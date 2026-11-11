// Determinism test for the *Deterministic decompression functions.
//
// determinism_test generate <dataDir>
//     Compresses synthetic clips and writes them to dataDir. Compression is not deterministic across platforms,
//     so the generated files are committed and every platform decompresses the same bytes.
// determinism_test hash <dataDir> [outFile]
//     Prints one "det <case> <hash>" line per test case. These must match on every platform and CPU feature level.
//     The cases run three times: in the default floating-point environment, with the flush mode Burst's
//     FloatMode.Deterministic sets ("[burst]" lines), and with every flush mode plus round-toward-zero ("[hostile]" lines).
//     Also prints "simd <case> <hash>" lines for the regular functions (expected to vary per platform) and
//     fails if the deterministic results deviate from the regular results by more than a small tolerance.
// determinism_test verify <dataDir> <expectedFile> [outFile]
//     Like hash, but also fails if any "det" line differs from expectedFile.
// determinism_test bench <dataDir>
//     Times the deterministic functions against the regular functions.

#include "compression.h"
#include "decompression.h"
#include "../src/fp_environment.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace
{
	const int   kNumBones    = 70;
	const int   kNumSamples  = 75;
	const float kSampleRate  = 30.f;
	const int   kNumScalars  = 40;
	const short kCompressionLevel = 2;

	struct AlignedBuffer
	{
		std::vector<uint8_t> storage;
		uint8_t*             data = nullptr;
		size_t               size = 0;

		void resize(size_t newSize)
		{
			storage.assign(newSize + 16, 0);
			uintptr_t address = reinterpret_cast<uintptr_t>(storage.data());
			data = storage.data() + ((16 - (address & 15)) & 15);
			size = newSize;
		}
	};

	bool readFile(const std::string& path, AlignedBuffer& buffer)
	{
		FILE* file = std::fopen(path.c_str(), "rb");
		if (!file)
			return false;
		std::fseek(file, 0, SEEK_END);
		long size = std::ftell(file);
		std::fseek(file, 0, SEEK_SET);
		buffer.resize(static_cast<size_t>(size));
		size_t read = std::fread(buffer.data, 1, buffer.size, file);
		std::fclose(file);
		return read == buffer.size;
	}

	bool writeFile(const std::string& path, const void* data, size_t size)
	{
		FILE* file = std::fopen(path.c_str(), "wb");
		if (!file)
			return false;
		size_t written = std::fwrite(data, 1, size, file);
		std::fclose(file);
		return written == size;
	}

	float floatFromBits(uint32_t bits)
	{
		float result;
		std::memcpy(&result, &bits, sizeof(float));
		return result;
	}

	uint32_t bitsFromFloat(float value)
	{
		uint32_t result;
		std::memcpy(&result, &value, sizeof(float));
		return result;
	}

	// Generation only runs on one machine, so it may use libm.
	void makeRotation(int bone, float t, float* outQuat)
	{
		float ax    = std::sin(1.7f * bone + 0.3f);
		float ay    = std::cos(1.3f * bone);
		float az    = 0.5f + 0.1f * bone;
		float len   = std::sqrt(ax * ax + ay * ay + az * az);
		float angle = (0.4f + 0.02f * bone) * std::sin(2.1f * t + 0.7f * bone) + 0.1f * bone;
		float s     = std::sin(angle * 0.5f) / len;
		outQuat[0]  = ax * s;
		outQuat[1]  = ay * s;
		outQuat[2]  = az * s;
		outQuat[3]  = std::cos(angle * 0.5f);
	}

	void makeSkeletonClip(bool withScales, std::vector<short>& parents, AlignedBuffer& aos)
	{
		parents.resize(kNumBones);
		for (int i = 0; i < kNumBones; i++)
			parents[i] = static_cast<short>(i == 0 ? 0 : (i - 1) / 3);

		aos.resize(sizeof(float) * 12 * kNumBones * kNumSamples);
		float* data = reinterpret_cast<float*>(aos.data);
		for (int bone = 0; bone < kNumBones; bone++)
		{
			int pattern = bone % 5;
			for (int sample = 0; sample < kNumSamples; sample++)
			{
				float  t = sample / kSampleRate;
				float* q = data + 12 * (bone * kNumSamples + sample);
				switch (pattern)
				{
					case 0:  // Animated rotation and translation
						makeRotation(bone, t, q);
						q[4] = 0.1f * bone + 0.3f * std::sin(3.f * t);
						q[5] = 0.25f * std::cos(1.5f * t + bone);
						q[6] = -0.05f * bone;
						break;
					case 1:  // Constant rotation, animated translation
						makeRotation(bone, 0.f, q);
						q[4] = 0.02f * bone;
						q[5] = 0.5f * std::sin(0.9f * t);
						q[6] = 0.f;
						break;
					case 2:  // Default track
						q[0] = 0.f; q[1] = 0.f; q[2] = 0.f; q[3] = 1.f;
						q[4] = 0.f; q[5] = 0.f; q[6] = 0.f;
						break;
					case 3:  // Animated rotation and non-uniform stretch
						makeRotation(bone, t, q);
						q[4] = 0.3f;
						q[5] = 0.1f * bone;
						q[6] = 0.f;
						break;
					default:  // Large translations
						makeRotation(bone, 0.25f * t, q);
						q[4] = 150.f + 20.f * std::sin(t);
						q[5] = -320.f + 5.f * t;
						q[6] = 75.f * std::cos(0.5f * t + bone);
						break;
				}
				q[7] = 0.f;
				bool stretch = pattern == 3;
				q[8]  = stretch ? 1.f + 0.2f * std::sin(2.f * t + bone) : 1.f;
				q[9]  = stretch ? 1.f - 0.1f * std::sin(1.f * t) : 1.f;
				q[10] = stretch ? 0.8f : 1.f;
				q[11] = (withScales && (bone % 4) == 1) ? 1.f + 0.25f * std::sin(1.2f * t + bone) : 1.f;
			}
		}
	}

	void* compressSkeleton(bool withScales, int& outSize, void*& outScales, int& outScalesSize)
	{
		std::vector<short> parents;
		AlignedBuffer      aos;
		makeSkeletonClip(withScales, parents, aos);
		const float* data = reinterpret_cast<const float*>(aos.data);

		outScales     = nullptr;
		outScalesSize = 0;
		std::vector<float> sampledScales;
		if (withScales)
		{
			// Mirrors AclUnity.Compression.CompressSkeletonClip in the Latios Framework.
			std::vector<float> scales(kNumBones * kNumSamples);
			for (int i = 0; i < kNumBones * kNumSamples; i++)
				scales[i] = data[12 * i + 11];
			std::vector<float> errors(kNumBones, 0.00001f);
			outScales = compressScalarsClip(kNumBones, kCompressionLevel, scales.data(), kNumSamples, kSampleRate, errors.data(), &outScalesSize);
			sampledScales.resize(kNumBones * kNumSamples);
			for (int sample = 0; sample < kNumSamples; sample++)
				sampleFloats(outScales, sampledScales.data() + sample * kNumBones, sample / kSampleRate, 3);
		}

		return compressSkeletonClip(parents.data(),
									kNumBones,
									kCompressionLevel,
									data,
									kNumSamples,
									kSampleRate,
									0.0001f,
									0.03f,
									&outSize,
									withScales ? sampledScales.data() : nullptr);
	}

	void* compressScalars(int& outSize)
	{
		std::vector<float> values(kNumScalars * kNumSamples);
		std::vector<float> errors(kNumScalars);
		for (int track = 0; track < kNumScalars; track++)
		{
			errors[track] = (track % 3) == 0 ? 0.001f : 0.00001f;
			for (int sample = 0; sample < kNumSamples; sample++)
			{
				float t = sample / kSampleRate;
				float v;
				switch (track % 4)
				{
					case 0: v = 0.5f; break;
					case 1: v = std::sin(1.3f * t + track); break;
					case 2: v = 1000.f * std::cos(0.2f * t) + track; break;
					default: v = 0.001f * t * t - 0.01f * track; break;
				}
				values[track * kNumSamples + sample] = v;
			}
		}
		return compressScalarsClip(kNumScalars, kCompressionLevel, values.data(), kNumSamples, kSampleRate, errors.data(), &outSize);
	}

	int generate(const std::string& dir)
	{
		int   size = 0, scalesSize = 0;
		void* scales = nullptr;
		void* clip   = compressSkeleton(false, size, scales, scalesSize);
		bool  ok     = writeFile(dir + "/skeleton.bin", clip, size);
		disposeCompressedTracksBuffer(clip);

		clip = compressSkeleton(true, size, scales, scalesSize);
		ok &= writeFile(dir + "/skeleton_scaled.bin", clip, size);
		ok &= writeFile(dir + "/skeleton_scaled_scales.bin", scales, scalesSize);
		disposeCompressedTracksBuffer(clip);
		disposeCompressedTracksBuffer(scales);

		clip = compressScalars(size);
		ok &= writeFile(dir + "/scalars.bin", clip, size);
		disposeCompressedTracksBuffer(clip);

		std::printf(ok ? "Generated test data in %s\n" : "Failed to write test data to %s\n", dir.c_str());
		return ok ? 0 : 1;
	}

	struct TestData
	{
		AlignedBuffer skeleton;
		AlignedBuffer skeletonScaled;
		AlignedBuffer skeletonScales;
		AlignedBuffer scalars;
		float         duration = 0.f;

		bool load(const std::string& dir)
		{
			duration = (kNumSamples - 1) / kSampleRate;
			return readFile(dir + "/skeleton.bin", skeleton) && readFile(dir + "/skeleton_scaled.bin", skeletonScaled) &&
				   readFile(dir + "/skeleton_scaled_scales.bin", skeletonScales) && readFile(dir + "/scalars.bin", scalars);
		}
	};

	std::vector<float> makeTimes(float duration)
	{
		// Multiples of 1/128 are exact, so every platform passes the same bit patterns.
		std::vector<float> times;
		for (int k = 0; static_cast<float>(k) * 0.0078125f <= duration; k++)
			times.push_back(static_cast<float>(k) * 0.0078125f);
		times.push_back(duration);
		const uint32_t awkward[] = { 0x3F9D70A4u, 0x3E4CCCCDu, 0x3DCCCCCDu, 0x400CCCCDu, 0x3F7FFFFFu, 0x00000001u };
		for (uint32_t bits : awkward)
		{
			float t = floatFromBits(bits);
			if (t <= duration)
				times.push_back(t);
		}
		return times;
	}

	struct Hasher
	{
		uint64_t hash = 14695981039346656037ull;

		void add(float value)
		{
			uint32_t bits = bitsFromFloat(value);
			for (int i = 0; i < 4; i++)
			{
				hash ^= (bits >> (8 * i)) & 0xffu;
				hash *= 1099511628211ull;
			}
		}
	};

	struct Comparison
	{
		double maxRelativeError = 0.0;
		int    failures         = 0;
		int    worstLane        = -1;
		float  worstDet         = 0.f;
		float  worstSimd        = 0.f;

		void add(float deterministic, float simd, int lane)
		{
			double scale = std::fmax(1.0, std::fabs(static_cast<double>(simd)));
			double error = std::fabs(static_cast<double>(deterministic) - static_cast<double>(simd)) / scale;
			if (error > maxRelativeError)
			{
				maxRelativeError = error;
				worstLane        = lane;
				worstDet         = deterministic;
				worstSimd        = simd;
			}
			if (!(error <= 1e-4))
				failures++;
		}
	};

	const float kSentinel = -12345.f;

	struct CaseResult
	{
		Hasher     det;
		Hasher     simd;
		Comparison comparison;

		// skipLane7 skips the translation padding, which is undefined for the non-blended functions.
		void addPose(const std::vector<float>& det_, const std::vector<float>& simd_, bool skipLane7)
		{
			for (size_t i = 0; i < det_.size(); i++)
			{
				if (skipLane7 && (i % 12) == 7)
					continue;
				det.add(det_[i]);
				simd.add(simd_[i]);
				comparison.add(det_[i], simd_[i], static_cast<int>(i % 12));
			}
		}

		void addValues(const std::vector<float>& det_, const std::vector<float>& simd_)
		{
			for (size_t i = 0; i < det_.size(); i++)
			{
				det.add(det_[i]);
				simd.add(simd_[i]);
				comparison.add(det_[i], simd_[i], -1);
			}
		}
	};

	struct Report
	{
		FILE*                    out = nullptr;
		std::string              tag;
		int                      failures = 0;
		std::vector<std::string> detLines;

		void emit(const std::string& name, const CaseResult& result)
		{
			char line[512];
			std::snprintf(line, sizeof(line), "det %s %016llx\n", name.c_str(), static_cast<unsigned long long>(result.det.hash));
			write(line);
			detLines.push_back(line);
			std::snprintf(line, sizeof(line), "simd %s %016llx\n", name.c_str(), static_cast<unsigned long long>(result.simd.hash));
			write(line);
			std::snprintf(line, sizeof(line), "maxerror %s %.3e lane %d det %.9g simd %.9g%s\n", name.c_str(), result.comparison.maxRelativeError,
						  result.comparison.worstLane, result.comparison.worstDet, result.comparison.worstSimd, result.comparison.failures ? " FAIL" : "");
			write(line);
			if (result.comparison.failures)
				failures++;
		}

		std::string prefix() const { return tag.empty() ? std::string() : "[" + tag + "] "; }

		void write(const char* line)
		{
			std::string text = prefix() + line;
			std::fputs(text.c_str(), stdout);
			if (out)
				std::fputs(text.c_str(), out);
		}
	};

	const unsigned long long kBoneMask[2]   = { 0xF0F0F0F0F0F0F0F5ull, 0x2Aull };
	const unsigned long long kScalarMask[1] = { 0x000000A5A5A5A5A5ull };

	// Computed before any floating-point environment changes, so every pass passes the same bit patterns.
	struct Inputs
	{
		std::vector<float> times;
		std::vector<float> reverseTimes;
		float              blendFirst = 0.375f;
		float              blendAdd   = 0.625f;
		// Just above the smallest normal float, so that its products with values below 1 are denormal.
		float              blendTiny = floatFromBits(0x00C00000u);
	};

	struct NamedResult
	{
		std::string name;
		CaseResult  result;
	};

	void runCases(const TestData& data, const Inputs& in, std::vector<NamedResult>& results)
	{
		const size_t poseFloats = 12 * kNumBones;

		struct SkeletonVariant { const char* name; const void* transforms; const void* scales; };
		const SkeletonVariant variants[] = {
			{ "skeleton", data.skeleton.data, nullptr },
			{ "scaled", data.skeletonScaled.data, data.skeletonScales.data },
		};

		for (const SkeletonVariant& variant : variants)
		{
			for (unsigned char mode = 0; mode < 4; mode++)
			{
				const std::string suffix = std::string(variant.name) + "_mode" + std::to_string(mode);
				CaseResult pose, blended, tiny, masked, maskedBlended, bone;
				std::vector<float> det(poseFloats), simd(poseFloats);
				std::vector<float> detBone(12), simdBone(12);
				for (size_t ti = 0; ti < in.times.size(); ti++)
				{
					const float t  = in.times[ti];
					const float rt = in.reverseTimes[ti];

					samplePoseDeterministic(variant.transforms, variant.scales, det.data(), t, mode);
					samplePose(variant.transforms, variant.scales, simd.data(), t, mode);
					pose.addPose(det, simd, true);

					samplePoseBlendedFirstDeterministic(variant.transforms, variant.scales, det.data(), in.blendFirst, t, mode);
					samplePoseBlendedAddDeterministic(variant.transforms, variant.scales, det.data(), in.blendAdd, rt, mode);
					samplePoseBlendedFirst(variant.transforms, variant.scales, simd.data(), in.blendFirst, t, mode);
					samplePoseBlendedAdd(variant.transforms, variant.scales, simd.data(), in.blendAdd, rt, mode);
					blended.addPose(det, simd, false);

					samplePoseBlendedFirstDeterministic(variant.transforms, variant.scales, det.data(), in.blendTiny, t, mode);
					samplePoseBlendedAddDeterministic(variant.transforms, variant.scales, det.data(), in.blendTiny, rt, mode);
					samplePoseBlendedFirst(variant.transforms, variant.scales, simd.data(), in.blendTiny, t, mode);
					samplePoseBlendedAdd(variant.transforms, variant.scales, simd.data(), in.blendTiny, rt, mode);
					tiny.addPose(det, simd, false);

					std::fill(det.begin(), det.end(), kSentinel);
					std::fill(simd.begin(), simd.end(), kSentinel);
					samplePoseMaskedDeterministic(variant.transforms, variant.scales, det.data(), kBoneMask, t, mode);
					samplePoseMasked(variant.transforms, variant.scales, simd.data(), kBoneMask, t, mode);
					masked.addPose(det, simd, true);

					std::fill(det.begin(), det.end(), kSentinel);
					std::fill(simd.begin(), simd.end(), kSentinel);
					samplePoseMaskedBlendedFirstDeterministic(variant.transforms, variant.scales, det.data(), kBoneMask, in.blendFirst, t, mode);
					samplePoseMaskedBlendedAddDeterministic(variant.transforms, variant.scales, det.data(), kBoneMask, in.blendAdd, rt, mode);
					samplePoseMaskedBlendedFirst(variant.transforms, variant.scales, simd.data(), kBoneMask, in.blendFirst, t, mode);
					samplePoseMaskedBlendedAdd(variant.transforms, variant.scales, simd.data(), kBoneMask, in.blendAdd, rt, mode);
					maskedBlended.addPose(det, simd, false);

					for (int i = 0; i < kNumBones; i++)
					{
						sampleBoneDeterministic(variant.transforms, variant.scales, detBone.data(), i, t, mode);
						sampleBone(variant.transforms, variant.scales, simdBone.data(), i, t, mode);
						bone.addPose(detBone, simdBone, true);
					}
				}
				results.push_back({ "samplePose_" + suffix, pose });
				results.push_back({ "samplePoseBlended_" + suffix, blended });
				results.push_back({ "samplePoseBlendedTiny_" + suffix, tiny });
				results.push_back({ "samplePoseMasked_" + suffix, masked });
				results.push_back({ "samplePoseMaskedBlended_" + suffix, maskedBlended });
				results.push_back({ "sampleBone_" + suffix, bone });
			}
		}

		for (unsigned char mode = 0; mode < 4; mode++)
		{
			const std::string suffix = "mode" + std::to_string(mode);
			const void* clip = data.scalars.data;
			CaseResult floats, blended, tiny, masked, maskedBlended, single;
			std::vector<float> det(kNumScalars), simd(kNumScalars);
			for (size_t ti = 0; ti < in.times.size(); ti++)
			{
				const float t  = in.times[ti];
				const float rt = in.reverseTimes[ti];

				sampleFloatsDeterministic(clip, det.data(), t, mode);
				sampleFloats(clip, simd.data(), t, mode);
				floats.addValues(det, simd);

				sampleFloatsBlendedFirstDeterministic(clip, det.data(), in.blendFirst, t, mode);
				sampleFloatsBlendedAddDeterministic(clip, det.data(), in.blendAdd, rt, mode);
				sampleFloatsBlendedFirst(clip, simd.data(), in.blendFirst, t, mode);
				sampleFloatsBlendedAdd(clip, simd.data(), in.blendAdd, rt, mode);
				blended.addValues(det, simd);

				sampleFloatsBlendedFirstDeterministic(clip, det.data(), in.blendTiny, t, mode);
				sampleFloatsBlendedAddDeterministic(clip, det.data(), in.blendTiny, rt, mode);
				sampleFloatsBlendedFirst(clip, simd.data(), in.blendTiny, t, mode);
				sampleFloatsBlendedAdd(clip, simd.data(), in.blendTiny, rt, mode);
				tiny.addValues(det, simd);

				std::fill(det.begin(), det.end(), kSentinel);
				std::fill(simd.begin(), simd.end(), kSentinel);
				sampleFloatsMaskedDeterministic(clip, det.data(), kScalarMask, t, mode);
				sampleFloatsMasked(clip, simd.data(), kScalarMask, t, mode);
				masked.addValues(det, simd);

				std::fill(det.begin(), det.end(), kSentinel);
				std::fill(simd.begin(), simd.end(), kSentinel);
				sampleFloatsMaskedBlendedFirstDeterministic(clip, det.data(), kScalarMask, in.blendFirst, t, mode);
				sampleFloatsMaskedBlendedAddDeterministic(clip, det.data(), kScalarMask, in.blendAdd, rt, mode);
				sampleFloatsMaskedBlendedFirst(clip, simd.data(), kScalarMask, in.blendFirst, t, mode);
				sampleFloatsMaskedBlendedAdd(clip, simd.data(), kScalarMask, in.blendAdd, rt, mode);
				maskedBlended.addValues(det, simd);

				for (int i = 0; i < kNumScalars; i++)
				{
					det[i]  = sampleFloatDeterministic(clip, i, t, mode);
					simd[i] = sampleFloat(clip, i, t, mode);
				}
				single.addValues(det, simd);
			}
			results.push_back({ "sampleFloats_" + suffix, floats });
			results.push_back({ "sampleFloatsBlended_" + suffix, blended });
			results.push_back({ "sampleFloatsBlendedTiny_" + suffix, tiny });
			results.push_back({ "sampleFloatsMasked_" + suffix, masked });
			results.push_back({ "sampleFloatsMaskedBlended_" + suffix, maskedBlended });
			results.push_back({ "sampleFloat_" + suffix, single });
		}
	}

	// The passes run the same cases under different caller floating-point environments.
	// The deterministic results must not change, while the regular functions may.
	enum class Environment
	{
		Default,
		BurstDeterministic,
		Hostile
	};

	const char* environmentTag(Environment environment)
	{
		switch (environment)
		{
			case Environment::BurstDeterministic: return "burst";
			case Environment::Hostile: return "hostile";
			default: return "";
		}
	}

	AclUnityFp::Register modifyEnvironment(AclUnityFp::Register value, Environment environment)
	{
		switch (environment)
		{
			case Environment::BurstDeterministic: return value | AclUnityFp::kBurstDeterministicBits;
			case Environment::Hostile: return (value & ~AclUnityFp::kRoundingBits) | AclUnityFp::kFlushBits | AclUnityFp::kRoundTowardZero;
			default: return value;
		}
	}

	bool readExpected(const char* expectedPath, std::vector<std::string>& expected)
	{
		FILE* file = std::fopen(expectedPath, "r");
		if (!file)
			return false;
		char buffer[512];
		while (std::fgets(buffer, sizeof(buffer), file))
		{
			std::string line(buffer);
			while (!line.empty() && (line.back() == '\n' || line.back() == '\r'))
				line.pop_back();
			if (line.compare(0, 4, "det ") == 0)
				expected.push_back(line + "\n");
		}
		std::fclose(file);
		return true;
	}

	int verifyAgainst(const std::vector<std::string>& expected, const Report& report, const char* expectedPath)
	{
		const std::string prefix     = report.prefix();
		int               mismatches = 0;
		if (expected.size() != report.detLines.size())
		{
			std::printf("%sMISMATCH expected %d deterministic cases but computed %d\n", prefix.c_str(), static_cast<int>(expected.size()),
						static_cast<int>(report.detLines.size()));
			mismatches++;
		}
		for (size_t i = 0; i < expected.size() && i < report.detLines.size(); i++)
		{
			if (expected[i] != report.detLines[i])
			{
				std::printf("%sMISMATCH expected: %s         computed: %s", prefix.c_str(), expected[i].c_str(), report.detLines[i].c_str());
				mismatches++;
			}
		}
		std::printf("%s%d deterministic case(s) differ from %s\n", prefix.c_str(), mismatches, expectedPath);
		return mismatches;
	}

	int hash(const std::string& dir, const char* outPath, const char* expectedPath)
	{
		TestData data;
		if (!data.load(dir))
		{
			std::printf("Failed to load test data from %s\n", dir.c_str());
			return 1;
		}

		std::vector<std::string> expected;
		if (expectedPath && !readExpected(expectedPath, expected))
		{
			std::printf("Failed to open %s\n", expectedPath);
			return 1;
		}

		Inputs in;
		in.times = makeTimes(data.duration);
		for (float t : in.times)
			in.reverseTimes.push_back(data.duration - t);

		FILE* out        = outPath ? std::fopen(outPath, "w") : nullptr;
		int   deviations = 0;
		int   mismatches = 0;
		const Environment environments[] = { Environment::Default, Environment::BurstDeterministic, Environment::Hostile };
		for (Environment environment : environments)
		{
			std::vector<NamedResult>   results;
			const AclUnityFp::Register saved = AclUnityFp::read();
			AclUnityFp::write(modifyEnvironment(saved, environment));
			runCases(data, in, results);
			AclUnityFp::write(saved);

			Report report;
			report.out = out;
			report.tag = environmentTag(environment);
			for (const NamedResult& result : results)
				report.emit(result.name, result.result);
			// The regular functions are only expected to agree in the default environment. Under other rounding modes, they can
			// select different keyframes.
			std::printf("%s%d case(s) deviate from the regular functions%s\n", report.prefix().c_str(), report.failures,
						environment == Environment::Default ? "" : " (informational)");
			if (environment == Environment::Default)
				deviations += report.failures;
			if (expectedPath)
				mismatches += verifyAgainst(expected, report, expectedPath);
		}
		if (out)
			std::fclose(out);

		if (deviations != 0)
			return 2;
		return mismatches == 0 ? 0 : 3;
	}

	template<typename Func>
	double timeIt(int iterations, Func func)
	{
		auto start = std::chrono::high_resolution_clock::now();
		for (int i = 0; i < iterations; i++)
			func(i);
		auto end = std::chrono::high_resolution_clock::now();
		return std::chrono::duration<double, std::micro>(end - start).count() / iterations;
	}

	int bench(const std::string& dir)
	{
		TestData data;
		if (!data.load(dir))
		{
			std::printf("Failed to load test data from %s\n", dir.c_str());
			return 1;
		}
		std::vector<float> pose(12 * kNumBones);
		std::vector<float> floats(kNumScalars);
		const int   iterations = 20000;
		const float step       = data.duration / iterations;
		const void* t          = data.skeletonScaled.data;
		const void* s          = data.skeletonScales.data;
		const void* f          = data.scalars.data;
		float       sink       = 0.f;

		for (int pass = 0; pass < 2; pass++)
		{
			double a = timeIt(iterations, [&](int i) { samplePose(t, s, pose.data(), i * step, 0); });
			double b = timeIt(iterations, [&](int i) { samplePoseDeterministic(t, s, pose.data(), i * step, 0); });
			double c = timeIt(iterations, [&](int i) { samplePoseBlendedAdd(t, s, pose.data(), 0.5f, i * step, 0); });
			double d = timeIt(iterations, [&](int i) { samplePoseBlendedAddDeterministic(t, s, pose.data(), 0.5f, i * step, 0); });
			double e = timeIt(iterations, [&](int i) { sampleBone(t, s, pose.data(), i % kNumBones, i * step, 0); });
			double g = timeIt(iterations, [&](int i) { sampleBoneDeterministic(t, s, pose.data(), i % kNumBones, i * step, 0); });
			double h = timeIt(iterations, [&](int i) { sampleFloats(f, floats.data(), i * step, 0); });
			double k = timeIt(iterations, [&](int i) { sampleFloatsDeterministic(f, floats.data(), i * step, 0); });
			double m = timeIt(iterations, [&](int i) { sink += sampleFloat(f, i % kNumScalars, i * step, 0); });
			double n = timeIt(iterations, [&](int i) { sink += sampleFloatDeterministic(f, i % kNumScalars, i * step, 0); });

			const AclUnityFp::Register saved = AclUnityFp::read();
			AclUnityFp::write(modifyEnvironment(saved, Environment::BurstDeterministic));
			double bs = timeIt(iterations, [&](int i) { samplePoseDeterministic(t, s, pose.data(), i * step, 0); });
			double gs = timeIt(iterations, [&](int i) { sampleBoneDeterministic(t, s, pose.data(), i % kNumBones, i * step, 0); });
			double ns = timeIt(iterations, [&](int i) { sink += sampleFloatDeterministic(f, i % kNumScalars, i * step, 0); });
			AclUnityFp::write(saved);

			if (pass == 0)
				continue;
			std::printf("perf samplePose            %8.3f us  deterministic %8.3f us  (%.2fx)  under Burst's flush mode %8.3f us\n", a, b, b / a, bs);
			std::printf("perf samplePoseBlendedAdd  %8.3f us  deterministic %8.3f us  (%.2fx)\n", c, d, d / c);
			std::printf("perf sampleBone            %8.3f us  deterministic %8.3f us  (%.2fx)  under Burst's flush mode %8.3f us\n", e, g, g / e, gs);
			std::printf("perf sampleFloats          %8.3f us  deterministic %8.3f us  (%.2fx)\n", h, k, k / h);
			std::printf("perf sampleFloat           %8.3f us  deterministic %8.3f us  (%.2fx)  under Burst's flush mode %8.3f us\n", m, n, n / m, ns);
		}
		return sink == 12345.f ? 1 : 0;
	}
}

int main(int argc, char** argv)
{
	if (argc >= 3 && std::strcmp(argv[1], "generate") == 0)
		return generate(argv[2]);
	if (argc >= 3 && std::strcmp(argv[1], "hash") == 0)
		return hash(argv[2], argc >= 4 ? argv[3] : nullptr, nullptr);
	if (argc >= 4 && std::strcmp(argv[1], "verify") == 0)
		return hash(argv[2], argc >= 5 ? argv[4] : nullptr, argv[3]);
	if (argc >= 3 && std::strcmp(argv[1], "bench") == 0)
		return bench(argv[2]);
	std::printf("usage:\n  determinism_test generate <dataDir>\n  determinism_test hash <dataDir> [outFile]\n"
				"  determinism_test verify <dataDir> <expectedFile> [outFile]\n  determinism_test bench <dataDir>\n");
	return 1;
}
