#include "decompression.h"

#include "acl/decompression/decompress.h"

using namespace acl;

namespace 
{
	class PoseTrackWriter : public track_writer
	{
	private:
		float* m_outputBuffer;

	public:
		PoseTrackWriter(float* outputBuffer) : m_outputBuffer(outputBuffer) {}
		
		RTM_FORCE_INLINE void RTM_SIMD_CALL write_rotation(uint32_t track_index, rtm::quatf_arg0 rotation)
		{
			rtm::quat_store(rotation, m_outputBuffer + 12 * track_index);
		}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_translation(uint32_t track_index, rtm::vector4f_arg0 translation)
		{
			rtm::vector_store(translation, m_outputBuffer + 12 * track_index + 4);
		}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_scale(uint32_t track_index, rtm::vector4f_arg0 scale)
		{
			rtm::vector_store(rtm::vector_set_w(scale, 1.f), m_outputBuffer + 12 * track_index + 8);
		}
	};

	class PoseBlendedFirstTrackWriter : public track_writer
	{
	private:
		float* m_outputBuffer;
		rtm::vector4f m_blendFactor;

	public:
		PoseBlendedFirstTrackWriter(float* outputBuffer, float blendFactor) : m_outputBuffer(outputBuffer), m_blendFactor(rtm::vector_broadcast(&blendFactor)) {}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_rotation(uint32_t track_index, rtm::quatf_arg0 rotation)
		{
			rtm::quat_store(rtm::vector_mul(rtm::quat_to_vector(rotation), m_blendFactor), m_outputBuffer + 12 * track_index);
		}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_translation(uint32_t track_index, rtm::vector4f_arg0 translation)
		{
			auto weightedTranslation = rtm::vector_set_w(translation, 1.0f);
			rtm::vector_store(rtm::vector_mul(weightedTranslation, m_blendFactor), m_outputBuffer + 12 * track_index + 4);
		}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_scale(uint32_t track_index, rtm::vector4f_arg0 scale)
		{
			rtm::vector_store(rtm::vector_mul(rtm::vector_set_w(scale, 1.f), m_blendFactor), m_outputBuffer + 12 * track_index + 8);
		}
	};

	// Note: RTM doesn't currently call out FMA for AVX. However, MSVC 2019 will generate it anyways.
	class PoseBlendedAddTrackWriter : public track_writer
	{
	private:
		float* m_outputBuffer;
		rtm::vector4f m_blendFactor;
		float m_uniformScale;

	public:
		PoseBlendedAddTrackWriter(float* outputBuffer, float blendFactor, float uniformScale) : 
			m_outputBuffer(outputBuffer), 
			m_blendFactor(rtm::vector_broadcast(&blendFactor)),
			m_uniformScale(uniformScale)
		{}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_rotation(uint32_t track_index, rtm::quatf_arg0 rotation)
		{
			auto dst = m_outputBuffer + 12 * track_index;
			auto prevRot = rtm::vector_load(dst);
			auto newRot = rtm::quat_to_vector(rotation);
			// Todo: Is there something faster than this branch?
			newRot = rtm::vector_dot(prevRot, newRot) < 0.0f ? rtm::vector_neg(newRot) : newRot;
			rtm::vector_store(rtm::vector_mul_add(newRot, m_blendFactor, prevRot), dst);
		}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_translation(uint32_t track_index, rtm::vector4f_arg0 translation)
		{
			auto dst = m_outputBuffer + 12 * track_index + 4;
			auto weightedTranslation = rtm::vector_set_w(translation, 1.0f);
			rtm::vector_store(rtm::vector_mul_add(weightedTranslation, m_blendFactor, rtm::vector_load(dst)), dst);
		}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_scale(uint32_t track_index, rtm::vector4f_arg0 scale)
		{
			auto dst = m_outputBuffer + 12 * track_index + 8;
			rtm::vector_store(rtm::vector_mul_add(rtm::vector_set_w(scale, m_uniformScale), m_blendFactor, rtm::vector_load(dst)), dst);
		}
	};

	class PoseMaskedTrackWriter : public PoseTrackWriter
	{
	private:
		const std::uint64_t* m_mask;

	public:
		PoseMaskedTrackWriter(float* outputBuffer, const std::uint64_t* mask) : PoseTrackWriter(outputBuffer), m_mask(mask) {}

		bool skip_track_rotation(uint32_t track_index) const { return (m_mask[track_index >> 6] & (1ull << (track_index & 0x3f))) != 0; }
		bool skip_track_translation(uint32_t track_index) const { return (m_mask[track_index >> 6] & (1ull << (track_index & 0x3f))) != 0; }
		bool skip_track_scale(uint32_t track_index) const { return (m_mask[track_index >> 6] & (1ull << (track_index & 0x3f))) != 0; }
	};

	class PoseBlendedFirstMaskedTrackWriter : public PoseBlendedFirstTrackWriter
	{
	private:
		const std::uint64_t* m_mask;

	public:
		PoseBlendedFirstMaskedTrackWriter(float* outputBuffer, const std::uint64_t* mask, float blendFactor) : PoseBlendedFirstTrackWriter(outputBuffer, blendFactor), m_mask(mask) {}

		bool skip_track_rotation(uint32_t track_index) const { return (m_mask[track_index >> 6] & (1ull << (track_index & 0x3f))) != 0; }
		bool skip_track_translation(uint32_t track_index) const { return (m_mask[track_index >> 6] & (1ull << (track_index & 0x3f))) != 0; }
		bool skip_track_scale(uint32_t track_index) const { return (m_mask[track_index >> 6] & (1ull << (track_index & 0x3f))) != 0; }
	};

	class PoseBlendedAddMaskedTrackWriter : public PoseBlendedAddTrackWriter
	{
	private:
		const std::uint64_t* m_mask;

	public:
		PoseBlendedAddMaskedTrackWriter(float* outputBuffer, const std::uint64_t* mask, float blendFactor, float uniformScale) : PoseBlendedAddTrackWriter(outputBuffer, blendFactor, uniformScale), m_mask(mask) {}

		bool skip_track_rotation(uint32_t track_index) const { return (m_mask[track_index >> 6] & (1ull << (track_index & 0x3f))) != 0; }
		bool skip_track_translation(uint32_t track_index) const { return (m_mask[track_index >> 6] & (1ull << (track_index & 0x3f))) != 0; }
		bool skip_track_scale(uint32_t track_index) const { return (m_mask[track_index >> 6] & (1ull << (track_index & 0x3f))) != 0; }
	};

	class UniformScaleTrackWriter : public track_writer
	{
	private:
		float* m_outputBuffer;

	public:
		UniformScaleTrackWriter(float* outputBuffer) : m_outputBuffer(outputBuffer) {}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_float1(uint32_t track_index, rtm::scalarf_arg0 value)
		{
			rtm::scalar_store(value, m_outputBuffer + track_index * 12 + 11);
		}
	};

	class UniformScaleBlendedFirstTrackWriter : public track_writer
	{
	private:
		float* m_outputBuffer;
		rtm::scalarf m_blendFactor;

	public:
		UniformScaleBlendedFirstTrackWriter(float* outputBuffer, float blendFactor) : m_outputBuffer(outputBuffer), m_blendFactor(rtm::scalar_set(blendFactor)) {}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_float1(uint32_t track_index, rtm::scalarf_arg0 value)
		{
			rtm::scalar_store(rtm::scalar_mul(value, m_blendFactor), m_outputBuffer + track_index * 12 + 11);
		}
	};

	class UniformScaleBlendedAddTrackWriter : public track_writer
	{
	private:
		float* m_outputBuffer;
		rtm::scalarf m_blendFactor;

	public:
		UniformScaleBlendedAddTrackWriter(float* outputBuffer, float blendFactor) : m_outputBuffer(outputBuffer), m_blendFactor(rtm::scalar_set(blendFactor)) {}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_float1(uint32_t track_index, rtm::scalarf_arg0 value)
		{
			auto dst = m_outputBuffer + track_index * 12 + 11;
			auto existing = rtm::scalar_load(dst);
			rtm::scalar_store(rtm::scalar_mul_add(value, m_blendFactor, existing), dst);
		}
	};

	class UniformScaleMaskedTrackWriter : public track_writer
	{
	private:
		float* m_outputBuffer;
		const std::uint64_t* m_mask;

	public:
		UniformScaleMaskedTrackWriter(float* outputBuffer, const std::uint64_t* mask) : m_outputBuffer(outputBuffer), m_mask(mask) {}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_float1(uint32_t track_index, rtm::scalarf_arg0 value)
		{
			// Todo: Need to add track skipping to ACL for scalar tracks
			auto ulong = m_mask[track_index >> 6];
			bool isBitSet = (ulong & (1ull << (track_index & 0x3f))) != 0;
			if (isBitSet)
				rtm::scalar_store(value, m_outputBuffer + track_index * 12 + 11);
		}
	};

	class UniformScaleBlendedFirstMaskedTrackWriter : public track_writer
	{
	private:
		float* m_outputBuffer;
		const std::uint64_t* m_mask;
		rtm::scalarf m_blendFactor;

	public:
		UniformScaleBlendedFirstMaskedTrackWriter(float* outputBuffer, const std::uint64_t* mask, float blendFactor) : m_outputBuffer(outputBuffer), m_mask(mask), m_blendFactor(rtm::scalar_set(blendFactor)) {}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_float1(uint32_t track_index, rtm::scalarf_arg0 value)
		{
			// Todo: Need to add track skipping to ACL for scalar tracks
			auto ulong = m_mask[track_index >> 6];
			bool isBitSet = (ulong & (1ull << (track_index & 0x3f))) != 0;
			if (isBitSet)
				rtm::scalar_store(rtm::scalar_mul(value, m_blendFactor), m_outputBuffer + track_index * 12 + 11);
		}
	};

	class UniformScaleBlendedAddMaskedTrackWriter : public track_writer
	{
	private:
		float* m_outputBuffer;
		const std::uint64_t* m_mask;
		rtm::scalarf m_blendFactor;

	public:
		UniformScaleBlendedAddMaskedTrackWriter(float* outputBuffer, const std::uint64_t* mask, float blendFactor) : m_outputBuffer(outputBuffer), m_mask(mask), m_blendFactor(rtm::scalar_set(blendFactor)) {}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_float1(uint32_t track_index, rtm::scalarf_arg0 value)
		{
			// Todo: Need to add track skipping to ACL for scalar tracks
			auto ulong = m_mask[track_index >> 6];
			bool isBitSet = (ulong & (1ull << (track_index & 0x3f))) != 0;
			if (isBitSet)
			{
				auto dst = m_outputBuffer + track_index * 12 + 11;
				rtm::scalar_store(rtm::scalar_mul_add(value, m_blendFactor, rtm::scalar_load(dst)), dst);
			}
		}
	};

	class BoneTrackWriter : public track_writer
	{
	private:
		float* m_outputBuffer;

	public:
		BoneTrackWriter(float* outputBuffer) : m_outputBuffer(outputBuffer) {}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_rotation(uint32_t track_index, rtm::quatf_arg0 rotation)
		{
			(void)track_index;
			rtm::quat_store(rotation, m_outputBuffer);
		}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_translation(uint32_t track_index, rtm::vector4f_arg0 translation)
		{
			(void)track_index;
			rtm::vector_store(translation, m_outputBuffer + 4);
		}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_scale(uint32_t track_index, rtm::vector4f_arg0 scale)
		{
			(void)track_index;
			rtm::vector_store(scale, m_outputBuffer + 8);
		}
	};

	class MultiFloatTrackWriter : public track_writer
	{
	private:
		float* m_outputBuffer;

	public:
		MultiFloatTrackWriter(float* outputBuffer) : m_outputBuffer(outputBuffer) {}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_float1(uint32_t track_index, rtm::scalarf_arg0 value)
		{
			rtm::scalar_store(value, m_outputBuffer + track_index);
		}
	};

	class MultiFloatBlendedFirstTrackWriter : public track_writer
	{
	private:
		float* m_outputBuffer;
		rtm::scalarf m_blendFactor;

	public:
		MultiFloatBlendedFirstTrackWriter(float* outputBuffer, float blendFactor) : m_outputBuffer(outputBuffer), m_blendFactor(rtm::scalar_set(blendFactor)) {}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_float1(uint32_t track_index, rtm::scalarf_arg0 value)
		{
			rtm::scalar_store(rtm::scalar_mul(value, m_blendFactor), m_outputBuffer + track_index);
		}
	};

	class MultiFloatBlendedAddTrackWriter : public track_writer
	{
	private:
		float* m_outputBuffer;
		rtm::scalarf m_blendFactor;

	public:
		MultiFloatBlendedAddTrackWriter(float* outputBuffer, float blendFactor) : m_outputBuffer(outputBuffer), m_blendFactor(rtm::scalar_set(blendFactor)) {}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_float1(uint32_t track_index, rtm::scalarf_arg0 value)
		{
			auto dst = m_outputBuffer + track_index;
			rtm::scalar_store(rtm::scalar_mul_add(value, m_blendFactor, rtm::scalar_load(dst)), dst);
		}
	};

	class MultiFloatMaskedTrackWriter : public track_writer
	{
	private:
		float* m_outputBuffer;
		const std::uint64_t* m_mask;

	public:
		MultiFloatMaskedTrackWriter(float* outputBuffer, const std::uint64_t* mask) : m_outputBuffer(outputBuffer), m_mask(mask) {}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_float1(uint32_t track_index, rtm::scalarf_arg0 value)
		{
			// Todo: Need to add track skipping to ACL for scalar tracks
			auto ulong = m_mask[track_index >> 6];
			bool isBitSet = (ulong & (1ull << (track_index & 0x3f))) != 0;
			if (isBitSet)
				rtm::scalar_store(value, m_outputBuffer + track_index);
		}
	};

	class MultiFloatBlendedFirstMaskedTrackWriter : public track_writer
	{
	private:
		float* m_outputBuffer;
		const std::uint64_t* m_mask;
		rtm::scalarf m_blendFactor;

	public:
		MultiFloatBlendedFirstMaskedTrackWriter(float* outputBuffer, const std::uint64_t* mask, float blendFactor) : m_outputBuffer(outputBuffer), m_mask(mask), m_blendFactor(rtm::scalar_set(blendFactor)) {}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_float1(uint32_t track_index, rtm::scalarf_arg0 value)
		{
			// Todo: Need to add track skipping to ACL for scalar tracks
			auto ulong = m_mask[track_index >> 6];
			bool isBitSet = (ulong & (1ull << (track_index & 0x3f))) != 0;
			if (isBitSet)
				rtm::scalar_store(rtm::scalar_mul(value, m_blendFactor), m_outputBuffer + track_index);
		}
	};

	class MultiFloatBlendedAddMaskedTrackWriter : public track_writer
	{
	private:
		float* m_outputBuffer;
		const std::uint64_t* m_mask;
		rtm::scalarf m_blendFactor;

	public:
		MultiFloatBlendedAddMaskedTrackWriter(float* outputBuffer, const std::uint64_t* mask, float blendFactor) : m_outputBuffer(outputBuffer), m_mask(mask), m_blendFactor(rtm::scalar_set(blendFactor)) {}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_float1(uint32_t track_index, rtm::scalarf_arg0 value)
		{
			// Todo: Need to add track skipping to ACL for scalar tracks
			auto ulong = m_mask[track_index >> 6];
			bool isBitSet = (ulong & (1ull << (track_index & 0x3f))) != 0;
			if (isBitSet)
			{
				auto dst = m_outputBuffer + track_index;
				rtm::scalar_store(rtm::scalar_mul_add(value, m_blendFactor, rtm::scalar_load(dst)), dst);
			}
		}
	};

	class SingleFloatTrackWriter : public track_writer
	{
	private:
		float* m_output;

	public:
		SingleFloatTrackWriter(float* outputBuffer) : m_output(outputBuffer) {}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_float1(uint32_t track_index, rtm::scalarf_arg0 value)
		{
			(void)track_index;
			rtm::scalar_store(value, m_output);
		}
	};

	class TransformDecompressionSettings : public default_transform_decompression_settings
	{
	public:
		// We perform these safety checks at the C# layer.
		// The most important check is ensuring that the compressed tracks object is aligned to a 16 byte boundary.
		// This will require custom offsets when working with BlobAssets.
		static constexpr bool skip_initialize_safety_checks() { return true; }

		// Force the version to the tagged 2.1 to decrease code size.
		static constexpr compressed_tracks_version16 version_supported() { return compressed_tracks_version16::v02_01_00; }
	};

	using TransformDecompressionContext = decompression_context<TransformDecompressionSettings>;

	class FloatDecompressionSettings : public decompression_settings
	{
	public:
		// We perform these safety checks at the C# layer.
		// The most important check is ensuring that the compressed tracks object is aligned to a 16 byte boundary.
		static constexpr bool skip_initialize_safety_checks() { return true; }

		static constexpr bool is_track_type_supported(track_type8 type) { return type == track_type8::float1f; }

		// Force the version to the tagged 2.1 to decrease code size.
		static constexpr compressed_tracks_version16 version_supported() { return compressed_tracks_version16::v02_01_00; }

		// Todo: Make a separate variant for this when a user requests this feature.
		static constexpr bool is_per_track_rounding_supported() { return false; }
	};

	using FloatDecompressionContext = decompression_context<FloatDecompressionSettings>;
}

// It is advantageous to perform as much trivial work between seek() and decompress_track[s]() because seek() prefetches.
// There isn't much but we can at least use that to configure the writer and to clamp index values.
ACL_UNITY_API void samplePose(const void* compressedTransformTracks, const void* compressedScaleTracks, float* outputBuffer, float time, unsigned char keyframeInterpolationMode)
{
	TransformDecompressionContext context;
	context.initialize(*static_cast<const compressed_tracks*>(compressedTransformTracks));
	context.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
	PoseTrackWriter writer(outputBuffer);
	if (compressedScaleTracks == nullptr)
		context.decompress_tracks(writer);
	else
	{
		FloatDecompressionContext scaleContext;
		// Todo: Is initializing a context here too heavy for prefetching?
		scaleContext.initialize(*static_cast<const compressed_tracks*>(compressedScaleTracks));
		context.decompress_tracks(writer);
		scaleContext.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
		UniformScaleTrackWriter scaleWriter(outputBuffer);
		scaleContext.decompress_tracks(scaleWriter);
	}
}

ACL_UNITY_API void samplePoseBlendedFirst(const void* compressedTransformTracks, const void* compressedScaleTracks, float* outputBuffer, float blendFactor, float time, unsigned char keyframeInterpolationMode)
{
	TransformDecompressionContext context;
	context.initialize(*static_cast<const compressed_tracks*>(compressedTransformTracks));
	context.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
	PoseBlendedFirstTrackWriter writer(outputBuffer, blendFactor);
	if (compressedScaleTracks == nullptr)
		context.decompress_tracks(writer);
	else
	{
		FloatDecompressionContext scaleContext;
		// Todo: Is initializing a context here too heavy for prefetching?
		scaleContext.initialize(*static_cast<const compressed_tracks*>(compressedScaleTracks));
		context.decompress_tracks(writer);
		scaleContext.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
		UniformScaleBlendedFirstTrackWriter scaleWriter(outputBuffer, blendFactor);
		scaleContext.decompress_tracks(scaleWriter);
	}
}

ACL_UNITY_API void samplePoseBlendedAdd(const void* compressedTransformTracks, const void* compressedScaleTracks, float* outputBuffer, float blendFactor, float time, unsigned char keyframeInterpolationMode)
{
	TransformDecompressionContext context;
	context.initialize(*static_cast<const compressed_tracks*>(compressedTransformTracks));
	context.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
	if (compressedScaleTracks == nullptr)
	{
		PoseBlendedAddTrackWriter writer(outputBuffer, blendFactor, 1.f);
		context.decompress_tracks(writer);
	}
	else
	{
		PoseBlendedAddTrackWriter writer(outputBuffer, blendFactor, 0.f);
		FloatDecompressionContext scaleContext;
		// Todo: Is initializing a context here too heavy for prefetching?
		scaleContext.initialize(*static_cast<const compressed_tracks*>(compressedScaleTracks));
		context.decompress_tracks(writer);
		scaleContext.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
		UniformScaleBlendedAddTrackWriter scaleWriter(outputBuffer, blendFactor);
		scaleContext.decompress_tracks(scaleWriter);
	}
}

ACL_UNITY_API void samplePoseMasked(const void* compressedTransformTracks, const void* compressedScaleTracks, float* outputBuffer, const std::uint64_t* mask, float time, unsigned char keyframeInterpolationMode)
{
	TransformDecompressionContext context;
	context.initialize(*static_cast<const compressed_tracks*>(compressedTransformTracks));
	context.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
	PoseMaskedTrackWriter writer(outputBuffer, mask);
	if (compressedScaleTracks == nullptr)
		context.decompress_tracks(writer);
	else
	{
		FloatDecompressionContext scaleContext;
		// Todo: Is initializing a context here too heavy for prefetching?
		scaleContext.initialize(*static_cast<const compressed_tracks*>(compressedScaleTracks));
		context.decompress_tracks(writer);
		scaleContext.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
		UniformScaleMaskedTrackWriter scaleWriter(outputBuffer, mask);
		scaleContext.decompress_tracks(scaleWriter);
	}
}

ACL_UNITY_API void samplePoseMaskedBlendedFirst(const void* compressedTransformTracks, const void* compressedScaleTracks, float* outputBuffer, const std::uint64_t* mask, float blendFactor, float time, unsigned char keyframeInterpolationMode)
{
	TransformDecompressionContext context;
	context.initialize(*static_cast<const compressed_tracks*>(compressedTransformTracks));
	context.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
	PoseBlendedFirstMaskedTrackWriter writer(outputBuffer, mask, blendFactor);
	if (compressedScaleTracks == nullptr)
		context.decompress_tracks(writer);
	else
	{
		FloatDecompressionContext scaleContext;
		// Todo: Is initializing a context here too heavy for prefetching?
		scaleContext.initialize(*static_cast<const compressed_tracks*>(compressedScaleTracks));
		context.decompress_tracks(writer);
		scaleContext.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
		UniformScaleBlendedFirstMaskedTrackWriter scaleWriter(outputBuffer, mask, blendFactor);
		scaleContext.decompress_tracks(scaleWriter);
	}
}

ACL_UNITY_API void samplePoseMaskedBlendedAdd(const void* compressedTransformTracks, const void* compressedScaleTracks, float* outputBuffer, const std::uint64_t* mask, float blendFactor, float time, unsigned char keyframeInterpolationMode)
{
	TransformDecompressionContext context;
	context.initialize(*static_cast<const compressed_tracks*>(compressedTransformTracks));
	context.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
	if (compressedScaleTracks == nullptr)
	{
		PoseBlendedAddMaskedTrackWriter writer(outputBuffer, mask, blendFactor, 1.f);
		context.decompress_tracks(writer);
	}
	else
	{
		PoseBlendedAddTrackWriter writer(outputBuffer, blendFactor, 0.f);
		FloatDecompressionContext scaleContext;
		// Todo: Is initializing a context here too heavy for prefetching?
		scaleContext.initialize(*static_cast<const compressed_tracks*>(compressedScaleTracks));
		context.decompress_tracks(writer);
		scaleContext.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
		UniformScaleBlendedAddMaskedTrackWriter scaleWriter(outputBuffer, mask, blendFactor);
		scaleContext.decompress_tracks(scaleWriter);
	}
}

ACL_UNITY_API void sampleBone(const void* compressedTransformTracks, const void* compressedScaleTracks, float* boneQvvs, int boneIndex, float time, unsigned char keyframeInterpolationMode)
{
	TransformDecompressionContext context;
	context.initialize(*static_cast<const compressed_tracks*>(compressedTransformTracks));
	context.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
	BoneTrackWriter writer(boneQvvs);
	uint32_t uindex = static_cast<uint32_t>(boneIndex);
	uindex = std::min(uindex, context.get_compressed_tracks()->get_num_tracks() - 1);
	if (compressedScaleTracks == nullptr)
		context.decompress_track(uindex, writer);
	else
	{
		FloatDecompressionContext scaleContext;
		// Todo: Is initializing a context here too heavy for prefetching?
		scaleContext.initialize(*static_cast<const compressed_tracks*>(compressedScaleTracks));
		context.decompress_track(uindex, writer);
		scaleContext.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
		SingleFloatTrackWriter scaleWriter(boneQvvs + 11);
		scaleContext.decompress_track(uindex, scaleWriter);
	}
}

ACL_UNITY_API void sampleFloats(const void* compressedFloatTracks, float* floatOutputBuffer, float time, unsigned char keyframeInterpolationMode)
{
	FloatDecompressionContext context;
	context.initialize(*static_cast<const compressed_tracks*>(compressedFloatTracks));
	context.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
	MultiFloatTrackWriter writer(floatOutputBuffer);
	context.decompress_tracks(writer);
}

ACL_UNITY_API void sampleFloatsBlendedFirst(const void* compressedFloatTracks, float* floatOutputBuffer, float blendFactor, float time, unsigned char keyframeInterpolationMode)
{
	FloatDecompressionContext context;
	context.initialize(*static_cast<const compressed_tracks*>(compressedFloatTracks));
	context.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
	MultiFloatBlendedFirstTrackWriter writer(floatOutputBuffer, blendFactor);
	context.decompress_tracks(writer);
}

ACL_UNITY_API void sampleFloatsBlendedAdd(const void* compressedFloatTracks, float* floatOutputBuffer, float blendFactor, float time, unsigned char keyframeInterpolationMode)
{
	FloatDecompressionContext context;
	context.initialize(*static_cast<const compressed_tracks*>(compressedFloatTracks));
	context.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
	MultiFloatBlendedAddTrackWriter writer(floatOutputBuffer, blendFactor);
	context.decompress_tracks(writer);
}

ACL_UNITY_API void sampleFloatsMasked(const void* compressedFloatTracks, float* floatOutputBuffer, const std::uint64_t* mask, float time, unsigned char keyframeInterpolationMode)
{
	FloatDecompressionContext context;
	context.initialize(*static_cast<const compressed_tracks*>(compressedFloatTracks));
	context.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
	MultiFloatMaskedTrackWriter writer(floatOutputBuffer, mask);
	context.decompress_tracks(writer);
}

ACL_UNITY_API void sampleFloatsMaskedBlendedFirst(const void* compressedFloatTracks, float* floatOutputBuffer, const std::uint64_t* mask, float blendFactor, float time, unsigned char keyframeInterpolationMode)
{
	FloatDecompressionContext context;
	context.initialize(*static_cast<const compressed_tracks*>(compressedFloatTracks));
	context.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
	MultiFloatBlendedFirstMaskedTrackWriter writer(floatOutputBuffer, mask, blendFactor);
	context.decompress_tracks(writer);
}

ACL_UNITY_API void sampleFloatsMaskedBlendedAdd(const void* compressedFloatTracks, float* floatOutputBuffer, const std::uint64_t* mask, float blendFactor, float time, unsigned char keyframeInterpolationMode)
{
	FloatDecompressionContext context;
	context.initialize(*static_cast<const compressed_tracks*>(compressedFloatTracks));
	context.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
	MultiFloatBlendedAddMaskedTrackWriter writer(floatOutputBuffer, mask, blendFactor);
	context.decompress_tracks(writer);
}

ACL_UNITY_API float sampleFloat(const void* compressedFloatTracks, int trackIndex, float time, unsigned char keyframeInterpolationMode)
{
	FloatDecompressionContext context;
	context.initialize(*static_cast<const compressed_tracks*>(compressedFloatTracks));
	context.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
	float result;
	SingleFloatTrackWriter writer(&result);
	uint32_t uindex = static_cast<uint32_t>(trackIndex);
	uindex = std::min(uindex, context.get_compressed_tracks()->get_num_tracks() - 1);
	context.decompress_track(uindex, writer);
	return result;
}
