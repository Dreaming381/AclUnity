#include "decompression.h"

#include "acl/decompression/decompress.h"

using namespace acl;

namespace 
{
	class PoseAOSTrackWriter : public track_writer
	{
	private:
		float* m_aosOutputBuffer;

	public:
		PoseAOSTrackWriter(float* aosOutputBuffer) : m_aosOutputBuffer(aosOutputBuffer) {}
		
		RTM_FORCE_INLINE void RTM_SIMD_CALL write_rotation(uint32_t track_index, rtm::quatf_arg0 rotation)
		{
			rtm::quat_store(rotation, m_aosOutputBuffer + 12 * track_index);
		}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_translation(uint32_t track_index, rtm::vector4f_arg0 translation)
		{
			rtm::vector_store(translation, m_aosOutputBuffer + 12 * track_index + 4);
		}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_scale(uint32_t track_index, rtm::vector4f_arg0 scale)
		{
			rtm::vector_store(scale, m_aosOutputBuffer + 12 * track_index + 8);
		}
	};

	class PoseAOSBlendedFirstTrackWriter : public track_writer
	{
	private:
		float* m_aosOutputBuffer;
		rtm::vector4f m_blendFactor;

	public:
		PoseAOSBlendedFirstTrackWriter(float* aosOutputBuffer, float blendFactor) : m_aosOutputBuffer(aosOutputBuffer), m_blendFactor(rtm::vector_broadcast(&blendFactor)) {}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_rotation(uint32_t track_index, rtm::quatf_arg0 rotation)
		{
			rtm::quat_store(rtm::vector_mul(rtm::quat_to_vector(rotation), m_blendFactor), m_aosOutputBuffer + 12 * track_index);
		}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_translation(uint32_t track_index, rtm::vector4f_arg0 translation)
		{
			rtm::vector_store(rtm::vector_mul(translation, m_blendFactor), m_aosOutputBuffer + 12 * track_index + 4);
		}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_scale(uint32_t track_index, rtm::vector4f_arg0 scale)
		{
			rtm::vector_store(rtm::vector_mul(scale, m_blendFactor), m_aosOutputBuffer + 12 * track_index + 8);
		}
	};

	// Todo: RTM doesn't currently use FMA for AVX.
	class PoseAOSBlendedAddTrackWriter : public track_writer
	{
	private:
		float* m_aosOutputBuffer;
		rtm::vector4f m_blendFactor;

	public:
		PoseAOSBlendedAddTrackWriter(float* aosOutputBuffer, float blendFactor) : m_aosOutputBuffer(aosOutputBuffer), m_blendFactor(rtm::vector_broadcast(&blendFactor)) {}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_rotation(uint32_t track_index, rtm::quatf_arg0 rotation)
		{
			auto dst = m_aosOutputBuffer + 12 * track_index;
			rtm::quat_store(rtm::vector_mul_add(rtm::quat_to_vector(rotation), m_blendFactor, rtm::vector_load(dst)), dst);
		}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_translation(uint32_t track_index, rtm::vector4f_arg0 translation)
		{
			auto dst = m_aosOutputBuffer + 12 * track_index + 4;
			rtm::vector_store(rtm::vector_mul_add(translation, m_blendFactor, rtm::vector_load(dst)), dst);
		}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_scale(uint32_t track_index, rtm::vector4f_arg0 scale)
		{
			auto dst = m_aosOutputBuffer + 12 * track_index + 8;
			rtm::vector_store(rtm::vector_mul_add(scale, m_blendFactor, rtm::vector_load(dst)), dst);
		}
	};

	class PoseSOATrackWriter : public track_writer
	{
	private:
		float* m_soaOutputBuffer;
		constexpr const static uint32_t m_rx = 0;
		const uint32_t m_ry;
		const uint32_t m_rz;
		const uint32_t m_rw;
		const uint32_t m_tx;
		const uint32_t m_ty;
		const uint32_t m_tz;
		const uint32_t m_sx;
		const uint32_t m_sy;
		const uint32_t m_sz;

	public:
		PoseSOATrackWriter(float* soaOutputBuffer, uint16_t boneCount)
			: m_soaOutputBuffer(soaOutputBuffer)
			, m_ry(boneCount)
			, m_rz(boneCount * 2)
			, m_rw(boneCount * 3)
			, m_tx(boneCount * 4)
			, m_ty(boneCount * 5)
			, m_tz(boneCount * 6)
			, m_sx(boneCount * 7)
			, m_sy(boneCount * 8)
			, m_sz(boneCount * 9)
		{}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_rotation(uint32_t track_index, rtm::quatf_arg0 rotation)
		{
			m_soaOutputBuffer[m_rx + track_index] = rtm::quat_get_x(rotation);
			m_soaOutputBuffer[m_ry + track_index] = rtm::quat_get_y(rotation);
			m_soaOutputBuffer[m_rz + track_index] = rtm::quat_get_z(rotation);
			m_soaOutputBuffer[m_rw + track_index] = rtm::quat_get_w(rotation);
		}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_translation(uint32_t track_index, rtm::vector4f_arg0 translation)
		{
			m_soaOutputBuffer[m_tx] = rtm::vector_get_x(translation);
			m_soaOutputBuffer[m_ty] = rtm::vector_get_y(translation);
			m_soaOutputBuffer[m_tz] = rtm::vector_get_z(translation);
		}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_scale(uint32_t track_index, rtm::vector4f_arg0 scale)
		{
			m_soaOutputBuffer[m_sx] = rtm::vector_get_x(scale);
			m_soaOutputBuffer[m_sy] = rtm::vector_get_y(scale);
			m_soaOutputBuffer[m_sz] = rtm::vector_get_z(scale);
		}
	};

	class BoneTrackWriter : public track_writer
	{
	private:
		float* m_aosOutputBuffer;

	public:
		BoneTrackWriter(float* aosOutputBuffer) : m_aosOutputBuffer(aosOutputBuffer) {}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_rotation(uint32_t track_index, rtm::quatf_arg0 rotation)
		{
			(void)track_index;
			rtm::quat_store(rotation, m_aosOutputBuffer);
		}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_translation(uint32_t track_index, rtm::vector4f_arg0 translation)
		{
			(void)track_index;
			rtm::vector_store(translation, m_aosOutputBuffer + 4);
		}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_scale(uint32_t track_index, rtm::vector4f_arg0 scale)
		{
			(void)track_index;
			rtm::vector_store(scale, m_aosOutputBuffer + 8);
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
	};

	using TransformDecompressionContext = decompression_context<TransformDecompressionSettings>;

	class FloatDecompressionSettings : public decompression_settings
	{
	public:
		// We perform these safety checks at the C# layer.
		// The most important check is ensuring that the compressed tracks object is aligned to a 16 byte boundary.
		static constexpr bool skip_initialize_safety_checks() { return true; }

		static constexpr bool is_track_type_supported(track_type8 type) { return type == track_type8::float1f; }
	};

	using FloatDecompressionContext = decompression_context<FloatDecompressionSettings>;
}

// It is advantageous to perform as much trivial work between seek() and decompress_track[s]() because seek() prefetches.
// There isn't much but we can at least use that to configure the writer and to clamp index values.
ACL_UNITY_API void samplePoseAOS(const void* compressedTransformTracks, float* aosOutputBuffer, float time, unsigned char keyframeInterpolationMode)
{
	TransformDecompressionContext context;
	context.initialize(*static_cast<const compressed_tracks*>(compressedTransformTracks));
	context.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
	PoseAOSTrackWriter writer(aosOutputBuffer);
	context.decompress_tracks(writer);
}

ACL_UNITY_API void samplePoseAOSBlendedFirst(const void* compressedTransformTracks, float* aosOutputBuffer, float blendFactor, float time, unsigned char keyframeInterpolationMode)
{
	TransformDecompressionContext context;
	context.initialize(*static_cast<const compressed_tracks*>(compressedTransformTracks));
	context.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
	PoseAOSBlendedFirstTrackWriter writer(aosOutputBuffer, blendFactor);
	context.decompress_tracks(writer);
}

ACL_UNITY_API void samplePoseAOSBlendedAdd(const void* compressedTransformTracks, float* aosOutputBuffer, float blendFactor, float time, unsigned char keyframeInterpolationMode)
{
	TransformDecompressionContext context;
	context.initialize(*static_cast<const compressed_tracks*>(compressedTransformTracks));
	context.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
	PoseAOSBlendedAddTrackWriter writer(aosOutputBuffer, blendFactor);
	context.decompress_tracks(writer);
}

ACL_UNITY_API void samplePoseSOA(const void* compressedTransformTracks, float* soaOutputBuffer, float time, unsigned char keyframeInterpolationMode)
{
	TransformDecompressionContext context;
	context.initialize(*static_cast<const compressed_tracks*>(compressedTransformTracks));
	context.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
	PoseSOATrackWriter writer(soaOutputBuffer, context.get_compressed_tracks()->get_num_tracks());
	context.decompress_tracks(writer);
}

ACL_UNITY_API void sampleBone(const void* compressedTransformTracks, float* boneQVV, int boneIndex, float time, unsigned char keyframeInterpolationMode)
{
	TransformDecompressionContext context;
	context.initialize(*static_cast<const compressed_tracks*>(compressedTransformTracks));
	context.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
	BoneTrackWriter writer(boneQVV);
	uint32_t uindex = static_cast<uint32_t>(boneIndex);
	uindex = std::min(uindex, context.get_compressed_tracks()->get_num_tracks() - 1);
	context.decompress_track(uindex, writer);
}

ACL_UNITY_API void sampleFloats(const void* compressedFloatTracks, float* floatOutputBuffer, float time, unsigned char keyframeInterpolationMode)
{
	FloatDecompressionContext context;
	context.initialize(*static_cast<const compressed_tracks*>(compressedFloatTracks));
	context.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
	MultiFloatTrackWriter writer(floatOutputBuffer);
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
