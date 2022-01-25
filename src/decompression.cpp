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
		
		void RTM_SIMD_CALL write_rotation(uint32_t track_index, rtm::quatf_arg0 rotation)
		{
			rtm::quat_store(rotation, m_aosOutputBuffer + 12 * track_index);
		}

		void RTM_SIMD_CALL write_translation(uint32_t track_index, rtm::vector4f_arg0 translation)
		{
			rtm::vector_store(translation, m_aosOutputBuffer + 12 * track_index + 4);
		}

		void RTM_SIMD_CALL write_scale(uint32_t track_index, rtm::vector4f_arg0 scale)
		{
			rtm::vector_store(scale, m_aosOutputBuffer + 12 * track_index + 8);
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

		void RTM_SIMD_CALL write_rotation(uint32_t track_index, rtm::quatf_arg0 rotation)
		{
			m_soaOutputBuffer[m_rx + track_index] = rtm::quat_get_x(rotation);
			m_soaOutputBuffer[m_ry + track_index] = rtm::quat_get_y(rotation);
			m_soaOutputBuffer[m_rz + track_index] = rtm::quat_get_z(rotation);
			m_soaOutputBuffer[m_rw + track_index] = rtm::quat_get_w(rotation);
		}

		void RTM_SIMD_CALL write_translation(uint32_t track_index, rtm::vector4f_arg0 translation)
		{
			m_soaOutputBuffer[m_tx] = rtm::vector_get_x(translation);
			m_soaOutputBuffer[m_ty] = rtm::vector_get_y(translation);
			m_soaOutputBuffer[m_tz] = rtm::vector_get_z(translation);
		}

		void RTM_SIMD_CALL write_scale(uint32_t track_index, rtm::vector4f_arg0 scale)
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

		void RTM_SIMD_CALL write_rotation(uint32_t track_index, rtm::quatf_arg0 rotation)
		{
			(void)track_index;
			rtm::quat_store(rotation, m_aosOutputBuffer);
		}

		void RTM_SIMD_CALL write_translation(uint32_t track_index, rtm::vector4f_arg0 translation)
		{
			(void)track_index;
			rtm::vector_store(translation, m_aosOutputBuffer + 4);
		}

		void RTM_SIMD_CALL write_scale(uint32_t track_index, rtm::vector4f_arg0 scale)
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

		void RTM_SIMD_CALL write_float1(uint32_t track_index, rtm::scalarf_arg0 value)
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

		void RTM_SIMD_CALL write_float1(uint32_t track_index, rtm::scalarf_arg0 value)
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

ACL_UNITY_API void samplePoseAOS(const void* compressedTransformTracks, float* aosOutputBuffer, float time, unsigned char keyframeInterpolationMode)
{
	TransformDecompressionContext context;
	PoseAOSTrackWriter writer(aosOutputBuffer);
	context.initialize(*static_cast<const compressed_tracks*>(compressedTransformTracks));
	context.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
	context.decompress_tracks(writer);
}

ACL_UNITY_API void samplePoseSOA(const void* compressedTransformTracks, float* soaOutputBuffer, float time, unsigned char keyframeInterpolationMode)
{
	TransformDecompressionContext context;
	context.initialize(*static_cast<const compressed_tracks*>(compressedTransformTracks));
	PoseSOATrackWriter writer(soaOutputBuffer, context.get_compressed_tracks()->get_num_tracks());
	context.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
	context.decompress_tracks(writer);
}

ACL_UNITY_API void sampleBone(const void* compressedTransformTracks, float* boneQVV, int boneIndex, float time, unsigned char keyframeInterpolationMode)
{
	TransformDecompressionContext context;
	BoneTrackWriter writer(boneQVV);
	context.initialize(*static_cast<const compressed_tracks*>(compressedTransformTracks));
	context.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
	context.decompress_track(static_cast<uint32_t>(boneIndex), writer);
}

ACL_UNITY_API void sampleFloats(const void* compressedFloatTracks, float* floatOutputBuffer, float time, unsigned char keyframeInterpolationMode)
{
	FloatDecompressionContext context;
	MultiFloatTrackWriter writer(floatOutputBuffer);
	context.initialize(*static_cast<const compressed_tracks*>(compressedFloatTracks));
	context.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
	context.decompress_tracks(writer);
}

ACL_UNITY_API float sampleFloat(const void* compressedFloatTracks, int trackIndex, float time, unsigned char keyframeInterpolationMode)
{
	FloatDecompressionContext context;
	float result;
	SingleFloatTrackWriter writer(&result);
	context.initialize(*static_cast<const compressed_tracks*>(compressedFloatTracks));
	context.seek(time, static_cast<sample_rounding_policy>(keyframeInterpolationMode));
	context.decompress_track(static_cast<uint32_t>(trackIndex), writer);
	return result;
}
