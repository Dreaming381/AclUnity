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
			rtm::vector_store(rtm::vector_mul(translation, m_blendFactor), m_outputBuffer + 12 * track_index + 4);
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
			rtm::vector_store(rtm::vector_mul_add(translation, m_blendFactor, rtm::vector_load(dst)), dst);
		}

		RTM_FORCE_INLINE void RTM_SIMD_CALL write_scale(uint32_t track_index, rtm::vector4f_arg0 scale)
		{
			auto dst = m_outputBuffer + 12 * track_index + 8;
			rtm::vector_store(rtm::vector_mul_add(rtm::vector_set_w(scale, m_uniformScale), m_blendFactor, rtm::vector_load(dst)), dst);
		}
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
