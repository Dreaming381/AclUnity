#include "compression.h"

// Include bug: transform_error_metric depends on core/track_types but does not include it
#include "acl/core/track_types.h"
#include "acl/compression/transform_error_metrics.h"
#include "acl/compression/compress.h"
#include "acl/core/ansi_allocator.h"


using namespace acl;

namespace
{
	class ParentScaleInverseTransformErrorMetric : public qvvf_matrix3x4f_transform_error_metric
	{
	private:
		struct MatrixAndInverseScale
		{
			rtm::matrix3x4f matrix;
			rtm::vector4f inverseScale;

			MatrixAndInverseScale() = default;
			MatrixAndInverseScale(const rtm::qvvf& qvvf)
			{
				matrix     = rtm::matrix_from_qvv(qvvf);
				auto scale = rtm::vector_set_w(qvvf.scale, 1.0f);
				scale = rtm::vector_reciprocal(scale);
				
				// Todo: Switch this to use vector select once is_finite returning a mask is supported in RTM
				if (!rtm::vector_is_finite(scale))
				{
					// Todo: Bug! Inlining the return values into the scalar_is_finite function causes ambiguous function error in MSVC
					// Todo: ^^ Was in 2.0.1 but may not be true in 2.0.4
					rtm::scalarf x = rtm::vector_get_x(scale);
					rtm::scalarf y = rtm::vector_get_y(scale);
					rtm::scalarf z = rtm::vector_get_z(scale);
					
					if (!rtm::scalar_is_finite(x))
						scale = rtm::vector_set_x(scale, 0.0f);
					if (!rtm::scalar_is_finite(y))
						scale = rtm::vector_set_y(scale, 0.0f);
					if (!rtm::scalar_is_finite(z))
						scale = rtm::vector_set_z(scale, 0.0f);
				}

				inverseScale = scale;
			}
		};

		const signed short* m_parentIndices = nullptr;

	public:
		ParentScaleInverseTransformErrorMetric(const signed short* parentIndices) : m_parentIndices(parentIndices) {}

		virtual const char* get_name() const override { return "ParentScaleInverseTransformErrorMetric"; }

		virtual size_t get_transform_size(bool has_scale) const override { return has_scale ? sizeof(MatrixAndInverseScale) : sizeof(rtm::qvvf); }

		virtual RTM_DISABLE_SECURITY_COOKIE_CHECK void convert_transforms(const convert_transforms_args& args, void* out_transforms) const override
		{
			const uint32_t* dirtyTransformIndices = args.dirty_transform_indices;
			const rtm::qvvf* qvvfTransforms = args.transforms;
			MatrixAndInverseScale* outTransforms = static_cast<MatrixAndInverseScale*>(out_transforms);

			const uint32_t numDirtyTransforms = args.num_dirty_transforms;
			for (uint32_t dirtyTransformIndex = 0; dirtyTransformIndex < numDirtyTransforms; ++dirtyTransformIndex)
			{
				const uint32_t transformIndex = dirtyTransformIndices[dirtyTransformIndex];

				outTransforms[transformIndex] = MatrixAndInverseScale(qvvfTransforms[transformIndex]);
			}
		}

		virtual RTM_DISABLE_SECURITY_COOKIE_CHECK void local_to_object_space(const local_to_object_space_args& args, void* out_object_transforms) const override
		{
			const uint32_t* dirtyTransformIndices = args.dirty_transform_indices;
			const uint32_t* parentTransformIndices = args.parent_transform_indices;
			const MatrixAndInverseScale* localTransforms = static_cast<const MatrixAndInverseScale*>(args.local_transforms);
			MatrixAndInverseScale* outObjectTransforms = static_cast<MatrixAndInverseScale*>(out_object_transforms);

			const uint32_t numDirtyTransforms = args.num_dirty_transforms;
			for (uint32_t dirtyTransformIndex = 0; dirtyTransformIndex < numDirtyTransforms; ++dirtyTransformIndex)
			{
				const uint32_t transformIndex = dirtyTransformIndices[dirtyTransformIndex];
				const uint32_t parentTransformIndex = parentTransformIndices[transformIndex];

				MatrixAndInverseScale objTransform;
				MatrixAndInverseScale localTransform = localTransforms[transformIndex];
				
				if (parentTransformIndex == k_invalid_track_index)
					objTransform.matrix = localTransform.matrix;	// Just copy the root as-is, it has no parent and thus local and object space transforms are equal
				else if (m_parentIndices[transformIndex] < 0)
				{
					// Todo: Need a more direct vector-matrix scaling multiply in RTM
					MatrixAndInverseScale parentObjectTransform = outObjectTransforms[parentTransformIndex];
					objTransform.matrix = rtm::matrix_mul(rtm::matrix_mul(localTransform.matrix, rtm::matrix_from_scale(parentObjectTransform.inverseScale)), parentObjectTransform.matrix);
				}
				else
					objTransform.matrix = rtm::matrix_mul(localTransforms[transformIndex].matrix, outObjectTransforms[parentTransformIndex].matrix);

				objTransform.inverseScale = localTransform.inverseScale;
				outObjectTransforms[transformIndex] = objTransform;
			}
		}
	};
}

ACL_UNITY_API void* compressSkeletonClip(const signed short* parentIndices, 
										 signed short        numBones, 
										 signed short        compressionLevel, 
										 const float*        aosClipData, 
										 int                 numSamples, 
										 float               sampleRate, 
										 float               maxDistanceError, 
										 float               sampledErrorDistanceFromBone, 
										 float               maxNegligibleTranslationDrift, 
										 float				 maxNegligibleScaleDrift,
										 int*                outCompressedSizeInBytes)
{
	ansi_allocator allocator;

	track_array_qvvf      trackArray(allocator, static_cast<uint32_t>(numBones));
	// Todo: make_ref should accept this input as const.
	auto*                 clipData = const_cast<rtm::qvvf*>(reinterpret_cast<const rtm::qvvf*>(aosClipData));
	track_desc_transformf trackDesc;
	trackDesc.constant_scale_threshold       = maxNegligibleScaleDrift;
	trackDesc.constant_translation_threshold = maxNegligibleTranslationDrift;
	trackDesc.precision                      = maxDistanceError;
	trackDesc.shell_distance                 = sampledErrorDistanceFromBone;
	for (short i = 0; i < numBones; i++)
	{
		trackDesc.output_index = static_cast<uint32_t>(i);
		short parentIndex      = static_cast<short>(abs(parentIndices[i]));
		if (parentIndex == i)
		{
			trackDesc.parent_index = k_invalid_track_index;
		}
		else
		{
			trackDesc.parent_index = static_cast<uint32_t>(parentIndex);
		}
		trackArray[i] = track_qvvf::make_ref(trackDesc, clipData + i * numSamples, static_cast<uint32_t>(numSamples), sampleRate);
	}

	auto compressionSettings  = get_default_compression_settings();
	compressionSettings.level = static_cast<compression_level8>(compressionLevel);
	ParentScaleInverseTransformErrorMetric errorMetric(parentIndices);
	compressionSettings.error_metric = &errorMetric;

	compressed_tracks* outCompressedTracks = nullptr;
	output_stats outputStats;

	compress_track_list(allocator, trackArray, compressionSettings, outCompressedTracks, outputStats);
	*outCompressedSizeInBytes = outCompressedTracks->get_size();
	
	return outCompressedTracks;
}

ACL_UNITY_API void* compressScalarsClip(signed short numTracks, 
										signed short compressionLevel, 
										const float* clipData, 
										int numSamples, 
										float sampleRate,
										float* maxErrors, 
										int* outCompressedSizeInBytes)
{
	ansi_allocator allocator;;

	track_array_float1f trackArray(allocator, static_cast<uint32_t>(numTracks));
	for (short i = 0; i < numTracks; i++)
	{
		track_desc_scalarf trackDesc;
		trackDesc.output_index = static_cast<uint32_t>(i);
		trackDesc.precision    = maxErrors[i];
		// Todo: make_ref should accept clipData input as const.
		trackArray[i] = track_float1f::make_ref(trackDesc, const_cast<float*>(clipData) + i * numSamples, static_cast<uint32_t>(numSamples), sampleRate);
	}

	auto compressionSettings = get_default_compression_settings();
	compressionSettings.level = static_cast<compression_level8>(compressionLevel);

	compressed_tracks* outCompressedTracks = nullptr;
	output_stats outputStats;

	compress_track_list(allocator, trackArray, compressionSettings, outCompressedTracks, outputStats);
	*outCompressedSizeInBytes = outCompressedTracks->get_size();

	return outCompressedTracks;
}

ACL_UNITY_API void disposeCompressedTracksBuffer(void* compressedTracksBuffer)
{
	ansi_allocator a;
	compressed_tracks* buffer = static_cast<compressed_tracks*>(compressedTracksBuffer);
	a.deallocate(buffer, buffer->get_size());
}
