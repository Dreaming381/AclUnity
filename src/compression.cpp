#include "compression.h"

// Include bug: transform_error_metric depends on core/track_types but does not include it
#include "acl/core/track_types.h"
#include "acl/compression/transform_error_metrics.h"
#include "acl/compression/compress.h"
#include "acl/core/ansi_allocator.h"

using namespace acl;

namespace
{
	namespace 
	{
		RTM_FORCE_INLINE rtm::qvvf RTM_SIMD_CALL qvvsMul(rtm::qvvf_arg0 child, rtm::qvvf_arg1 parent)
		{
			const rtm::quatf rotation = rtm::quat_mul(child.rotation, parent.rotation);
			const float scale = rtm::vector_get_w(parent.scale);
			const rtm::vector4f nonUniform = rtm::vector_mul(parent.scale, scale);
			const rtm::vector4f translation = rtm::vector_add(rtm::quat_mul_vector3(rtm::vector_mul(child.translation, nonUniform), parent.rotation), parent.translation);
			const rtm::vector4f scaleStretch = rtm::vector_set_w(child.scale, scale * rtm::vector_get_w(child.scale));
			return rtm::qvv_set(rotation, translation, scaleStretch);
		}
	}
	
	class QvvsNoScaleTransformErrorMetric : public qvvf_transform_error_metric
	{
	public:
		virtual const char* get_name() const override { return "QvvsNoScaleTransformErrorMetric"; }

		virtual RTM_DISABLE_SECURITY_COOKIE_CHECK void local_to_object_space(const local_to_object_space_args& args, void* out_object_transforms) const override
		{
			const uint32_t* dirty_transform_indices = args.dirty_transform_indices;
			const uint32_t* parent_transform_indices = args.parent_transform_indices;
			const rtm::qvvf* local_transforms_ = static_cast<const rtm::qvvf*>(args.local_transforms);
			rtm::qvvf* out_object_transforms_ = static_cast<rtm::qvvf*>(out_object_transforms);

			const uint32_t num_dirty_transforms = args.num_dirty_transforms;
			for (uint32_t dirty_transform_index = 0; dirty_transform_index < num_dirty_transforms; ++dirty_transform_index)
			{
				const uint32_t transform_index = dirty_transform_indices[dirty_transform_index];
				const uint32_t parent_transform_index = parent_transform_indices[transform_index];

				rtm::qvvf obj_transform;
				if (parent_transform_index == k_invalid_track_index)
					obj_transform = local_transforms_[transform_index];	// Just copy the root as-is, it has no parent and thus local and object space transforms are equal
				else
					obj_transform = rtm::qvv_normalize(qvvsMul(local_transforms_[transform_index], out_object_transforms_[parent_transform_index]));

				out_object_transforms_[transform_index] = obj_transform;
			}
		}
	};

	class QvvsTransformErrorMetric : public QvvsNoScaleTransformErrorMetric
	{
	private:
		const float* m_sampledScales = nullptr;
		const size_t m_numTransforms = 0;

	public:
		QvvsTransformErrorMetric(const float* sampledScales, size_t numTransforms) : m_sampledScales(sampledScales), m_numTransforms(numTransforms) {}

		virtual size_t get_transform_size(bool has_scale) const override { (void)has_scale; return sizeof(rtm::qvvf); }
		virtual bool needs_conversion(bool has_scale) const override { (void)has_scale; return true; }

		virtual RTM_DISABLE_SECURITY_COOKIE_CHECK void convert_transforms(const convert_transforms_args& args, void* out_transforms) const override
		{
			const uint32_t* dirtyTransformIndices = args.dirty_transform_indices;
			const rtm::qvvf* qvvfTransforms = args.transforms;
			rtm::qvvf* outTransforms = static_cast<rtm::qvvf*>(out_transforms);

			const uint32_t numDirtyTransforms = args.num_dirty_transforms;

			if (args.is_lossy)
			{
				for (uint32_t dirtyTransformIndex = 0; dirtyTransformIndex < numDirtyTransforms; ++dirtyTransformIndex)
				{
					const uint32_t transformIndex = dirtyTransformIndices[dirtyTransformIndex];
					const float scale = m_sampledScales[args.sample_index * m_numTransforms + transformIndex];

					outTransforms[transformIndex] = qvvfTransforms[transformIndex];
					outTransforms[transformIndex].scale = rtm::vector_set_w(qvvfTransforms[transformIndex].scale, scale);
				}
			}
			else
			{
				for (uint32_t dirtyTransformIndex = 0; dirtyTransformIndex < numDirtyTransforms; ++dirtyTransformIndex)
				{
					const uint32_t transformIndex = dirtyTransformIndices[dirtyTransformIndex];

					outTransforms[transformIndex] = qvvfTransforms[transformIndex];
				}
			}
		}

		virtual RTM_DISABLE_SECURITY_COOKIE_CHECK void convert_transforms_no_scale(const convert_transforms_args& args, void* out_transforms) const override
		{
			convert_transforms(args, out_transforms);
		}

		virtual RTM_DISABLE_SECURITY_COOKIE_CHECK void local_to_object_space_no_scale(const local_to_object_space_args& args, void* out_object_transforms) const override
		{
			local_to_object_space(args, out_object_transforms);
		}

		virtual RTM_DISABLE_SECURITY_COOKIE_CHECK rtm::scalarf RTM_SIMD_CALL calculate_error(const calculate_error_args& args) const override
		{
			const rtm::qvvf& raw_transform_ = *static_cast<const rtm::qvvf*>(args.transform0);
			const rtm::qvvf& lossy_transform_ = *static_cast<const rtm::qvvf*>(args.transform1);

			const float rtScale = rtm::vector_get_w(raw_transform_.scale);
			const rtm::qvvf rtPatched = rtm::qvv_set(raw_transform_.rotation, raw_transform_.translation, rtm::vector_mul(raw_transform_.scale, rtScale));

			const float ltScale = rtm::vector_get_w(lossy_transform_.scale);
			const rtm::qvvf ltPatched = rtm::qvv_set(lossy_transform_.rotation, lossy_transform_.translation, rtm::vector_mul(lossy_transform_.scale, ltScale));

			// Note that because we have scale, we must measure all three axes
			const rtm::vector4f vtx0 = args.shell_point_x;
			const rtm::vector4f vtx1 = args.shell_point_y;
			const rtm::vector4f vtx2 = args.shell_point_z;

			const rtm::vector4f raw_vtx0 = rtm::qvv_mul_point3(vtx0, rtPatched);
			const rtm::vector4f raw_vtx1 = rtm::qvv_mul_point3(vtx1, rtPatched);
			const rtm::vector4f raw_vtx2 = rtm::qvv_mul_point3(vtx2, rtPatched);

			const rtm::vector4f lossy_vtx0 = rtm::qvv_mul_point3(vtx0, ltPatched);
			const rtm::vector4f lossy_vtx1 = rtm::qvv_mul_point3(vtx1, ltPatched);
			const rtm::vector4f lossy_vtx2 = rtm::qvv_mul_point3(vtx2, ltPatched);

			const rtm::scalarf vtx0_error = rtm::vector_distance3(raw_vtx0, lossy_vtx0);
			const rtm::scalarf vtx1_error = rtm::vector_distance3(raw_vtx1, lossy_vtx1);
			const rtm::scalarf vtx2_error = rtm::vector_distance3(raw_vtx2, lossy_vtx2);

			return rtm::scalar_max(rtm::scalar_max(vtx0_error, vtx1_error), vtx2_error);
		}

		virtual RTM_DISABLE_SECURITY_COOKIE_CHECK rtm::scalarf RTM_SIMD_CALL calculate_error_no_scale(const calculate_error_args& args) const override
		{
			return calculate_error(args);
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
										 int*                outCompressedSizeInBytes,
										 float*              sampledScales)
{
	ansi_allocator allocator;

	track_array_qvvf      trackArray(allocator, static_cast<uint32_t>(numBones));
	// Todo: make_ref should accept this input as const.
	// Todo: Is this reinterpret undefined behavior, where using void* for aosClipData instead would be more appropriate?
	auto*                 clipData = const_cast<rtm::qvvf*>(reinterpret_cast<const rtm::qvvf*>(aosClipData));
	track_desc_transformf trackDesc;
	trackDesc.precision                      = maxDistanceError;
	trackDesc.shell_distance                 = sampledErrorDistanceFromBone;
	for (short i = 0; i < numBones; i++)
	{
		trackDesc.output_index = static_cast<uint32_t>(i);
		short parentIndex      = static_cast<short>(parentIndices[i]);
		if (parentIndex == i || parentIndex == -1)
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

	QvvsNoScaleTransformErrorMetric errorMetricA;
	QvvsTransformErrorMetric errorMetricB(sampledScales, numBones);
	if (sampledScales == nullptr)
		compressionSettings.error_metric = &errorMetricA;
	else
		compressionSettings.error_metric = &errorMetricB;

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
