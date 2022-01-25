#pragma once

#include "acl_unity_defines.h"

extern "C"
{
	/// <summary>
	/// Decompresses the pose at the given sample time and stores it in AOS format
	/// </summary>
	/// <param name="compressedTransformTracks">The 16-byte aligned pointer to the compressed tracks data</param>
	/// <param name="aosOutputBuffer">A pointer to the buffer where the decompressed pose should be stored. See the detailed documentation for the layout.</param>
	/// <param name="time">The time at which to sample the pose in seconds</param>
	/// <param name="keyframeInterpolationMode">The method used for interpolating between the two keyframes sampled: 
	/// 0 = lerp, 1 = round to next sample, 2 = round to previous sample, 3 = round to nearest sample</param>
	/// <remarks>
	/// The resulting layout stored in aosOutputBuffer is as follows:
	/// Where t = translation, r = rotation, s = scale, and ~ = a padding float value; a bone is stored as follows:
	/// floats 0-3:  r.x, r.y, r.z, r.w,
	/// floats 4-7:  t.x, t.y, t.z, ~
	/// floats 8-11: s.x, s.y, s.z, ~
	/// 
	/// In total, each bone is 12 floats or 48 bytes long. So the first 12 float values correspond to bone 0, then next 12 values correspond to bone 1, ect.
	/// </remarks>
	ACL_UNITY_API void samplePoseAOS(const void* compressedTransformTracks, float* aosOutputBuffer, float time, unsigned char keyframeInterpolationMode);

	/// <summary>
	/// Decompresses the pose at the given sample time and stores it in SOA format
	/// </summary>
	/// <param name="compressedTransformTracks">The 16-byte aligned pointer to the compressed tracks data</param>
	/// <param name="soaOutputBuffer">A pointer to the buffer where the decompressed pose should be stored. See the detailed documentation for the layout.</param>
	/// <param name="time">The time at which to sample the pose in seconds</param>
	/// <param name="keyframeInterpolationMode">The method used for interpolating between the two keyframes sampled: 
	/// 0 = lerp, 1 = round to next sample, 2 = round to previous sample, 3 = round to nearest sample</param>
	/// <remarks>
	/// Each component of the transform is stored as an array, followed by the next transform. For a skeleton with three bones: b0, b1, and b2;
	/// Where t = translation, r = rotation, s = scale; the layout is as follows:
	/// floats 0-2:   b0 r.x, b1 r.x, b2 r.x
	/// floats 3-5:   b0 r.y, b1 r.y, b2 r.y
	/// floats 6-8:   b0 r.z, b1 r.z, b2 r.z
	/// floats 9-11:  b0 r.w, b1 r.w, b2 r.w
	/// floats 12-14: b0 t.x, b1 t.x, b2 t.x
	/// floats 15-17: b0 t.y, b1 t.y, b2 t.y
	/// floats 18-20: b0 t.z, b1 t.z, b2 t.z
	/// floats 21-23: b0 s.x, b1 s.x, b2 s.x
	/// floats 24-26: b0 s.y, b1 s.y, b2 s.y
	/// floats 27-29: b0 s.z, b1 s.z, b2 s.z
	/// 
	/// In total each bone stores 10 floats or 40 bytes, so this format is more compact than AOS
	/// </remarks>
	ACL_UNITY_API void samplePoseSOA(const void* compressedTransformTracks, float* soaOutputBuffer, float time, unsigned char keyframeInterpolationMode);

	/// <summary>
	/// Decompresses the bone for the given boneIndex at the fiven sample time and stores it in a QVV (AOS format)
	/// </summary>
	/// <param name="compressedTransformTracks">The 16-byte aligned pointer to the compressed tracks data</param>
	/// <param name="boneQVV">A pointer to the buffer where the decompressed bone should be stored. See the detailed documentation for the layout.</param>
	/// <param name="boneIndex">The individual bone in the skeleton that should be sampled</param>
	/// <param name="time">The time at which to sample the pose in seconds</param>
	/// <param name="keyframeInterpolationMode">The method used for interpolating between the two keyframes sampled: 
	/// 0 = lerp, 1 = round to next sample, 2 = round to previous sample, 3 = round to nearest sample</param>
	/// <remarks>
	/// The resulting layout stored in boneQVV is as follows:
	/// Where t = translation, r = rotation, s = scale, and ~ = a padding float value; the bone is stored as follows:
	/// floats 0-3:  r.x, r.y, r.z, r.w,
	/// floats 4-7:  t.x, t.y, t.z, ~
	/// floats 8-11: s.x, s.y, s.z, ~
	/// 
	/// In total, the bone is 12 floats or 48 bytes long.
	/// </remarks>
	ACL_UNITY_API void sampleBone(const void* compressedTransformTracks, float* boneQVV, int boneIndex, float time, unsigned char keyframeInterpolationMode);

	/// <summary>
	/// Decompresses the scalar values at the given sample time and stores the results in the floatOutputBuffer
	/// </summary>
	/// <param name="compressedFloatTracks">The 16-byte aligned pointer to the compressed tracks data</param>
	/// <param name="floatOutputBuffer">A pointer to the buffer where the decompressed sampled values should be stored.
	/// Each track stores a single float value at its respective index.</param>
	/// <param name="time">The time at which to sample the pose in seconds</param>
	/// <param name="keyframeInterpolationMode">The method used for interpolating between the two keyframes sampled: 
	/// 0 = lerp, 1 = round to next sample, 2 = round to previous sample, 3 = round to nearest sample</param>
	ACL_UNITY_API void sampleFloats(const void* compressedFloatTracks, float* floatOutputBuffer, float time, unsigned char keyframeInterpolationMode);

	/// <summary>
	/// Decompresses a single scalar value for the given trackIndex at the given sample time
	/// </summary>
	/// <param name="compressedFloatTracks">The 16-byte aligned pointer to the compressed tracks data</param>
	/// <param name="trackIndex">The individual track index that should be sampled</param>
	/// <param name="time">The time at which to sample the pose in seconds</param>
	/// <param name="keyframeInterpolationMode">The method used for interpolating between the two keyframes sampled: 
	/// 0 = lerp, 1 = round to next sample, 2 = round to previous sample, 3 = round to nearest sample</param>
	/// <returns>The sampled value for the specified track</returns>
	ACL_UNITY_API float sampleFloat(const void* compressedFloatTracks, int trackIndex, float time, unsigned char keyframeInterpolationMode);
}