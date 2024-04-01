#pragma once

#include "acl_unity_defines.h"

extern "C"
{
	/// <summary>
	/// Decompresses the pose at the given sample time and stores it in AOS format
	/// </summary>
	/// <param name="compressedTransformTracks">The 16-byte aligned pointer to the compressed tracks data</param>
	/// <param name="compressedScaleTracks">The 16-byte aligned pointer to the compressed scale tracks, or nullptr if all uniform scales are 1f.</param>
	/// <param name="aosOutputBuffer">A pointer to the buffer where the decompressed pose should be stored. See the detailed documentation for the layout.</param>
	/// <param name="time">The time at which to sample the pose in seconds</param>
	/// <param name="keyframeInterpolationMode">The method used for interpolating between the two keyframes sampled: 
	/// 0 = lerp, 1 = round to next sample, 2 = round to previous sample, 3 = round to nearest sample</param>
	/// <remarks>
	/// The resulting layout stored in aosOutputBuffer is as follows:
	/// Where t = translation, r = rotation, s = stretch, u = uniform scale, and ~ = a padding float value; a bone is stored as follows:
	/// floats 0-3:  r.x, r.y, r.z, r.w,
	/// floats 4-7:  t.x, t.y, t.z, ~
	/// floats 8-11: s.x, s.y, s.z, u
	/// 
	/// In total, each bone is 12 floats or 48 bytes long. So the first 12 float values correspond to bone 0, then next 12 values correspond to bone 1, ect.
	/// </remarks>
	ACL_UNITY_API void samplePose(const void* compressedTransformTracks, const void* compressedScaleTracks, float* aosOutputBuffer, float time, unsigned char keyframeInterpolationMode);

	/// <summary>
	/// Decompresses the pose at the given sample time and stores it in AOS format, scaling each result by the blend factor
	/// </summary>
	/// <param name="compressedTransformTracks">The 16-byte aligned pointer to the compressed tracks data</param>
	/// <param name="compressedScaleTracks">The 16-byte aligned pointer to the compressed scale tracks, or nullptr if all uniform scales are 1f.</param>
	/// <param name="aosOutputBuffer">A pointer to the buffer where the decompressed pose should be stored. See the detailed documentation for the layout.</param>
	/// <param name="blendFactor">A scale factor to apply to all decompressed values. Rotations are left unnormalized.</param>
	/// <param name="time">The time at which to sample the pose in seconds</param>
	/// <param name="keyframeInterpolationMode">The method used for interpolating between the two keyframes sampled: 
	/// 0 = lerp, 1 = round to next sample, 2 = round to previous sample, 3 = round to nearest sample</param>
	/// <remarks>
	/// The resulting layout stored in aosOutputBuffer is as follows:
	/// Where t = translation, r = rotation, s = stretch, u = uniform scale, and b = total accumulated blending; a bone is stored as follows:
	/// floats 0-3:  r.x, r.y, r.z, r.w,
	/// floats 4-7:  t.x, t.y, t.z, b
	/// floats 8-11: s.x, s.y, s.z, u
	/// 
	/// In total, each bone is 12 floats or 48 bytes long. So the first 12 float values correspond to bone 0, then next 12 values correspond to bone 1, ect.
	/// </remarks>
	ACL_UNITY_API void samplePoseBlendedFirst(const void* compressedTransformTracks, const void* compressedScaleTracks, float* aosOutputBuffer, float blendFactor, float time, unsigned char keyframeInterpolationMode);

	/// <summary>
	/// Decompresses the pose at the given sample time, scales each value by the blend factor, and adds it to the existing value in the buffer in AOS format
	/// </summary>
	/// <param name="compressedTransformTracks">The 16-byte aligned pointer to the compressed tracks data</param>
	/// <param name="compressedScaleTracks">The 16-byte aligned pointer to the compressed scale tracks, or nullptr if all uniform scales are 1f.</param>
	/// <param name="aosOutputBuffer">A pointer to the buffer where the decompressed pose should be stored. See the detailed documentation for the layout.</param>
	/// <param name="blendFactor">A scale factor to apply to all decompressed values. Rotations are left unnormalized.</param>
	/// <param name="time">The time at which to sample the pose in seconds</param>
	/// <param name="keyframeInterpolationMode">The method used for interpolating between the two keyframes sampled: 
	/// 0 = lerp, 1 = round to next sample, 2 = round to previous sample, 3 = round to nearest sample</param>
	/// <remarks>
	/// The resulting layout stored in aosOutputBuffer is as follows:
	/// Where t = translation, r = rotation, s = stretch, u = uniform scale, and b = total accumulated blending; a bone is stored as follows:
	/// floats 0-3:  r.x, r.y, r.z, r.w,
	/// floats 4-7:  t.x, t.y, t.z, b
	/// floats 8-11: s.x, s.y, s.z, u
	/// 
	/// In total, each bone is 12 floats or 48 bytes long. So the first 12 float values correspond to bone 0, then next 12 values correspond to bone 1, ect.
	/// </remarks>
	ACL_UNITY_API void samplePoseBlendedAdd(const void* compressedTransformTracks, const void* compressedScaleTracks, float* aosOutputBuffer, float blendFactor, float time, unsigned char keyframeInterpolationMode);

	/// <summary>
	/// Decompresses the pose at the given sample time and stores it in AOS format only for bones selected in the mask
	/// </summary>
	/// <param name="compressedTransformTracks">The 16-byte aligned pointer to the compressed tracks data</param>
	/// <param name="compressedScaleTracks">The 16-byte aligned pointer to the compressed scale tracks, or nullptr if all uniform scales are 1f.</param>
	/// <param name="aosOutputBuffer">A pointer to the buffer where the decompressed pose should be stored. See the detailed documentation for the layout.</param>
	/// <param name="mask">A pointer to bitmask represented by an array of unsigned 64 bit integers where the bone indices corresponding to set bits are sampled.</param>
	/// <param name="time">The time at which to sample the pose in seconds</param>
	/// <param name="keyframeInterpolationMode">The method used for interpolating between the two keyframes sampled: 
	/// 0 = lerp, 1 = round to next sample, 2 = round to previous sample, 3 = round to nearest sample</param>
	/// <remarks>
	/// The resulting layout stored in aosOutputBuffer is as follows:
	/// Where t = translation, r = rotation, s = stretch, u = uniform scale, and ~ = a padding float value; a bone is stored as follows:
	/// floats 0-3:  r.x, r.y, r.z, r.w,
	/// floats 4-7:  t.x, t.y, t.z, ~
	/// floats 8-11: s.x, s.y, s.z, u
	/// 
	/// In total, each bone is 12 floats or 48 bytes long. So the first 12 float values correspond to bone 0, then next 12 values correspond to bone 1, ect.
	/// </remarks>
	ACL_UNITY_API void samplePoseMasked(const void* compressedTransformTracks, const void* compressedScaleTracks, float* outputBuffer, const unsigned long long* mask, float time, unsigned char keyframeInterpolationMode);

	/// <summary>
	/// Decompresses the pose at the given sample time and stores it in AOS format only for bones selected in the mask, scaling each result by the blend factor
	/// </summary>
	/// <param name="compressedTransformTracks">The 16-byte aligned pointer to the compressed tracks data</param>
	/// <param name="compressedScaleTracks">The 16-byte aligned pointer to the compressed scale tracks, or nullptr if all uniform scales are 1f.</param>
	/// <param name="aosOutputBuffer">A pointer to the buffer where the decompressed pose should be stored. See the detailed documentation for the layout.</param>
	/// <param name="mask">A pointer to bitmask represented by an array of unsigned 64 bit integers where the bone indices corresponding to set bits are sampled.</param>
	/// <param name="blendFactor">A scale factor to apply to all decompressed values. Rotations are left unnormalized.</param>
	/// <param name="time">The time at which to sample the pose in seconds</param>
	/// <param name="keyframeInterpolationMode">The method used for interpolating between the two keyframes sampled: 
	/// 0 = lerp, 1 = round to next sample, 2 = round to previous sample, 3 = round to nearest sample</param>
	/// <remarks>
	/// The resulting layout stored in aosOutputBuffer is as follows:
	/// Where t = translation, r = rotation, s = stretch, u = uniform scale, and b = total accumulated blending; a bone is stored as follows:
	/// floats 0-3:  r.x, r.y, r.z, r.w,
	/// floats 4-7:  t.x, t.y, t.z, b
	/// floats 8-11: s.x, s.y, s.z, u
	/// 
	/// In total, each bone is 12 floats or 48 bytes long. So the first 12 float values correspond to bone 0, then next 12 values correspond to bone 1, ect.
	/// </remarks>
	ACL_UNITY_API void samplePoseMaskedBlendedFirst(const void* compressedTransformTracks, const void* compressedScaleTracks, float* outputBuffer, const unsigned long long* mask, float blendFactor, float time, unsigned char keyframeInterpolationMode);

	/// <summary>
	/// Decompresses the pose at the given sample time, scales each value by the blend factor, and adds it to the existing value in the buffer in AOS format only for bones selected in the mask
	/// </summary>
	/// <param name="compressedTransformTracks">The 16-byte aligned pointer to the compressed tracks data</param>
	/// <param name="compressedScaleTracks">The 16-byte aligned pointer to the compressed scale tracks, or nullptr if all uniform scales are 1f.</param>
	/// <param name="aosOutputBuffer">A pointer to the buffer where the decompressed pose should be stored. See the detailed documentation for the layout.</param>
	/// <param name="mask">A pointer to bitmask represented by an array of unsigned 64 bit integers where the bone indices corresponding to set bits are sampled.</param>
	/// <param name="blendFactor">A scale factor to apply to all decompressed values. Rotations are left unnormalized.</param>
	/// <param name="time">The time at which to sample the pose in seconds</param>
	/// <param name="keyframeInterpolationMode">The method used for interpolating between the two keyframes sampled: 
	/// 0 = lerp, 1 = round to next sample, 2 = round to previous sample, 3 = round to nearest sample</param>
	/// <remarks>
	/// The resulting layout stored in aosOutputBuffer is as follows:
	/// Where t = translation, r = rotation, s = stretch, u = uniform scale, and b = total accumulated blending; a bone is stored as follows:
	/// floats 0-3:  r.x, r.y, r.z, r.w,
	/// floats 4-7:  t.x, t.y, t.z, b
	/// floats 8-11: s.x, s.y, s.z, u
	/// 
	/// In total, each bone is 12 floats or 48 bytes long. So the first 12 float values correspond to bone 0, then next 12 values correspond to bone 1, ect.
	/// </remarks>
	ACL_UNITY_API void samplePoseMaskedBlendedAdd(const void* compressedTransformTracks, const void* compressedScaleTracks, float* outputBuffer, const unsigned long long* mask, float blendFactor, float time, unsigned char keyframeInterpolationMode);
	
	/// <summary>
	/// Decompresses the bone for the given boneIndex at the given sample time and stores it in a QVV (AOS format)
	/// </summary>
	/// <param name="compressedTransformTracks">The 16-byte aligned pointer to the compressed tracks data</param>
	/// <param name="compressedScaleTracks">The 16-byte aligned pointer to the compressed scale tracks, or nullptr if all uniform scales are 1f.</param>
	/// <param name="boneQvvs">A pointer to the buffer where the decompressed bone should be stored. See the detailed documentation for the layout.</param>
	/// <param name="boneIndex">The individual bone in the skeleton that should be sampled</param>
	/// <param name="time">The time at which to sample the pose in seconds</param>
	/// <param name="keyframeInterpolationMode">The method used for interpolating between the two keyframes sampled: 
	/// 0 = lerp, 1 = round to next sample, 2 = round to previous sample, 3 = round to nearest sample</param>
	/// <remarks>
	/// The resulting layout stored in boneQVV is as follows:
	/// Where t = translation, r = rotation, s = stretch, u = uniform scale, and ~ = a padding float value; the bone is stored as follows:
	/// floats 0-3:  r.x, r.y, r.z, r.w,
	/// floats 4-7:  t.x, t.y, t.z, ~
	/// floats 8-11: s.x, s.y, s.z, u
	/// 
	/// In total, the bone is 12 floats or 48 bytes long.
	/// </remarks>
	ACL_UNITY_API void sampleBone(const void* compressedTransformTracks, const void* compressedScaleTracks, float* boneQvvs, int boneIndex, float time, unsigned char keyframeInterpolationMode);

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
	/// Decompresses the scalar values at the given sample time, multiplies them by the blend factor, and stores the results in the floatOutputBuffer
	/// </summary>
	/// <param name="compressedFloatTracks">The 16-byte aligned pointer to the compressed tracks data</param>
	/// <param name="floatOutputBuffer">A pointer to the buffer where the decompressed sampled values should be stored.
	/// Each track stores a single float value at its respective index.</param>
	/// <param name="time">The time at which to sample the pose in seconds</param>
	/// <param name="keyframeInterpolationMode">The method used for interpolating between the two keyframes sampled: 
	/// 0 = lerp, 1 = round to next sample, 2 = round to previous sample, 3 = round to nearest sample</param>
	ACL_UNITY_API void sampleFloatsBlendedFirst(const void* compressedFloatTracks, float* floatOutputBuffer, float blendFactor, float time, unsigned char keyframeInterpolationMode);

	/// <summary>
	/// Decompresses the scalar values at the given sample time, multiplies it by the blend factor, and adds the results to the existing values in the floatOutputBuffer
	/// </summary>
	/// <param name="compressedFloatTracks">The 16-byte aligned pointer to the compressed tracks data</param>
	/// <param name="floatOutputBuffer">A pointer to the buffer where the decompressed sampled values should be stored.
	/// Each track stores a single float value at its respective index.</param>
	/// <param name="mask">A pointer to bitmask represented by an array of unsigned 64 bit integers where the track indices corresponding to set bits are sampled.</param>
	/// <param name="time">The time at which to sample the pose in seconds</param>
	/// <param name="keyframeInterpolationMode">The method used for interpolating between the two keyframes sampled: 
	/// 0 = lerp, 1 = round to next sample, 2 = round to previous sample, 3 = round to nearest sample</param>
	ACL_UNITY_API void sampleFloatsBlendedAdd(const void* compressedFloatTracks, float* floatOutputBuffer, float blendFactor, float time, unsigned char keyframeInterpolationMode);

	/// <summary>
	/// Decompresses the scalar values at the given sample time and stores the results in the floatOutputBuffer only for the tracks selected in the mask
	/// </summary>
	/// <param name="compressedFloatTracks">The 16-byte aligned pointer to the compressed tracks data</param>
	/// <param name="floatOutputBuffer">A pointer to the buffer where the decompressed sampled values should be stored.
	/// Each track stores a single float value at its respective index.</param>
	/// <param name="mask">A pointer to bitmask represented by an array of unsigned 64 bit integers where the track indices corresponding to set bits are sampled.</param>
	/// <param name="time">The time at which to sample the pose in seconds</param>
	/// <param name="keyframeInterpolationMode">The method used for interpolating between the two keyframes sampled: 
	/// 0 = lerp, 1 = round to next sample, 2 = round to previous sample, 3 = round to nearest sample</param>
	ACL_UNITY_API void sampleFloatsMasked(const void* compressedFloatTracks, float* floatOutputBuffer, const unsigned long long* mask, float time, unsigned char keyframeInterpolationMode);

	/// <summary>
	/// Decompresses the scalar values at the given sample time for the tracks selected in the mask, multiplies them by the blend factor, and stores the results in the floatOutputBuffer
	/// </summary>
	/// <param name="compressedFloatTracks">The 16-byte aligned pointer to the compressed tracks data</param>
	/// <param name="floatOutputBuffer">A pointer to the buffer where the decompressed sampled values should be stored.
	/// Each track stores a single float value at its respective index.</param>
	/// <param name="mask">A pointer to bitmask represented by an array of unsigned 64 bit integers where the track indices corresponding to set bits are sampled.</param>
	/// <param name="time">The time at which to sample the pose in seconds</param>
	/// <param name="keyframeInterpolationMode">The method used for interpolating between the two keyframes sampled: 
	/// 0 = lerp, 1 = round to next sample, 2 = round to previous sample, 3 = round to nearest sample</param>
	ACL_UNITY_API void sampleFloatsMaskedBlendedFirst(const void* compressedFloatTracks, float* floatOutputBuffer, const unsigned long long* mask, float blendFactor, float time, unsigned char keyframeInterpolationMode);

	/// <summary>
	/// Decompresses the scalar values at the given sample time for the tracks selected in the mask, multiplies them by the blend factor, 
	/// and adds the results to the existing values in the floatOutputBuffer
	/// </summary>
	/// <param name="compressedFloatTracks">The 16-byte aligned pointer to the compressed tracks data</param>
	/// <param name="floatOutputBuffer">A pointer to the buffer where the decompressed sampled values should be stored.
	/// Each track stores a single float value at its respective index.</param>
	/// <param name="mask">A pointer to bitmask represented by an array of unsigned 64 bit integers where the track indices corresponding to set bits are sampled.</param>
	/// <param name="time">The time at which to sample the pose in seconds</param>
	/// <param name="keyframeInterpolationMode">The method used for interpolating between the two keyframes sampled: 
	/// 0 = lerp, 1 = round to next sample, 2 = round to previous sample, 3 = round to nearest sample</param>
	ACL_UNITY_API void sampleFloatsMaskedBlendedAdd(const void* compressedFloatTracks, float* floatOutputBuffer, const unsigned long long* mask, float blendFactor, float time, unsigned char keyframeInterpolationMode);

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