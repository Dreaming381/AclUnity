#pragma once

#define ACL_NO_ALLOCATOR_TRACKING

#include "acl_unity_defines.h"

extern "C"
{
	/// <summary>
	/// Compresses a skeletal animation clip
	/// </summary>
	/// <param name="parentIndices">An array of indices to the corresponding parent bone. 
	/// If the index is itself, it has no parent. If the index is negated, the bone will not inherit scale.</param>
	/// <param name="numBones">The number of bones in the skeleton</param>
	/// <param name="compressionLevel">The level of compression to apply. 0 = lowest, fastest; 4 = highest, slower</param>
	/// <param name="aosClipData">A sequence of uniformly sampled keyframe AOS poses of the animation clip. See the detailed documentation for the layout.</param>
	/// <param name="numSamples">The number of uniformly sampled keyframe AOS poses</param>
	/// <param name="sampleRate">The sample rate of the animation clip</param>
	/// <param name="maxDistanceError">The distance a virtual vertex is allowed to deviate from the source animation in Unity units. Recommended default is 0.0001</param>
	/// <param name="sampledErrorDistanceFromBone">How far away the virtual vertex is from the bone in Unity units. Higher values are more accurate. Recommended default is 0.03</param>
	/// <param name="maxNegligibleTranslationDrift">If a bone slides less than this distance in Unity units, it can be considered constant. Recommended default is 0.00001</param>
	/// <param name="maxNegligibleScaleDrift">If a bone's scale value changes less than this value, it can be considered constant. Recommended default is 0.00001</param>
	/// <param name="outCompressedSizeInBytes">This function writes the number of bytes of compressed animation to this variable.</param>
	/// <returns>A pointer to a 16 byte aligned block of memory containing compressed animation data. The size in bytes is stored in outCompressedSizeInBytes.
	/// The caller owns the memory and is responsible for disposing it once it is done copying it.</returns>
	/// <remarks>
	/// The expected layout in aosClipData is as follows:
	/// Where t = translation, r = rotation, s = scale, and ~ = a padding float value; a bone is stored as follows:
	/// floats 0-3:  r.x, r.y, r.z, r.w,
	/// floats 4-7:  t.x, t.y, t.z, ~
	/// floats 8-11: s.x, s.y, s.z, ~
	/// 
	/// In total, each bone is 12 floats or 48 bytes long. So the first 12 float values correspond to bone 0, then next 12 values correspond to bone 1, ect.
	/// All samples for a single bone are stored consecutively. Then the next bone is stored for all keyframes in the same format and so on.
	/// 
	/// WARNING: The data must be 16-byte aligned.
	/// 
	/// Currently compression levels 2 and below are identical.
	/// </remarks>
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
											 int*                outCompressedSizeInBytes);

	/// <summary>
	/// Compresses an animation clip containing multiple scalar values (tracks)
	/// </summary>
	/// <param name="numTracks">The number of scalar tracks to compress</param>
	/// <param name="compressionLevel">The level of compression to apply. 0 = lowest, fastest; 4 = highest, slower</param>
	/// <param name="clipData">A sequence of uniformly sampled keyframe values for each scalar in the animation clip. See the detailed documentation for the layout.</param>
	/// <param name="numSamples">The number of uniformly sampled keyframes</param>
	/// <param name="sampleRate">The sample rate of the animation clip</param>
	/// <param name="maxError">An array specifying the maximum allowed deviation of each scalar value from the source animation</param>
	/// <param name="outCompressedSizeInBytes">This function writes the number of bytes of compressed animation to this variable.</param>
	/// <returns>A pointer to a 16 byte aligned block of memory containing compressed animation data. The size in bytes is stored in outCompressedSizeInBytes.
	/// The caller owns the memory and is responsible for disposing it once it is done copying it.</returns>
	/// <remarks>
	/// All samples for a single scalar track must be stored consecutively in clipData. Then the next track is stored for all keyframes and so on.
	/// 
	/// Currently compression levels 2 and below are identical.
	/// </remarks>
	ACL_UNITY_API void* compressScalarsClip(signed short numTracks, 
											signed short compressionLevel, 
											const float* clipData, 
											int			 numSamples, 
											float		 sampleRate, 
											float*		 maxErrors, 
											int*		 outCompressedSizeInBytes);
	
	/// <summary>
	/// Disposes a buffer generated from one of the compression functions
	/// </summary>
	/// <param name="compressedTracksBuffer">The buffer to dispose</param>
	ACL_UNITY_API void disposeCompressedTracksBuffer(void* compressedTracksBuffer, void* allocator);
}