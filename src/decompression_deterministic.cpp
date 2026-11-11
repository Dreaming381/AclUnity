// Cross-platform deterministic decompression.
// RTM and ACL fall back to their scalar implementations, which only use IEEE-754 operations that are exactly rounded
// (add, sub, mul, div, sqrt, float/int conversions). This avoids fused multiply-add on ARM64 and rsqrt/rcp estimates on x86.
// The build must also disable floating-point contraction for this file (see CMakeLists.txt), which the pragmas below repeat.
// The version namespaces keep every RTM and ACL inline function in this file distinct from the SIMD ones in decompression.cpp.
// Each export runs its implementation with IEEE rounding and denormal handling, because callers such as Burst's
// FloatMode.Deterministic set flush modes that differ between x86 and ARM.
#define RTM_NO_INTRINSICS
#define RTM_ENABLE_VERSION_NAMESPACE
#define ACL_ENABLE_VERSION_NAMESPACE

#if defined(_MSC_VER) && !defined(__clang__)
	#pragma fp_contract(off)
	#pragma float_control(precise, on)
	#define ACL_UNITY_NOINLINE __declspec(noinline)
#else
	#if defined(__clang__)
		#pragma STDC FP_CONTRACT OFF
	#endif
	#define ACL_UNITY_NOINLINE __attribute__((noinline))
#endif

#define ACL_UNITY_SAMPLE_NAME(name) name##DeterministicInner
#define ACL_UNITY_SAMPLE_API static ACL_UNITY_NOINLINE
#include "decompression_impl.h"

#include "fp_environment.h"

namespace
{
	// The implementations must stay out of line so that the compiler cannot move their math across the register writes.
	class IeeeEnvironmentScope
	{
	private:
		AclUnityFp::Register m_saved;
		bool                 m_changed;

	public:
		IeeeEnvironmentScope()
		{
			m_saved                   = AclUnityFp::read();
			AclUnityFp::Register ieee = AclUnityFp::toIeee(m_saved);
			m_changed                 = ieee != m_saved;
			if (m_changed)
				AclUnityFp::write(ieee);
		}

		~IeeeEnvironmentScope()
		{
			if (m_changed)
				AclUnityFp::write(m_saved);
		}
	};
}

ACL_UNITY_API void samplePoseDeterministic(const void* compressedTransformTracks, const void* compressedScaleTracks, float* aosOutputBuffer, float time, unsigned char keyframeInterpolationMode)
{
	IeeeEnvironmentScope scope;
	samplePoseDeterministicInner(compressedTransformTracks, compressedScaleTracks, aosOutputBuffer, time, keyframeInterpolationMode);
}

ACL_UNITY_API void samplePoseBlendedFirstDeterministic(const void* compressedTransformTracks, const void* compressedScaleTracks, float* aosOutputBuffer, float blendFactor, float time, unsigned char keyframeInterpolationMode)
{
	IeeeEnvironmentScope scope;
	samplePoseBlendedFirstDeterministicInner(compressedTransformTracks, compressedScaleTracks, aosOutputBuffer, blendFactor, time, keyframeInterpolationMode);
}

ACL_UNITY_API void samplePoseBlendedAddDeterministic(const void* compressedTransformTracks, const void* compressedScaleTracks, float* aosOutputBuffer, float blendFactor, float time, unsigned char keyframeInterpolationMode)
{
	IeeeEnvironmentScope scope;
	samplePoseBlendedAddDeterministicInner(compressedTransformTracks, compressedScaleTracks, aosOutputBuffer, blendFactor, time, keyframeInterpolationMode);
}

ACL_UNITY_API void samplePoseMaskedDeterministic(const void* compressedTransformTracks, const void* compressedScaleTracks, float* outputBuffer, const unsigned long long* mask, float time, unsigned char keyframeInterpolationMode)
{
	IeeeEnvironmentScope scope;
	samplePoseMaskedDeterministicInner(compressedTransformTracks, compressedScaleTracks, outputBuffer, mask, time, keyframeInterpolationMode);
}

ACL_UNITY_API void samplePoseMaskedBlendedFirstDeterministic(const void* compressedTransformTracks, const void* compressedScaleTracks, float* outputBuffer, const unsigned long long* mask, float blendFactor, float time, unsigned char keyframeInterpolationMode)
{
	IeeeEnvironmentScope scope;
	samplePoseMaskedBlendedFirstDeterministicInner(compressedTransformTracks, compressedScaleTracks, outputBuffer, mask, blendFactor, time, keyframeInterpolationMode);
}

ACL_UNITY_API void samplePoseMaskedBlendedAddDeterministic(const void* compressedTransformTracks, const void* compressedScaleTracks, float* outputBuffer, const unsigned long long* mask, float blendFactor, float time, unsigned char keyframeInterpolationMode)
{
	IeeeEnvironmentScope scope;
	samplePoseMaskedBlendedAddDeterministicInner(compressedTransformTracks, compressedScaleTracks, outputBuffer, mask, blendFactor, time, keyframeInterpolationMode);
}

ACL_UNITY_API void sampleBoneDeterministic(const void* compressedTransformTracks, const void* compressedScaleTracks, float* boneQvvs, int boneIndex, float time, unsigned char keyframeInterpolationMode)
{
	IeeeEnvironmentScope scope;
	sampleBoneDeterministicInner(compressedTransformTracks, compressedScaleTracks, boneQvvs, boneIndex, time, keyframeInterpolationMode);
}

ACL_UNITY_API void sampleFloatsDeterministic(const void* compressedFloatTracks, float* floatOutputBuffer, float time, unsigned char keyframeInterpolationMode)
{
	IeeeEnvironmentScope scope;
	sampleFloatsDeterministicInner(compressedFloatTracks, floatOutputBuffer, time, keyframeInterpolationMode);
}

ACL_UNITY_API void sampleFloatsBlendedFirstDeterministic(const void* compressedFloatTracks, float* floatOutputBuffer, float blendFactor, float time, unsigned char keyframeInterpolationMode)
{
	IeeeEnvironmentScope scope;
	sampleFloatsBlendedFirstDeterministicInner(compressedFloatTracks, floatOutputBuffer, blendFactor, time, keyframeInterpolationMode);
}

ACL_UNITY_API void sampleFloatsBlendedAddDeterministic(const void* compressedFloatTracks, float* floatOutputBuffer, float blendFactor, float time, unsigned char keyframeInterpolationMode)
{
	IeeeEnvironmentScope scope;
	sampleFloatsBlendedAddDeterministicInner(compressedFloatTracks, floatOutputBuffer, blendFactor, time, keyframeInterpolationMode);
}

ACL_UNITY_API void sampleFloatsMaskedDeterministic(const void* compressedFloatTracks, float* floatOutputBuffer, const unsigned long long* mask, float time, unsigned char keyframeInterpolationMode)
{
	IeeeEnvironmentScope scope;
	sampleFloatsMaskedDeterministicInner(compressedFloatTracks, floatOutputBuffer, mask, time, keyframeInterpolationMode);
}

ACL_UNITY_API void sampleFloatsMaskedBlendedFirstDeterministic(const void* compressedFloatTracks, float* floatOutputBuffer, const unsigned long long* mask, float blendFactor, float time, unsigned char keyframeInterpolationMode)
{
	IeeeEnvironmentScope scope;
	sampleFloatsMaskedBlendedFirstDeterministicInner(compressedFloatTracks, floatOutputBuffer, mask, blendFactor, time, keyframeInterpolationMode);
}

ACL_UNITY_API void sampleFloatsMaskedBlendedAddDeterministic(const void* compressedFloatTracks, float* floatOutputBuffer, const unsigned long long* mask, float blendFactor, float time, unsigned char keyframeInterpolationMode)
{
	IeeeEnvironmentScope scope;
	sampleFloatsMaskedBlendedAddDeterministicInner(compressedFloatTracks, floatOutputBuffer, mask, blendFactor, time, keyframeInterpolationMode);
}

ACL_UNITY_API float sampleFloatDeterministic(const void* compressedFloatTracks, int trackIndex, float time, unsigned char keyframeInterpolationMode)
{
	IeeeEnvironmentScope scope;
	return sampleFloatDeterministicInner(compressedFloatTracks, trackIndex, time, keyframeInterpolationMode);
}
