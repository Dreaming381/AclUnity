// Must be included before any other RTM or ACL header.
// RTM does not emit FMA on x86, so we replace its multiply-add helper macros when AVX2 (and therefore FMA) is enabled.
// This works because RTM's function bodies expand these macros where they are defined, and the macro header is only included once.
#pragma once

#include "rtm/math.h"
#include "rtm/impl/macros.vector4.impl.h"

#if defined(RTM_FMA_INTRINSICS) && defined(RTM_SSE2_INTRINSICS) && !defined(RTM_NEON_INTRINSICS)
	#undef RTM_VECTOR4F_MULV_ADD
	#undef RTM_VECTOR4F_MULS_ADD
	#undef RTM_VECTOR4F_NEG_MULV_SUB
	#undef RTM_VECTOR4F_NEG_MULS_SUB

	#define RTM_VECTOR4F_MULV_ADD(v0, v1, v2) _mm_fmadd_ps((v0), (v1), (v2))
	#define RTM_VECTOR4F_MULS_ADD(v0, s1, v2) _mm_fmadd_ps((v0), _mm_set_ps1((s1)), (v2))
	#define RTM_VECTOR4F_NEG_MULV_SUB(v0, v1, v2) _mm_fnmadd_ps((v0), (v1), (v2))
	#define RTM_VECTOR4F_NEG_MULS_SUB(v0, s1, v2) _mm_fnmadd_ps((v0), _mm_set_ps1((s1)), (v2))
#endif
