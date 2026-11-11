// Access to the floating-point control register (MXCSR on x86, FPCR on ARM64, FPSCR on ARMv7).
// Status flags are left untouched by toIeee(), so a register that only differs in its flags compares equal.
#pragma once

#include <cstdint>

#if defined(_M_X64) || defined(_M_IX86) || defined(__x86_64__) || defined(__i386__)
	#include <xmmintrin.h>
	#define ACL_UNITY_FP_X86
#elif defined(_M_ARM64) || defined(__aarch64__)
	#if defined(_MSC_VER) && !defined(__clang__)
		#include <intrin.h>
		#if !defined(ARM64_SYSREG)
			#define ARM64_SYSREG(op0, op1, crn, crm, op2) (((op0 & 1) << 14) | ((op1 & 7) << 11) | ((crn & 15) << 7) | ((crm & 15) << 3) | ((op2 & 7) << 0))
		#endif
		#if !defined(ARM64_FPCR)
			#define ARM64_FPCR ARM64_SYSREG(3, 3, 4, 4, 0)
		#endif
	#endif
	#define ACL_UNITY_FP_ARM64
#elif defined(__arm__)
	#define ACL_UNITY_FP_ARM32
#endif

namespace AclUnityFp
{
	using Register = uint64_t;

#if defined(ACL_UNITY_FP_X86)
	// FTZ (bit 15) and DAZ (bit 6)
	static constexpr Register kFlushBits = 0x8040;
	// Rounding control (bits 13-14)
	static constexpr Register kRoundingBits = 0x6000;
	// What Burst's FloatMode.Deterministic sets
	static constexpr Register kBurstDeterministicBits = 0x8000;
	static constexpr Register kRoundTowardZero = 0x6000;

	inline Register read() { return _mm_getcsr(); }
	inline void     write(Register value) { _mm_setcsr(static_cast<unsigned int>(value)); }
#elif defined(ACL_UNITY_FP_ARM64)
	// FZ (bit 24), plus AH (bit 1) and FIZ (bit 0) from FEAT_AFP, which are RES0 without it
	static constexpr Register kFlushBits = (1ull << 24) | (1ull << 1) | (1ull << 0);
	// RMode (bits 22-23)
	static constexpr Register kRoundingBits = 3ull << 22;
	static constexpr Register kBurstDeterministicBits = 1ull << 24;
	static constexpr Register kRoundTowardZero = 3ull << 22;

	#if defined(_MSC_VER) && !defined(__clang__)
	inline Register read() { return static_cast<Register>(_ReadStatusReg(ARM64_FPCR)); }
	inline void     write(Register value) { _WriteStatusReg(ARM64_FPCR, static_cast<__int64>(value)); }
	#else
	inline Register read()
	{
		Register value;
		__asm__ __volatile__("mrs %0, fpcr" : "=r"(value) : : "memory");
		return value;
	}
	inline void write(Register value) { __asm__ __volatile__("msr fpcr, %0" : : "r"(value) : "memory"); }
	#endif
#elif defined(ACL_UNITY_FP_ARM32)
	// FZ (bit 24)
	static constexpr Register kFlushBits = 1ull << 24;
	// RMode (bits 22-23)
	static constexpr Register kRoundingBits = 3ull << 22;
	static constexpr Register kBurstDeterministicBits = 1ull << 24;
	static constexpr Register kRoundTowardZero = 3ull << 22;

	inline Register read()
	{
		uint32_t value;
		__asm__ __volatile__("vmrs %0, fpscr" : "=r"(value) : : "memory");
		return value;
	}
	inline void write(Register value)
	{
		uint32_t value32 = static_cast<uint32_t>(value);
		__asm__ __volatile__("vmsr fpscr, %0" : : "r"(value32) : "memory");
	}
#else
	static constexpr Register kFlushBits = 0;
	static constexpr Register kRoundingBits = 0;
	static constexpr Register kBurstDeterministicBits = 0;
	static constexpr Register kRoundTowardZero = 0;

	inline Register read() { return 0; }
	inline void     write(Register) {}
#endif

	// Round-to-nearest with denormals neither flushed nor treated as zero.
	inline Register toIeee(Register value) { return value & ~(kFlushBits | kRoundingBits); }
}
