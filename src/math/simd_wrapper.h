#ifndef _SLAM_UTILITY_SIMD_WRAPPER_H_
#define _SLAM_UTILITY_SIMD_WRAPPER_H_

#include <cstdint>

#if defined(__x86_64__) || defined(_M_X64)
    #include <immintrin.h>
    #define SIMD_X86
#elif defined(__arm__) || defined(__aarch64__)
    #include <arm_neon.h>
    #define SIMD_ARM
#endif

namespace slam_utility {
namespace simd {

#ifdef SIMD_X86
    using uint8x16_t = __m128i;
    using float32x4_t = __m128;
#elif defined(SIMD_ARM)
    using uint8x16_t = uint8x16_t;
    using float32x4_t = float32x4_t;
#endif

inline float32x4_t LoadF(const float* p) {
#ifdef SIMD_X86
    return _mm_loadu_ps(p);
#elif defined(SIMD_ARM)
    return vld1q_f32(p);
#else
    return float32x4_t{};
#endif
}

inline void StoreF(float* p, float32x4_t v) {
#ifdef SIMD_X86
    _mm_storeu_ps(p, v);
#elif defined(SIMD_ARM)
    vst1q_f32(p, v);
#endif
}

inline float32x4_t SetF(float v) {
#ifdef SIMD_X86
    return _mm_set1_ps(v);
#elif defined(SIMD_ARM)
    return vdupq_n_f32(v);
#else
    return float32x4_t{};
#endif
}

inline float32x4_t AddF(float32x4_t a, float32x4_t b) {
#ifdef SIMD_X86
    return _mm_add_ps(a, b);
#elif defined(SIMD_ARM)
    return vaddq_f32(a, b);
#else
    return a;
#endif
}

inline float32x4_t SubF(float32x4_t a, float32x4_t b) {
#ifdef SIMD_X86
    return _mm_sub_ps(a, b);
#elif defined(SIMD_ARM)
    return vsubq_f32(a, b);
#else
    return a;
#endif
}

inline float32x4_t MulF(float32x4_t a, float32x4_t b) {
#ifdef SIMD_X86
    return _mm_mul_ps(a, b);
#elif defined(SIMD_ARM)
    return vmulq_f32(a, b);
#else
    return a;
#endif
}

inline float32x4_t LerpF(float32x4_t v0, float32x4_t v1, float32x4_t s) {
    float32x4_t one = SetF(1.0f);
    return AddF(MulF(SubF(one, s), v0), MulF(s, v1));
}

inline float32x4_t FloorF(float32x4_t v) {
#ifdef SIMD_X86
    return _mm_floor_ps(v);
#elif defined(SIMD_ARM)
    return vrndmq_f32(v);
#else
    return v;
#endif
}

inline float32x4_t ConvertU8ToF32(uint8x16_t v) {
#ifdef SIMD_X86
    __m128i zero = _mm_setzero_si128();
    __m128i v16 = _mm_unpacklo_epi8(v, zero);
    __m128i v32 = _mm_unpacklo_epi16(v16, zero);
    return _mm_cvtepi32_ps(v32);
#elif defined(SIMD_ARM)
    uint16x8_t v16 = vmovl_u8(vget_low_u8(v));
    uint32x4_t v32 = vmovl_u16(vget_low_u16(v16));
    return vcvtq_f32_u32(v32);
#else
    return float32x4_t{};
#endif
}

inline uint8x16_t LoadU(const uint8_t* p) {
#ifdef SIMD_X86
    return _mm_loadu_si128(reinterpret_cast<const __m128i*>(p));
#elif defined(SIMD_ARM)
    return vld1q_u8(p);
#else
    return uint8x16_t{};
#endif
}

inline void StoreU(uint8_t* p, uint8x16_t v) {
#ifdef SIMD_X86
    _mm_storeu_si128(reinterpret_cast<__m128i*>(p), v);
#elif defined(SIMD_ARM)
    vst1q_u8(p, v);
#endif
}

inline uint8x16_t CompareGreater(uint8x16_t a, uint8x16_t b) {
#ifdef SIMD_X86
    __m128i offset = _mm_set1_epi8(static_cast<char>(0x80));
    return _mm_cmpgt_epi8(_mm_xor_si128(a, offset), _mm_xor_si128(b, offset));
#elif defined(SIMD_ARM)
    return vcgtq_u8(a, b);
#else
    return a;
#endif
}

inline uint8x16_t ShiftLeft1(uint8x16_t v) {
#ifdef SIMD_X86
    __m128i v16 = _mm_slli_epi16(v, 1);
    return _mm_and_si128(v16, _mm_set1_epi8(static_cast<char>(0xFE)));
#elif defined(SIMD_ARM)
    return vshlq_n_u8(v, 1);
#else
    return v;
#endif
}

inline uint8x16_t AndOne(uint8x16_t v) {
#ifdef SIMD_X86
    return _mm_and_si128(v, _mm_set1_epi8(1));
#elif defined(SIMD_ARM)
    return vandq_u8(v, vdupq_n_u8(1));
#else
    return v;
#endif
}

inline uint8x16_t Or(uint8x16_t a, uint8x16_t b) {
#ifdef SIMD_X86
    return _mm_or_si128(a, b);
#elif defined(SIMD_ARM)
    return vorrq_u8(a, b);
#else
    return a;
#endif
}

} // namespace simd
} // namespace slam_utility

#endif
