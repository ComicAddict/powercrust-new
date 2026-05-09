/*
 * simd_math.h -- SIMD-accelerated 3-D math helpers for PowerCrust
 *
 * Provides two inline functions:
 *
 *   sqdist_fast(a, b)
 *       Squared Euclidean distance between 3-D points a[] and b[].
 *       Uses SSE2 to process two components in parallel; the third
 *       component is handled as a scalar FMA.
 *       Falls back to scalar code when SSE2 is unavailable.
 *
 *   max_edge_sq_avx2(indices)
 *       Maximum squared edge length among all 6 pairs of 4 points.
 *       Uses AVX2+FMA to evaluate the first 4 pairs simultaneously,
 *       then finishes the remaining 2 pairs with SSE2.
 *       Falls back to scalar code when AVX2/FMA is unavailable.
 *
 * Compile with -mavx2 -mfma (GCC/Clang) to enable full acceleration.
 * SSE2 is always available on x86-64, so the sqdist_fast path is
 * always active on that architecture.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================
 * SSE2 path: sqdist_fast
 * ========================================================= */

#if defined(__SSE2__)
#  include <immintrin.h>

static inline double sqdist_fast(const double a[3], const double b[3])
{
    /* Load a[0..1] and b[0..1] into 128-bit registers (unaligned). */
    __m128d va = _mm_loadu_pd(a);
    __m128d vb = _mm_loadu_pd(b);
    __m128d d  = _mm_sub_pd(va, vb);
    d          = _mm_mul_pd(d, d);          /* [dx^2, dy^2] */

    /* Horizontal sum: move high lane to low, add. */
    __m128d hi  = _mm_unpackhi_pd(d, d);   /* [dy^2, dy^2] */
    __m128d sum = _mm_add_sd(d, hi);       /* [dx^2+dy^2, ...] */

    double result;
    _mm_store_sd(&result, sum);

    double dz = a[2] - b[2];
    return result + dz * dz;
}

/* =========================================================
 * AVX2 + FMA path: max_edge_sq_avx2
 *
 * 6 pairs from 4 points (labelled 0-3):
 *   batch1 (4 wide): (0,1), (0,2), (0,3), (1,2)
 *   batch2 (scalar): (1,3), (2,3)
 * ========================================================= */

#  if defined(__AVX2__) && defined(__FMA__)

static inline double max_edge_sq_avx2(const double indices[4][3])
{
    /*
     * _mm256_set_pd(e3, e2, e1, e0) stores:
     *   lane0=e0, lane1=e1, lane2=e2, lane3=e3
     *
     * Lane mapping for batch1:
     *   lane0: pair (0,1)    lane1: pair (0,2)
     *   lane2: pair (0,3)    lane3: pair (1,2)
     */

    /* --- X coordinates --- */
    __m256d ax = _mm256_set_pd(indices[1][0], indices[0][0],
                               indices[0][0], indices[0][0]);
    __m256d bx = _mm256_set_pd(indices[2][0], indices[3][0],
                               indices[2][0], indices[1][0]);
    __m256d dx = _mm256_sub_pd(ax, bx);
    __m256d sq = _mm256_mul_pd(dx, dx);

    /* --- Y coordinates (fused multiply-add: sq += dy*dy) --- */
    __m256d ay = _mm256_set_pd(indices[1][1], indices[0][1],
                               indices[0][1], indices[0][1]);
    __m256d by = _mm256_set_pd(indices[2][1], indices[3][1],
                               indices[2][1], indices[1][1]);
    __m256d dy = _mm256_sub_pd(ay, by);
    sq = _mm256_fmadd_pd(dy, dy, sq);

    /* --- Z coordinates --- */
    __m256d az = _mm256_set_pd(indices[1][2], indices[0][2],
                               indices[0][2], indices[0][2]);
    __m256d bz = _mm256_set_pd(indices[2][2], indices[3][2],
                               indices[2][2], indices[1][2]);
    __m256d dz = _mm256_sub_pd(az, bz);
    sq = _mm256_fmadd_pd(dz, dz, sq);

    /* Horizontal max across 4 lanes. */
    double b1[4];
    _mm256_storeu_pd(b1, sq);
    double m = b1[0];
    if (b1[1] > m) m = b1[1];
    if (b1[2] > m) m = b1[2];
    if (b1[3] > m) m = b1[3];

    /* Remaining two pairs using sqdist_fast (SSE2). */
    double d13 = sqdist_fast(indices[1], indices[3]);
    double d23 = sqdist_fast(indices[2], indices[3]);
    if (d13 > m) m = d13;
    if (d23 > m) m = d23;
    return m;
}

#  else /* SSE2 but no AVX2/FMA: scalar fallback for max_edge_sq */

static inline double max_edge_sq_avx2(const double indices[4][3])
{
    double m = 0.0;
    int i, j;
    for (i = 0; i < 4; i++)
        for (j = i + 1; j < 4; j++) {
            double d = sqdist_fast(indices[i], indices[j]);
            if (d > m) m = d;
        }
    return m;
}

#  endif /* AVX2+FMA */

#else /* No SSE2: full scalar fallback */

static inline double sqdist_fast(const double a[3], const double b[3])
{
    double dx = a[0]-b[0], dy = a[1]-b[1], dz = a[2]-b[2];
    return dx*dx + dy*dy + dz*dz;
}

static inline double max_edge_sq_avx2(const double indices[4][3])
{
    double m = 0.0;
    int i, j;
    for (i = 0; i < 4; i++)
        for (j = i + 1; j < 4; j++) {
            double d = sqdist_fast(indices[i], indices[j]);
            if (d > m) m = d;
        }
    return m;
}

#endif /* SSE2 */

#ifdef __cplusplus
}
#endif
