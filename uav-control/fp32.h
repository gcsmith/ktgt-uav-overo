#ifndef _UAV_FP32__H_
#define _UAV_FP32__H_

#define USE_FP_22_10
#undef USE_FP_INT64

// 32-bit fixed point

#define FP32_BP2 (FP32_BP << 1)
#define FP32_BPH (FP32_BP >> 1)

#ifdef USE_FP_22_10
#define FP32_BP         10
#define FP32_MUL(x, y)  (int32_t)(((x) * (y)) >> FP32_BP)
#define FP32_DIV(x, y)  (int32_t)(((x) << FP32_BP) / (y))
#else
#define FP32_BP         16
#define FP32_MUL(x, y)  (int32_t)((((x) >> FP32_BPH) * ((y) >> FP32_BPH)))
#ifdef USE_FP_INT64
#define FP32_DIV(x, y)  (int32_t)((((int64_t)(x) << FP32_BP2) / (y)) >> FP32_BP)
#else
#define FP32_DIV(x, y)  (int32_t)((((x) << 4) / ((y) >> 5)) << 7)
#endif
#endif

#define FP32_DECIMALS(f)    ((f) & ((1 << FP32_BP) - 1))
#define FP32_FROM_INT(i)    ((int32_t)(i) << FP32_BP)
#define FP32_TO_INT(f)      ((f) >> FP32_BP)
#define FP32_TO_FRAC(f)     (FP32_DECIMALS(f) / (float)(1 << (FP32_BP - 1)))
#define FP32_TO_FLOAT(f)    (FP32_TO_INT(f) + FP32_TO_FRAC(f))

#endif // _UAV_FP32__H_

