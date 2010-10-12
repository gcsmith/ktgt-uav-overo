#ifndef _UAV_FP32__H_
#define _UAV_FP32__H_

// 32-bit fixed point

#define FP32_BP  16
#define FP32_BP2 32
#define FP32_BPH 8

#define FP32_FROM_INT(i)    ((int32_t)(i) << FP32_BP)
#define FP32_TO_INT(f)      ((f) >> FP32_BP)
#define FP32_DECIMALS(f)    ((f) & 0xff)
#define FP32_MUL(x, y)      (int32_t)((((x) >> FP32_BPH) * ((y) >> FP32_BPH)))
#if 1
#define FP32_DIV(x, y)      (int32_t)((((x) << 6) / ((y) >> 6)) << 4)
#else
#define FP32_DIV(x, y)      (int32_t)((((int64_t)(x) << FP32_BP2) / (y)) >> FP32_BP)
#endif

#endif // _UAV_FP32__H_

