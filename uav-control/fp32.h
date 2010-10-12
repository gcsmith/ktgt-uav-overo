#ifndef _UAV_FP32__H_
#define _UAV_FP32__H_

// 32-bit fixed point

#define FP32_BP  16
#define FP32_BP2 32

#define FP32_FROM_INT(i)    ((int32_t)(i) << FP32_BP)
#define FP32_TO_INT(f)      ((f) >> FP32_BP)
#define FP32_DECIMALS(f)    ((f) & 0xff)
#define FP32_MUL(x, y)      (int32_t)(((int64_t)(x) * (int64_t)(y)) >> FP32_BP)
#define FP32_DIV(x, y)      (int32_t)((((int64_t)(x) << FP32_BP2) / (y)) >> FP32_BP)

// 16-bit fixed point

#define FP16_BP  8
#define FP16_BP2 16

#define FP16_FROM_INT(i)    ((int16_t)(i) << FP16_BP)
#define FP16_TO_INT(f)      ((f) >> FP16_BP)
#define FP16_DECIMALS(f)    ((f) & 0xff)
#define FP16_MUL(x, y)      (int16_t)(((int32_t)(x) * (int32_t)(y)) >> FP16_BP)
#define FP16_DIV(x, y)      (int16_t)((((int32_t)(x) << FP16_BP2) / (y)) >> FP16_BP)

#endif // _UAV_FP32__H_

