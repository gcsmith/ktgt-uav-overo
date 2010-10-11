#ifndef _UAV_FP32__H_
#define _UAV_FP32__H_

#define INT_TO_FP32(fixed)      ((fixed) << 8)
#define FP32_TO_INT(myInt)      ((myInt) >> 8)
#define FP32_DECIMALS(fixed)    ((fixed) & 0xff)
#define FP32_MUL(x, y)          (((x) * (y) + 128) >> 8)
#define FP32_DIV(x, y)          (((x) << 8)/(y))

#endif // _UAV_FP32__H_

