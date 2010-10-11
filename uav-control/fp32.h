#ifndef _UAV_FP32__H_
#define _UAV_FP32__H_

#define FP32_Y                  11
#define FP32_Yp1                12
#define FP32_2_TO_Ym1           1024    //10^2

#define INT_TO_FP32(fixed)      ((fixed) << FP32_Y)
#define FP32_TO_INT(myInt)      ((myInt) >> FP32_Y)
#define FP32_DECIMALS(fixed)    ((fixed) & 0xff)
#define FP32_MUL(x, y)          (((x) * (y) + FP32_2_TO_Ym1 ) >> FP32_Y)
#define FP32_DIV(x, y)          ((((x) << (FP32_Y + 1))/(y)) / 2)

#endif // _UAV_FP32__H_

