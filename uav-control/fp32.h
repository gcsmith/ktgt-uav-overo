#ifndef _UAV_FP32__H_
#define _UAV_FP32__H_

// 32-bit fixed point

#define FP32_BP  16
#define FP32_BP2 32
#define FP32_BPH 8
/*
#define FP32_FROM_INT(i)    ((int32_t)(i) << FP32_BP)
#define FP32_TO_INT(f)      ((f) >> FP32_BP)
#define FP32_DECIMALS(f)    ((f) & 0xff)
#define FP32_MUL(x, y)      (int32_t)((((x) >> FP32_BPH) * ((y) >> FP32_BPH)))
#if 1
#define FP32_DIV(x, y)      (int32_t)((((x) << 6) / ((y) >> 6)) << 4)
#else
#define FP32_DIV(x, y)      (int32_t)((((int64_t)(x) << FP32_BP2) / (y)) >> FP32_BP)
#endif
*/
///*
// Y = 11
#define FP32_Y                  11
#define FP32_Yp1                12
#define FP32_2_TO_Ym1           1024    //2^10
#define FP32_2_TO_Y_MINUS_1     2047    //(2^11) - 1
//#define TW0_TO_31 2147483648
//*/
/*
// Y = 12
#define FP32_Y                  12
#define FP32_Yp1                13
#define FP32_2_TO_Ym1           2048    //2^11
#define FP32_2_TO_Y_MINUS_1     4095    //(2^12) - 1
*/
/*
#define INT_TO_FP32(fixed)      ((fixed) << FP32_Y)
#define FP32_TO_INT(myInt)      ((myInt) >> FP32_Y)
#define FP32_DECIMALS(fixed)    ((fixed) & 0xff)
#define FP32_MUL(x, y)          (((x) * (y) + FP32_2_TO_Ym1 ) >> FP32_Y)
#define FP32_DIV(x, y)          ((((x) << (FP32_Yp1))/(y)) / 2)
*/


int INT_TO_FP32(int myint);

int FP32_TO_INT(int fixed);

int FP32_DECIMALS(int fixed);

int FP32_DIV(int a , int b);

int FP32_MUL(int a, int b);



#endif // _UAV_FP32__H_


