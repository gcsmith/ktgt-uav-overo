#ifndef _UAV_FIXED_POINT__H_
#define _UAV_FIXED_POINT__H_

#define INT_2_FIX(fixed) ((fixed) << 8)
#define FIX_2_INT(myInt) ((myInt) >> 8)
#define FIX_DECIMALS(fixed) ((fixed) & 0xff)
#define FIX_MULT(x, y) (((x) * (y) + 128) >> 8)
#define FIX_DIV(x, y) (((x) << 8)/(y))

int int2fix(int fixed);

int fix2int(int myint);

int fixdecimals(int fixed);

//define MULTX_Y(A,B) (Full-sizeint+1) (A.full * b.full + 2^(Y-1))>>Y
//define DIVX_Y(A,B) (Full-sizeint+1) (( (A.full << Y+1)/ b.full)  ////// + 1)/2

int fixmul(int x, int y);

int fixdiv(int x, int y);

#endif

