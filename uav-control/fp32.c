#include "fp32.h"

const int TWO_TO_31 = 2147483648;

int INT_TO_FP32(int myint){
    char sign = 0;
    int temp = 0;
    if(myint < 0){
        sign = 1;    
    }
    temp = myint << FP32_Y; 
    if(sign){
        temp = temp | 2147483648 ;   
    } 
    return temp;
}

int FP32_TO_INT(int fixed){
    char sign = 0;
    int temp = 0;
    if(fixed < 0){
        sign = 1;    
    }
    temp = fixed >> FP32_Y; 
    if(sign){
        temp = temp |2147483648 ;   
    } 
    return temp;
}

int FP32_DECIMALS(int fixed){
    return (fixed) & (FP32_2_TO_Y_MINUS_1) ;
}

int FP32_DIV(int a , int b){
    char sign = 0;
    int temp = 0;
    if( (a < 0 || b < 0) && !(a < 0 && b < 0) ){
        sign = 1;
    }
    temp = ((a << FP32_Yp1) / b) / 2;
    if(sign){
        temp = temp | 2147483648;
    }
    return temp;
}

int FP32_MUL(int a, int b){
    char sign = 0;
    int temp = 0;
    if( (a < 0 || b < 0) && !(a < 0 && b < 0) ){
        sign = 1;
    }
   
    temp = ((a * b) + FP32_2_TO_Ym1) >> FP32_Y;
    if(sign){
        temp = temp | 2147483648;
    }
    return temp;
}



