//#include <stdio.h>
//#define FIXED_Y 8

#define INT_2_FIX (int fixed) (fixed << 8)
#define FIX_2_INT (int myInt) (myInt >> 8)
#define FIX_DECIMALS (int fixed) (fixed & 0xff)
#define FIX_MULT (int x, int y) ((x * y + 128) >> 8)
#define FIX _DIV (int x, int y) ((x << 8)/y)

int int2fix(int fixed){
    return fixed << 8;
}

int fix2int(int myint){
    return myint >> 8;
}
int fixdecimals(int fixed){
    return fixed & 0xff;
}

//define MULTX_Y(A,B) (Full-sizeint+1) (A.full * b.full + 2^(Y-1))>>Y
//define DIVX_Y(A,B) (Full-sizeint+1) (( (A.full << Y+1)/ b.full)  ////// + 1)/2

int fixmul(int x, int y){
    return (x * y + 128) >> 8;   
}

int fixdiv(int x, int y){
    return ((x << 8 )/ y );// + 1) / 2;   
}


/*
int main(){
    int x = 10;
    int y = 5;
    double xd = (double)x;
    double yd = (double)y/2;
    
    int xf = int2fix(x);
    int yf = fixdiv(int2fix(y),int2fix(2));
    
    int result = 0;
    printf("int: %ld\n", sizeof(int));
    printf("double div: %f\n", xd / yd);
    printf("xf: %d\n", xf);
    printf("yf: %d\n", yf);
    printf("xf: %d\n", fix2int(xf));
    printf("yf: %d\n", fix2int(yf));
    printf("fix div: %d\n", fixdiv(xf,yf));
    printf("fix div: %d\n", fix2int(fixdiv(xf,yf)));
    
    


}
*/
