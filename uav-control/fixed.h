int int2fix(int fixed);

int fix2int(int myint);

int fixdecimals(int fixed);

//define MULTX_Y(A,B) (Full-sizeint+1) (A.full * b.full + 2^(Y-1))>>Y
//define DIVX_Y(A,B) (Full-sizeint+1) (( (A.full << Y+1)/ b.full)  ////// + 1)/2

int fixmul(int x, int y);

int fixdiv(int x, int y);
