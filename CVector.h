#ifndef _CVECTOR_H_
#define _CVECTOR_H_

//#include "arduino.h"
//#include "RVector.h"
//#include "Matrix.h"
#include <SoftwareSerial.h>

// Column Vector
class CVector {
  private:
    float* data;
    int col;
    
  public:
    CVector();
    CVector(const int c);
    ~CVector();
    int get_col(void) { return col; }
    void set_data(const int c, float val) { data[c] = val;}
    float get_data(const int c) { return data[c];}
    void print(void);
    
    //void add(CVector* cvec, CVector* out_cvec);
    //void sub(CVector* cvec, CVector* out_cvec);
    //float mul_rvec(RVector* rvec); // [1 x c] * [c x 1] = float
    //void mul_mat(Matrix* mat, CVector* out_cvec); // [1 x c] * [c x r] =[1 x r]
    //void mul_scalar(const float g, CVector* out_cvec);
};

#endif // _CVECTOR_H_

