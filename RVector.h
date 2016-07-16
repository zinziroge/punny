#ifndef _RVECTOR_H_
#define _RVECTOR_H_

//#include "arduino.h"
//#include "CVector.h"
//#include "Matrix.h"
#include <SoftwareSerial.h>

// Row Vector
class RVector {
  private:
    float* data;
    int row;
    
  public:
    RVector();
    RVector(const int row);
    ~RVector();
    int get_row(void) { return row; }
    void set_data(const int r, float val) { data[r] = val;}
    float get_data(const int r) { return data[r];}
    void print(void);
    
    //void add(RVector* rvec, RVector* out_rvec);
    //void sub(RVector* rvec, RVector* out_rvec);
    //void mul_cvec(CVector* cvec, Matrix* out_mat); // [r x 1] * [1 x c] = [r x c]
    //void mul_scalar(const float g, RVector* out_rvec);
};

#endif // _RVECTOR_H_

