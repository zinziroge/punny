#ifndef _MATRIX_H_
#define _MATRIX_H_

#include <SoftwareSerial.h>
//#include "arduino.h"
//#include "RVector.h"
//#include "CVector.h"

// Matrix
class Matrix {
  private:
    float* data;
    int row, col;

  public:
    Matrix();
    Matrix(const int r, const int c);
    ~Matrix();
    int get_row(void) {
      return row;
    }
    int get_col(void) {
      return col;
    }
    int get_n(const int t_r, const int t_c) const {
      return col * t_r + t_c;
    };
    void set_data(const int r, const int c, float val) const {
      data[get_n(r, c)] = val;
    }
    void set_data(const int n, float val) {
      data[n] = val;
    }
    float get_data(const int r, const int c) {
      return data[get_n(r, c)];
    }
    float get_data(const int n) {
      return data[n];
    }

    void add(Matrix* mat, Matrix* out_mat);
    void sub(Matrix* mat, Matrix* out_mat);
    void mul_mat(Matrix* mat, Matrix* out_mat);
    //void mul_rvec(RVector* rvec, RVector* out_rvec);
    void mul_scalar(const float g, Matrix* out_mat);

    void print(void);
};

#endif // _MATRIX_H_

