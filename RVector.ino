#include "RVector.h"
#include "CVector.h"
#include "Matrix.h"

// RVector
RVector::RVector() {
  row = 2;
  data = (float*)malloc(row * sizeof(float));
}

RVector::RVector(const int t_r) {
  row = t_r;
  data = (float*)malloc(row * sizeof(float));
}

RVector::~RVector() {
  free(data);
}

/*void RVector::add(RVector* in_rvec, RVector* out_rvec) {
  for(int r=0; r < out_rvec->get_row(); r+=1) {
    out_rvec->set_data(r, this->data[r] + in_rvec->get_data(r));
  }
}

void RVector::sub(RVector* in_rvec, RVector* out_rvec) {
  for(int r=0; r < out_rvec->get_row(); r+=1) {
    out_rvec->set_data(r, this->data[r] - in_rvec->get_data(r));
  }
}*/

// [r x 1] * [1 x c] = [r x c]
/*void RVector::mul_cvec(CVector* cvec, Matrix* out_mat) {
  for(int r=0; r < row; r+=1) {
    for(int c=0; c < cvec->get_col(); c+=1) {
      int n = cvec->get_col() * r + c;
      out_mat->set_data(n, this->data[r] * cvec->get_data(c));
    }
  }
}*/

// scalar mul
/*void RVector::mul_scalar(const float g, RVector* out_rvec) {
  for(int i=0; i < out_rvec->get_row(); i+=1) {
    out_rvec->set_data(i, g * this->data[i]);
  }
}*/

void RVector::print(void) {
  for(int r=0; r < row; r+=1) {
    Serial.print("|"); Serial.print(data[r]); Serial.println("|");
  }
  Serial.println("");
}
