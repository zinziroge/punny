#include "RVector.h"
#include "CVector.h"
#include "Matrix.h"

// CVector
CVector::CVector() {
   col = 2;
   data = (float*)malloc(col * sizeof(float));
}

CVector::CVector(const int t_c) {
  col = t_c;
  data = (float*)malloc(col * sizeof(float));
}

CVector::~CVector() {
  free(data);
}

/*void CVector::add( CVector* in_cvec, CVector* out_cvec) {
  for(int c=0; c < out_cvec->get_col(); c+=1) {
    out_cvec->set_data(c, this->data[c] + in_cvec->get_data(c));
  }
}

void CVector::sub( CVector* in_cvec, CVector* out_cvec) {
  for(int c=0; c < out_cvec->get_col(); c+=1) {
    out_cvec->set_data(c, this->data[c] - in_cvec->get_data(c));
  }
}*/

// [1 x c] * [c x 1] = float
/*
float CVector::mul_rvec(RVector rvec) { 
  float out_val = 0.0f;
  
  for(int c=0; c < col; c+=1) {
    out_val += data[c] * rvec.get_data(c);
  }

  return out_val;
}
*/

// [1 x c] * [c x r] =[1 x r]
/*void CVector::mul_mat(Matrix* mat, CVector* out_cvec) {
  for(int r=0; r < out_cvec->get_col(); r+=1) {
    float tmp = 0.0f;
    for(int c=0; c < this->col; c+=1) {
      int n = mat->get_n(r, c);
      tmp += this->data[c] * mat->get_data(n);
    }
    out_cvec->set_data(r, tmp);
  }
}*/

// scalar mul
/*void CVector::mul_scalar(const float g, CVector* out_cvec) {
  for(int c=0; c < out_cvec->get_col(); c+=1) {
    out_cvec->set_data(c, g * this->data[c]);
  }
}*/

void CVector::print(void) {
  Serial.print("|");
  for(int c=0; c < col; c+=1) {
    Serial.print(data[c]);
    Serial.print(" ");
  }
  Serial.println("|");
  Serial.println("");
}

