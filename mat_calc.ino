#include "mat_calc.h"

// RVecor
void add_(RVector* rvec_1, RVector* rvec_2, RVector* out_rvec) {
  for(int r=0; r < out_rvec->get_row(); r+=1) {
    out_rvec->set_data(r, rvec_1->get_data(r) + rvec_2->get_data(r));
  }
}

void sub(RVector* rvec_1, RVector* rvec_2, RVector* out_rvec) {
  for(int r=0; r < out_rvec->get_row(); r+=1) {
    out_rvec->set_data(r, rvec_1->get_data(r) - rvec_2->get_data(r));
  }
}

void mul(RVector* rvec_1, float s, RVector* out_rvec) {
  for(int r=0; r < out_rvec->get_row(); r+=1) {
    out_rvec->set_data(r, s*rvec_1->get_data(r));
  }
}

// [r x 1] * [1 x c] = [r x c]
void mul(RVector* rvec, CVector* cvec, Matrix* out_mat) {
  for(int r=0; r < rvec->get_row(); r+=1) {
    for(int c=0; c < cvec->get_col(); c+=1) {
      int n = cvec->get_col() * r + c;
      out_mat->set_data(n, rvec->get_data(r) * cvec->get_data(c));
    }
  }
}

void zero(RVector* rvec) {
  for(int r=0; r < rvec->get_row(); r+=1) {
    rvec->set_data(r, 0);
  }
}

// CVector
void add_(CVector* cvec_1, CVector* cvec_2, CVector* out_cvec) {
  for(int c=0; c < out_cvec->get_col(); c+=1) {
    out_cvec->set_data(c, cvec_1->get_data(c) + cvec_2->get_data(c));
  }
}

void sub(CVector* cvec_1, CVector* cvec_2, CVector* out_cvec) {
  for(int c=0; c < out_cvec->get_col(); c+=1) {
    out_cvec->set_data(c, cvec_1->get_data(c) - cvec_2->get_data(c));
  }
}

void mul(CVector* cvec_1, float s, CVector* out_cvec) {
  for(int c=0; c < out_cvec->get_col(); c+=1) {
    out_cvec->set_data(c, s*cvec_1->get_data(c));
  }
}

float mul(CVector* cvec, RVector* rvec) { 
  float out_val = 0.0f;
  
  for(int c=0; c < cvec->get_col(); c+=1) {
    out_val += cvec->get_data(c) * rvec->get_data(c);
  }

  return out_val;
}

// [1 x c] * [c x r] =[1 x r]
void mul(CVector* cvec, Matrix* mat, CVector* out_cvec) {
  for(int c=0; c < out_cvec->get_col(); c+=1) {
    float tmp = 0.0f;
    for(int i=0; i < mat->get_row(); i+=1) {
      int n = mat->get_n(i, c);
      tmp += cvec->get_data(i) * mat->get_data(n);
    }
    out_cvec->set_data(c, tmp);
  }
}

void zero(CVector* cvec) {
  for(int c=0; c < cvec->get_col(); c+=1) {
    cvec->set_data(c, 0);
  }
}

// Matrix
void add_(Matrix* mat_1, Matrix* mat_2, Matrix* out_mat) {
  for(int r=0; r < out_mat->get_row(); r+=1) {
    for(int c=0; c < out_mat->get_col(); c+=1) {
      int n = out_mat->get_n(r, c);
      out_mat->set_data(n, mat_1->get_data(n) + mat_2->get_data(n));
    }
  }  
}

void sub(Matrix* mat_1, Matrix* mat_2, Matrix* out_mat) {
  for(int r=0; r < out_mat->get_row(); r+=1) {
    for(int c=0; c < out_mat->get_col(); c+=1) {
      int n = out_mat->get_n(r, c);
      out_mat->set_data(n, mat_1->get_data(n) - mat_2->get_data(n));
    }
  }   
}

// [r x c] * [c x m] = [r x m]
void mul(Matrix* mat_1, Matrix* mat_2, Matrix* out_mat) {
  for(int r=0; r < out_mat->get_row(); r+=1) {
    for(int c=0; c < out_mat->get_col(); c+=1) {
      float tmp = 0.0f;
      for(int i=0; i < out_mat->get_row(); i+=1) {
         tmp += mat_1->get_data(r, i) * mat_2->get_data(i, c);
      }
      out_mat->set_data(r, c, tmp);
    }
  }    
}

// [r x c] * [c x 1] = [r x 1]
void mul(Matrix* mat, RVector* rvec, RVector* out_rvec) {
  for(int r=0; r < out_rvec->get_row(); r+=1) {
    out_rvec->set_data(r, 0.0f);
    for(int c=0; c < mat->get_col(); c+=1) {
      int n = mat->get_n(r, c);
      out_rvec->set_data(r, out_rvec->get_data(r) + mat->get_data(n) * rvec->get_data(c));
    }
  }  
}

// scaler mul
void mul(Matrix* mat, const float g, Matrix* out_mat) {
  for(int r=0; r < out_mat->get_row(); r+=1) {
    for(int c=0; c < out_mat->get_col(); c+=1) {
      int n = out_mat->get_n(r, c);
      out_mat->set_data(n, g * mat->get_data(n));
    }
  }
}

void eye(Matrix* mat) {
  for(int r=0; r < mat->get_row(); r+=1) {
    for(int c=0; c < mat->get_col(); c+=1) {
      int n = mat->get_n(r, c);
      if( r==c ) 
        mat->set_data(n, 1);
      else
        mat->set_data(n, 0);
    }
  }
}

