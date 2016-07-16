//#include "RVector.h"
//#include "CVector.h"
//#include "Matrix.h"

Matrix::Matrix() {
  row = 2;
  col = 2;
  data = (float*)malloc(row * col * sizeof(float));
}

Matrix::Matrix(const int t_r, const int t_c) {
  row = t_r;
  col = t_c;
  data = (float*)malloc(row * col * sizeof(float));
}

Matrix::~Matrix() {
  free(data);
}

/*void Matrix::add(Matrix* mat, Matrix* out_mat) {
  for(int r=0; r < row; r+=1) {
    for(int c=0; c < col; c+=1) {
      int n = get_n(r, c);
      out_mat->set_data(n, this->data[n] + mat->get_data(n));
    }
  }  
}

void Matrix::sub(Matrix* mat, Matrix* out_mat) {
  for(int r=0; r < row; r+=1) {
    for(int c=0; c < col; c+=1) {
      int n = get_n(r, c);
      out_mat->set_data(n, this->data[n] - mat->get_data(n));
    }
  }  
}*/

// [r x c] * [c x m] = [r x m]
void Matrix::mul_mat(Matrix* mat, Matrix* out_mat) {
  for(int r=0; r < row; r+=1) {
    float tmp = 0.0f;
    for(int c=0; c < col; c+=1) {
      tmp += this->data[get_n(r,c)] * mat->get_data(mat->get_n(c, r));
    }
    int n = get_n(out_mat->get_row(), out_mat->get_col());
    out_mat->set_data(n, tmp);
  }    
}

// [r x c] * [c x 1] = [r x 1]
/*
void Matrix::mul_rvec(RVector* rvec, RVector* out_rvec) {
  for(int r=0; r < row; r+=1) {
    out_rvec->set_data(r, 0.0f);
    for(int c=0; c < c; c+=1) {
      n = get_n(r, c);
      out_rvec->set_data(r, out_rvec->get_data(r) + data[n] * rvec->get_data(c));
    }
  }  
}
*/

// scaler mul
void Matrix::mul_scalar(const float g, Matrix* out_mat) {
  for(int r=0; r < row; r+=1) {
    for(int c=0; c < col; c+=1) {
      int n = get_n(r, c);
      out_mat->set_data(n, g * data[n]);
    }
  }
}

void Matrix::print(void) {
  for(int r=0; r < row; r+=1) {
    Serial.print("|");
    for(int c=0; c < col; c+=1) {
      int n = get_n(r, c);
      Serial.print(data[n]); Serial.print(" ");
    }
    Serial.println("|");
  }
  Serial.println("");
}
