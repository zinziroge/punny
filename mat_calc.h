#ifndef _MAT_CALC_H_
#define _MAT_CALC_H_

#include "arduino.h"
#include "RVector.h"
#include "CVector.h"
#include "Matrix.h"

// RVecor
void add_(RVector* rvec_1, RVector* rvec_2, RVector* out_rvec);
void sub(RVector* rvec_1, RVector* rvec_2, RVector* out_rvec);
void mul(RVector* rvec_1, float s, RVector* out_rvec);
// [r x 1] * [1 x c] = [r x c]
void mul(RVector* rvec, CVector* cvec, Matrix* out_mat);

// CVector
void add_(CVector* cvec_1, CVector* cvec_2, CVector* out_cvec);
void sub(CVector* cvec_1, CVector* cvec_2, CVector* out_cvec);
void mul(CVector* cvec_1, float s, CVector* out_cvec);
float mul(CVector* cvec, RVector* rvec);
// [1 x c] * [c x r] =[1 x r]
void mul(CVector* cvec, Matrix* mat, CVector* out_cvec);


// Matrix
void add_(Matrix* mat_1, Matrix* mat_2, Matrix* out_mat);
void sub(Matrix* mat_1, Matrix* mat_2, Matrix* out_mat);
// [r x c] * [c x m] = [r x m]
void mul(Matrix* mat_1, Matrix* mat_2, Matrix* out_mat);
// [r x c] * [c x 1] = [r x 1]
void mul(Matrix* mat, RVector* rvec, RVector* out_rvec);
// scaler mul
void mul(Matrix* mat, const float g, Matrix* out_mat);

void eye(Matrix* mat);

#endif _MAT_CALC_H_
