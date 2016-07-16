#ifndef _KALMAN_FILTEER_H_
#define _KALMAN_FILTEER_H_

#include "mat_calc.h"

class KalmanFilter {
  private:
    Matrix* A;
    Matrix* At;
    RVector* B;
    RVector* C;
    CVector* Ct;
    RVector* x[3];
    Matrix* P[3];
    float sz;
    Matrix* sw;
    Matrix* Pm;
    RVector* kg;
    RVector* tmp_vec_1;
    RVector* tmp_vec_2;
    Matrix* tmp_mat_1;
    Matrix* tmp_mat_2;
    
    int init(const float dt);
    void estimation(const int i, const float omega[3]);
    float filtering(const int i, const float acc_angle[3]);
    float calc_convariance(const int i);
    void calc_kalman_gain(const int i, const float s);
    void update_state_vector(const int i, const float inn);
    
  public:
    KalmanFilter();
    ~KalmanFilter();

    int exec(const float dt, const float omega[3], const float acc_angle[3],  float kf_acc_angle[3]);
};

#endif // _KALMAN_FILTEER_H_

