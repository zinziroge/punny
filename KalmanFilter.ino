#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() {
  A = new Matrix(2, 2);
  At = new Matrix(2, 2);
  B = new RVector(2);
  C = new RVector(2);
  Ct = new CVector(2);
  x[0] = new RVector(2);
  x[1] = new RVector(2);
  x[2] = new RVector(2);
  P[0] = new Matrix(2, 2);
  P[1] = new Matrix(2, 2);
  P[2] = new Matrix(2, 2);
  sw = new Matrix(2, 2);
  Pm = new Matrix(2, 2);
  kg = new RVector(2);
  
  tmp_vec_1 = new RVector(2);
  tmp_vec_2 = new RVector(2);
  tmp_mat_1 = new Matrix(2, 2);
  tmp_mat_2 = new Matrix(2, 2);

  C->set_data(0, 1.0f);
  C->set_data(1, 0.0f);
  Ct->set_data(0, 1.0f);
  Ct->set_data(1, 0.0f);
  sz = 0.3;
  sw->set_data(0, 0, 0.001);
  sw->set_data(0, 1, 0.0);
  sw->set_data(1, 0, 0.0);
  sw->set_data(1, 1, 0.003);
  eye(P[0]);
  eye(P[1]);
  eye(P[2]);
  zero(x[0]);
  zero(x[1]);
  zero(x[2]);
}

KalmanFilter::~KalmanFilter() {
  delete A;
  delete B;
  delete C;
}

int KalmanFilter::init(const float dt) {
  A->set_data(0,0, 1);
  A->set_data(0,1, dt);
  A->set_data(1,0, 0);
  A->set_data(1,1, 1);
  At->set_data(0,0, 1);
  At->set_data(0,1, 0);
  At->set_data(1,0, dt);
  At->set_data(1,1, 1);

  B->set_data(0, dt);
  B->set_data(1, 0);

  return 0;
}

int KalmanFilter::exec(const float dt, const float omega[3], const float acc_angle[3], float kf_acc_angle[3]) {
  //Serial.println("start");
  init(dt);

  for(int i=0; i < 3; i+=1) {
    float inn;
    float s;
    unsigned long cur_t, pre_t;
    
#ifdef _DEBUG
    Serial.println("esti");
    pre_t = micros();
#endif
    estimation(i, omega);
#ifdef _DEBUG
    cur_t = micros();
    Serial.println(cur_t - pre_t);
#endif
    
#ifdef _DEBUG
    Serial.println("filtering");
    pre_t = micros();
#endif
    inn = filtering(i, acc_angle);
#ifdef _DEBUG
    cur_t = micros();
    Serial.println(cur_t - pre_t);
#endif

#ifdef _DEBUG
    Serial.println("conv");
    pre_t = micros();
#endif
    s = calc_convariance(i);
#ifdef _DEBUG
    cur_t = micros();
    Serial.println(cur_t - pre_t);
#endif

#ifdef _DEBUG
    Serial.println("kalman gain");
    pre_t = micros();
#endif
    calc_kalman_gain(i, s);
#ifdef _DEBUG
    cur_t = micros();
    Serial.println(cur_t - pre_t);
#endif
  
#ifdef _DEBUG
    Serial.println("update");
    pre_t = micros();
#endif
    update_state_vector(i, inn);
#ifdef _DEBUG
    cur_t = micros();
    Serial.println(cur_t - pre_t);
#endif
  }

  kf_acc_angle[0] = x[0]->get_data(0);
  kf_acc_angle[1] = x[1]->get_data(0);
  kf_acc_angle[2] = x[2]->get_data(0);
  
  return 0;
}

void KalmanFilter::estimation(const int i, const float omega[3]) {
  mul(A, x[i], tmp_vec_1);
  mul(B, omega[i], tmp_vec_2);
  add_(tmp_vec_1, tmp_vec_2, x[i]);

  mul(A, P[i], tmp_mat_1);
  mul(tmp_mat_1, At, Pm);

  //x[i]->print();
}

float KalmanFilter::filtering(const int i, const float acc_angle[3]) {
  float tmp = mul(Ct, x[i]);
  float inn = acc_angle[i] - tmp;
  return inn;
}

float KalmanFilter::calc_convariance(const int i) {
  float s;
  mul(P[i], C, tmp_vec_1);
  s = mul(Ct, tmp_vec_1) + sz;
  return s;
}

void KalmanFilter::calc_kalman_gain(const int i, const float s) {
  mul(Pm, C, kg);
  kg->set_data(0, kg->get_data(0)/s);
  kg->set_data(1, kg->get_data(1)/s);
}

void KalmanFilter::update_state_vector(const int i, const float inn) {
  mul(kg, inn, tmp_vec_1);
  add_(x[i], tmp_vec_1, x[i]);
  
  mul(kg, Ct, tmp_mat_1);
  mul(tmp_mat_1, Pm, tmp_mat_2);
  sub(Pm, tmp_mat_2, tmp_mat_2);
  add_(tmp_mat_2, sw, P[i]);
}
