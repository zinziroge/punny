//#define _DEBUG

#include <SoftwareSerial.h>
#include <MsTimer2.h>
#include "RVector.h"
#include "CVector.h"
#include "Matrix.h"
#include "mat_calc.h"
#include "KalmanFilter.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

//#define OUTPUT_READABLE_ACCELGYRO
//#define OUTPUT_KF_ANGLE
#define OUTPUT_KF_ANGLE_BLE

int16_t ax, ay, az;
int16_t gx, gy, gz;


#define PS_PIN_0 0
#define PS_PIN_1 1
#define PS_PIN_2 2

#define PITCH 0
#define YAW   1
#define ROLL  2

#define GYRO_LSB  131.0
#define ACC_LSB   16384.0

SoftwareSerial ble_ser(8, 9); // rxPin, txPin
unsigned long t;

KalmanFilter kf;
float acc_angle[3];
float kf_angle[3];
float omega[3];
float integ_angle[3];
float dt;
unsigned long pre_t;

void acc2euler(
    const int ax, const int ay, const int az, 
    float* euler_p, float* euler_y, float* euler_r ) {
    //*euler_p = atan2(ax, ay)/ACC_LSB;
    //*euler_y = atan2(ay, ay)/ACC_LSB;
    //*euler_r = atan2(ax, az)/ACC_LSB;
    *euler_p = atan2(ax, az) * 180 / 3.141592;
    *euler_y = atan2(ay, az) * 180 / 3.141592;
    *euler_r = atan2(ax, ay) * 180 / 3.141592;
}

void sensor() {
    unsigned long cur_t = micros();
    
    // read raw accel/gyro measurements from device
#ifdef _DEBUG
    Serial.println("getMotion6");
    pre_t = micros();
#endif
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
#ifdef _DEBUG
    cur_t = micros();
    Serial.println(cur_t - pre_t);
#endif
    
    dt = (cur_t - pre_t) / 1000000.0f;
    
    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        Serial.print("a/g:\t");
        Serial.print(cur_t); Serial.print("\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.println(gz);
    #endif

    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif

    
    omega[PITCH] = gx / GYRO_LSB;
    omega[YAW] = gz / GYRO_LSB;
    omega[ROLL] = gy / GYRO_LSB;
    integ_angle[PITCH] += dt * omega[PITCH];
    integ_angle[YAW] += dt * omega[YAW];
    integ_angle[ROLL] += dt * omega[ROLL];
    
    #ifdef _DEBUG
    Serial.println("acc2euler");
    pre_t = micros();
    #endif
    
    acc2euler(ax, ay, az, &acc_angle[PITCH], &acc_angle[YAW], &acc_angle[ROLL]);
    #ifdef _DEBUG
    cur_t = micros();
    Serial.println(cur_t - pre_t);
    #endif
    kf.exec(dt, omega, acc_angle, kf_angle);

    #ifdef OUTPUT_KF_ANGLE
        Serial.print("a/g:\t");
        Serial.print(cur_t); Serial.print("\t");
        //Serial.print(acc_angle[PITCH]*1000); Serial.print("\t");
        //Serial.print(acc_angle[YAW]*1000); Serial.print("\t");
        //Serial.print(acc_angle[ROLL]*1000); Serial.print("\t");
        Serial.print(integ_angle[PITCH]*1000); Serial.print("\t");
        Serial.print(integ_angle[YAW]*1000); Serial.print("\t");
        Serial.print(integ_angle[ROLL]*1000); Serial.print("\t");
        Serial.print(kf_angle[PITCH]*1000); Serial.print("\t");
        Serial.print(kf_angle[YAW]*1000); Serial.print("\t");
        Serial.println(kf_angle[ROLL]*1000);
    #endif

    #ifdef OUTPUT_KF_ANGLE_BLE
        /*
        ble_ser.print(dt); ble_ser.print("\t");
        ble_ser.print(acc_angle[PITCH]*1000.0); ble_ser.print("\t");
        ble_ser.print(kf_angle[PITCH]*1000.0); ble_ser.print("\t");
        ble_ser.println("");
        */
        ble_ser.print(acc_angle[PITCH]);
        ble_ser.print("\t");
        ble_ser.print(acc_angle[YAW]);
        ble_ser.println("");
        Serial.print(acc_angle[PITCH]);
        Serial.print("\t");
        Serial.print(acc_angle[YAW]);
        Serial.println("");
    #endif
    
    pre_t = cur_t;
}

Matrix* a_mat;
Matrix* b_mat;
Matrix* c_mat;
RVector* rvec;
RVector* rvec2;
CVector* cvec;
CVector* cvec2;

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
    
  Serial.begin(9600);

  pinMode(8, INPUT);
  pinMode(9, OUTPUT);
  ble_ser.begin(9600);
  //ble_ser.listen();

  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  //MsTimer2::set(100, sensor);
  //MsTimer2::start();

  a_mat = new Matrix(2, 2);
  b_mat = new Matrix(2, 2);
  c_mat = new Matrix(2, 2);

  rvec = new RVector(2);
  rvec2 = new RVector(2);
  cvec = new CVector(2);
  cvec2 = new CVector(2);
}


  
void test() {
  a_mat->set_data(0,0, 1);
  a_mat->set_data(0,1, 2);
  a_mat->set_data(1,0, 3);
  a_mat->set_data(1,1, 4);
  b_mat->set_data(0,0, 5);
  b_mat->set_data(0,1, 6);
  b_mat->set_data(1,0, 7);
  b_mat->set_data(1,1, 8);
  c_mat->set_data(0,0, 0);
  c_mat->set_data(0,1, 0);
  c_mat->set_data(1,0, 0);
  c_mat->set_data(1,1, 0);
  rvec->set_data(0,9);
  rvec->set_data(1,10);
  cvec->set_data(0,11);
  cvec->set_data(1,12);

  // mat * mat = mat
  mul(a_mat, b_mat, c_mat);
  c_mat->print();

  // rvec * cvec = mat
  mul(rvec, cvec, c_mat);
  c_mat->print();

  // mat * rvec = rvec
  mul(a_mat, rvec, rvec2);
  rvec2->print();

  // rvec * cvec = c_mat
  mul(rvec, cvec, c_mat);
  c_mat->print();

  // cvec * a_mat = cvec
  mul(cvec, a_mat, cvec2);
  cvec2->print();

  // cvec * rvec = scaler
  Serial.println(mul(cvec, rvec));
}

void loop() {
//    unsigned long cur_t = micros();
//    Serial.println("sensor");
//    pre_t = micros();
    sensor();
//    cur_t = micros();
//    Serial.println(cur_t - pre_t);
//  test();
//  delay(1000);
}

/*
void loop() {
  // put your main code here, to run repeatedly:
  int pressure_sensor_0 = analogRead(PS_PIN_0);
  int pressure_sensor_1 = analogRead(PS_PIN_1);
  int pressure_sensor_2 = analogRead(PS_PIN_2);
  int ble_ser_val;

  t = micros();
  
  ble_ser.print(t);
  ble_ser.print(" ");
  ble_ser.print(pressure_sensor_1);
  //ble_ser.print(" ");
  //if( ble_ser.available() ) {
  //    ble_ser_val = ble_ser.read();
  //    //Serial.print(ble_ser_val);
  //}
  ble_ser.print("\r\n");
  Serial.print(pressure_sensor_0);
  Serial.print(" ");
  Serial.print(pressure_sensor_1);
  Serial.print(" ");
  Serial.print(pressure_sensor_2);
  Serial.print("\n");

  delay(100);
}
*/

