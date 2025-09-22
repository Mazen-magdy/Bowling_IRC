// ====================     INCLUDES     ====================
#include <Adafruit_AHRS.h>
#include <Adafruit_AHRS_FusionInterface.h>
#include <Adafruit_AHRS_Madgwick.h>
#include <Adafruit_AHRS_Mahony.h>
#include <Adafruit_AHRS_NXPFusion.h>
#include <Adafruit_Sensor_Set.h>
#include <Adafruit_Simple_AHRS.h>
#include <Adafruit_MPU6050.h>
// ====================     DEFINES      ====================

// Motor 1 Pins
#define E1_Pin1 3 // has interrupt
#define E1_Pin2 4 // don't have interrupt
#define M1_d1 8
#define M1_d2 12
#define M1_en 9

// Motor 2 Pins
#define E2_Pin1 2 // has interrupt
#define E2_Pin2 7 // don't have interrupt
#define M2_d1 10
#define M2_d2 11
#define M2_en 5

// ====================      CLASSES     ====================

class Encoder{
  private:
    long counter = 0;
    float countPerRevolution = 0;
    float circumference = 0;
    float countPerCm = 0;
    int E1 = 5; // encoder pin 1
    int E2 = 6; // encoder pin 2
  public:
    long counter1 = this -> counter;
    float target = 0;

    Encoder(long counts,int Cm, int e1,int e2 ){ // e1,e2 is the encoder pins 

      circumference = Cm;
      countPerRevolution = counts;
      countPerCm = countPerRevolution / circumference;
      E1 = e1;
      E2 = e2;
    }
    float Get_Moved_distance_from_launch(){
      return (float)counter / countPerCm;
    }
    float Get_Moved_Distance_From_Point(float distance){
      return this ->Get_Moved_distance_from_launch() - distance  ;
    }
    // float Get_Distance_From_Point(float distance){
    //   return distance - (float)counter /  turnsToCm ; 
    // }
    void set_target(float cm){
      target = cm + this -> Get_Moved_distance_from_launch();
    }
    float distance_from_target(){
      return target - this -> Get_Moved_distance_from_launch();
    }
    float get_Moved_Counts_From_Point(long point){
      return  this -> Get_Moved_distance_from_launch() - point;
    }
    void Counter(){
      if(digitalRead(E1) == LOW && digitalRead(E2) == HIGH)
      {
          counter++;
      }
      else if(digitalRead(E1) == HIGH && digitalRead(E2) == LOW)
      {
          counter++;
      }
      else{counter--;}
      counter1 = counter;
    } 
};

// ====================      INSTANCES     ====================
// --- encoder
Encoder E1(990,40.84,E1_Pin1,E1_Pin2); // For Motor 1 
Encoder E2(990,40.84,E2_Pin1,E2_Pin2); // For Motor 2 

// --- mpu 
Adafruit_MPU6050 mpu;
Adafruit_Madgwick filter;  // or Adafruit_Mahony filter

// ==================== GLOBAL VARIABLES ====================

bool user_Data = false;

// Motor 1
volatile bool     M1_timer_tiked   = false;
volatile uint32_t M1_last_millis   = 0;
volatile long     M1_encoder_read  = 0;

// Motor 2
volatile bool     M2_timer_tiked   = false;
volatile uint32_t M2_last_millis   = 0;
volatile long     M2_encoder_read  = 0;

// Sync + Control flags
bool  flag_A = false, flag_B = false;
long  async_two_motors = 0;
float Ksync = 0.2;


// ==================== PID STATE VARIABLES ====================

// --- Motor 1 PID ---
float M1_integral       = 0;
float M1_last_D_filtered= 0;
float M1_alpha          = 0.5;
int   M1_last_Error     = 0;
float M1_P, M1_I, M1_D;
float M1_startPoint;            // in cm
float M1_last_measure   = 0;
float M1_clearance      = 0.5;
bool  M1_target_set     = false;
// --- Motor 2 PID ---
float M2_integral       = 0;
float M2_last_D_filtered= 0;
float M2_alpha          = 0.5;
int   M2_last_Error     = 0;
float M2_P, M2_I, M2_D;
float M2_startPoint;            // in cm
float M2_last_measure   = 0;
float M2_clearance      = 0.5;
bool  M2_target_set     = false;

// ==================== INTERRUPT SERVICE ROUTINES ====================

// --- Timer Interrupts -----
ISR(TIMER1_COMPA_vect) {
  // Motor 1 feedback update
  M1_timer_tiked  = true;
  M1_encoder_read = E1.distance_from_target(); // distance between current and the target
} 

ISR(TIMER1_COMPB_vect) {
  // Motor 2 feedback update
  M2_timer_tiked  = true;
  M2_encoder_read = E2.distance_from_target();
}

// --- External interrupts -----
void E1_ISR() {
  E1.Counter();
}

void E2_ISR() {
  E2.Counter();
}
// ==================== YAW VARIABLES ====================
unsigned long lastUpdate  = 0;
// --- refrence point
float yaw0;  
bool yaw0_flag            = false;
float yaw;
float start_Yaw;
bool start_Yaw_flag       = false;
// --- needed variables for PID controlling ---
float last_Yaw;
float yaw_Target;
float yaw_last_error;
unsigned long last_Time; // in ms
float yaw_Integral;
float yaw_Clearence         = 5;
float yaw_P,yaw_I,yaw_D;
float yaw_last_D_filtered;
float yaw_Alpha             = 0.5;
// --- stop flag ---
bool yaw_Reached            = false;
// ==================== MPU FUNCTION ====================
//--- luxary
void get_Yaw_angle_10ms(){
  unsigned long now = millis();

  if (now - lastUpdate >= 10) {   // 10 ms = 100 Hz
    lastUpdate = now;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    filter.updateIMU(g.gyro.x, g.gyro.y, g.gyro.z,
                     a.acceleration.x, a.acceleration.y, a.acceleration.z);
    if(!yaw0_flag){
      yaw0 = filter.getYaw();
      yaw0_flag = true;
    }
    yaw =filter.getYaw() - yaw0;
    // normalize angle to -180..180
    if (yaw > 180) yaw -= 360;
    if (yaw < -180) yaw += 360;
    Serial.print("Yaw: "); Serial.println(filter.getYaw());
  }
}
// --- feedback function ---
float get_Yaw_angle(){

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  filter.updateIMU(g.gyro.x, g.gyro.y, g.gyro.z,
                    a.acceleration.x, a.acceleration.y, a.acceleration.z);
  if(!yaw0_flag){
    yaw0 = filter.getYaw();
    yaw0_flag = true;
  }
  yaw =filter.getYaw() - yaw0;
  // normalize angle to -180..180
  if (yaw > 180) yaw -= 360;
  if (yaw < -180) yaw += 360;
  Serial.print("Yaw: "); Serial.println(filter.getYaw());
  return yaw;
} // needs Delay for 10ms

// ==================== ROTATION PID CONTROLLER ====================

void yaw_PID_Controller(){
  if(yaw_Reached) return;

  if(millis() - last_Time == 10){

      yaw = get_Yaw_angle();
      float error = yaw_Target - yaw;

      if(error < yaw_Clearence){
        //stop
        yaw_Reached = true;
        return;
      }

      float dt = 0.01;

      yaw_Integral += (error + yaw_last_error) / 2 * dt;
      yaw_Integral  = constrain(yaw_Integral, -1000, 1000); 

      // Derivative (filtered)
     float D_filtered = yaw_Alpha * (yaw - last_Yaw) / dt +
                       (1 - yaw_Alpha) * yaw_last_D_filtered;

      float V = yaw_P * error + yaw_I * yaw_Integral - yaw_D * D_filtered;

       Motor1_Motion(V/2);
       Motor2_Motion(-1 * V/2);

       yaw_last_error = error;
       yaw_last_D_filtered = D_filtered;
       last_Yaw = yaw;
  }
}

// ==================== MOTION PID FUNCTIONS ====================

void PID_motion_M1() {
  if (M1_timer_tiked) {
    cli();
    float dt         = (millis() - M1_last_millis) / 1000.0;
    int encoder_read = M1_encoder_read;
    M1_timer_tiked   = false;
    sei();

    int e            = encoder_read;

    if (e <= M1_clearance) {
      Motor1_Motion(0); // stop motor
      flag_A = true;
      if (flag_B) stop_motors_PID_controller();
      return;
    }

    float measure = E1.Get_Moved_Distance_From_Point(M1_startPoint);

    // Integral
    M1_integral += (e + M1_last_Error) / 2 * dt;
    M1_integral  = constrain(M1_integral, -1000, 1000);

    // Derivative (filtered)
    float D_filtered = M1_alpha * (measure - M1_last_measure) / dt +
                       (1 - M1_alpha) * M1_last_D_filtered;

    // Sync term
    async_two_motors = E1.Get_Moved_Distance_From_Point(M1_startPoint) - E2.Get_Moved_Distance_From_Point(M2_startPoint);

    // PID output
    float V = M1_P * e + M1_I * M1_integral - M1_D * D_filtered
              - Ksync * async_two_motors;

    Motor1_Motion(V);

    // Store last values
    M1_last_millis     = millis();
    M1_last_Error      = e;
    M1_last_D_filtered = D_filtered;
    M1_last_measure    = measure;
  }
}

void PID_motion_M2() {
  if (M2_timer_tiked) {
    cli();
    float dt         = (millis() - M2_last_millis) / 1000.0;
    int encoder_read = M2_encoder_read;
    M2_timer_tiked   = false;
    sei();

    int e = encoder_read;

    if (e <= M2_clearance) {
      Motor1_Motion(0); // stop motor
      flag_B = true;
      if (flag_A) stop_motors_PID_controller();
      return;
    }

    float measure = E2.Get_Moved_Distance_From_Point(M2_startPoint);

    // Integral
    M2_integral += (e + M2_last_Error) / 2 * dt;
    M2_integral  = constrain(M2_integral, -1000, 1000);

    // Derivative (filtered)
    float D_filtered = M2_alpha * (measure - M2_last_measure) / dt +
                       (1 - M2_alpha) * M2_last_D_filtered;

    // Sync term
    async_two_motors = E1.Get_Moved_Distance_From_Point(M1_startPoint) - E2.Get_Moved_Distance_From_Point(M2_startPoint);

    // PID output
    float V = M2_P * e + M2_I * M2_integral - M2_D * D_filtered
              + Ksync * async_two_motors;

    Motor1_Motion(V);

    // Store last values
    M2_last_millis     = millis();
    M2_last_Error      = e;
    M2_last_D_filtered = D_filtered;
    M2_last_measure    = measure;
  }
}


// ==================== CONTROLLER CONTROL ====================

void run_motors_PID_controller() {
  cli();
  TCCR1A = (1 << WGM11);
  TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS10) | (1 << CS11);
  ICR1   = 4999;
  OCR1A  = 2499;
  OCR1B  = 4999;
  TIMSK1 |= (1 << OCIE1A) | (1 << OCIE1B); // enable interrupt
  sei();
}

void stop_motors_PID_controller() {
  TCCR1B = 0;   // stop Timer1
  TIMSK1 &= ~((1 << OCIE1A) | (1 << OCIE1B));
}

// ==================== MOTOR CONTROL ====================
void Motor1_Motion(float speed){
  bool dir = (speed >= 0) ? 1 : 0;
  speed = map(speed,0,12,0,255);
  switch(dir){
      case 0: // CCW
        digitalWrite(M1_d1,HIGH);
        digitalWrite(M1_d2,LOW);
        analogWrite (M1_en,speed);
        break;
      case 1: // CW 
        digitalWrite(M1_d1, LOW);
        digitalWrite(M1_d2,HIGH);
        analogWrite (M1_en,speed);
        break;
      default :
        return;
  }
}

void Motor2_Motion(float speed){
  bool dir = (speed >= 0) ? 1 : 0;
  speed = map(speed,0,12,0,255);
  switch(dir){
      case 0: // CCW
        digitalWrite(M1_d1,HIGH);
        digitalWrite(M1_d2,LOW);
        analogWrite (M1_en,speed);
        break;
      case 1: // CW 
        digitalWrite(M1_d1, LOW);
        digitalWrite(M1_d2,HIGH);
        analogWrite (M1_en,speed);
        break;
      default :
        return;
  }
}

// ==================== ARDUINO MAIN ====================

void setup() {
  // setup interrupt on Timer1 with OCR1A , OCR1B
  Serial.begin(9600);
  Serial.println("started");

  pinMode(E1_Pin1, INPUT);
  pinMode(E1_Pin2, INPUT);
  pinMode(E2_Pin1, INPUT);
  pinMode(E2_Pin2, INPUT);
  
  pinMode(M1_d1, OUTPUT);
  pinMode(M1_d2, OUTPUT);
  pinMode(M1_en, OUTPUT);
  pinMode(M2_d1, OUTPUT);
  pinMode(M2_d2, OUTPUT);
  pinMode(M2_en, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(E1_Pin1), E1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(E2_Pin1), E2_ISR, CHANGE);
}

void loop() {
  // ================= MOTION PID CONTROL TESTING =================
    // get data from user
    if(!user_Data){
      Serial.println("Enter P I D for M1");
      while(!Serial.available());
      M1_P = Serial.read();
      Serial.print("M1_P "); Serial.println(M1_P);
      while(!Serial.available());
      M1_I = Serial.read();
      Serial.print("M1_I "); Serial.println(M1_I);
      while(!Serial.available());
      M1_D = Serial.read();
      Serial.print("M1_D "); Serial.println(M1_D);

      Serial.println("Enter P I D for M2");
      while(!Serial.available());
      M2_P = Serial.read();
      Serial.print("M2_P "); Serial.println(M2_P);
      while(!Serial.available());
      M2_I = Serial.read();
      Serial.print("M2_I "); Serial.println(M2_I);
      while(!Serial.available());
      M2_D = Serial.read();
      Serial.print("M2_D "); Serial.println(M2_D);
    }

    // ---- PID_Intialization ----
      if(!M1_target_set){
        M1_startPoint = E1.Get_Moved_distance_from_launch();
        E1.set_target(122.52);
        M1_target_set = true;
      }
      if(!M2_target_set){
        M2_startPoint = E2.Get_Moved_distance_from_launch();
        E2.set_target(122.52);
        M2_target_set = true;
      }
    //
    run_motors_PID_controller(); 

    PID_motion_M1();
    PID_motion_M2();
  //
  // // ================= ROTATION PID CONTROL TESTING =================
  //   // get data from user
  //   if(!user_Data){
  //     Serial.println("Please enter target angle : ");
  //     while(!Serial.available());
  //     yaw_Target = Serial.read();
  //     Serial.print("traget angle : "); Serial.println(yaw_Target);
  //   }
  //   yaw_PID_Controller();
  // //


  user_Data = true; // prevent input in the next all loop just first time it asks
}
