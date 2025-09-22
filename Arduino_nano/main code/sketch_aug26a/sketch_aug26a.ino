
#include <math.h>
// -------------- Global Variables ----------------//

// ----------------------defines----------------------//
#define E1_Pin1 3 // has interrupt
#define E1_Pin2 4 // don't have interrupt
#define M1_d1 8
#define M1_d2 12
#define M1_en 9

#define E2_Pin1 2 // has interrupt
#define E2_Pin2 7 // don't have interrupt
#define M2_d1 10
#define M2_d2 11
#define M2_en 5
// motion PID


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
    float get_Moved_counts_From_Point(long point){
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
Encoder E1(990,40.84,E1_Pin1,E1_Pin2);
Encoder E2(990,40.84,E2_Pin1,E2_Pin2);

void PID_controller(float target1,int direction1,float target2,int direction2);
// ---------------------- interrupt functions -----------------//
  void E1_ISR() {
    E1.Counter();
  }

  void E2_ISR() {
    E2.Counter();
  }

void setup() {
  // put your setup code here, to run once:
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
  // put your main code here, to run repeatedly:
  float distance = 0;
  Serial.println("enter Distance");
  while(!Serial.available());
  distance = Serial.parseFloat();
  int direction = 0;
  Serial.println("enter direction");
  while(!Serial.available());
  direction = Serial.parseInt();
  PID_controller(distance,direction,distance,direction);

  Serial.println(E1.target);
  Serial.println(E1.Get_Moved_distance_from_launch());
}

// function definitions

// void PID_controller(float target1,int direction1,float target2,int direction2){  // directions is 0 : CCW , 1 : CW
//   unsigned long init_time = millis();
  
//   E1.set_target(target1);
//   E2.set_target(target2);

//   float clearance = 0.5; // 0.5 cm
//   // can record that point we started to use PID which can be used later
  
//   float P = 0,I = 0,D = 0;
//   float diffrential1 = 0;
//   float integral1 = 0;

//   float diffrential2 = 0;
//   float integral2 = 0;
  
//   int dt = 0 ;
//   float last_error1 = target1;
//   float last_error2 = target2;
//   unsigned long last_time = init_time;
  
//   float voltage1 = 0;
//   float voltage2 = 0;
 
//   P =5;
//   I = 0.00000001;
//   D = 1;
//   // Serial.println("please enter P , I ,D : ");
//     Serial.println("Enter P:");
//   while (!Serial.available());  
//   P = Serial.parseFloat();

//   Serial.println("Enter I:");
//   while (!Serial.available());  
//   I = Serial.parseFloat();

//   Serial.println("Enter D:");
//   while (!Serial.available());  
//   D = Serial.parseFloat();

//   Serial.print("Using PID: P="); Serial.print(P);
//   Serial.print(" I="); Serial.print(I);
//   Serial.print(" D="); Serial.println(D);


//   while(true){
//     float e1 = E1.distance_from_target();
//     // float e2 = E2.distance_from_target();

//     unsigned long current_time = millis();
//     dt = (current_time - last_time) / 1000.0;
//     if (dt <= 0) dt = 0.001; // safety against divide by zero

//     integral1 += (float)(e1 + last_error1) / 2 * dt;
//     integral1 = constrain(integral1,-1000,1000);

//     diffrential1 = ((float)(e1- last_error1)) / dt;

//     voltage1 = P * e1 + I * integral1 + D * diffrential1;
   
//     Serial.print("Error: "); Serial.println(e1);
//     Serial.print("Integral: "); Serial.println(integral1);
//     Serial.print("diffrential1: "); Serial.println(diffrential1);
//     Serial.print("Voltage: "); Serial.println(voltage1);

//     // --- Send to motor ---
//     motor1_motion(abs(voltage1), (voltage1 >= 0) ? direction1 : !direction1);

    
//     // integral2 += (float)(e2 + last_error2) / 2 * dt;
//     // diffrential2 = ((float)(e2- last_error2)) / dt;
//     // voltage2 = P * e2 + I * integral2 + D * diffrential2;
//     // if(voltage2 < 0){
//     //   if(direction2 == 0)
//     //     motor2_motion(voltage2 * -1,1);
//     //   else
//     //     motor2_motion(voltage2 * -1,0);

//     // }
//     // else
//     //   motor2_motion(voltage2,direction2);

//     last_error1 = e1;
//     // last_error2 = e2;
//     last_time = current_time;
//     // delay(100);

//     // --- Stop condition ---
//     if (abs(e1) <= clearance) {
//       motor1_motion(0, 0);
//       break;
//     }
//   }
// }

void PID_controller(float target1, int direction1, float target2, int direction2) {  
  // directions: 0 = CCW, 1 = CW

  unsigned long init_time = millis();

  // Set targets
  E1.set_target(target1);
  E2.set_target(target2);

  float clearance = 1; // cm tolerance

  // PID gains (default values, will be overridden by user input)
  float P = 5.0, I = 0.0001, D = 1.0;
  
  // --- Read P, I, D from Serial (blocking until input) ---
  Serial.println("Enter P:");
  while (!Serial.available());  
  P = Serial.parseFloat();

  Serial.println("Enter I:");
  while (!Serial.available());  
  I = Serial.parseFloat();

  Serial.println("Enter D:");
  while (!Serial.available());  
  D = Serial.parseFloat();

  Serial.print("Using PID: P="); Serial.print(P);
  Serial.print(" I="); Serial.print(I);
  Serial.print(" D="); Serial.println(D);

  // --- Variables ---
  float last_error1 = target1;
  float last_error2 = target2;

  float integral1 = 0;
  float integral2 = 0;

  unsigned long last_time = init_time;

  // for serial debugging 
  unsigned long lastPrint = 0;

  while (true) {
    float e1 = E1.distance_from_target();
    float e2 = E2.distance_from_target();
    Serial.print("Error1: "); Serial.println(e1);
    Serial.print("Error2: "); Serial.println(e2);

    if (abs(e1) <= clearance && abs(e2) <=clearance) {
          motor_motion(0, 0);
          break;
    }
    unsigned long current_time = millis();
    float dt = (current_time - last_time) / 1000.0; // convert to seconds
    if (dt <= 0) dt = 0.001; // safety against divide by zero

    // --- PID calculations ---
    integral1 += (e1 + last_error1)/2 * dt;

    // Anti-windup clamp
    if (integral1 > 1000) integral1 = 1000;
    if (integral1 < -1000) integral1 = -1000;
    
    integral2 += (e2 + last_error2)/2 * dt;

    // Anti-windup clamp
    if (integral2 > 1000) integral2 = 1000;
    if (integral2 < -1000) integral2 = -1000;

    float derivative1 = (e1 - last_error1) / dt;
    float voltage1 = P * e1 + I * integral1 + D * derivative1;
    if (abs(e1) <= clearance) {
              voltage1 = 0;
    }
    float derivative2 = (e1 - last_error1) / dt;
    float voltage2 = P * e1 + I * integral1 + D * derivative1;
    if (abs(e2) <= clearance) {
              voltage2 = 0;
              break;
    }
    // Debug info
    
    Serial.print("Integral: "); Serial.println(integral1);
    Serial.print("Derivative: "); Serial.println(derivative1);
    Serial.print("Voltage: "); Serial.println(voltage1);
    Serial.println("------------------------------------");
    Serial.print("Integra2: "); Serial.println(integral2);
    Serial.print("Derivative2: "); Serial.println(derivative2);
    Serial.print("Voltage2: "); Serial.println(voltage2);
    Serial.println("------------------------------------");

    // --- Send to motor ---
    // if (abs(voltage1) < .5) voltage1 = 0; // ignore tiny values
    // motor1_motion(abs(voltage1), (voltage1 >= 0) ? 1 : 0);
    // motor2_motion(abs(voltage2), (voltage2 >= 0) ? 1 : 0);
    motor_motion(voltage1,voltage2);
    // Update history
    last_error1 = e1;
    last_error2 = e2;
    last_time = current_time;

    // --- Stop condition ---
    
    // if (millis() - lastPrint >= 1000) {   // print every 100ms
    //     Serial.print("Error: ");
    //     Serial.println(e1);
    //     lastPrint = millis();
    // }
  }
}



// void motor1_motion(float speed, int direction){
//   speed = map(speed,0,12,0,255);
//     switch(direction){
//       case 0: // CCW
//         digitalWrite(M1_d1,HIGH);
//         digitalWrite(M1_d2,LOW);
//         analogWrite (M1_en,speed);
//         break;
//       case 1: // CW 
//         digitalWrite(M1_d1, LOW);
//         digitalWrite(M1_d2,HIGH);
//         analogWrite (M1_en,speed);
//         break;
//       default :
//         return;
//     }
// }
// void motor2_motion(float speed, int direction){
//   speed = map(speed,0,12,0,255);
//     switch(direction){
//       case 0: // CCW
//         digitalWrite(M2_d1,HIGH);
//         digitalWrite(M2_d2,LOW);
//         analogWrite (M2_en,speed);
//         break;
//       case 1: // CW 
//         digitalWrite(M2_d1, LOW);
//         digitalWrite(M2_d2,HIGH);
//         analogWrite (M2_en,speed);
//         break;
//       default :
//         return;
//     }

// }

void motor_motion(float speed1, float speed2){
  bool dir1 = (speed1 >= 0) ? 1 : 0;
  bool dir2 = (speed2 >= 0) ? 1 : 0;
  speed1 = map(speed1,0,12,0,255);
  speed2 = map(speed2,0,12,0,255);
  switch(dir1){
      case 0: // CCW
        digitalWrite(M1_d1,HIGH);
        digitalWrite(M1_d2,LOW);
        analogWrite (M1_en,speed1);
        break;
      case 1: // CW 
        digitalWrite(M1_d1, LOW);
        digitalWrite(M1_d2,HIGH);
        analogWrite (M1_en,speed1);
        break;
      default :
        return;
  }
  switch(dir2){
      case 0: // CCW
        digitalWrite(M2_d1,HIGH);
        digitalWrite(M2_d2,LOW);
        analogWrite (M2_en,speed2);
        break;
      case 1: // CW 
        digitalWrite(M2_d1, LOW);
        digitalWrite(M2_d2,HIGH);
        analogWrite (M2_en,speed2);
        break;
      default :
        return;
  }

}