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
    int counter = 0;
    int countPerRevolution = 0;
    int circumference = 0;
    int countPerCm = 0;
    float target = 0;
    int E1 = 5; // encoder pin 1
    int E2 = 6; // encoder pin 2
  public:
    Encoder(int counts,int Cm, int e1,int e2 ){ // e1,e2 is the encoder pins 

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
      return this - >Get_Moved_distance_from_launch() - distance  ;
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
    } 
};
Encoder E1(1000,40.84,E1_Pin1,E1_Pin2);
Encoder E2(1000,40.84,E2_Pin1,E2_Pin2);

void PID_controller(float target1,int direction1,float target2,int direction2);
void motor_motion(float speed, int direction);

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
  pinMode(M1_d1, OUTPUT);
  pinMode(M1_d2, OUTPUT);
  pinMode(M1_en, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(E1_Pin1), E1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(E2_Pin1), E2_ISR, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
  PID_controller(2,0,2,1);

}

// function definitions

void PID_controller(float target1,int direction1,float target2,int direction2){  // directions is 0 : CCW , 1 : CW
  int init_time = millis();
  E1.set_target(target1);
  E2.set_target(target2);
  int clearance = 0.5; // 0.5 cm
  // can record that point we started to use PID which can be used later
  int P = 0,I = 0,D = 0;
  float diffrential1 = 0;
  float integral1 = 0;

  float diffrential2 = 0;
  float integral2 = 0;
  
  int dt = 0 ;
  int last_error1 = target1;
  int last_error2 = target2;
  int last_time = init_time;
  
  float voltage1 = 0;
  float voltage2 = 0;

  Serial.print("Enter P ,I ,D values : ");
  P = Serial.read();
  I = Serial.read();
  D = Serial.read();

  while(true || Serial.read() == 's'){
    float e1 = E1.distance_from_target();
    float e2 = E2.distance_from_target();
  
    if(abs(e1) < clearance && abs(e2) > clearance){

      int current_time = millis();
      dt = (current_time - last_time);

      integral1 += (float)(e1 + last_error1) / 2 * dt;
      diffrential1 = ((float)(e1- last_error1)) / dt;
      voltage1 = P * e1 + I * integral1 + D * diffrential1;
      
      if(voltage1 < 0){
          if(direction1 == 0)
            motor1_motion(voltage1 * -1,1);
          else
            motor1_motion(voltage1 * -1,0);

        }
        else
          motor1_motion(voltage1,direction2);


      integral2 += (float)(e2 + last_error2) / 2 * dt;
      diffrential2 = ((float)(e2- last_error2)) / dt;
      voltage2 = P * e2 + I * integral2 + D * diffrential2;
      if(voltage2 < 0){
        if(direction2 == 0)
          motor2_motion(voltage2 * -1,1);
        else
          motor2_motion(voltage2 * -1,0);

      }
      else
        motor2_motion(voltage2,direction2);

      last_error1 = e1;
      last_error2 = e2;
      last_time = current_time;
    }
    else{break;}
  }
}
void motor1_motion(float speed, int direction){
  switch(direction){
    switch(direction){
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
}
void motor2_motion(float speed, int direction){
  switch(direction){
    switch(direction){
      case 0: // CCW
        digitalWrite(M2_d1,HIGH);
        digitalWrite(M2_d2,LOW);
        analogWrite (M2_en,speed);
        break;
      case 1: // CW 
        digitalWrite(M2_d1, LOW);
        digitalWrite(M2_d2,HIGH);
        analogWrite (M2_en,speed);
        break;
      default :
        return;
    }
  }
}

