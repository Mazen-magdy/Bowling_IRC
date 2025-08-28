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
volatile int counter = 0; // stand by the number of pulses given
volatile int state1 = LOW;
volatile int state2 = LOW;
int completeTurn = 1024;

class Encoder{
  private:
    int counter = 0;
    int turnsToCm = 0;
  public:
    Encoder(int counts,int Cm ){
      turnsToCm = Cm;
      completeTurn = counts;
    }
    float Get_Moved_distance_from_launch(){
      return (float)counter / turnsToCm;
    }
    float Get_Moved_Distance_From_Point(float distance){
      return (float)counter /  turnsToCm - distance  ;
    }
    float Get_Distance_From_Point(float distance){
      return distance - (float)counter /  turnsToCm ; 
    }
    void Counter(){
      if(digitalRead(E1_Pin1) == LOW && digitalRead(E1_Pin2) == HIGH)
      {
        if(state1 == HIGH)
          counter++;
        else 
          counter--;
      }
      else if(digitalRead(E1_Pin1) == HIGH && digitalRead(E1_Pin2) == LOW)
      {
        if(state1 == LOW)
          counter++;
        else 
          counter--;
      }
      state1 = digitalRead(E1_Pin1);
      state2 = digitalRead(E1_Pin2);
    } 
};
Encoder E1(900,900);
Encoder E2(900,900);

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
  
}

// function definitions

void PID_controller(float target1,int direction1,float target2,int direction2){  // directions is 0 : CCW , 1 : CW
  int init_time = millis();
  int clearance = 0.5; // 0.5 cm
  // can record that point we started to use PID which can be used later
  int P1 = 0,I1 = 0,D1 = 0;
  float diffrential1 = 0;
  float integral1 = 0;

  int P2 = 0,I2 = 0,D2 = 0;
  float diffrential2 = 0;
  float integral2 = 0;
  
  int dt = 0 ;
  int last_error1 = target1;
  int last_error2 = target2;
  int last_time = init_time;
  
  float voltage1 = 0;
  float voltage2 = 0;

  while(true){
    float e1 = E1.Get_Distance_From_Point(target1);
    float e2 = E2.Get_Distance_From_Point(target1);
  
    if(abs(e1) < clearance && abs(e2) > clearance){

      int current_time = millis();
      dt = (current_time - last_time);

      integral1 += (float)(e1 + last_error1) / 2 * dt;
      diffrential1 = ((float)(e1- last_error1)) / dt;
      voltage1 = P1 * e1 + I1 * integral1 + D1 * diffrential1;
      
      motor1_motion(voltage1 , direction1);

      integral2 += (float)(e2 + last_error2) / 2 * dt;
      diffrential2 = ((float)(e2- last_error2)) / dt;
      voltage2 = P2 * e2 + I2 * integral2 + D2 * diffrential2;

      motor1_motion(voltage2,direction2);

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

