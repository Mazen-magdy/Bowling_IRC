
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
volatile int counter = 0; // stand by the number of pulses given
int completeTurn = 1024;

void Counter(){
      if(digitalRead(E1_Pin1) == LOW && digitalRead(E1_Pin2) == HIGH)
      {
          counter++;
      }
      else if(digitalRead(E1_Pin1) == HIGH && digitalRead(E1_Pin2) == LOW)
      {
          counter++;
      }
      else{counter--;}
} 

void run_motor(int d1,int d2,int en){
  digitalWrite(d1,HIGH);
  digitalWrite(d2,LOW);
  digitalWrite(en,HIGH);
}

void stop_motor(int d1,int d2,int en){
  digitalWrite(d1,HIGH);
  digitalWrite(d2,LOW);
  digitalWrite(en,0);
}




volatile int counter1 = 0; // stand by the number of pulses given
void Counter1(){
      if(digitalRead(E2_Pin1) == LOW && digitalRead(E2_Pin2) == HIGH)
      {
          counter1++;
      }
      else if(digitalRead(E2_Pin1) == HIGH && digitalRead(E2_Pin2) == LOW)
      {
          counter1++;
      }
      else{counter1--;}
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
  attachInterrupt(digitalPinToInterrupt(E1_Pin1), Counter, CHANGE);
  attachInterrupt(digitalPinToInterrupt(E2_Pin1), Counter1, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(counter);
  

}
