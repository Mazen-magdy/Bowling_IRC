#define IR 2
#define E1_Pin1 3 // has interrupt
#define E1_Pin2 4 // don't have interrupt
#define M1_d1 8
#define M1_d2 12
#define M1_en 9

volatile int counter = 0; // stand by the number of pulses given
int completeTurnCounts = 0;
int completeTurnTime = 0;
int hits = 0;
long startTime = 0;

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


void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(IR,INPUT);
  pinMode(E1_Pin1, INPUT);
  pinMode(E1_Pin2, INPUT);
  pinMode(M1_d1, OUTPUT);
  pinMode(M1_d2, OUTPUT);
  pinMode(M1_en, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(E1_Pin1), Counter, CHANGE);
}

long counter1 =0;
int lastRead = 1;
int currentRead = 1;
void loop() {
  // -----------------there is a delay in triggering solve it 
  // put your main code here, to run repeatedly:
  run_motor(M1_d1,M1_d2,M1_en);
  currentRead =  digitalRead(IR);
  if( currentRead == 0 && lastRead == 1 ){
    if( hits == 0){
      hits = 1;
      startTime = millis();
      counter1 = counter;
    }
    else{
      completeTurnCounts = counter - counter1;
      completeTurnTime = millis() - startTime; // with max speed
      hits = 0;
      Serial.print("counts ");
      Serial.println(completeTurnCounts);
      Serial.print("Time ");
      Serial.println(completeTurnTime);
    }
    lastRead = digitalRead(IR);
  }
  lastRead = currentRead;
  //Serial.println(digitalRead(IR));
  // Serial.print("counter ");
  // Serial.println(counter);
}
