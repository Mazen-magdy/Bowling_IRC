

volatile bool M1_timer_tiked = false;
volatile uint32_t M1_last_millis = 0;
volatile long M1_encoder_read = 0;

volatile bool M2_timertiked = false;
volatile uint32_t M2_last_millis = 0;
volatile long M2_encoder_read = 0;

ISR(TIMER1_COMPA_vect){
  // M1 control flag , feedback , dt
  M1_timer_tiked = true;
  M1_encoder_read = E1.getDistance;
}
ISR(TIMER1_COMPB_vect){
  // Motor 2 flag , feedback, dt
  M2_timer_tiked = true;
}
bool flag_A =false,flag_B = false;
long async_two_motors = E1.getMovedDistance - E2.getMovedDistance;
long Ksync = 0.2;


float M1_integral = 0;
float M1_last_D_filtered = 0;
float M1_alpha = 0.5; // related to D calculations
int M1_last_Error = 0;
float M1_P,M1_I,M1_D;
int M1_startPoint; // it sets when running PID
float M1_last_measure;
float M1_clearance = 0.5;
void PID_motion_M1(){
  if(M1_timer_tiked){
    cli();
    // we made because if we returned from interrupt in the middle of calculations not complete with modified variables 
    float dt = (millis()  - M1_last_millis) / 1000.0 ;
    int enocder_read = M1_encoder_read;
    M1_timer_tiked = false ;
    sei();
    int e = getDistanceFromTarget(enocder_read);
    if(e <= M1_clearance)
      {
        stop_motor(); // stops motor
        flag_A = true;
        if(flag_B)
          stop_PID_controller();
        return;
      }
    float measure = getMovedDistanceFromPoint(enocder_read,startPoint);
    M1_integral += (e + M1_last_Error) / 2  * dt;
    M1_integral = constrain(M1_integral ,-1000 ,1000); // windup shield 
    // in derivative we will derive over measurement and we will get the avg of all derivatives to avoid noise
    float D_filtered = M1_alpha * (measure-M1_last_measure) / dt + (1-M1_alpha) * M1_last_D_filtered; // noise cancller

    async_two_motors = E1.getMovedDistance - E2.getMovedDistance; // get the difference between them 
    float V = M1_P * e + M1_I * M1_integral - M1_D * D_filtered - Ksync * async_two_motors ;
  
    launch_motor(V); // inside it will move motor into the direction V tells 
    // inherit values
    M1_last_millis = millis();
    M1_last_Error = e;
    M1_last_D_filtered = D_filtered;
    M1_last_measure = measure;
  }
}


float M2_integral = 0;
float M2_last_D_filtered = 0;
float M2_alpha = 0.5; // related to D calculations
int   M2_last_Error = 0;
float M2_P,M2_I,M2_D;
int   M2_startPoint; // it sets when running PID
float M2_last_measure;
float M2_clearance = 0.5;
void PID_motion_M2(){
  if(M2_timer_tiked){
    cli();
    // we made because if we returned from interrupt in the middle of calculations not complete with modified variables 
    float dt = (millis()  - M2_last_millis) / 1000.0 ;
    int enocder_read = M2_encoder_read;
    M2_timer_tiked = false ;
    sei();
    int e = getDistanceFromTarget(enocder_read);
    if(e <= M2_clearance)
      {
        stop_motor(); // stops motor
        flag_B = true;
        if(flag_A)
          stop_PID_controller();
        return;
      }
    float measure = getMovedDistanceFromPoint(enocder_read,startPoint);
    M2_integral += (e + M2_last_Error) / 2  * dt;
    M2_integral = constrain(M2_integral ,-1000 ,1000); // windup shield 
    // in derivative we will derive over measurement and we will get the avg of all derivatives to avoid noise
    float D_filtered = M2_alpha * (measure-M2_last_measure) / dt + (1-M2_alpha) * M2_last_D_filtered; // noise cancller

    async_two_motors = E1.getMovedDistance - E2.getMovedDistance; // get the difference between them 
    float V = M2_P * e + M2_I * M2_integral - M2_D * D_filtered + Ksync * async_two_motors ;
    //
    launch_motor(V); // inside it will move motor into the direction V tells 
    // inherit values
    M2_last_millis = millis();
    M2_last_Error = e;
    M2_last_D_filtered = D_filtered;
    M2_last_measure = measure;
  }
}




void run_motors_PID_controller(){
  cli(); // stop interrupt when setting 
  TCCR1A = (1<< WGM11);
  TCCR1B = (1<< WGM12) | (1<< WGM13) 
          | (1<< CS10)| (1<< CS11);
  ICR1 = 4999;
  OCR1A = 2499;
  OCR1B = 4999;
  TIMSK1 |= (1 << OCIE1A) | (1 << OCIE1B); // enable interrupt
  sei();
}
void stop_motors_PID_controller(){
    TCCR1B = 0;   // stop Timer1
  TIMSK1 &= ~((1 << OCIE1A) | (1 << OCIE1B));

}
void setup() {
  // put your setup code here, to run once:
  // setup interrupt on timer 1 with OCR1A , OCR2B
  
}



void loop() {
  // put your main code here, to run repeatedly:
  if(true)  // condition before running pid
    run_motors_pid_controller()
  PID_motion_M1();
}
