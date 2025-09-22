#include <Adafruit_AHRS.h>
#include <Adafruit_AHRS_FusionInterface.h>
#include <Adafruit_AHRS_Madgwick.h>
#include <Adafruit_AHRS_Mahony.h>
#include <Adafruit_AHRS_NXPFusion.h>
#include <Adafruit_Sensor_Set.h>
#include <Adafruit_Simple_AHRS.h>
#include <Adafruit_MPU6050.h>
// ======== GLOBAL VARIABLES =========

// ======= INSTANCES =========
Adafruit_MPU6050 mpu;
Adafruit_Madgwick filter;  // or Adafruit_Mahony filter

// ======== MPU ===========
unsigned long lastUpdate = 0;
float yaw0;
bool yaw0_flag = false;
float yaw;
float yaw_Target;

void get_Yaw_angle(){
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
// ======== PID ROTATION ===========

void yaw_PID_Controller(){
    
}


// ======= ARDUINO MAIN CODE =========
void setup() {
  // put your setup code here, to run once:
 if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1);
  }
  filter.begin(100); // sampling frequency (Hz)
}

void loop() {
  // put your main code here, to run repeatedly:
  get_Yaw_angle();
}




