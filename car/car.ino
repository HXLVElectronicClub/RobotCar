// comment this line out if you are not using L239D and drive the motor directly.
#define L239D_DRIVE
// use shift register to save pins
#define USE_74HC595

// Select IR or Bluetooth
//#define USE_IR
#define USE_BLUETOOTH
// Use MPU6050 Gyroscope and Accelerometer
#define USE_MPU6050
// use shift register to save pins
#define USE_74HC595

// push demo
/*-------------------------------
   Define used pins
-------------------------------*/
#ifndef L239D_DRIVE
  #define LEFT 8
  #define RIGHT 9
#else
    #define LEFT         5
    #define LEFTR        4
    #define RIGHT        3
    #define RIGHTR       2
    #define ENABLE_LEFT  9
    #define ENABLE_RIGHT 6
#endif

#ifdef USE_74HC595
    #define SHIFT_IN    2
    #define SHIFT_CLK   3
    #define SHIFT_LATCH 4
#endif

#ifdef USE_74HC595
    #define SHIFT_IN    3
    #define SHIFT_CLK   4
    #define SHIFT_LATCH 5
#endif

#ifdef USE_MPU6050
  #define INTERRUPT_PIN 2
#endif

#define SENSOR_LEFT A0
#define SENSOR_RIGHT A1
#define IRDATA 11
#define ECHO_PIN 8
#define TRIG_PIN 7

#ifdef USE_BLUETOOTH
  #define BT_TX 12
  #define BT_RX 13
#endif
/*-------------------------------*/

/*---------------------------------
 * include libraries
 *---------------------------------*/
#ifdef USE_BLUETOOTH
  #include "ArduinoBlue.h"
  #include <SoftwareSerial.h>
#endif

#ifdef USE_IR
  #include "IRremote.h"
#endif
#ifdef USE_MPU6050
  #include "I2Cdev.h"
  #include "MPU6050_6Axis_MotionApps20.h"
  #include "Wire.h"
#endif

#include "Move.h"
//Distance sensor
#include "SR04.h"

/*---------------------------------
 * Function declaration
 *--------------------------------*/
// Read IR sensor/or bluetooth control, and call coresponding function
void IRControl();
// Read Bluetooth data from phone
void BTControl();
// Move according to IR data
bool DoMove(long);
// Auto move, avoid obstacal by distance sensor, Need Ultrasonic distance sensor
void AutoMove();
// Read Displacement data from MPU6050
void readDisplacement();
// Trace the path drew from bluetooth, need bluetooth and MP6050 sensor
void TracePath();

/*---------------------------------
 * Controller/sensor control code instance
 *--------------------------------*/
#ifdef USE_IR
  IRrecv irrecv(IRDATA);
#endif
#ifdef USE_BLUETOOTH
  SoftwareSerial softSerial(BT_TX,BT_RX);
  ArduinoBlue phone(softSerial);
#endif
  SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
#ifdef USE_MPU6050
  MPU6050 mpu;
  // MPU control/status vars
  bool dmpReady = false;  // set true if DMP init was successful
  uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
  void dmpDataReady() {mpuInterrupt = true;}
#endif

/*----------------------------------
 * code start
 *--------------------------------*/
void setup() {
  Serial.begin(115200);
#ifndef L239D_DRIVE
  MotorPins(LEFT,RIGHT);
#else
  #ifdef USE_74HC595
    MotorPins_shift(SHIFT_IN,SHIFT_CLK,SHIFT_LATCH,ENABLE_LEFT,ENABLE_RIGHT);
  #else
    MotorPins(LEFT, LEFTR, RIGHT, RIGHTR, ENABLE_LEFT, ENABLE_RIGHT);
  #endif 
  SetSpeedRatio(1,0.9);
#endif
#ifdef USE_IR
  irrecv.enableIRIn();
#endif
#ifdef USE_BLUETOOTH
  softSerial.begin(9600);
#endif
#ifdef USE_MPU6050
  uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
  Wire.begin();
  Wire.setClock(400000);
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();

    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
#endif
}

void loop() {
#ifdef USE_BLUETOOTH
  BTControl();
#endif
#ifdef USE_IR
  IRControl();
#endif
}

void goAndStop() {
  MoveForward(1000);
  Stop();
}

void trace() {
  int sensor1 = analogRead(SENSOR_LEFT);
  int sensor2 = analogRead(SENSOR_RIGHT);
  
  if (sensor1 > sensor2) {
    TurnLeft();
  } else {
    TurnRight();
  }
  delay(100);
}

bool automove=false;

#ifdef USE_IR
bool nocmd=true;
long t = 0;
void IRControl() {
  decode_results results;
  if (irrecv.decode(&results)) {
    Serial.print("IR: 0x");
    Serial.println(results.value, HEX);
    
    if (DoMove(results.value)) {
      t=millis();
      nocmd = false;
    }
    irrecv.resume();
  } else if (automove) {
    Serial.println("AutoMove");
    AutoMove();
  } else if (!nocmd) {
    if (millis() - t > 300) {
      Stop();
      Serial.print("NoCmd:");
      Serial.println(millis()-t);
      nocmd = true;
    }
  }
}

/*   
  case 0xFFA25D: Serial.println("POWER"); break;
  case 0xFFE21D: Serial.println("VOL STOP"); break;
  case 0xFF629D: Serial.println("MODE"); break;
  case 0xFF22DD: Serial.println("PAUSE");    break;
  case 0xFF02FD: Serial.println("FAST BACK");    break;
  case 0xFFC23D: Serial.println("FAST FORWARD");   break;
  case 0xFFE01F: Serial.println("EQ");    break;
  case 0xFFA857: Serial.println("VOL-");    break;
  case 0xFF906F: Serial.println("VOL+");    break;
  case 0xFF9867: Serial.println("RETURN");    break;
  case 0xFFB04F: Serial.println("USB SCAN");    break;
  case 0xFF6897: Serial.println("0");    break;
  case 0xFF30CF: Serial.println("1");    break;
  case 0xFF18E7: Serial.println("2");    break;
  case 0xFF7A85: Serial.println("3");    break;
  case 0xFF10EF: Serial.println("4");    break;
  case 0xFF38C7: Serial.println("5");    break;
  case 0xFF5AA5: Serial.println("6");    break;
  case 0xFF42BD: Serial.println("7");    break;
  case 0xFF4AB5: Serial.println("8");    break;
  case 0xFF52AD: Serial.println("9");    break;
  case 0xFFFFFFFF: Serial.println(" REPEAT");break;  
  */
long lastmove;
bool DoMove(long value) {
  switch(value) {
      case 0xFFFFFFFF: // repeat
        DoMove(lastmove);
        break;
      case 0xFF629D: // Mode
        automove = !automove;
        break;
      case 0xFF18E7: // 2
        Serial.println("MoveFW");
        MoveForward();
        lastmove = value;
        break;
      case 0xFF30CF: // 1
      case 0xFF10EF: // 4
        TurnLeft();
        Serial.println("MoveLT");
        lastmove = value;
        break;
      case 0xFF38C7: // 5
        MoveBackward();
        Serial.println("MOVEBW");
        lastmove = value;
        break;
      case 0xFF7A85: // 3
      case 0xFF5AA5: // 6
        Serial.println("MoveRT");
        lastmove = value; 
        TurnRight();
        break;
      default:
        lastmove = 0;
        return false;
    }
    //delay(100);
    return true;
}
#endif

void AutoMove() {
  long a = sr04.Distance();
  Serial.print(a);
  Serial.println("cm");
  if (a <= 15) {
    Stop();
    MoveBackward(100);
    TurnLeft();
  } else {
    MoveForward();
  }
}

#ifdef USE_BLUETOOTH
void BTControl() {
  
  int throttle = phone.getThrottle() - 49;
  int steering = phone.getSteering() - 49;

  if (phone.isPathAvailable()) {
    TracePath();
  } else if (throttle == 0) {
    // If throttle is zero, don't move.
    Stop();
  } else {
    MoveCar(throttle, steering);
  }
}

  // THROTTLE AND STEERING CONTROL
  // throttle values after subtracting 49:
  //     50 = max forward throttle
  //     0 = no throttole
  //     -49 = max reverse throttle
  // steering values after subtracting 49:
  //     50 = max right
  //     0 = straight
  //     -49 = max left
void MoveCar(int throttle, int steering) {
  if (throttle > 50) throttle = 50;
  if (throttle < -50) throttle = -50;
  
  // Determine forwards or backwards.
  if (throttle > 20) {
    // Forward
    MoveForward();
  }
  else {
    // Backward
    MoveBackward();
  }
  
  // Map throttle to PWM range.
  int mappedSpeed = map(abs(throttle), 0, 50, MINIMUM_MOTOR_SPEED, 255);
  // Map steering to PWM range.
  int reducedSpeed = map(abs(steering), 0, 50, mappedSpeed, MINIMUM_MOTOR_SPEED);

  int leftMotorSpeed, rightMotorSpeed;
  if (steering > 0) {
    // Turn Right: reduce right motor speed
    leftMotorSpeed = mappedSpeed;
    rightMotorSpeed = reducedSpeed;
  }
  else {
    // Turn Left: reduce left motor speed
    leftMotorSpeed = reducedSpeed;
    rightMotorSpeed = mappedSpeed;
  }

  SetLeftSpeed(leftMotorSpeed);
  SetRightSpeed(rightMotorSpeed);
  
  // Print Debug Info
  Serial.print("throttle: "); Serial.print(throttle);
  Serial.print("\tsteering: "); Serial.print(steering);
  Serial.print("\tmappedSpeed: "); Serial.print(mappedSpeed);
  Serial.print("\treducedSpeed: "); Serial.print(reducedSpeed);
  Serial.print("\tleftMotorSpeed: "); Serial.print(leftMotorSpeed);
  Serial.print("\trightMotorSpeed: "); Serial.println(rightMotorSpeed);
}

#ifdef USE_MPU6050
// Trace the path from phone
// current implementation:
// 1) turn to the right direction
// 2) move forward certain time, time is calculated by distance
void TracePath() {
  int l    = phone.getPathLength();
  Serial.print("Path data length:");Serial.println(l);
  if (l>0) {
    float *xs = phone.getPathArrayX();
    float *ys = phone.getPathArrayY();

    // the angle heading to
    float heading;
    float ypr[3]; // vector to store yaw/pitch/roll
    readMPU6050(ypr);
    heading = ypr[0];

    for (int i=1; i<l; i++) {
      // double theta_p = theta;
      // Calculate the direction and speed
      //Serial.print("Theta(degree):");Serial.print(theta*180/M_PI);
      float deltax = xs[i]-xs[i-1];
      float deltay = ys[i]-ys[i-1];
      float mod = sqrt(deltax*deltax+deltay*deltay);
      // float x_ = deltax * cos(theta) + deltay*sin(theta);
      // float y_ = deltay * cos(theta) - deltax*sin(theta); 

      float dir;
      if (deltay == 0) {
        dir = 90;
        if (deltax > 0) {
          dir = -90;
        }
      } else {
        dir = atan(deltax/deltay)*180/M_PI;
      }

      // Start to move
      SetLeftSpeed(255);
      SetRightSpeed(255);

      // Turn to direction according to current angle
      if (dir-heading > 0 || dir-heading<-180) {
        TurnRight();
        while(dir-heading>0 || dir-heading<-180) {
          readMPU6050(ypr);
          heading = ypr[0];
          // Serial.print("dir:");Serial.print(dir);
          // Serial.print("-->");
          // Serial.print("Heading:");Serial.println(heading);
        };
      } else {
        TurnLeft();
        while(dir-heading <=0 && dir-heading >= -180) {
          readMPU6050(ypr);
          heading = ypr[0];
          // Serial.print("dir:");Serial.print(dir);
          // Serial.print("<--");
          // Serial.print("Heading:");Serial.println(heading);
        }
      }
      
      // after this, should heading to the target point
      MoveForward(mod*20);
      //Serial.print("Moving forward:");Serial.println(mod*10);
      
      // Serial.print("\tdelta X:");Serial.print(deltax);
      // Serial.print("\tdelta Y:");Serial.print(deltay);
      // Serial.print("\tX_:");Serial.print(x_);
      // Serial.print("\tY_:");Serial.print(y_);      

      // float deltax = xs[i]-xs[0];
      // float deltay = ys[i]-ys[0];
      // if (deltay == 0) {
      //   theta = 90;
      //   if (deltax<0) {
      //     theta = -90;
      //   }
      // } else {
      //   theta = atan(deltax/deltay)*180/M_PI;
      // }
      // Serial.print("deltax:");Serial.print(deltax);
      // Serial.print("\tdeltay:");Serial.print(deltay);
      // deltax = xs[i]-xs[i-1];
      // deltay = ys[i]-ys[i-1];
      // Serial.print("\tmod:");Serial.print(mod);
      // Serial.print("\ttheta:");Serial.println(theta);
      
      // while(1) {
      //   float ypr[3];
      //   VectorInt16 ds;
      //   if (readMPU6050(&v, ypr,&ds)) {
      //     s.x += ds.x;
      //     s.y += ds.y;
      //     if (ds.x!=0 || ds.y != 0) {
      //       Serial.print("\tV:");Serial.print(v.x);
      //       Serial.print("\t");Serial.print(v.y);
      //       //Serial.print("\tYaw:");Serial.print(ypr[0]);
      //       Serial.print("\tdSx:");Serial.print(ds.x);
      //       Serial.print("\tdSy:"); Serial.print(ds.y);
      //       Serial.print("\tSx:"); Serial.print(s.x);
      //       Serial.print("\tSy:"); Serial.print(s.y);
      //       Serial.println();
      //     }
      //   }
      // }
      // while (s.x < xs[i] && s.y < ys[i]) {
      //   float[3] ypr;
      //   VectorInt16 ds;
      //   readMPU6050(&v,ypr,&ds);
      //   s.x += ds.x;
      //   s.y += ds.y;
      //   if (ypr[0] > )  

      // }
    }
  }
}

// Read MPU6050 sensor
// Currently only read the yaw in degree ( I cannot let the displacement work)
// potentially it can return volocity and displacement
bool readMPU6050(float *ypr) {
  // static unsigned long lastTime;
  // static int zerocount;
  
  uint8_t fifoBuffer[64]; // FIFO storage buffer
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorInt16 aa;         // [x, y, z]            accel sensor measurements
  VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
  VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
  VectorFloat gravity;    // [x, y, z]            gravity vector
  
  if (!dmpReady) return false;
  if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return false;
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  // mpu.dmpGetAccel(&aa, fifoBuffer);
  // mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  // mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

  // unsigned long currTime = millis();
  // unsigned long dtime = currTime - lastTime;
  // lastTime = currTime;

  ypr[0] = ypr[0]*180/M_PI;
  ypr[1] = ypr[1]*180/M_PI;
  ypr[2] = ypr[2]*180/M_PI;

  // VectorInt16 a;
  // a.x = aaWorld.x/128; // remove noise
  // a.y = aaWorld.y/128;
  // a.z = aaWorld.z/128;

  // // Serial.print("areal\t");
  // // Serial.print(aaReal.x);
  // // Serial.print("\t");
  // // Serial.print(aaReal.y);
  // // Serial.print("\t");
  // // Serial.print(aaReal.z);
  // // Serial.print("\ta:");Serial.print(a.x);Serial.print(a.y);
  // // Serial.print("\tyaw:");Serial.print(ypr[0]);
  // // Serial.println();
  
  // if (a.x == 0 && a.y == 0) {
  //     zerocount++;
  // // if we see 10 times a == 0, 
  // // force speed to 0
  //     if (zerocount > 10) {
  //     ds->x += v->x*dtime;
  //     ds->y += v->y*dtime;
  //     v->x = 0;
  //     v->y = 0;
  //     zerocount = 0;
  //     }
  // } else {
  //     ds->x += (v->x*dtime + a.x*dtime*dtime/2);
  //     v->x += a.x*dtime;
  //     ds->y += v->y*dtime + a.y*dtime*dtime/2;
  //     v->y += a.y*dtime;
  //     zerocount = 0;
  // }
  return true;
}

#endif
#endif
