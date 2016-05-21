#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include "Kalman.h"

MPU6050 mpu; // adres 0x68 (AD0 LOW)

#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_EULER


#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
//float pitch_kalman;
//float pitch_dmp;
int16_t dmpAccel[3];
int16_t dmpGyro[3];
int16_t ax, ay, az;
int16_t gx, gy, gz;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
//uint8_t guiPacket[20] = {0x10, 0x02, 0, 0, 0, 0, 0 , 0, 0, 0, 0, 0, 0, 0



// kalman i komplementarny zmienne
uint32_t timer;
double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
bool oneTime = false;


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() 
{
  mpuInterrupt = true;
}

void setup() {
  
  Wire.begin();
  TWBR = 24; 

  Serial.begin(115200);


  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
delay(2);
  //  // wait for ready
  //  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //  while (Serial.available() && Serial.read()); // empty buffer
  //  while (!Serial.available());                 // wait for data
  //  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

//  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
//  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(25);
  mpu.setYGyroOffset(12);
  mpu.setZGyroOffset(-9);
  //-4503  -1159 830 28  8 -10

  mpu.setXAccelOffset(-4503);
  mpu.setYAccelOffset(-1159);
  mpu.setZAccelOffset(830);
  //  Your offsets:  -4496 -1171 828 26  5 -10
  //
  //Data is printed as: acelX acelY acelZ giroX giroY giroZ

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

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
  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  //#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(ay, az) * RAD_TO_DEG;
  double pitch = atan2(ax , sqrt(long(ay) * ay + long(az) * az)) * RAD_TO_DEG;
  //#else // Eq. 28 and 29
  //  double roll  = 2;// atan(ay / sqrt(ax * ax + az * az)) * RAD_TO_DEG;
  //  double pitch = atan2(-ax, az) * RAD_TO_DEG;
  //#endif
  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;
  timer = micros();
}

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (oneTime) {
      oneTime = false;
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
#if 1
      //Serial.print("\t");
      //        Serial.print(ax);
      //        Serial.print("\t");
      //        Serial.print(ay);
      //        Serial.print("\t");
      //        Serial.print(az);
      //
      //       // mpu.dmpGetGyro(dmpGyro,fifoBuffer);
      //        Serial.print("\t");
      //        Serial.print(gx);
      //        Serial.print("\t");
      //        Serial.print(gy);
      //        Serial.print("\t");
      //        Serial.print(gz);
      //        Serial.print("\t");
#endif


      double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
      timer = micros();

      // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
      // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
      // It is then converted from radians to degrees
      //#ifdef RESTRICT_PITCH // Eq. 25 and 26
      double roll  = atan2(ay, az) * RAD_TO_DEG;
      double pitch = atan2(ax , sqrt(long(ay) * ay + long(az) * az)) * RAD_TO_DEG;//
      //#else // Eq. 28 and 29
      //  double roll  = atan2(ay / sqrt(ax * ax + az * az)) * RAD_TO_DEG;
      //  double pitch = atan2(-ax, az) * RAD_TO_DEG;
      //#endif

      //Serial.print("\t");
      //Serial.print(ay);
      //            Serial.print("\t");
      //            Serial.print(long(ay)*ay);
      //            Serial.print("\t");
      //            Serial.print(az);
      //            Serial.print("\t");
      //            Serial.print(long(az)*az);
      //            Serial.print("\t");
      double gyroXrate = -gx / 16.4; // Convert to deg/s
      double gyroYrate = -gy / 16.4; // Convert to deg/s

#ifdef RESTRICT_PITCH
      // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
      if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
        kalmanX.setAngle(roll);
        compAngleX = roll;
        kalAngleX = roll;
        gyroXangle = roll;
      } else
        kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

      if (abs(kalAngleX) > 90)
        gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
      // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
      if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
        kalmanY.setAngle(pitch);
        compAngleY = pitch;
        kalAngleY = pitch;
        gyroYangle = pitch;
      } else
        kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

      if (abs(kalAngleY) > 90)
        gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

      gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
      gyroYangle += gyroYrate * dt;
      //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
      //gyroYangle += kalmanY.getRate() * dt;

      compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
      compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

      // Reset the gyro angle when it has drifted too much
      if (gyroXangle < -180 || gyroXangle > 180)
        gyroXangle = kalAngleX;
      if (gyroYangle < -180 || gyroYangle > 180)
        gyroYangle = kalAngleY;

      /* Print Data */
#if 1 // Set to 1 to activate
      Serial.print(ax); Serial.print("\t");
      Serial.print(ay); Serial.print("\t");
      Serial.print(az); Serial.print("\t");
      Serial.print(gx); Serial.print("\t");
      Serial.print(gy); Serial.print("\t");
      Serial.print(gz); Serial.print("\t");
     // Serial.print("\t");
#endif
    //  Serial.print("\t");
      Serial.print(roll); Serial.print("\t");
      Serial.print(gyroXangle); Serial.print("\t");
      Serial.print(compAngleX); Serial.print("\t");
      Serial.print(kalAngleX); Serial.print("\t");

      //Serial.print("\t");

      Serial.print(pitch); Serial.print("\t");
      Serial.print(gyroYangle); Serial.print("\t");
      Serial.print(compAngleY); Serial.print("\t");
      Serial.print(kalAngleY); Serial.print("\t");

#if 0 // Set to 1 to print the temperature
      Serial.print("\t");
      double temperature = (double)tempRaw / 340.0 + 36.53;
      Serial.print(temperature); Serial.print("\t");
#endif

      Serial.print("\r\n");


      // other program behavior stuff here
      // .
      // .
      // .
      // if you are really paranoid you can frequently test in between other
      // stuff to see if mpuInterrupt is true, and if so, "break;" from the
      // while() loop to immediately process the MPU data
      // .
      // .
      // .
    }
  }
  oneTime = true;
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_QUATERNION
    // display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    Serial.print("quat\t");
    Serial.print(q.w);
    Serial.print("\t");
    Serial.print(q.x);
    Serial.print("\t");
    Serial.print(q.y);
    Serial.print("\t");
    Serial.println(q.z);
#endif

#if 1//OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[2] * 180 / M_PI);
    Serial.print("\t\r\n");
#endif
#if 0
    mpu.dmpGetAccel(dmpAccel, fifoBuffer);
    Serial.print("\t");
    Serial.print(dmpAccel[0]);
    Serial.print("\t");
    Serial.print(dmpAccel[1]);
    Serial.print("\t");
    Serial.print(dmpAccel[2]);

    mpu.dmpGetGyro(dmpGyro, fifoBuffer);
    Serial.print("\t");
    Serial.print(dmpGyro[0]);
    Serial.print("\t");
    Serial.print(dmpGyro[1]);
    Serial.print("\t");
    Serial.print(dmpGyro[2]);
#endif



#ifdef OUTPUT_TEAPOT
    // display quaternion values in InvenSense Teapot demo format:
    teapotPacket[2] = fifoBuffer[0];
    teapotPacket[3] = fifoBuffer[1];
    teapotPacket[4] = fifoBuffer[4];
    teapotPacket[5] = fifoBuffer[5];
    teapotPacket[6] = fifoBuffer[8];
    teapotPacket[7] = fifoBuffer[9];
    teapotPacket[8] = fifoBuffer[12];
    teapotPacket[9] = fifoBuffer[13];
    Serial.write(teapotPacket, 14);
    teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

    // delay(2);
  }

}
