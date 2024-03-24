#include <I2C.h>
#include <Wire.h>
#include <MD25.h>

// ================================================================
// ===                          MD25                            ===
// ================================================================

#define MD25ADDRESS         0x58     
MD25 controller;

// ================================================================
// ===                     variables                            ===
// ================================================================
 
 int pulseDiffR;
 int pulseDiffL;

const unsigned long timeInterval = 200; //  in milliseconds
unsigned long prevTime = 0;
 int prevPulseCountR = 0;
 int prevPulseCountL = 0;
float rpmR=0;
float rpmL=0;



float rpmLinPwm;
float rpmRinPwm;

//tilt variables
 float roll_angle;
 float setangle = 0;
 float desiredSignal; //desired signal for both motors


// pids
 float errorintegral = 0; //pidtilt
float errorprev=0; 
 float kp = 6; 
 float ki = 1;
 float kd =1;




/*
 float errorintegral2 = 0; //pidmotorR
 float kp2 =1;
 float ki2 =0 ;


 float errorintegral3 = 0; //pidmotorL
 float kp3 =1;
 float ki3 =0 ;
*/

int OutputR;
int OutputL;






 unsigned volatile long currmic; // for pid1
 unsigned volatile long prevmic = 0;
 unsigned volatile long currmic2; //for pid2
 unsigned volatile long prevmic2 = 0;
 unsigned volatile long currmic3; // for pid3
 unsigned volatile long prevmic3 = 0;





/*
//distance from encoder
int distancecounter=0;//cant reset it to zero, the number has to be constantly updated from the last value
int revolutions= distancecounter*(1/360); // pulses*(rev/pulse)
int wheelcircumference = 10; //in cm
int distance;//in cm
*/




#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu(0x68);
//MPU6050 mpu(0x69); // <-- use for AD0 high

//choose degree format voidloop #ifdef
//#define OUTPUT_READABLE_YAWPITCHROLL//noifdef//







// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===                      VOID SETUP                       ===
// ================================================================

void setup() {



  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();


   // Create an instance of the MD25 motor controller
  controller = MD25(0xB0 >> 1);

  // Set the I2C address of the MD25 motor controller
  controller.changeAddress(MD25ADDRESS);

  // Set the mode to Encoder Mode 1
  controller.setMode(1);








  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);





  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
//**********************************************//



  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));


 //********no typing required// while (Serial.available() && Serial.read()); // empty buffer
 // while (!Serial.available());                 // wait for data
 // while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  //************* supply your own gyro offsets here, scaled for min sensitivity (whats show on the serialmonitor came from library)
  mpu.setXGyroOffset(100);
  mpu.setYGyroOffset(7);
  mpu.setZGyroOffset(51);
  mpu.setXAccelOffset(-1951);
  mpu.setYAccelOffset(-325);
  mpu.setZAccelOffset(1155);
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    
    mpu.PrintActiveOffsets();  ////           X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro
                               //OFFSETS      -1951,    -325,    1155,      100,       7,      51


    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
//    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();


    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();



     controller.resetEncoders();
     





  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

 
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() 

{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
  
  { 
    
 currmic=micros(); // for pid1

 //currmic2=micros(); //for pid2

 //currmic3=micros(); // for pid3
//----------------------------------------------------------------------------





     // Get the Latest packet 
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetGyro(&gy, fifoBuffer);
    
 
    roll_angle = ypr[2] * 180 / M_PI;


    //float gx_rad = gy.x * PI / 180.0; //degree per sec to rad per sec



    pidtilt(); // get desie speed for both motors
    




//----------------------------------------------------------------------------
 //timebase

 unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - prevTime;

  if (elapsedTime >= timeInterval) {



    
 
    int currentPulseCountR = controller.getEncoder1();
    int currentPulseCountL = controller.getEncoder2();

    pulseDiffR = currentPulseCountR - prevPulseCountR;
    pulseDiffL = currentPulseCountL - prevPulseCountL;


   // pulseDiffpersec= pulseDiff/(elapsedTime/1000);
    rpmR = (pulseDiffR * (60000)) / (elapsedTime*360.0); //*60 to minute  // encoder pulse:360
    rpmL = (pulseDiffR * (60000)) / (elapsedTime*360.0); //*60 to minute  // encoder pulse:360



    

    prevPulseCountR=currentPulseCountR;
    prevPulseCountL=currentPulseCountL;

    prevTime=currentTime;




  }
    
//----------------------------------------------------------------------------

    //RpmRToPwm();
    //RpmLToPwm();


    //pidRmotor();
    //pidLmotor();

    
//----------------------------------------------------------------------------



      rightrotforward();
      leftrotforward();

// Setting stop rotating range, make the robot run unsmoothly. So we take it of

/*
   if (roll_angle > 2 || roll_angle > 2) 
    {
      rightrotforward();
      leftrotforward();
    } 

    else if (roll_angle < -2 || roll_angle < -3)
    {
      rightrotbackward();
      leftrotbackward();
    } 

     else if (roll_angle >= -2 && roll_angle <= 2 || roll_angle >= -2 && roll_angle <= 2)
     {
      rightrotstop();
      leftrotstop();
     }

*/
/*
        if (desiredSignal > 10 ) 
    {
      rightrotforward();
      leftrotforward();
    } 

    else if (desiredSignal < -10 )
    {
      rightrotbackward();
      leftrotbackward();
    } 

     else if (desiredSignal >= -10 && desiredSignal <= 10  )
     {
      rightrotstop();
      leftrotstop();
     }
*/

/*
   if (roll_angle>-28.5) 
    {
      rightrotforward();
      leftrotforward();
    } 

    else if (roll_angle<-26.5)
    {
      rightrotbackward();
      leftrotbackward();
    } 

     else if (roll_angle >= -28.5 && roll_angle <= -26.5)
     {
      rightrotstop();
      leftrotstop();
     }

*/




 
//----------------------------------------------------------------------------

   //important variables

  Serial.print(" Angle :  ");  
  Serial.print(roll_angle);  

  Serial.print("  | desire signal in PWM ");
  Serial.print(desiredSignal);



  Serial.print("|   rpmRinpwm ");   //0.5 for left and right adjustment(if the balance between left and right is not center)
  Serial.print(rpmR);


  Serial.print("|   rpmLinpwm ");   
  Serial.print(rpmL);







  }

//----------------------------------------------------------------------------

  //plotter

   Serial.print(10); 
   Serial.print(',');  
   Serial.print(roll_angle);    
   Serial.print(','); 
   Serial.print(setangle);    
   Serial.print(','); 
   Serial.println(-10); 

  }




// ================================================================
// ===                    define motor function                 ===
// ================================================================


void pidtilt()//1
   {
     float deltaT = (currmic - prevmic) / 1000000;
     prevmic = currmic;

     float error = setangle- roll_angle ;    

     errorintegral = errorintegral + error*deltaT; //integral

     float errorderivative = (error-errorprev)*deltaT;// deriative
    
     float ut = kp*error + ki*errorintegral + kd * errorderivative;  

     desiredSignal = ut;
     
     if (desiredSignal > 127) // in pwm signal unit 
     {
       desiredSignal = 127;
     }
     if (desiredSignal < -127)
     {
       desiredSignal = -127;
     }
     
     errorprev=error;

      OutputR= desiredSignal;
      OutputL= desiredSignal;





   }
/*
void pidRmotor() //2
{
     float deltaT2 = (currmic2 - prevmic2) / 1000000;
     prevmic2 = currmic2;

    float  error2 =  desiredSignal - rpmR  ; //inpwm

     errorintegral2 = errorintegral2 + error2*deltaT2;
    
     float ut2 = kp2*error2 + ki2*errorintegral2 ;  

     OutputR = ut2;  

     if (OutputR > 127)
     {
       OutputR = 127;
     }
     if (OutputR < -127)
     {
       OutputR = -127;
     }
}


void pidLmotor() //3
{
     float deltaT3 = (currmic3 - prevmic3) / 1000000;
     prevmic3 = currmic3;

     float error3 =  desiredSignal - rpmL  ; //inrpm

     errorintegral3 = errorintegral3 + error3*deltaT3;
    
     float ut3 = kp3*error3 + ki3*errorintegral3 ;  

     OutputL = ut3;  

     if (OutputL > 127)
     {
       OutputL = 127;
     }
     if (OutputL < -127)
     {
       OutputL = -127;
     }
}

*/
// ================================================================
// ===                    define motor function                 ===
// ================================================================


//using x 0.5 to adjust speed between left and right. in case center of gravity is not center


//right motor
void rightrotforward() {

controller.setMotor1Speed(0.5*OutputR);

 Serial.print(" <R Forward> ");



}

void rightrotbackward()
{

controller.setMotor1Speed(0.5*OutputR);

  Serial.print(" <R Backward> ");




}
void  rightrotstop()
{

controller.setMotor1Speed(0);

Serial.print(" <R stop> ");



}
  //left motor
void leftrotforward() {

controller.setMotor2Speed(0.5*OutputL);

 Serial.print(" <L Forward> ");



}

void leftrotbackward()
{

controller.setMotor2Speed(0.5*OutputL);

  Serial.print(" <L Backward> ");




}
void  leftrotstop()
{
controller.setMotor2Speed(0);

  Serial.print(" <L stop> ");


}


