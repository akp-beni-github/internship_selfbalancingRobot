


#include <I2C.h>
#include <Wire.h>
#include <MD25.h>

// ================================================================
// ===                          MD25                            ===
// ================================================================

 #define MD25ADDRESS         0x58     
MD25 controller;


///
 int pulseDiffR;
 int pulseDiffL;

const unsigned long timeInterval = 100; //  in milliseconds
unsigned long prevTime = 0;
unsigned int prevPulseCountR = 0;
unsigned int prevPulseCountL = 0;
float rpmR=0;
float rpmL=0;



///






void setup() {

  Serial.begin(4800);





    Wire.begin();


   // Create an instance of the MD25 motor controller
  controller = MD25(0xB0 >> 1);

  // Set the I2C address of the MD25 motor controller
  controller.changeAddress(MD25ADDRESS);

  // Set the mode to Encoder Mode 1
  controller.setMode(1);




    controller.resetEncoders();




}

void loop() {



  unsigned long currentTime; 
  unsigned long elapsedTime;






 //timebase for rpm reader

 currentTime = millis();
 elapsedTime = currentTime - prevTime;
   

   int currentPulseCountR = controller.getEncoder1();
   int currentPulseCountL = controller.getEncoder2();


 


     if (elapsedTime >= timeInterval) {




float pwr=100*(sin(currentTime/1000)>0);
float vt=100*(sin(currentTime/1000)>0); //pwnm since it used to control motor
float vtinrpm= map(vt,0,127,0,200);

    
 
    int currentPulseCountR = controller.getEncoder1();
    //int currentPulseCountL = controller.getEncoder2();

    pulseDiffR = currentPulseCountR - prevPulseCountR;
    //pulseDiffL = currentPulseCountL - prevPulseCountL;


   // pulseDiffpersec= pulseDiff/(elapsedTime/1000);
    rpmR = (pulseDiffR * (60000)) / (elapsedTime*360.0); //*60 to minute  // encoder pulse:360
    //rpmL = (pulseDiffR * (60000)) / (elapsedTime*360.0); //*60 to minute  // encoder pulse:360

    //rpmRinpwm=map(rpmR,0,200,0,127);
    

    prevPulseCountR=currentPulseCountR;
   // prevPulseCountL=currentPulseCountL;

    


///encode pid
float errorderivative;
float eprev;

float kp=10;
float ki=1;
float kd=5;
float e = vtinrpm-rpmR;
float deltaT = ((float) (elapsedTime))/( 1000 );
float eintegral = eintegral + e*deltaT+ e*errorderivative;
 errorderivative = (e-eprev)*deltaT;// deriative
float u = kp*e + ki*eintegral;   


    // motor power
    float vtmotor = (u);
  if( vtmotor > 127 ){
    vtmotor = 127;
  }
    else if( vtmotor < -127 ){
    vtmotor = -127;
  }
  eprev=e;

    controller.setMotor1Speed(vt);  //thru controller > jitter occurs
    controller.setMotor1Speed(vtmotor); // directly witjout pid controller
  



     prevTime=currentTime;

    //erial.print(210);
    //Serial.print(',');
    //Serial.print(-10);
    Serial.print(vtinrpm);
    Serial.print(',');
    Serial.print(rpmR);


    Serial.println();



     }



}

