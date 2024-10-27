#include <PWMServo.h>

PWMServo servo1;
PWMServo servo2;
PWMServo servo3;
PWMServo servo4;
PWMServo servo5;
PWMServo servo6;
PWMServo servo7;

unsigned long currentMillis;
unsigned long previousMillis = 0;        // set up timers
long interval = 10;                      // time constant for timer
int function;   // check value

float LFB;
float LLR;
float LT;
float LFBFiltered;
float LLRFiltered;
float LTFiltered;

float eyeball;
float eyeballPrev;

float jaw;
float neck1;
float neck2;
float neck3;

int lidFlag = 0;
unsigned long previousLidMillis = 0;        // set up timers

void setup() {
  servo1.attach(2);    // eyeball 
  servo2.attach(3);    // eyeball
  servo3.attach(4);    // lid
  servo4.attach(5);    // lid
  servo5.attach(6);    // jaw
  servo6.attach(7);
  servo7.attach(8);
  
  servo1.write(90);     // default positions
  servo2.write(90);
  servo3.write(85);     // lid: 42 closed, 85 open
  servo4.write(100);    // lid: 148 closed,100 open
  servo5.write(66);     // jaw: 66 closed,120 open 
  servo6.write(90);
  servo7.write(90);
  Serial1.begin(115200);
}

void loop() {

    if (Serial1.available() > 0){           // receive IMU data from serial IMU
        //Serial.println("data");
        function = Serial1.parseInt();
          if (function == 500) {                   // look for check value to check it's the start of the data
              LFB = Serial1.parseFloat();
              LLR = Serial1.parseFloat();
              LT = Serial1.parseFloat();
              if (Serial1.read() == '\n') {     // end of IMU data 
              }
          }
     }

    currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {       // start of timed loop
        previousMillis = currentMillis;

    LFBFiltered = filter(LFB, LFBFiltered,30);
    LLRFiltered = filter(LLR, LLRFiltered,30);
    LTFiltered = filter(LT, LTFiltered,20);

    // filter & scale stick
    eyeball = 90+(LTFiltered/5);
    servo1.write(eyeball-5);
    servo2.write(eyeball-10);

    // look for big changes and blink
    if (lidFlag == 0 && eyeball > eyeballPrev+2.5 || eyeball < eyeballPrev-2.5){
      lidFlag = 1;
    }
    if (lidFlag == 1) {
      servo3.write(42);     // 42 closed, 85 open
      servo4.write(153);    // 153 closed,100 open 
      lidFlag = 2;
      previousLidMillis = currentMillis;           
    }
    else if (lidFlag == 2 && currentMillis - previousLidMillis >= 150) {
      servo3.write(85);     // 42 closed, 85 open
      servo4.write(100);    // 148 closed,100 open 
      lidFlag = 0;    
    } 
    eyeballPrev = eyeball;      // bookmark previous value   

    jaw = abs(LLRFiltered/4);   // jaw servo value
    servo5.write(66 + jaw);     // jaw: 66 closed,120 open

    neck1 = LLRFiltered/4;
    neck2 = LLRFiltered/4;

    neck3 = LFBFiltered/4;

    servo6.write((90-neck1)-neck3);
    servo7.write((90-neck2)+neck3);
     

    }   // end of timed loop 
}


// motion filter to filter motions and compliance

float filter(float prevValue, float currentValue, int filter) {  
  float lengthFiltered =  (prevValue + (currentValue * filter)) / (filter + 1);  
  return lengthFiltered;  
}



