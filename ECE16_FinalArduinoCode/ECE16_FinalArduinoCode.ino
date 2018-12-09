#include <SPI.h>
#include "I2Cdev.h"
//#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <AltSoftSerial.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#if (SSD1306_LCDHEIGHT != 32)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

AltSoftSerial BTserial;

int heartrate_pin = A2; //Pin of heartrate
int motorPin = 3; //Pin for Button

//button variables
int button = 4; // button pin number
int buttonstate;
int last_buttonstate = HIGH; //button hasn't been pressed yet
bool in_sleep = 0; 

//MPU
const int MPU_addr=0x68;  // I2C address of the MPU-6050, can be changed to 0x69
MPU6050 IMU(MPU_addr);

//Interupt Pin
const int interruptPin = 2;
volatile bool ipinReady = false;

String s; //String to Clear Buffer

//IMU Variables
int16_t ax, ay, az, tp, gx, gy, gz;

//Variable for heartrate
unsigned long int heartrate_val = 0;

//motorStart time value
unsigned long int motorStart = 0;

//Sampling Variables 
unsigned long int samplePeriod = 40000; // the target sample period, 5000 microseconds, 200Hz
unsigned long int startTime = 0;
unsigned long int elapsedTime = 0;
unsigned long int currentTime = 0;
unsigned long int lastTime = 0;
bool newRead = false;
bool sending = false;

// Check the interrupt pin to see if there is data available in the MPU's buffer
void interruptPinISR() {
  ipinReady = true;
}

void printData(String heart, String steps, String warning) { 
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.print("Heart Rate: ");
  display.println(heart.toInt());
  display.print("Number of Steps: ");
  display.println(steps.toInt());
  display.print("WARNING: ");
  display.println(warning);
  display.display();
}

void readData() {
  //Serial.println("reading imu");
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);                    // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  
  //Accelerometer (3 Axis)
  ax=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  ay=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  az=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  
  //Temperature
  tp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  
  //Gyroscope (3 Axis)
  gx=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gy=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gz=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
  //heart rate IR pin
  heartrate_val=analogRead(heartrate_pin);
}

//timer 
void pollData() {
  //Serial.println("pollData");
  if (ipinReady && !newRead) {
  //Serial.println("in polldata first if");
    currentTime = micros();
    if ((currentTime - lastTime) >= samplePeriod) {
     // Serial.println("in polldata second if");
      elapsedTime = (currentTime - startTime)/1e3;
      lastTime = currentTime;

      readData();
      newRead = true;
    }
  }  
}

//send all data from IMU and IR
void sendData() {
  BTserial.print(elapsedTime);
  BTserial.print(' ');
  BTserial.print(gx);
  BTserial.print(' ');
  BTserial.print(gy);
  BTserial.print(' ');
  BTserial.print(gz);
  BTserial.print(' ');
  BTserial.print(heartrate_val);
  BTserial.println(' ');

}

//sleep function is its own looping function that polls the button pin
//if the button is high and is staying high, continue looping, if the 
//button has been pressed, send the wakeup commands and send the reconnect
//signal to python
void sleep()
{
    
  while(in_sleep){
    buttonstate = digitalRead(button);

    display.display();
    display.clearDisplay();
      
    if (buttonstate == HIGH) last_buttonstate = HIGH; //register a toggle when button is released 
  
    else if (buttonstate == LOW && last_buttonstate == HIGH ){
      last_buttonstate = LOW;
      in_sleep = 0;
      BTserial.write("AT+qwertyuiopqwertyuiopqwertyuiopqwertyuiopqwertyuiopqwertyuiopqwertyuiopqwertyuiop");
      delay(1000);
      BTserial.write("AT+ADTY0");
      delay(500);
      BTserial.write("AT+RESET");
      delay(1000);
      BTserial.println(-1); //send to python, have python register a -1
      delay(500);
      BTserial.print(-1);
      delay(500);                         //want delays to avoid write-over errors
      s = BTserial.readStringUntil('\n'); // clear any possible buffer
      
    }
  }
}

/* awake 
the main loop send the code here when the wearable is awake */
void awake (){
  pollData();

  sending = true;
  if (newRead && sending) {
   // Serial.println("in firs if in loop");
    sendData();
    newRead = false;
  }

  if (BTserial.available() > 0) {  
    
    String dataFromPython =  BTserial.readStringUntil('\n');
    
    //check if heartbeat is too high - ring motor
    if (dataFromPython == "h") {
      motorStart = millis()
      printData(heart, steps, "Your heart beat is too High!");
      digitalWrite(motorPin, LOW); 
    }
    //check if heartbeat is too low - ring motor
    else if (dataFromPython == "l") { 
      motorStart = millis()
      printData(heart, steps, "Your heart beat is too Low!");
      digitalWrite(motorPin, LOW); 
    }
    
    //functionality that tells you to move
    else if (dataFromPython == "b") {  
      motorStart = millis()
      printData(heart, steps, "Remember to Move!!");
      digitalWrite(motorPin, LOW);    
    }
    
    //can add functionality here for our customizations 
    
    //if all is good, then print heartbeat and steps 
    else {
      //split data for heart rate and step count
      int idx = dataFromPython.indexOf(","); //index to split string
      String steps = dataFromPython.substring(0, idx);
      String heart = dataFromPython.substring(idx+1);
      
      //print both on OLED!
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0,0);
      display.print("Heart Rate: ");
      display.println(heart.toInt());
      display.print("Number of Steps: ");
      display.println(steps.toInt());
      display.display();
    } 
  }
}
void setup(){
  
  //Initialize Button Pin
  pinMode (button, INPUT);
  
  //Initialize Motor Pin
  pinMode(motorPin, OUTPUT);
  digitalWrite(motorPin, HIGH); 

  // Intialize the IMU and the DMP ont he IMU
  IMU.initialize();
  IMU.dmpInitialize();
  IMU.setDMPEnabled(true);

  // Initialize OLED
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  display.clearDisplay();
  
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(MPU_addr);   // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  //Serial.begin(9600);  
  //while (!Serial);
  BTserial.begin(9600); 
  // Set the timer interrupt
  startTime = micros();

  // Create an interrupt for pin2, which is connected to the INT pin of the MPU6050
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), interruptPinISR, RISING);
}

/* Main Loop
this main loop is meant to poll the button while the wearable is in the awake state.
While awake, send to the awake function, if button is pressed, send disconnect message
to python, send sleep commands, and send to the "sleep" loop*/
void loop(){

  buttonstate = digitalRead(button); // check button state
  if (buttonstate == HIGH){    
      last_buttonstate = HIGH;
      awake();
  }
  if ((millis() - motorStart) > 2000) // Stop motor statement 
      digitalWrite(motorPin, HIGH);

  else if (buttonstate == LOW &&  last_buttonstate == HIGH) { //initialize Bluetooth 
        last_buttonstate = LOW;
        in_sleep = 1;
        BTserial.print(-2);
        delay(500);
        BTserial.write("AT");
        delay(500);
        BTserial.write("AT+ADTY3");
        delay(1000);
        BTserial.write("AT+RESET");
        delay(1000);
        BTserial.write("AT+SLEEP");
        sleep();
      }  
  
}
