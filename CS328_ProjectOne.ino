


/*********************Reset Button***********************/
#define ButtonOne 13
/*******************Motor Variables**********************/
#define MotorPWM_A 4 //left motor
#define MotorPWM_B 5 //right motor

#define INA1A 32
#define INA2A 34
#define INA1B 30
#define INA2B 36

int16_t LeftSpeed = 150;
int16_t RightSpeed = 150;
int16_t PWM_Speed = 0;
int16_t current_Time = 0;

/******************Encoder Variables******************/
#define EncoderOne 3
#define EncoderTwo 2

static volatile int16_t countOne=0;
static volatile int16_t countTwo=0;
int16_t tickLeft = 0;
int16_t tickRight = 0;
float Distance = 0;
/*****************************************************/

int16_t TurnCounter = 0;


float rotation = 3.125;
float rpmRight=0;
float rpmLeft=0;


/*******************Program Functions****************************/
void Forward(int &lSpeed, int &rSpeed, int rpmR, int rpmL);
void Forward(int speed);
void Stop();
void Reverse(int speed);
void TurnLeft(int rotation);
void colorSwitch(int l);

void CounterOne()
{
  countOne++;
  tickRight++;
  Distance++;
}
void CounterTwo()
{
    countTwo++;
    tickLeft++;
    Distance++;
}

/***********************************LCD**************************************/
#include <LCD-I2C.h>
//LCD constructor
LCD_I2C lcd(0x27, 16, 2);
/*************************************LED***********************************/
#include "Adafruit_NeoPixel.h"
//LED Pin
#define led 38
//LED constructor
Adafruit_NeoPixel strip(1, led, NEO_GRB + NEO_KHZ800);
/***************************************************************************/



void setup() {
/***********************ENCODER SETUP**************************************/
  pinMode(EncoderOne, INPUT_PULLUP);
  pinMode(EncoderTwo, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EncoderOne), CounterOne, FALLING);
  attachInterrupt(digitalPinToInterrupt(EncoderTwo), CounterTwo, FALLING);

  /*****************MOTOR SETUP*********************/
    pinMode(MotorPWM_A, OUTPUT);
    pinMode(MotorPWM_B, OUTPUT);
    pinMode(INA1A, OUTPUT);
    pinMode(INA2A, OUTPUT);
    pinMode(INA1B, OUTPUT);
    pinMode(INA2B, OUTPUT);
/****************************************************/

//LCD Init
   lcd.begin();
   lcd.display();
   lcd.backlight();

//LED Init
  strip.begin();
  strip.show();
  strip.setBrightness(10);

//Button setup
  pinMode(ButtonOne, INPUT_PULLUP);
//Serial Communication 
  Serial.begin(9600);
  delay(2000);

    
  
}


void loop(){
  //track time since start
 int time  = millis();
 //Reset encoder counters
 countOne =0;
 countTwo =0;
 //delay for 50ms to get accurate reading
  delay(50);
  //set LED to Green for forward travel
  colorSwitch(3);

  //if we have made less than 4 turns
  if(TurnCounter < 4){
  //go forward at 150
   Forward(LeftSpeed, RightSpeed, countOne, countTwo);
   //if distance on current path is 3 feet (790 ticks)
   if(tickLeft > 790 && tickRight > 790) { 
    //store current distance so it doesnt update while turning
    int x = Distance; 
    //make 90 degree left turn
   TurnLeft(95); 
   //set distance to stored value
   Distance = x; 
   //reset current path distance (ticks)
   tickLeft = 0; tickRight =0; 
   //increment number of turns
   TurnCounter++;
   }

  }
  //if reset button ButtonOne is pressed: wait one second, reset number of turns
  if(digitalRead(ButtonOne) == LOW ) {delay(1000); TurnCounter = 0;}
   Serial.println(countOne);

//if fewer than 4 turns, display the time and distance on the lcd
//Distance/43.89 = inches traveled
if(TurnCounter<4){
   lcd.setCursor(0, 0);
  lcd.print("Time: "); 
  lcd.print(time);
  lcd.setCursor(0, 1);
  lcd.print("Distance: ");
  lcd.print(Distance/43.89);
  }


}

// Method: Forward
// Input: L&R speed – value [0-255] | L&R RPM - value to compare L&R motor speed
// Rotate the motors in a clockwise fashion
void Forward(int &lSpeed, int &rSpeed, int rpmR, int rpmL)
{
if(rpmR > rpmL) {rSpeed--; lSpeed++;}
else if(rpmL > rpmR) {lSpeed--; rSpeed++;}

  analogWrite(MotorPWM_A, lSpeed);
  analogWrite(MotorPWM_B, rSpeed);

  // Left Motor
  digitalWrite(INA1A, HIGH);
  digitalWrite(INA2A, LOW);

  // Right Motor
  digitalWrite(INA1B, HIGH);
  digitalWrite(INA2B, LOW);
}

//Method: Turn Left
//Input: rotation value, or how far to turn
//Rotates right motor normally and left motor in reverse
void TurnLeft(int rotation)
{
  //reset count variable
  countTwo=0;

  Stop();
  //variable to track R,G,B cycle while turning
  int lightCounter = 0;
  //calculate time since start
  int startTime = millis();
  //while encoder ticks are less than given rotation
  while(countTwo < rotation)
  {
    //time calculation for 5hz light cycle
    int currentTime = millis();
    if(currentTime - startTime > 67)
    {
      //reset start time
      startTime = millis();
      //cycle 0-2 for R,G,B
     if(lightCounter < 3) lightCounter++;
     else lightCounter = 0;
      
    }
//Set LED to appropriate color
  colorSwitch(lightCounter);
//rotate motors in opposite directions
    analogWrite(MotorPWM_A, 100);
    digitalWrite(INA1A, LOW);
    digitalWrite(INA2A, HIGH);

    analogWrite(MotorPWM_B, 100);
    digitalWrite(INA1B, HIGH);
    digitalWrite(INA2B, LOW);

  }
  Stop();
}

//Method: Forward
//overloaded forward for non-corrected movement
void Forward(int speed)
{
  analogWrite(MotorPWM_A, speed);
  analogWrite(MotorPWM_B, speed);

  // Left Motor
  digitalWrite(INA1A, HIGH);
  digitalWrite(INA2A, LOW);

  // Right Motor
  digitalWrite(INA1B, HIGH);
  digitalWrite(INA2B, LOW);
}
//Method: Reverse
//non-corrected reverse travel
void Reverse(int speed)
{
  analogWrite(MotorPWM_A, speed);
  analogWrite(MotorPWM_B, speed);

  // Left Motor
  digitalWrite(INA1A, LOW);
  digitalWrite(INA2A, HIGH);

 // Right Motor
  digitalWrite(INA1B, LOW);
  digitalWrite(INA2B, HIGH);
}

//Method: Stop
//Stops both motors by writing 0 to them
void Stop()
{
  // Left Motor
  digitalWrite(INA1A, LOW);
  digitalWrite(INA2A, HIGH);

 // Right Motor
  digitalWrite(INA1B, LOW);
  digitalWrite(INA2B, HIGH);

 analogWrite(MotorPWM_A, 0);
  analogWrite(MotorPWM_B, 0);

}

//Method: Color Switch
//Input: lightCounter, value to track which color to display
//Displays color LED in red, green, blue based on input value
void colorSwitch(int l)
{
  switch(l){
    case 0: strip.setPixelColor(0, strip.Color(255, 0, 0));
    strip.show();
    break;
    case 1:
    strip.setPixelColor(0, strip.Color(0, 255, 0));
    strip.show();
    break;
    case 2:
    strip.setPixelColor(0, strip.Color(0, 0, 255));
    strip.show();
    break;
    case 3:
    strip.setPixelColor(0, strip.Color(0, 255, 0));
    strip.show();
    break;
  }
}