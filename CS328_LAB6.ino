/***********************************LCD**************************************/
#include <LCD-I2C.h>
//LCD constructor
LCD_I2C lcd(0x27, 16, 2);

/**********************************BLUETOOTH******************************/
#include <SoftwareSerial.h>
#define RX 7
#define TX 6
SoftwareSerial BTSerial(RX, TX); // RX, TX for Bluetooth 



/*******************Motor Variables**********************/
#define MotorPWM_A 4 //left motor
#define MotorPWM_B 5 //right motor

#define INA1A 32
#define INA2A 34
#define INA1B 30
#define INA2B 36

/*******************Program Functions****************************/

void Forward(int speed);
void Stop();
void TurnLeft(int speed);
void MoveSequence(int num, int speed);

/************************GLOBALS***********************/
int originTime;
int num =0;
int speed = random(50,256);

void setup() {
  /*************************BlUETOOTH SETUP*****************************/
  pinMode(RX, INPUT);
  pinMode(TX, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  BTSerial.begin(38400); 
  Serial.begin(9600); 


  /*****************MOTOR SETUP*********************/
    pinMode(MotorPWM_A, OUTPUT);
    pinMode(MotorPWM_B, OUTPUT);
    pinMode(INA1A, OUTPUT);
    pinMode(INA2A, OUTPUT);
    pinMode(INA1B, OUTPUT);
    pinMode(INA2B, OUTPUT);
/****************************************************/

/***************************LCD Setup***********************/
   lcd.begin();
   lcd.display();
   lcd.backlight();
/*************************Program Setup***********************/
delay(2000);
originTime = millis();
MoveSequence(num,speed);

}

void loop() {

int currentTime = millis();
//Check if 5s has elapsed
if((currentTime - originTime) > 5000)
{
  //reset timer
  originTime = millis();
  //correctly cycle through forward, left, stop
  if(num < 2) {num++;}
  else {num = 0;}
//randomly assign speed
 speed = random(50,256);
//perform correct move
  MoveSequence(num, speed);
}

}


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
//Method: TurnLeft, rotate left at given speed
void TurnLeft(int speed){
analogWrite(MotorPWM_A, speed);
    digitalWrite(INA1A, LOW);
    digitalWrite(INA2A, HIGH);

    analogWrite(MotorPWM_B, speed);
    digitalWrite(INA1B, HIGH);
    digitalWrite(INA2B, LOW);
}
//Methood: MoveSequence, performs correct move from index 0-2 num for forward, left, or stop at given speed.
void MoveSequence(int num, int speed){
  switch (num){
    case 0://FORWARD
    Forward(speed);
    //write LCD Data
    lcd.clear();
    delay(75);
    lcd.setCursor(0, 0);
    lcd.print("   Forward "); 
    lcd.setCursor(0, 1);
    lcd.print("Speed: ");
    lcd.print(speed);
    //Write BT Data
    BTSerial.print("Forward, ");
    BTSerial.print(speed);  
     BTSerial.print(", "); 
    
    break;
    case 1://TURN LEFT
    TurnLeft(speed);
    //Write LCD Data
    lcd.clear();
    delay(75);
    lcd.setCursor(0, 0);
    lcd.print("   Turn Left "); 
    lcd.setCursor(0, 1);
    lcd.print("Speed: ");
    lcd.print(speed);
    //Write BT Data
    BTSerial.print("Turn, "); 
    BTSerial.print(speed); 
     BTSerial.print(", "); 
    
    break;
    case 2://STOP
    Stop();
    speed = 0;
    //Write LCD Data
    lcd.clear();
    delay(75);
    lcd.setCursor(0, 0);
    lcd.print("   Stop "); 
    lcd.setCursor(0, 1);
    lcd.print("Speed: ");
    lcd.print(speed);
    //Write BT Data
    BTSerial.print("Stop, "); 
    BTSerial.print(speed); 
    BTSerial.print(", "); 
    
    break;
    default://ERROR
    lcd.clear();
    delay(75);
    lcd.setCursor(0, 0);
    lcd.print("   Error "); 
    break;
  }
}