// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
#define MotorPWM_A 4 //left motor
#define MotorPWM_B 5 //right motor

#define INA1A 32 
#define INA2A 34
#define INA1B 30
#define INA2B 36
#define button_motor 13
#define switch_motor 12
bool button_released = true;
 bool forward = true;
 int motorSpeed = 50;
void setup() {
    pinMode(MotorPWM_A, OUTPUT);
    pinMode(MotorPWM_B, OUTPUT);
    pinMode(INA1A, OUTPUT);
    pinMode(INA2A, OUTPUT);
    pinMode(INA1B, OUTPUT);
    pinMode(INA2B, OUTPUT);
    pinMode(button_motor, INPUT_PULLUP);
    pinMode(switch_motor, INPUT_PULLUP);
}
void loop(){

 if(digitalRead(switch_motor) == LOW){
  forward = true;
 }else if(digitalRead(switch_motor) == HIGH){forward = false;}

  if (digitalRead(button_motor) == LOW && button_released){
    motorSpeed+= 50;
    if(motorSpeed > 255){motorSpeed=0;}
    delay(100);
    button_released = false;
  }else if(digitalRead(button_motor) == HIGH){
    button_released = true;
  }
  Forward(motorSpeed, forward);
}

// Method: Forward
// Input: speed – value [0-255]
// Rotate the motor in a clockwise fashion
void Forward(int speed, bool forward)
{
  analogWrite(MotorPWM_A, speed);
  analogWrite(MotorPWM_B, speed);

  // Left Motor
  if(forward){
  digitalWrite(INA1A, LOW);
  digitalWrite(INA2A, HIGH);
  }else{digitalWrite(INA1A, HIGH);
  digitalWrite(INA2A, LOW);}

  // Right Motor
  digitalWrite(INA1B, HIGH);
  digitalWrite(INA2B, LOW);
}
