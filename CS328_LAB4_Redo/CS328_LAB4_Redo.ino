

#define MotorPWM_A 4 //left motor
#define MotorPWM_B 5 //right motor

#define ButtonOne 13

#define INA1A 32
#define INA2A 34
#define INA1B 30
#define INA2B 36
int16_t LeftSpeed = 75;
int16_t RightSpeed = 75;
int16_t PWM_Speed = 0;
int16_t current_Time = 0;


#define EncoderOne 3
#define EncoderTwo 2
static volatile int16_t countOne=0;
static volatile int16_t countTwo=0;
int16_t tickLeft = 0;
int16_t tickRight = 0;

float rotation = 3.125;
float rpmRight=0;
float rpmLeft=0;

void Forward(int &lSpeed, int &rSpeed, int rpmR, int rpmL);
void Forward(int speed);
void Stop();
void Reverse(int speed);
void CounterOne()
{
  countOne++;
  tickRight++;
}
void CounterTwo()
{
    countTwo++;
    tickLeft++;
}
void setup() {

  pinMode(EncoderOne, INPUT_PULLUP);
  pinMode(EncoderTwo, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EncoderOne), CounterOne, FALLING);
  attachInterrupt(digitalPinToInterrupt(EncoderTwo), CounterTwo, FALLING);

    pinMode(MotorPWM_A, OUTPUT);
    pinMode(MotorPWM_B, OUTPUT);
    pinMode(INA1A, OUTPUT);
    pinMode(INA2A, OUTPUT);
    pinMode(INA1B, OUTPUT);
    pinMode(INA2B, OUTPUT);

    pinMode(ButtonOne, INPUT_PULLUP);
    Serial.begin(9600);
    delay(2000);
  
}
void loop(){
 int time  = millis();
 countOne =0;
 countTwo =0;
  delay(50);
//8.17" per tire rotation
//alleged 192 ticks per tire rotation, for us closer to 175??

   Forward(LeftSpeed, RightSpeed, countOne, countTwo);
   if(digitalRead(ButtonOne) == LOW ) {tickLeft = 335; tickRight = 335; }
   if(tickLeft > 800 && tickRight > 800) Forward(0);
    
   Serial.println(countOne);

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