// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
#define MotorPWM_A 4 //left motor
#define MotorPWM_B 5 //right motor

#define INA1A 32
#define INA2A 34
#define INA1B 30
#define INA2B 36
int16_t LeftSpeed = 200;
int16_t RightSpeed = 200;
int16_t PWM_Speed = 0;



#define EncoderOne 3
#define EncoderTwo 2
static volatile int16_t countOne=0;
static volatile int16_t countTwo=0;
float rotation = 3.125;
float rpmRight=0;
float rpmLeft=0;

void Forward(int &lSpeed, int &rSpeed, int rpmR, int rpmL);
void Forward(int speed);

void CounterOne()
{
  countOne++;
}
void CounterTwo()
{
    countTwo++;
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

    Serial.begin(9600);
    delay(2000);
     //Forward(100);
}
void loop(){
 
  if(PWM_Speed < 255){PWM_Speed+=5;}
   countOne=0;
   countTwo=0;
   delay(100);
    rpmRight = countOne*rotation;
    rpmLeft = countTwo*rotation;
   //Forward(LeftSpeed, RightSpeed, countOne, countTwo);
    Forward(PWM_Speed);
    Serial.println("PWM: ");
    Serial.print(PWM_Speed);
    Serial.println("RPM Right: ");
    Serial.print(rpmRight);
    Serial.println("RPM Left: ");
    Serial.print(rpmLeft);
}
// Method: Forward
// Input: speed – value [0-255]
// Rotate the motor in a clockwise fashion
void Forward(int &lSpeed, int &rSpeed, int rpmR, int rpmL)
{
if(rpmR > rpmL) {--rSpeed; ++lSpeed;}
else if(rpmL > rpmR) {--lSpeed; ++rSpeed;}

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
