//*******************************************************************************************************//
// Adam Wilkins & Joseph Somogie
// adam.wilkins@snhu.edu
// joseph.somogie@snhu.edu
// Lab 4 - DC Motor Control
//*******************************************************************************************************//


// Defines for motor and encoder pins
//*******************************************************************************************************//
#define MotorPWM_A 4 // Left motor
#define MotorPWM_B 5 // Right motor

#define INA1A 32 // Left motor input 1
#define INA2A 34 // Left motor input 2
#define INA1B 30 // Right motor input 1
#define INA2B 36 // Right motor input 2

#define EncoderOne 3 // Left Encoder
#define EncoderTwo 2 // Right Encoder
//*******************************************************************************************************//


// Variables Defined
//*******************************************************************************************************//
int16_t PWM_Speed = 0; // Motor speed 0-255

static volatile int16_t countOne=0; // Count for interrupt left encoder
static volatile int16_t countTwo=0; // Count for interrput right encoder

float rotation = 3.125; // Used to calculate rpm of each wheel

float rpmLeft=0; // rpm for left wheel
float rpmRight=0; // rpm for right wheel
//*******************************************************************************************************//


// Function Declarations
//*******************************************************************************************************//
void Forward(int speed);
//*******************************************************************************************************//


// Interrupt functions
//*******************************************************************************************************//
void CounterOne() // Interrupt function for left wheel
{
  countOne++;
}
void CounterTwo() // Interrupt function for right wheel
{
    countTwo++;
}
//*******************************************************************************************************//


// Setup
//*******************************************************************************************************//
void setup() {
  // Input Pinmodes
  //*******************************************************************************************************//
  pinMode(EncoderOne, INPUT_PULLUP);
  pinMode(EncoderTwo, INPUT_PULLUP);
  //*******************************************************************************************************//

  // Output Pinmodes
  //*******************************************************************************************************//
  pinMode(MotorPWM_A, OUTPUT);
  pinMode(MotorPWM_B, OUTPUT);
  pinMode(INA1A, OUTPUT);
  pinMode(INA2A, OUTPUT);
  pinMode(INA1B, OUTPUT);
  pinMode(INA2B, OUTPUT);
  //*******************************************************************************************************//

  // Attach interrups for encoders for left and right
  //*******************************************************************************************************//
  attachInterrupt(digitalPinToInterrupt(EncoderOne), CounterOne, FALLING);
  attachInterrupt(digitalPinToInterrupt(EncoderTwo), CounterTwo, FALLING);
  //*******************************************************************************************************//


  Serial.begin(9600);
  delay(2000);
}
//*******************************************************************************************************//


// Main Loop
//*******************************************************************************************************//
void loop(){
  // If statement to check if the speed is at its max if not increse speed by 5
  //*******************************************************************************************************//
  if(PWM_Speed < 255)
  {
    PWM_Speed+=5;
  }
  //*******************************************************************************************************//

  // Set counts to 0
  //*******************************************************************************************************//
  countOne=0;
  countTwo=0;
  //*******************************************************************************************************//

  delay(100); // Wait 0.1 seconds

  // Calculate rpm for both left and right motors by multiplying interupt count by rotations
  //*******************************************************************************************************//
  rpmRight = countOne*rotation;
  rpmLeft = countTwo*rotation;
  //*******************************************************************************************************//

  Forward(PWM_Speed); // Call forward fucntion with speed to move car forward

  // Prints the values of pwm, rpm left, and rpm right to the serial monitor
  //*******************************************************************************************************//
  Serial.println("PWM: ");
  Serial.print(PWM_Speed);
  Serial.println("RPM Right: ");
  Serial.print(rpmRight);
  Serial.println("RPM Left: ");
  Serial.print(rpmLeft);
  //*******************************************************************************************************//
}
//*******************************************************************************************************//

// Forward function writes to the motor to move both wheels forward
//*******************************************************************************************************//
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
//*******************************************************************************************************//
