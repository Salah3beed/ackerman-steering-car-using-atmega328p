#include <PID_v1.h>
#include <Arduino_FreeRTOS.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#define echoPin A4
#define trigPin A5
#define F_MOTOR_CW 8
#define F_MOTOR_CCW 9
#define B_MOTORS_B 6
#define B_MOTORS_F 7
#define REF_F_MOTOR 15
#define MAX_LEFT_POT 720  // Maximum potentiometer value on the hacked servo when turning left
#define MAX_RIGHT_POT 290 // Maximum potentiometer value on the hacked servo when turning right
#define ALLOWANCE_DISTANCE 30.0 // The distance below which the car would start decrasing its speed
#define ABOUT_TO_CRASH 5  // The distance below which (and included) the car stops its motion
#define REF_INTERVAL 430; // The interval of the potentiometer (MAX_LEFT_POT-MAX_RIGHT_POT)
#define MAX_PWM 255
#define PWM_Pin 4
#define PWM_Pin_Forward 5
int backward;
float pitch;
double roll;
RF24 radio(0, 2); // CE, CSN 

const byte address[6] = "00001";

// Defining a placeholder (structure) to hold all the data to be recieved from the transmitter 
struct Data_Package
{
  int backward;
  float roll = 454;
  float pitch = 0.0;
};

Data_Package data;  // Create a variable with the above structure

double actual = 0.0;
float E = 0.0;
float Kd = 0.0;

// FOR CONTROL 
double Kp = 5;
int Ki = 0;
int Kdd = 0;

double U;
//PID myPID(&actual, &U, &roll,Kp,Ki,Kdd, DIRECT);
long duration;  // variable for the duration of sound wave travel

int distance; // variable for the distance measurement

/*----------------------------------------------------------*/
/*---------------------- Tasks Section ---------------------*/
/*----------------------------------------------------------*/

void UltraSonic(void *pvParameters) // This is an UltraSonic task.
{
  (void) pvParameters;

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT);  // Sets the echoPin as an INPUT

  for (;;)  // A Task shall never return or exit.
  {
    digitalWrite(trigPin, LOW);

    vTaskDelay(0.1 / portTICK_PERIOD_MS);

    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds)

    digitalWrite(trigPin, HIGH);

    vTaskDelay(0.1 / portTICK_PERIOD_MS);

    digitalWrite(trigPin, LOW);

    // Reads the echoPin, returns the sound wave travel time in microseconds

    duration = pulseIn(echoPin, HIGH);

    // Calculating the distance

    distance = duration *0.034 / 2; // Speed of sound wave divided by 2 (go and back) distance

    if (backward == 0)  // checking if the moving is forward motion
    {
      if (distance < ABOUT_TO_CRASH)
      {
        Kd = 0;
      }
      else if (distance < ALLOWANCE_DISTANCE)
      {
        Kd = distance / ALLOWANCE_DISTANCE;
      }
    }
    else
    {
      Kd = 1;
    }

    // Kd will then be used in the left motors PWM
  }
}

void Receive(void *pvParameters)  // This is an NRF recieve task.
{
  (void) pvParameters;
  pinMode(A0, INPUT);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  for (;;)  // A Task shall never return or exit.
  {
    // Serial.println("I'm in the receive");
    // Check whether there is data to be received
    if (radio.available())
    {
      radio.read(&data, sizeof(Data_Package));  // Read the whole data and store it into the 'data' structure
    }

    backward = data.backward;
    pitch = data.pitch;
    roll = data.roll;
    Serial.println("I'm Finished");
    Serial.println(roll);
  }
}

void ServoAngle(void *pvParameters) // This is an ServoHacked task.
{
  (void) pvParameters;

  pinMode(REF_F_MOTOR, INPUT);
  pinMode(PWM_Pin, OUTPUT); /*declare D3 pin as an output pin */
  pinMode(F_MOTOR_CW, OUTPUT);
  pinMode(F_MOTOR_CCW, OUTPUT);
  //myPID.SetMode(AUTOMATIC);

  for (;;)  // A Task shall never return or exit.
  {
    //Kp = map(analogRead(A0), 0, 1023, 0, 40); //This line was used to Online Tune the system using a potentiometer
    actual = analogRead(REF_F_MOTOR);
    //myPID.Compute();  // This was used when we used D and I in the controller but we only were satisfied with P
    E = roll - actual;
    if (E < 0)
    {
      E = E *(-1);
      digitalWrite(F_MOTOR_CCW, LOW);
      digitalWrite(F_MOTOR_CW, HIGH);
    }
    else
    {
      digitalWrite(F_MOTOR_CCW, HIGH);
      digitalWrite(F_MOTOR_CW, LOW);
    }

    U = Kp * E;
    analogWrite(PWM_Pin, U / REF_INTERVAL *MAX_PWM);
    // END SERVO CONTROL

  }
}

void BackMotors(void *pvParameters) // This is an Back Motors task.
{
  (void) pvParameters;
  pinMode(PWM_Pin_Forward, OUTPUT);
  pinMode(B_MOTORS_B, OUTPUT);
  pinMode(B_MOTORS_F, OUTPUT);

  for (;;)  // A Task shall never return or exit. {
    if (backward == 1)
    {
      digitalWrite(B_MOTORS_F, LOW);
      digitalWrite(B_MOTORS_B, HIGH);
      analogWrite(PWM_Pin_Forward, Kd *pitch);

    }
  else
  {
    digitalWrite(B_MOTORS_F, HIGH);
    digitalWrite(B_MOTORS_B, LOW);
    analogWrite(PWM_Pin_Forward, Kd *pitch);
  }

  // Any code below this comment in this task will be read by the ardunio!!!
  //analogWrite(PWM_Pin_Forward, pitch);

  //dtostrf(roll, 5, 2, data.f);  // Converting double into charecter array (for sending with NRF)

  // END BACK MOTORS CONTROL

}

void setup()
{
  Serial.begin(9600);
  while (!Serial);  // Wait for Serial terminal to open port before starting program

  Serial.println("");
  Serial.println("******************************");
  Serial.println("        Program start         ");
  Serial.println("******************************");

  xTaskCreate(
    UltraSonic, "UltraSonic", 128 // Stack size
  , NULL, 2 // priority
  , NULL);

  xTaskCreate(
    Receive, "Receive", 128 // Stack size
  , NULL, 2 // priority
  , NULL);

  xTaskCreate(
    ServoAngle, "ServoAngle", 128 // Stack size
  , NULL, 2 // priority
  , NULL);

  xTaskCreate(
    BackMotors, "BackMotors", 128 // Stack size
  , NULL, 2 // priority
  , NULL);

  vTaskStartScheduler();

}

void loop() {}
