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

#define MAX_LEFT_POT 720
#define MAX_RIGHT_POT 290

#define ALLOWANCE_DISTANCE 30.0
#define ABOUT_TO_CRASH  5

#define MAX_PWM 250
#define PWM_Pin 4 /* give PWM_Pin name to D3 pin */
#define PWM_Pin_Forward  5 /* give PWM_Pin name to D5 pin */

RF24 radio(0,2); // CE, CSN
const byte address[6] = "00001";

struct Data_Package {
  boolean backward;
  float roll=485;
  float pitch=0.0;
};

Data_Package data; // Create a variable with the above structure

float actual=0.0;
float E=0.0;
float Kd=0.0;


// FOR CONTROL 
float Kp;
float U=1;


long duration; // variable for the duration of sound wave travel

int distance; // variable for the distance measurement

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  pinMode(PWM_Pin,OUTPUT);  /*declare D3 pin as an output pin */
  pinMode(PWM_Pin_Forward,OUTPUT);  /*declare D3 pin as an output pin */
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(15 ,INPUT);
  pinMode(A0 ,INPUT);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT

}
void loop() {

Kp=map(analogRead(A0),0,1023,0,40);

Kp=5;
//
//Serial.println(Kp);


  // Check whether there is data to be received
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
  }

  Serial.println(data.roll);
  // SERVO CONTROL
  actual = analogRead(REF_F_MOTOR);
//  E=(data.roll)-actual;
//Serial.println(actual);
  E=(data.roll)-actual;


  /* Produce 50% duty cycle PWM on D3 */
  if(E<0){
    E=E*(-1 );
       digitalWrite(F_MOTOR_CCW,LOW);
    digitalWrite(F_MOTOR_CW,HIGH);
  }
  else{
     digitalWrite(F_MOTOR_CCW,HIGH);
    digitalWrite(F_MOTOR_CW,LOW);

  }

  U = Kp * E;
  
  analogWrite(PWM_Pin,(U/1023)*MAX_PWM);


// END SERVO CONTROL

// ULTRA SONIC

 digitalWrite(trigPin, LOW);

  delayMicroseconds(100);

  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds)

  digitalWrite(trigPin, HIGH);

  delayMicroseconds(100);

  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds

  duration = pulseIn(echoPin, HIGH);

  // Calculating the distance

  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back) distance
if(! data.backward){
if(distance<ABOUT_TO_CRASH ){
  Kd=0;
}
else if(distance<ALLOWANCE_DISTANCE){
  Kd = distance/ALLOWANCE_DISTANCE;
}
}
else{
  Kd=1;
}
Serial.println(distance);
//
//Serial.println(Kd);

// END ULTRASONIC

// BACK MOTORS CONTROL
  /* Produce 50% duty cycle PWM on D3 */
  if(data.backward){
    
    digitalWrite(B_MOTORS_F,LOW);
    digitalWrite(B_MOTORS_B,HIGH);

  }
  else{
    digitalWrite(B_MOTORS_F,HIGH);
    digitalWrite(B_MOTORS_B,LOW);

  }
  analogWrite(PWM_Pin_Forward, Kd*data.pitch);

  //dtostrf(roll, 5, 2, data.f); // Converting double into charecter array (for sending with NRF)

// END BACK MOTORS CONTROL
  
}
