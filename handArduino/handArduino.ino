#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#define echoPin 9 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 4 //attach pin D3 Arduino to pin Trig of HC-SR04
RF24 radio(7, 8); // CE, CSN
void calculate_IMU_error();
const byte address[6] = "00001";
struct Data_Package {
  boolean backward;
  float roll=485;
  float pitch=0.0;
  float yaw;
  float e = 3.141592;
  char f[10] = "Hello There";
};
Data_Package data; //Create a variable with the above structure
float AccX, AccY, AccZ;
float accAngleX, accAngleY;
float AccErrorX, AccErrorY;
float roll;
float pitch;
const int MPU = 0x68; // MPU6050 I2C address
int c = 0;
float ref=0.0;
long duration; // variable for the duration of sound wave travel

int distance; // variable for the distance measurement
void setup() {
  Serial.begin(9600);
    radio.begin();
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MIN);
    radio.stopListening();
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT

}

void loop() {
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI)-AccErrorX; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) -AccErrorY; // AccErrorY ~(-1.58)
  // === Read gyroscope data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  data.roll =  accAngleX;
  data.pitch =  accAngleY;
  
  if(pitch<0){
    data.pitch = -1 * data.pitch;
    data.backward=true;
  }else{
    data.backward=false;
  }
  data.pitch =   map(data.pitch,0,90,0,255);
  data.roll = map(data.roll,-90,90,75,485); 
radio.write(&data, sizeof(Data_Package));
Serial.println(data.pitch);
Serial.println(data.roll);


 digitalWrite(trigPin, LOW);

  //delayMicroseconds(100);

  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds)

  digitalWrite(trigPin, HIGH);

  //delayMicroseconds(100);

  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds

  //duration = pulseIn(echoPin, HIGH);

  // Calculating the distance

  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back) distance

//Serial.println(distance);
  
  
  }

  void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
   
 
}
