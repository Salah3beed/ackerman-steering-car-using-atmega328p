/*
 * main.c
 *
 * Created: 5/21/2022 3:50:42 PM
 *  Author: Salah
 */ 

#include <avr/io.h>
#define F_CPU 16000000L
#include <util/delay.h>
#define GyroXH 0x43
#define GyroYH 0x45
#define GyroZH 0x47
int16_t GyroX, GyroY, GyroZ;
int16_t GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;
void i2c_write(unsigned char data)
{
	TWDR = data ;
	TWCR = (1<< TWINT)|(1<<TWEN);
	while ((TWCR & (1 <<TWINT)) == 0);
}
//***************************************************
unsigned char i2c_read(unsigned char isLast)
{
	if (isLast == 0) //send ACK
	TWCR = (1<< TWINT)|(1<<TWEN)|(1<<TWEA); //send ACK
	else
	TWCR = (1<< TWINT)|(1<<TWEN); //send NACK
	while ((TWCR & (1 <<TWINT)) == 0);
	return TWDR;
}

void i2c_start(void)
{
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while ((TWCR & (1 << TWINT)) == 0);
}
void i2c_stop()
{
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
}
void i2c_init(void)
{
	TWSR=0x00; //set prescaler bits to zero
	TWBR=0x0C; //SCL frequency is 50K for XTAL = 16M
	TWCR=0x04; //enable the TWI module
}

int16_t read_mpu(unsigned char address){
	int16_t x;
	i2c_start();
	i2c_write(0b11010000);
	i2c_write(address); //transmit data
	i2c_stop();
	//reading a byte
	i2c_start(); //transmit START condition
	i2c_write(0b11010001); //transmit SLA + R(1)
	x = ((i2c_read(1))<<8); //read one byte of data
	i2c_stop();
	i2c_start();
	i2c_write(0b11010000);
	i2c_write(address+1); //transmit data
	i2c_stop();
	i2c_start(); //transmit START condition
	i2c_write(0b11010001); //transmit SLA + R(1)
	x |= i2c_read(1);
	i2c_stop(); //transmit STOP condition
	return x;
}

void calculate_IMU_error() {
	// We can call this function in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
	// Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
	// Read accelerometer values 200 times
	while (c < 200) {
		GyroX = read_mpu(GyroXH);
		GyroY = read_mpu(GyroYH);
		GyroZ = read_mpu(GyroZH);
		// Sum all readings
		GyroErrorX = GyroErrorX + (GyroX / 131.0);
		GyroErrorY = GyroErrorY + (GyroY / 131.0);
		GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
		c++;
	}
	//Divide the sum by 200 to get the error value
	GyroErrorX = GyroErrorX / 200;
	GyroErrorY = GyroErrorY / 200;
	GyroErrorZ = GyroErrorZ / 200;
}

int main (void)
{
	DDRD = 0b01001100;
	int16_t data = 0;
	i2c_init();
	//writing a byte
	i2c_start(); //transmit START condition
	i2c_write(0b11010000); //transmit SLA + W(0)
	i2c_write(0x6B);
	i2c_write(0x00);
	i2c_stop();
	TCCR0A = 0b10000001;
	TCCR0B = 0b00000010;
	
	while(1){
	
	GyroY = read_mpu(GyroYH) - GyroErrorY;
	if (GyroY>=0)
	{
		PORTD= 0b00001000;
		
	}
	else{
		GyroY = (-1) *GyroY;
		PORTD= 0b00000100;
	}
	OCR0A = GyroY;
	_delay_ms(200);
	calculate_IMU_error();
}
}


