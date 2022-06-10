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
void MPU_WakeUP(){
		i2c_start(); //transmit START condition
		i2c_write(0b11010000); //transmit SLA + W(0)
		i2c_write(0x6B);
		i2c_write(0x00);
		i2c_stop();
}
int main (void)
{
	DDRD = 0b01001100;
	i2c_init();
	MPU_WakeUP();
	TCCR0A = 0b10000001;
	TCCR0B = 0b00000010;
	
	while(1){
	
	GyroY = read_mpu(GyroXH);
	if (GyroY>=0)
	{
		PORTD= 0b00001000;
		
	}
	else{
		GyroY = (-1) *GyroY;
		PORTD= 0b00000100;
	}
	OCR0A = GyroY/131;
	_delay_ms(100);

}
}


