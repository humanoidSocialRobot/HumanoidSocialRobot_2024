/*
 * pca9685.c
 *
 *  Created on: 20.01.2019
 *      Author: Mateusz Salamon
 *		mateusz@msalamon.pl
 *      Documented & edited on: 24/4/2024
 *      Documented & edited By: Yara Mohamed\Salma Metwally
 *
 */

#ifndef PCA9685_H_
#define PCA9685_H_

// Motor Connections to channels identifications

#define RingR_channel0 0
#define PinkyR_channel1 1
#define MiddleR_channel2 2
#define IndexR_channel3 3
#define ThumbR_channel4 4

#define PinkyL_channel5 5
#define MiddleL_channel6 6
#define RingL_channel7 7
#define IndexL_channel8 8
#define ThumbL_channel9 9

#define RestR_channel10 10   //LABEL D
#define RestL_channel11 11   //LABEL D'
#define ElbowR_channel12 12  //LABEL C
#define ElbowL_channel13 13  //LABEL C'
#define ShoulderOutR_channel14 14  //LABEL B
#define ShoulderOutL_channel15 15  //LABEL B'



//
//	Enable Servo control mode
//
#define PCA9685_SERVO_MODE

#ifdef PCA9685_SERVO_MODE
//
//	Servo min and max values for TURINGY TG9e Servos
//
#define SERVO_MIN	110
#define SERVO_MAX	500
#define MIN_ANGLE	0.0
#define MAX_ANGLE	180.0
#endif

//
//	Adjustable address 0x80 - 0xFE
//
#define PCA9685_ADDRESS 0x50 << 1

//
//	Registers
//
#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

#define PCA9685_MODE1 		0x0
#define PCA9685_PRESCALE 	0xFE

#define PCA9685_LED0_ON_L 	0x6
#define PCA9685_LED0_ON_H	0x7
#define PCA9685_LED0_OFF_L	0x8
#define PCA9685_LED0_OFF_H 	0x9

#define PCA9685_ALLLED_ON_L 	0xFA
#define PCA9685_ALLLED_ON_H 	0xFB
#define PCA9685_ALLLED_OFF_L 	0xFC
#define PCA9685_ALLLED_OFF_H 	0xFD

#define PCA9685_MODE1_ALCALL_BIT	0
typedef enum
{
	PCA9685_MODE1_SUB1_BIT 	= 3,
	PCA9685_MODE1_SUB2_BIT	= 2,
	PCA9685_MODE1_SUB3_BIT	= 1
}SubaddressBit;
#define PCA9685_MODE1_SLEEP_BIT		4
#define PCA9685_MODE1_AI_BIT		5
#define PCA9685_MODE1_EXTCLK_BIT	6
#define PCA9685_MODE1_RESTART_BIT	7


typedef enum
{
	PCA9685_OK 		= 0,
	PCA9685_ERROR	= 1
}PCA9685_STATUS;

PCA9685_STATUS PCA9685_SoftwareReset(void);
PCA9685_STATUS PCA9685_SleepMode(uint8_t Enable);
PCA9685_STATUS PCA9685_RestartMode(uint8_t Enable);
PCA9685_STATUS PCA9685_AutoIncrement(uint8_t Enable);

#ifndef PCA9685_SERVO_MODE
PCA9685_STATUS PCA9685_SetPwmFrequency(uint16_t Frequency);
#endif

PCA9685_STATUS PCA9685_SetPwm(uint8_t Channel, uint16_t OnTime, uint16_t OffTime);
PCA9685_STATUS PCA9685_SetPin(uint8_t Channel, uint16_t Value, uint8_t Invert);
#ifdef PCA9685_SERVO_MODE
PCA9685_STATUS PCA9685_SetServoAngle(uint8_t Channel, float Angle);
#endif

PCA9685_STATUS PCA9685_Init(I2C_HandleTypeDef *hi2c);

#endif /* PCA9685_H_ */
