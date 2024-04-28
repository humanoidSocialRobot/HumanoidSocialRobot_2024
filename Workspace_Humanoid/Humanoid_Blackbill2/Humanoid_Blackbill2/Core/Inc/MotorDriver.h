/*
 * MotorDriver.h
 *
 *  Created on: Apr 28, 2024
 *      Author: salma & yara
 */

#ifndef INC_MOTORDRIVER_H_
#define INC_MOTORDRIVER_H_
#include "main.h"

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

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
    TIM_HandleTypeDef *timer;  // Pointer to the timer handle
    uint32_t channel;           // Channel number
}MotorConfig;

typedef struct {
    GPIO_TypeDef *enablePort;  // Pointer to the GPIO port for motor enable control
    uint16_t EnableRight;          // Pin number 1 for direction control
    uint16_t EnableLeft;          // Pin number 2 for direction control
    GPIO_TypeDef *ISPort;     // Pointer to the GPIO port for direction control
    uint16_t IS_Right;        // Pin number for motor enable control
    uint16_t IS_Left;        // Pin number for motor enable control
    TIM_HandleTypeDef *timer;  // Pointer to the timer handle for PWM
    uint32_t PWM_Left_Channel;           // Channel number
    uint32_t PWM_Right_Channel;           // Channel number

}Dc_Motor;
/* USER CODE END PTD */



 void CloseHand(void);
 void OpenHand(void);
 void HelloWelcome(void);



#endif /* INC_MOTORDRIVER_H_ */
