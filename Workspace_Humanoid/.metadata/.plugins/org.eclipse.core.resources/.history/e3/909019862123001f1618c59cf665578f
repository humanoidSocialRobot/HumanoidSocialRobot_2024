/*
 * MotorDriver.h
 *
 *  Created on: Apr 28, 2024
 *      Author: salma & yara
 */

#ifndef INC_MOTORDRIVER_H_
#define INC_MOTORDRIVER_H_
#include "main.h"


/**
 * @file MotorDriver.h
 * Motor Connections to channels identifications
 * @brief Channel definitions for right and left hand fingers, rests, elbows, and shoulders.
 */

// Right Hand Fingers
#define RING_FINGER_RIGHT_CHANNEL 0    ///< Channel for Right Ring Finger
#define PINKY_FINGER_RIGHT_CHANNEL 1   ///< Channel for Right Pinky Finger
#define MIDDLE_FINGER_RIGHT_CHANNEL 2  ///< Channel for Right Middle Finger
#define INDEX_FINGER_RIGHT_CHANNEL 3   ///< Channel for Right Index Finger
#define THUMB_FINGER_RIGHT_CHANNEL 4   ///< Channel for Right Thumb Finger

// Left Hand Fingers
#define PINKY_FINGER_LEFT_CHANNEL 5    ///< Channel for Left Pinky Finger
#define MIDDLE_FINGER_LEFT_CHANNEL 6   ///< Channel for Left Middle Finger
#define RING_FINGER_LEFT_CHANNEL 7     ///< Channel for Left Ring Finger
#define INDEX_FINGER_LEFT_CHANNEL 8    ///< Channel for Left Index Finger
#define THUMB_FINGER_LEFT_CHANNEL 9    ///< Channel for Left Thumb Finger

// Rest Channels
#define REST_RIGHT_CHANNEL 10   ///< Channel for Right Hand Rest  (Label D)
#define REST_LEFT_CHANNEL 11    ///< Channel for Left Hand Rest  (Label D')

// Elbows
#define ELBOW_RIGHT_CHANNEL 12  ///< Channel for Right Elbow (Label C)
#define ELBOW_LEFT_CHANNEL 13   ///< Channel for Left Elbow (Label C')

// Shoulders
#define SHOULDER_OUT_RIGHT_CHANNEL 14  ///< Channel for Right Shoulder Out (Label B)
#define SHOULDER_OUT_LEFT_CHANNEL 15   ///< Channel for Left Shoulder Out (Label B')


/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
    TIM_HandleTypeDef *timer;  // Pointer to the timer handle
    uint32_t channel;           // Channel number
}MotorConfig_t;

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

}Dc_Motor_t;
/* USER CODE END PTD */



 void MotorDriver_CloseHand(void);
 void MotorDriver_OpenHandandRelease(MotorConfig_t *Motor , uint16_t CCRX_Value );
 void MotorDriver_ShakeHand(MotorConfig_t *Motor ,uint16_t CCRX_Value);
 void MotorDriver_HoldObject_OneHand_Right(MotorConfig_t *Motor,uint16_t CCRX_Value );
 void MotorDriver_HoldObject_OneHand_Left(MotorConfig_t *Motor , uint16_t CCRX_Value );
 void MotorDriver_RelaseObject_OneHand_Right(MotorConfig_t *Motor,uint16_t CCRX_Value );
 void MotorDriver_HoldObject_twoHand(MotorConfig_t *Motor_Right,MotorConfig_t *Motor_Left );



#endif /* INC_MOTORDRIVER_H_ */
