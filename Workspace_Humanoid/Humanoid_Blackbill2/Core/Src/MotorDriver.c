/*
 * MotorDriver.c
 *
 *  Created on: Apr 28, 2024
 *      Author: Salma & Yara
 *
 */

#include "MotorDriver.h"
#include "pca9685.h"
#include "math.h"


/**
  * @fn     MotorDriver_ReleaseObject_OneHand_Right
  * @brief  Opens the right hand by setting the angles of servo motors to simulate an open hand gesture.
  * @param  Motor: Pointer to the MotorConfig_t struct representing the Servo motor to control.
  * @param  CCRX_Value: Value of capture compare registers.
  * @note   This function sets the angles of servo motors connected to the PCA9685 device to simulate an open hand gesture.
  *         Each servo motor corresponds to a finger of the hand, and the angles are adjusted accordingly to open the hand.
  */
void MotorDriver_ReleaseObject_OneHand_Right(MotorConfig_t *Motor, uint16_t CCRX_Value) {
	//open
		  //Pinky Right hand (1)
		   PCA9685_SetServoAngle(PINKY_FINGER_RIGHT_CHANNEL ,  140);
		  //Ring Right hand (2)
		  PCA9685_SetServoAngle(RING_FINGER_RIGHT_CHANNEL,  140);
		  //middle Right hand (3)
		  PCA9685_SetServoAngle(MIDDLE_FINGER_RIGHT_CHANNEL,  140);
		  //index Right hand (4)
		  PCA9685_SetServoAngle(INDEX_FINGER_RIGHT_CHANNEL,  170);
		  //thumb Right hand (5)
<<<<<<< HEAD
		  PCA9685_SetServoAngle(THUMB_FINGER_RIGHT_CHANNEL,  15);
=======
<<<<<<< HEAD
		  PCA9685_SetServoAngle(THUMB_FINGER_RIGHT_CHANNEL,  15);
=======
		  PCA9685_SetServoAngle(ThumbR_channel4,  15);
>>>>>>> 31b1562211f11990ad6a5c449c1d18fd7d17800d
>>>>>>> 3981abae83c7ab2ccf4265e126a3ec4fe737dd7a
		  HAL_Delay(4000);
		  // C elbow right
		 PCA9685_SetServoAngle(12,  80);
		// D rest right
		  PCA9685_SetServoAngle(10,  10);
		  HAL_Delay(1500);
		  //shoulder release

    // A Shoulder IN Right, value =60
//    if (Motor->channel == TIM_CHANNEL_1) {
//        Motor->timer->Instance->CCR1 = CCRX_Value;
//    } else if (Motor->channel == TIM_CHANNEL_2) {
//        Motor->timer->Instance->CCR2 = CCRX_Value;
//    } else if (Motor->channel == TIM_CHANNEL_3) {
//        Motor->timer->Instance->CCR3 = CCRX_Value;
//    } else if (Motor->channel == TIM_CHANNEL_4) {
//        Motor->timer->Instance->CCR4 = CCRX_Value;
//    } else {
//        // Handle invalid channel if necessary
//    }
//    HAL_TIM_PWM_Start(Motor->timer, Motor->channel);
}

/**
  * @fn     MotorDriver_CloseHand
  * @brief  Closes the hand by setting the angles of servo motors to simulate a closed hand gesture.
  * @note   This function sets the angles of servo motors connected to the PCA9685 device to simulate a closed hand gesture.
  *         Each servo motor corresponds to a finger of the hand, and the angles are adjusted accordingly to close the hand.
  */
void MotorDriver_CloseHand(void) {
    // Pinky Right hand (1)
    PCA9685_SetServoAngle(PINKY_FINGER_RIGHT_CHANNEL, 10);
    // Ring Right hand (2)
    PCA9685_SetServoAngle(RING_FINGER_RIGHT_CHANNEL, 0);
    // Middle Right hand (3)
    PCA9685_SetServoAngle(MIDDLE_FINGER_RIGHT_CHANNEL, 0);
    // Index Right hand (4)
    PCA9685_SetServoAngle(INDEX_FINGER_RIGHT_CHANNEL, 75);
    // Thumb Right hand (5)
    PCA9685_SetServoAngle(THUMB_FINGER_RIGHT_CHANNEL, 75);

    // Pinky Left hand (1')
    PCA9685_SetServoAngle(PINKY_FINGER_LEFT_CHANNEL, 180);
    // Ring Left hand (2')
    PCA9685_SetServoAngle(RING_FINGER_LEFT_CHANNEL, 180);
    // Middle Left hand (3')
    PCA9685_SetServoAngle(MIDDLE_FINGER_LEFT_CHANNEL, 0);
    // Index Left hand (4')
    PCA9685_SetServoAngle(INDEX_FINGER_LEFT_CHANNEL, 180);
    // Thumb Left hand (5')
    PCA9685_SetServoAngle(THUMB_FINGER_LEFT_CHANNEL, 90);
}

/**
  * @fn     MotorDriver_ShakeHand
  * @brief  Welcomes with a hand gesture by controlling servo motors.
  * @param  Motor: Pointer to the MotorConfig_t struct representing the Servo motor to control.
  * @param  CCRX_Value: Value of capture compare registers.
  * @note   This function performs the following actions:
  *           - Sets servo angles for the fingers of the right hand to simulate an open hand gesture.
  *           - Sets servo angles for other parts of the right arm to a specific position.
  *           - Sets the PWM duty cycle for the Servo motor to a predefined value based on the specified motor channel.
  *           - Starts the PWM output for the specified Servo motor channel.
  */
void MotorDriver_ShakeHand(MotorConfig_t *Motor, uint16_t CCRX_Value) {
    // A Shoulder IN Right
    if (Motor->channel == TIM_CHANNEL_1) {
        Motor->timer->Instance->CCR1 = CCRX_Value; //90 duty cycle
    } else if (Motor->channel == TIM_CHANNEL_2) {
        Motor->timer->Instance->CCR2 = CCRX_Value;
    } else if (Motor->channel == TIM_CHANNEL_3) {
        Motor->timer->Instance->CCR3 = CCRX_Value;
    } else if (Motor->channel == TIM_CHANNEL_4) {
        Motor->timer->Instance->CCR4 = CCRX_Value;
    } else {
        // Handle invalid channel if necessary
    }
    HAL_TIM_PWM_Start(Motor->timer, Motor->channel);
    	HAL_Delay(5000);
	  //open
	  //Pinky Right hand (1)
<<<<<<< HEAD
	  PCA9685_SetServoAngle( PINKY_FINGER_RIGHT_CHANNEL  ,  140);
=======
<<<<<<< HEAD
	  PCA9685_SetServoAngle( PINKY_FINGER_RIGHT_CHANNEL  ,  140);
	  //Ring Right hand (2)
	  PCA9685_SetServoAngle(RING_FINGER_RIGHT_CHANNEL,  140);
	  //middle Right hand (3)
	  PCA9685_SetServoAngle(MIDDLE_FINGER_RIGHT_CHANNEL,  140);
	  //index Right hand (4)
	  PCA9685_SetServoAngle(INDEX_FINGER_RIGHT_CHANNEL,  170);
	  //thumb Right hand (5)
	  PCA9685_SetServoAngle(THUMB_FINGER_RIGHT_CHANNEL,  15);
=======
	  PCA9685_SetServoAngle( PinkyR_channel1 ,  140);
>>>>>>> 3981abae83c7ab2ccf4265e126a3ec4fe737dd7a
	  //Ring Right hand (2)
	  PCA9685_SetServoAngle(RING_FINGER_RIGHT_CHANNEL,  140);
	  //middle Right hand (3)
	  PCA9685_SetServoAngle(MIDDLE_FINGER_RIGHT_CHANNEL,  140);
	  //index Right hand (4)
	  PCA9685_SetServoAngle(INDEX_FINGER_RIGHT_CHANNEL,  170);
	  //thumb Right hand (5)
<<<<<<< HEAD
	  PCA9685_SetServoAngle(THUMB_FINGER_RIGHT_CHANNEL,  15);
=======
	  PCA9685_SetServoAngle(ThumbR_channel4,  15);
>>>>>>> 31b1562211f11990ad6a5c449c1d18fd7d17800d
>>>>>>> 3981abae83c7ab2ccf4265e126a3ec4fe737dd7a
	  HAL_Delay(1000);
	 //close
	 //pinky Right hand (1)
	 PCA9685_SetServoAngle(1,  10);
	 //Ring Right hand (2)
	 PCA9685_SetServoAngle(0,  0);
	 //middle Right hand (3)
	 PCA9685_SetServoAngle(2,  0);
	 //index Right hand (4)
	 PCA9685_SetServoAngle(3,  75);
	 //thumb Right hand (5)
	 PCA9685_SetServoAngle(4,  75);
	 HAL_Delay(3000);
	 //open
	  //Pinky Right hand (1)
<<<<<<< HEAD
=======
<<<<<<< HEAD
>>>>>>> 3981abae83c7ab2ccf4265e126a3ec4fe737dd7a
		  PCA9685_SetServoAngle( PINKY_FINGER_RIGHT_CHANNEL  ,  140);
		  //Ring Right hand (2)
		  PCA9685_SetServoAngle(RING_FINGER_RIGHT_CHANNEL,  140);
		  //middle Right hand (3)
		  PCA9685_SetServoAngle(MIDDLE_FINGER_RIGHT_CHANNEL,  140);
		  //index Right hand (4)
		  PCA9685_SetServoAngle(INDEX_FINGER_RIGHT_CHANNEL,  170);
		  //thumb Right hand (5)
		  PCA9685_SetServoAngle(THUMB_FINGER_RIGHT_CHANNEL,  15);
<<<<<<< HEAD
=======
	  HAL_Delay(4000);
	  //shoulder release
	    if (Motor->channel == TIM_CHANNEL_1) {
	        Motor->timer->Instance->CCR1 = CCRX_Value-30.00; //60 duty cycle
	    } else if (Motor->channel == TIM_CHANNEL_2) {
	        Motor->timer->Instance->CCR2 = CCRX_Value-30.00;
	    } else if (Motor->channel == TIM_CHANNEL_3) {
	        Motor->timer->Instance->CCR3 = CCRX_Value;
	    } else if (Motor->channel == TIM_CHANNEL_4) {
	        Motor->timer->Instance->CCR4 = CCRX_Value=30.00;
=======
	   PCA9685_SetServoAngle( PinkyR_channel1 ,  140);
	  //Ring Right hand (2)
	  PCA9685_SetServoAngle(RingR_channel0,  140);
	  //middle Right hand (3)
	  PCA9685_SetServoAngle(MiddleR_channel2,  140);
	  //index Right hand (4)
	  PCA9685_SetServoAngle(IndexR_channel3,  170);
	  //thumb Right hand (5)
	  PCA9685_SetServoAngle(ThumbR_channel4,  15);
>>>>>>> 3981abae83c7ab2ccf4265e126a3ec4fe737dd7a
	  HAL_Delay(4000);
	  //shoulder release
	    if (Motor->channel == TIM_CHANNEL_1) {
	        Motor->timer->Instance->CCR1 = CCRX_Value-30.00; //60 duty cycle
	    } else if (Motor->channel == TIM_CHANNEL_2) {
	        Motor->timer->Instance->CCR2 = CCRX_Value-30.00;
	    } else if (Motor->channel == TIM_CHANNEL_3) {
	        Motor->timer->Instance->CCR3 = CCRX_Value;
	    } else if (Motor->channel == TIM_CHANNEL_4) {
<<<<<<< HEAD
	        Motor->timer->Instance->CCR4 = CCRX_Value=30.00;
=======
	        Motor->timer->Instance->CCR4 = CCRX_Value=30;
>>>>>>> 31b1562211f11990ad6a5c449c1d18fd7d17800d
>>>>>>> 3981abae83c7ab2ccf4265e126a3ec4fe737dd7a
	    } else {
	        // Handle invalid channel if necessary
	    }
	    HAL_TIM_PWM_Start(Motor->timer, Motor->channel);

	  HAL_Delay(4000);
}

/**
  * @fn     MotorDriver_HoldObject_OneHand_Right
  * @brief  Holds an object with the right hand by positioning the fingers and arms.
  * @param  Motor: Pointer to the MotorConfig_t struct representing the Servo motor to control.
  * @param  CCRX_Value: Value of capture compare registers.
  * @note   This function performs the following actions:
  *           - Sets servo angles for the fingers of the right hand to a gripping position.
  *           - Sets servo angles for the fingers of the left hand to release the object.
  *           - Sets servo angles for other parts of the arms to specific positions.
  *           - Sets the PWM duty cycle for the specified Servo motor channel to a predefined value.
  *           - Starts the PWM output for the specified Servo motor channel.
  */
void MotorDriver_HoldObject_OneHand_Right(MotorConfig_t *Motor, uint16_t CCRX_Value) {
    // A Shoulder IN Right
    if (Motor->channel == TIM_CHANNEL_1) {
        Motor->timer->Instance->CCR1 = CCRX_Value;  //90 duty cycle
    } else if (Motor->channel == TIM_CHANNEL_2) {
        Motor->timer->Instance->CCR2 = CCRX_Value;
    } else if (Motor->channel == TIM_CHANNEL_3) {
        Motor->timer->Instance->CCR3 = CCRX_Value;
    } else if (Motor->channel == TIM_CHANNEL_4) {
        Motor->timer->Instance->CCR4 = CCRX_Value;
    } else {
        // Handle invalid channel if necessary
    }
    HAL_TIM_PWM_Start(Motor->timer, Motor->channel);


    HAL_Delay(3000);
    // C elbow right
<<<<<<< HEAD
=======
<<<<<<< HEAD
>>>>>>> 3981abae83c7ab2ccf4265e126a3ec4fe737dd7a
	  PCA9685_SetServoAngle(12,  10);
	  // D rest right
	  PCA9685_SetServoAngle(10,  100);
	  HAL_Delay(4000);
<<<<<<< HEAD
    	  //open
    	  //Pinky Right hand (1)
    		  PCA9685_SetServoAngle( PINKY_FINGER_RIGHT_CHANNEL  ,  140);
    		  //Ring Right hand (2)
    		  PCA9685_SetServoAngle(RING_FINGER_RIGHT_CHANNEL,  140);
    		  //middle Right hand (3)
    		  PCA9685_SetServoAngle(MIDDLE_FINGER_RIGHT_CHANNEL,  140);
    		  //index Right hand (4)
    		  PCA9685_SetServoAngle(INDEX_FINGER_RIGHT_CHANNEL,  170);
    		  //thumb Right hand (5)
    		  PCA9685_SetServoAngle(THUMB_FINGER_RIGHT_CHANNEL,  15);
=======
    	  //open
    	  //Pinky Right hand (1)
    		  PCA9685_SetServoAngle( PINKY_FINGER_RIGHT_CHANNEL  ,  140);
    		  //Ring Right hand (2)
    		  PCA9685_SetServoAngle(RING_FINGER_RIGHT_CHANNEL,  140);
    		  //middle Right hand (3)
    		  PCA9685_SetServoAngle(MIDDLE_FINGER_RIGHT_CHANNEL,  140);
    		  //index Right hand (4)
    		  PCA9685_SetServoAngle(INDEX_FINGER_RIGHT_CHANNEL,  170);
    		  //thumb Right hand (5)
    		  PCA9685_SetServoAngle(THUMB_FINGER_RIGHT_CHANNEL,  15);
=======
    	  PCA9685_SetServoAngle(12,  10);
    	  // D rest right
    	  PCA9685_SetServoAngle(10,  100);
    	  HAL_Delay(4000);
    	  //open
    	  //Pinky Right hand (1)
    	   PCA9685_SetServoAngle( PinkyR_channel1 ,  140);
    	  //Ring Right hand (2)
    	  PCA9685_SetServoAngle(RingR_channel0,  140);
    	  //middle Right hand (3)
    	  PCA9685_SetServoAngle(MiddleR_channel2,  140);
    	  //index Right hand (4)
    	  PCA9685_SetServoAngle(IndexR_channel3,  170);
    	  //thumb Right hand (5)
    	  PCA9685_SetServoAngle(ThumbR_channel4,  15);
>>>>>>> 31b1562211f11990ad6a5c449c1d18fd7d17800d
>>>>>>> 3981abae83c7ab2ccf4265e126a3ec4fe737dd7a
    	  HAL_Delay(2000);
       //close
    	 //pinky Right hand (1)
    	 PCA9685_SetServoAngle(1,  10);
    	 //Ring Right hand (2)
    	 PCA9685_SetServoAngle(0,  0);
    	 //middle Right hand (3)
    	 PCA9685_SetServoAngle(2,  0);
    	 //index Right hand (4)
    	 PCA9685_SetServoAngle(3,  75);
    	 //thumb Right hand (5)
    	 PCA9685_SetServoAngle(4,  75);

}

/**
  * @fn     MotorDriver_HoldObject_OneHand_Left
  * @

brief  Holds an object with the left hand by positioning the fingers and arms.
  * @param  Motor: Pointer to the MotorConfig_t struct representing the Servo motor to control.
  * @param  CCRX_Value: Value of capture compare registers.
  * @note   This function performs the following actions:
  *           - Sets servo angles for the fingers of the right hand to a gripping position.
  *           - Sets servo angles for the fingers of the left hand to release the object.
  *           - Sets servo angles for other parts of the arms to specific positions.
  *           - Sets the PWM duty cycle for the specified Servo motor channel to a predefined value.
  *           - Starts the PWM output for the specified Servo motor channel.
  */
void MotorDriver_HoldObject_OneHand_Left(MotorConfig_t *Motor, uint16_t CCRX_Value) {
	 // A Shoulder IN Left, value 90
//	    if (Motor->channel == TIM_CHANNEL_1) {
//	        Motor->timer->Instance->CCR1 = CCRX_Value;
//	    } else if (Motor->channel == TIM_CHANNEL_2) {
//	        Motor->timer->Instance->CCR2 = CCRX_Value;
//	    } else if (Motor->channel == TIM_CHANNEL_3) {
//	        Motor->timer->Instance->CCR3 = CCRX_Value;
//	    } else if (Motor->channel == TIM_CHANNEL_4) {
//	        Motor->timer->Instance->CCR4 = CCRX_Value;
//	    } else {
//	        // Handle invalid channel if necessary
//	    }
//	    HAL_TIM_PWM_Start(Motor->timer, Motor->channel);
//
//	    HAL_Delay(3000);
	    // C elbow left
	   	  PCA9685_SetServoAngle(13,  10);
	   	  // D rest left
	   	  PCA9685_SetServoAngle(15,  100);
	   	  HAL_Delay(4000);
	// Pinky Left hand (1')
    PCA9685_SetServoAngle(PINKY_FINGER_LEFT_CHANNEL, 180);
    // Ring Left hand (2')
    PCA9685_SetServoAngle(RING_FINGER_LEFT_CHANNEL, 180);
    // Middle Left hand (3')
    PCA9685_SetServoAngle(MIDDLE_FINGER_LEFT_CHANNEL, 0);
    // Index Left hand (4')
    PCA9685_SetServoAngle(INDEX_FINGER_LEFT_CHANNEL, 180);
    // Thumb Left hand (5')
    PCA9685_SetServoAngle(THUMB_FINGER_LEFT_CHANNEL, 180);
    HAL_Delay(2000);

    // Close Hand
    // Pinky Left hand (1')
    PCA9685_SetServoAngle(PINKY_FINGER_LEFT_CHANNEL, 0);
    // Ring Left hand (2')
    PCA9685_SetServoAngle(RING_FINGER_LEFT_CHANNEL, 0);
    // Middle Left hand (3')
    PCA9685_SetServoAngle(MIDDLE_FINGER_LEFT_CHANNEL, 180);
    // Index Left hand (4')
    PCA9685_SetServoAngle(INDEX_FINGER_LEFT_CHANNEL, 0);
    // Thumb Left hand (5')
    PCA9685_SetServoAngle(THUMB_FINGER_LEFT_CHANNEL, 90);


}

/**
  * @fn     MotorDriver_HoldObject_TwoHands
  * @brief  Holds an object with both hands by positioning the fingers and arms.
  * @param  Motor_Right: Pointer to the MotorConfig_t struct representing the Right Servo motor to control.
  * @param  Motor_Left: Pointer to the MotorConfig_t struct representing the Left Servo motor to control.
  * @note   This function performs the following actions:
  *           - Sets servo angles for the fingers of both hands to a gripping position.
  *           - Sets servo angles for other parts of the arms to specific positions.
  *           - Sets the PWM duty cycle for the specified Servo motor channels to a predefined value.
  *           - Starts the PWM output for the specified Servo motor channels.
  */
void MotorDriver_HoldObject_TwoHands(MotorConfig_t *Motor_Right, MotorConfig_t *Motor_Left) {
    // Right hand
    // Pinky Right hand (1)
    PCA9685_SetServoAngle(PINKY_FINGER_RIGHT_CHANNEL, 140);
    // Ring Right hand (2)
    PCA9685_SetServoAngle(RING_FINGER_RIGHT_CHANNEL, 140);
    // Middle Right hand (3)
    PCA9685_SetServoAngle(MIDDLE_FINGER_RIGHT_CHANNEL, 140);
    // Index Right hand (4)
    PCA9685_SetServoAngle(INDEX_FINGER_RIGHT_CHANNEL, 170);
    // Thumb Right hand (5)
    PCA9685_SetServoAngle(THUMB_FINGER_RIGHT_CHANNEL, 15);

    // Left hand
    // Pinky Left hand (1')
    PCA9685_SetServoAngle(PINKY_FINGER_LEFT_CHANNEL, 0);
    // Ring Left hand (2')
    PCA9685_SetServoAngle(RING_FINGER_LEFT_CHANNEL, 0);
    // Middle Left hand (3')
    PCA9685_SetServoAngle(MIDDLE_FINGER_LEFT_CHANNEL, 180);
    // Index Left hand (4')
    PCA9685_SetServoAngle(INDEX_FINGER_LEFT_CHANNEL, 0);
    // Thumb Left hand (5')
    PCA9685_SetServoAngle(THUMB_FINGER_LEFT_CHANNEL, 180);

    // Arm positions
    // D Rest Right
    PCA9685_SetServoAngle(REST_RIGHT_CHANNEL, 10);
    // D' Rest Left
    PCA9685_SetServoAngle(REST_LEFT_CHANNEL, 150);
    // C Elbow Right
    PCA9685_SetServoAngle(ELBOW_RIGHT_CHANNEL, 150);
    // C' Elbow Left
    PCA9685_SetServoAngle(ELBOW_LEFT_CHANNEL, 150);
    // B Arm Right
    PCA9685_SetServoAngle(SHOULDER_OUT_RIGHT_CHANNEL, 90);
    // B' Arm Left
    PCA9685_SetServoAngle(SHOULDER_OUT_LEFT_CHANNEL, 50);

    // A Shoulder IN Right
    if (Motor_Right->channel == TIM_CHANNEL_1) {
        Motor_Right->timer->Instance->CCR1 = 45.2;
    } else if (Motor_Right->channel == TIM_CHANNEL_2) {
        Motor_Right->timer->Instance->CCR2 = 45.2;
    } else if (Motor_Right->channel == TIM_CHANNEL_3) {
        Motor_Right->timer->Instance->CCR3 = 45.2;
    } else if (Motor_Right->channel == TIM_CHANNEL_4) {
        Motor_Right->timer->Instance->CCR4 = 45.2;
    } else {
        // Handle invalid channel if necessary
    }
    HAL_TIM_PWM_Start(Motor_Right->timer, Motor_Right->channel);

    // A' Shoulder IN Left
    if (Motor_Left->channel == TIM_CHANNEL_1) {
        Motor_Left->timer->Instance->CCR1 = 45.2;
    } else if (Motor_Left->channel == TIM_CHANNEL_2) {
        Motor_Left->timer->Instance->CCR2 = 45.2;
    } else if (Motor_Left->channel == TIM_CHANNEL_3) {
        Motor_Left->timer->Instance->CCR3 = 45.2;
    } else if (Motor_Left->channel == TIM_CHANNEL_4) {
        Motor_Left->timer->Instance->CCR4 = 45.2;
    } else {
        // Handle invalid channel if necessary
    }
    HAL_TIM_PWM_Start(Motor_Left->timer, Motor_Left->channel);
}
void MotorDriver_OpenHand_RightHand(){
	 //open
	    	  //Pinky Right hand (1)
	    		  PCA9685_SetServoAngle( PINKY_FINGER_RIGHT_CHANNEL  ,  140);
	    		  //Ring Right hand (2)
	    		  PCA9685_SetServoAngle(RING_FINGER_RIGHT_CHANNEL,  140);
	    		  //middle Right hand (3)
	    		  PCA9685_SetServoAngle(MIDDLE_FINGER_RIGHT_CHANNEL,  140);
	    		  //index Right hand (4)
	    		  PCA9685_SetServoAngle(INDEX_FINGER_RIGHT_CHANNEL,  170);
	    		  //thumb Right hand (5)
	    		  PCA9685_SetServoAngle(THUMB_FINGER_RIGHT_CHANNEL,  15);
}
void MotorDriver_OpenHand_LefttHand()
{
        // Pinky Left hand (1')
	    PCA9685_SetServoAngle(PINKY_FINGER_LEFT_CHANNEL, 180);
	    // Ring Left hand (2')
	    PCA9685_SetServoAngle(RING_FINGER_LEFT_CHANNEL, 180);
	    // Middle Left hand (3')
	    PCA9685_SetServoAngle(MIDDLE_FINGER_LEFT_CHANNEL, 0);
	    // Index Left hand (4')
	    PCA9685_SetServoAngle(INDEX_FINGER_LEFT_CHANNEL, 180);
	    // Thumb Left hand (5')
	    PCA9685_SetServoAngle(THUMB_FINGER_LEFT_CHANNEL, 180);


	}
void MotorDriver_CloseHand_RightHand()
{
	          //close
	    	 //pinky Right hand (1)
	    	 PCA9685_SetServoAngle(1,  10);
	    	 //Ring Right hand (2)
	    	 PCA9685_SetServoAngle(0,  0);
	    	 //middle Right hand (3)
	    	 PCA9685_SetServoAngle(2,  0);
	    	 //index Right hand (4)
	    	 PCA9685_SetServoAngle(3,  75);
	    	 //thumb Right hand (5)
	    	 PCA9685_SetServoAngle(4,  75);

}
void MotorDriver_CloseHand_LeftHand(){

	    // Close Hand
	    // Pinky Left hand (1')
	    PCA9685_SetServoAngle(PINKY_FINGER_LEFT_CHANNEL, 0);
	    // Ring Left hand (2')
	    PCA9685_SetServoAngle(RING_FINGER_LEFT_CHANNEL, 0);
	    // Middle Left hand (3')
	    PCA9685_SetServoAngle(MIDDLE_FINGER_LEFT_CHANNEL, 180);
	    // Index Left hand (4')
	    PCA9685_SetServoAngle(INDEX_FINGER_LEFT_CHANNEL, 0);
	    // Thumb Left hand (5')
	    PCA9685_SetServoAngle(THUMB_FINGER_LEFT_CHANNEL, 90);
}
void MotorDriver_OpenHand_TwoHand(){


	       // Pinky Left hand (1')
		    PCA9685_SetServoAngle(PINKY_FINGER_LEFT_CHANNEL, 180);
		    // Ring Left hand (2')
		    PCA9685_SetServoAngle(RING_FINGER_LEFT_CHANNEL, 180);
		    // Middle Left hand (3')
		    PCA9685_SetServoAngle(MIDDLE_FINGER_LEFT_CHANNEL, 0);
		    // Index Left hand (4')
		    PCA9685_SetServoAngle(INDEX_FINGER_LEFT_CHANNEL, 180);
		    // Thumb Left hand (5')
		    PCA9685_SetServoAngle(THUMB_FINGER_LEFT_CHANNEL, 180);

		    //open
	       //Pinky Right hand (1)
		  PCA9685_SetServoAngle( PINKY_FINGER_RIGHT_CHANNEL  ,  140);
		  //Ring Right hand (2)
		  PCA9685_SetServoAngle(RING_FINGER_RIGHT_CHANNEL,  140);
		  //middle Right hand (3)
		  PCA9685_SetServoAngle(MIDDLE_FINGER_RIGHT_CHANNEL,  140);
		  //index Right hand (4)
		  PCA9685_SetServoAngle(INDEX_FINGER_RIGHT_CHANNEL,  170);
		  //thumb Right hand (5)
		  PCA9685_SetServoAngle(THUMB_FINGER_RIGHT_CHANNEL,  15);

}
void MotorDriver_CloseHand_TwoHand(){
	        // Close Hand
		    // Pinky Left hand (1')
		    PCA9685_SetServoAngle(PINKY_FINGER_LEFT_CHANNEL, 0);
		    // Ring Left hand (2')
		    PCA9685_SetServoAngle(RING_FINGER_LEFT_CHANNEL, 0);
		    // Middle Left hand (3')
		    PCA9685_SetServoAngle(MIDDLE_FINGER_LEFT_CHANNEL, 180);
		    // Index Left hand (4')
		    PCA9685_SetServoAngle(INDEX_FINGER_LEFT_CHANNEL, 0);
		    // Thumb Left hand (5')
		    PCA9685_SetServoAngle(THUMB_FINGER_LEFT_CHANNEL, 90);

	          //close
	    	 //pinky Right hand (1)
	    	 PCA9685_SetServoAngle(1,  10);
	    	 //Ring Right hand (2)
	    	 PCA9685_SetServoAngle(0,  0);
	    	 //middle Right hand (3)
	    	 PCA9685_SetServoAngle(2,  0);
	    	 //index Right hand (4)
	    	 PCA9685_SetServoAngle(3,  75);
	    	 //thumb Right hand (5)
	    	 PCA9685_SetServoAngle(4,  75);




}


