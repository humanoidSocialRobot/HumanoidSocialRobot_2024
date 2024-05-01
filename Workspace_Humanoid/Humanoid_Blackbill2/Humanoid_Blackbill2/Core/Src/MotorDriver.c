/*
 * MotorDriver.c
 *
 *  Created on: Apr 28, 2024
 *      Author: salma & yara
 *
 */


#include "MotorDriver.h"
#include "pca9685.h"
#include "math.h"


/**
  * @fn     OpenHand
  * @brief  Opens the hand by setting the angles of servo motors to simulate an open hand gesture.
  * @note   This function sets the angles of servo motors connected to the PCA9685 device to simulate an open hand gesture.
  *         Each servo motor corresponds to a finger of the hand, and the angles are adjusted accordingly to open the hand.
  */
 void MotorDriver_OpenHandandRelease(MotorConfig_t *Motor , uint16_t CCRX_Value )
 {
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
     HAL_Delay(2000);
     //A Shoulder IN Right  , value =
	if(Motor->channel == 1){
	Motor->timer->Instance->CCR1=CCRX_Value;
	}
	else if(Motor->channel == 2){
	Motor->timer->Instance->CCR2=CCRX_Value;
	}
	else if(Motor->channel == 3){
	Motor->timer->Instance->CCR3=CCRX_Value;
	}
	else if(Motor->channel == 4){
	Motor->timer->Instance->CCR4=CCRX_Value;
	}else{}
	HAL_TIM_PWM_Start(&(Motor->timer), Motor->channel);
     	// D rest right
     	PCA9685_SetServoAngle(10,  10);
     	// C elbow right
     	PCA9685_SetServoAngle(12,  150);
 }

 /**
   * @fn     CloseHand
   * @brief  Closes the hand by setting the angles of servo motors to simulate a closed hand gesture.
   * @note   This function sets the angles of servo motors connected to the PCA9685 device to simulate a closed hand gesture.
   *         Each servo motor corresponds to a finger of the hand, and the angles are adjusted accordingly to close the hand.
   */
 void MotorDriver_CloseHand(void)
  {

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

 	 //pinkey Left hand (1')
 	 PCA9685_SetServoAngle(5,  180);
 	 //Ring Left hand (2')
 	 PCA9685_SetServoAngle(7,  180);
 	 //middle Left hand (3')
 	 PCA9685_SetServoAngle(6,  0);
 	 //index Left hand (4')
 	 PCA9685_SetServoAngle(8,  180);
 	 //thumb Left hand (5')
 	 PCA9685_SetServoAngle(9,  90);

  }

 /**
   * @fn     MotorDriver_WelcomingWithHand
   * @brief  Welcomes with a hand gesture by controlling servo motors.
   * @param  Motor: Pointer to the MotorConfig_t struct representing the Servo motor to control.
   * @note   This function performs the following actions:
   *           - Sets servo angles for the fingers of the right hand to simulate an open hand gesture.
   *           - Sets servo angles for other parts of the right arm to a specific position.
   *           - Sets the PWM duty cycle for the Servo motor to a predefined value based on the specified motor channel.
   *           - Starts the PWM output for the specified Servo motor channel.
   */
 void MotorDriver_ShakeHand(MotorConfig_t *Motor , uint16_t CCRX_Value  ){


	//A Shoulder IN Right
	if(Motor->channel == 1){
	Motor->timer->Instance->CCR1=CCRX_Value;
	}
	else if(Motor->channel == 2){
	Motor->timer->Instance->CCR2=CCRX_Value;
	}
	else if(Motor->channel == 3){
	Motor->timer->Instance->CCR3=CCRX_Value;
	}
	else if(Motor->channel == 4){
	Motor->timer->Instance->CCR4=CCRX_Value;
	}else{}
	HAL_TIM_PWM_Start(&(Motor->timer), Motor->channel);

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
 }


 /**
   * @fn     MotorDriver_HoldObject_OneHand
   * @brief  Holds an object with one hand by positioning the fingers and arms.
   * @param  Motor: Pointer to the MotorConfig_t struct representing the Servo motor to control. TODO ADD PARAMETER
   * @note   This function performs the following actions:
   *           - Sets servo angles for the fingers of the right hand to a gripping position.
   *           - Sets servo angles for the fingers of the left hand to release the object.
   *           - Sets servo angles for other parts of the arms to specific positions.
   *           - Sets the PWM duty cycle for the specified Servo motor channel to a predefined value.
   *           - Starts the PWM output for the specified Servo motor channel.
   */
 void MotorDriver_HoldObject_OneHand(MotorConfig_t *Motor , uint16_t CCRX_Value )
 {
	 	    //A Shoulder IN Right
	        if(Motor->channel == 1){
	 		Motor->timer->Instance->CCR1=CCRX_Value;
	 		}
	 		else if(Motor->channel == 2){
	 		Motor->timer->Instance->CCR2=CCRX_Value;
	 		}
	 		else if(Motor->channel == 3){
	 		Motor->timer->Instance->CCR3=CCRX_Value;
	 		}
	 		else if(Motor->channel == 4){
	 		Motor->timer->Instance->CCR4=CCRX_Value;
	 		}else{}
	 		HAL_TIM_PWM_Start(&(Motor->timer), Motor->channel);
		  // C elbow right
		  PCA9685_SetServoAngle(12,  55);
		  // D rest right
		  PCA9685_SetServoAngle(10,  80);
		  HAL_Delay(3000);
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
		  HAL_Delay(5000);
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
 MotorDriver_HoldObject_OneHand_Left(MotorConfig_t *Motor , uint16_t CCRX_Value )
 {
 // pinkey Left hand (1')
 	  PCA9685_SetServoAngle(5,  180);
 	  //Ring Left hand (2')
 	  PCA9685_SetServoAngle(7,  180);
 	  //middle Left hand (3')
 	  PCA9685_SetServoAngle(6,  0);
 	  //index Left hand (4')
 	  PCA9685_SetServoAngle(8,  180);
 	  //thumb Left hand (5')
 	  PCA9685_SetServoAngle(9,  180);
 	 	HAL_Delay(1000);

 	 	//A Shoulder IN Left  //value 69.5
 	   if(Motor->channel == 1){
 	 	 		Motor->timer->Instance->CCR1=CCRX_Value;
 	 	 		}
 	 	 		else if(Motor->channel == 2){
 	 	 		Motor->timer->Instance->CCR2=CCRX_Value;
 	 	 		}
 	 	 		else if(Motor->channel == 3){
 	 	 		Motor->timer->Instance->CCR3=CCRX_Value;
 	 	 		}
 	 	 		else if(Motor->channel == 4){
 	 	 		Motor->timer->Instance->CCR4=CCRX_Value;
 	 	 		}else{}
 	 	 		HAL_TIM_PWM_Start(&(Motor->timer), Motor->channel);

 	 	HAL_Delay(1000);
 	 	//// D'
 	 	PCA9685_SetServoAngle(11,  30);
 	 	HAL_Delay(1000);
 	 	////Motor C'
 	 	PCA9685_SetServoAngle(13,  40);
 	 	HAL_Delay(3000);
 	 	// pinkey Left hand (1')
 	 	PCA9685_SetServoAngle(5,  0);
 	 	//Ring Left hand (2')
 	 	PCA9685_SetServoAngle(7,  0);
 	 	//middle Left hand (3')
 	 	PCA9685_SetServoAngle(6,  180);
 	 	//index Left hand (4')
 	 	PCA9685_SetServoAngle(8,  0);
 	 	//thumb Left hand (5')
 	 	PCA9685_SetServoAngle(9,  90);
 }

 void MotorDriver_HoldObject_twoHand(MotorConfig_t *Motor_Right,MotorConfig_t *Motor_Left ){


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

 	 //Pinky Left hand (1')
 	 PCA9685_SetServoAngle(PinkyL_channel5,  0);
 	 //Ring Left hand (2')
 	 PCA9685_SetServoAngle(RingL_channel7,  0);
 	 //middle Left hand (3')
 	 PCA9685_SetServoAngle(MiddleL_channel6,  180);
 	 //index Left hand (4')
 	 PCA9685_SetServoAngle(IndexL_channel8,  0);
 	 //thumb Left hand (5')
 	 PCA9685_SetServoAngle(ThumbL_channel9,  180);

    // D rest right
 	PCA9685_SetServoAngle(10,  10);
 	// D'
 	PCA9685_SetServoAngle(11,  150);
 	// C elbow right
 	PCA9685_SetServoAngle(12,  150);
 	// C'
 	PCA9685_SetServoAngle(13,  150);
 	// B arm right
 	PCA9685_SetServoAngle(14,  90);
 	// B' left arm
 	PCA9685_SetServoAngle(15,  50);
 	//A Shoulder IN Right
 	if(Motor_Right->channel == 1){
 		Motor_Right->timer->Instance->CCR1=45.2;
	}
	else if(Motor_Right->channel == 2){
		Motor_Right->timer->Instance->CCR2=45.2;
	}
	else if(Motor_Right->channel == 3){
		Motor_Right->timer->Instance->CCR3=45.2;
	}
	else if(Motor_Right->channel == 4){
		Motor_Right->timer->Instance->CCR4=45.2;
	}else{}
	HAL_TIM_PWM_Start(&(Motor_Right->timer), Motor_Right->channel);
	if(Motor_Left->channel == 1){
		Motor_Left->timer->Instance->CCR1=45.2;
	}
	else if(Motor_Left->channel == 2){
		Motor_Left->timer->Instance->CCR2=45.2;
	}
	else if(Motor_Left->channel == 3){
		Motor_Left->timer->Instance->CCR3=45.2;
	}
	else if(Motor_Left->channel == 4){
		Motor_Left->timer->Instance->CCR4=45.2;
	}else{}
	HAL_TIM_PWM_Start(&(Motor_Left->timer), Motor_Left->channel);
 }

