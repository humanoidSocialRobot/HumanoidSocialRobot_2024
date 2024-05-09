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

#include "main.h"
#include "pca9685.h"
#include "math.h"

I2C_HandleTypeDef *pca9685_i2c;
/**
  * @fn     PCA9685_SetBit.
  * @brief  Sets a specific bit in a register of the PCA9685 device.
  * @param  Register: The address of the register where the bit needs to be set.
  *                  This parameter can be one of the following values:
  *                      @arg @ref PCA9685_SUBADR1           Allows you to set the I2C address that the PCA9685 device responds to when it is in Slave mode.
  *                      @arg @ref PCA9685_SUBADR2           Allows you to set the I2C address that the PCA9685 device responds to when it is in Slave mode.
  *                      @arg @ref PCA9685_SUBADR3           Allows you to set the I2C address that the PCA9685 device responds to when it is in Slave mode.
  *                      @arg @ref PCA9685_MODE1 	         Mode 1 register address (full)
  *                      @arg @ref PCA9685_PRESCALE          Pre-scale register address
  *                      @arg @ref PCA9685_LED0_ON_L         Start address of LEDn_ON_L registers
  *                      @arg @ref PCA9685_LED0_ON_H         Start address of LEDn_ON_H registers
  *                      @arg @ref PCA9685_LED0_OFF_L        Start address of LEDn_OFF_L registers
  *                      @arg @ref PCA9685_LED0_OFF_H        Start address of LEDn_OFF_H registers
  *                      @arg @ref PCA9685_ALLLED_ON_L       All_LED_ON_L register address
  *                      @arg @ref PCA9685_ALLLED_ON_H       All_LED_ON_H register address
  *                      @arg @ref PCA9685_ALLLED_OFF_L      All_LED_OFF_L register address
  *                      @arg @ref PCA9685_ALLLED_OFF_H      All_LED_OFF_H register address
  *                      @arg @ref PCA9685_MODE1_ALCALL_     if a general call address is received on the I2C bus, and the ALCALL and AI bits are set, the PCA9685 device will acknowledge that address.
  *                      @arg @ref PCA9685_MODE1_AI_BIT
  *                      @arg @ref PCA9685_MODE1_SLEEP_BIT   It determines whether the PCA9685 device is in sleep mode or not.
  *                      @arg @ref PCA9685_MODE1_EXTCLK_BIT  It determines whether the PCA9685 device uses an external clock input instead of its internal oscillator.
  *                      @arg @ref PCA9685_MODE1_RESTART_BIT When this bit is set to 1, the device restarts the PWM outputs immediately upon receiving a new value in the LEDn_ON registers. This can result in abrupt changes in PWM output duty cycles.
  *                      @arg @ref MODE2_REGISTER It determines whether the PCA9685 device uses an external clock input instead of its internal oscillator.
  * @param  Bit: The position of the bit to set (0 to 7).
  * @param  Value: The value to set the bit (0 or 1).
  * @retval PCA9685_STATUS: PCA9685_OK if the operation is successful, PCA9685_ERROR otherwise.
  */
PCA9685_STATUS PCA9685_SetBit(uint8_t Register, uint8_t Bit, uint8_t Value)
{
	uint8_t tmp;
    // Convert any non-zero value to 1
	if(Value) Value = 1;
    // Read the current value from the register
	if(HAL_OK != HAL_I2C_Mem_Read(pca9685_i2c, PCA9685_ADDRESS, Register, 1, &tmp, 1, 10))
	{
		return PCA9685_ERROR;
	}
    // Clear the bit position
	tmp &= ~((1<<PCA9685_MODE1_RESTART_BIT)|(1<<Bit));
    // Set the bit to the specified value
	tmp |= (Value&1)<<Bit;
    // Write the modified value back to the register
	if(HAL_OK != HAL_I2C_Mem_Write(pca9685_i2c, PCA9685_ADDRESS, Register, 1, &tmp, 1, 10))
	{
		return PCA9685_ERROR;
	}

	return PCA9685_OK;
}

/**
  * @fn     PCA9685_SoftwareReset
  * @brief  Performs a software reset on the PCA9685 device.
  * @note   This function sends a software reset command to the PCA9685 device, which resets all internal registers to their default values.
  * @retval PCA9685_STATUS: PCA9685_OK if the software reset operation is successful, PCA9685_ERROR otherwise.
  */
PCA9685_STATUS PCA9685_SoftwareReset(void)
{
	uint8_t cmd = 0x6;
	// Send the software reset command via I2C
	if(HAL_OK == HAL_I2C_Master_Transmit(pca9685_i2c, 0x00, &cmd, 1, 10))
	{
		return PCA9685_OK;
	}
	// Return error status if the transmission fails
	return PCA9685_ERROR;
}

/**
  * @fn     PCA9685_SleepMode
  * @brief  Sets the sleep mode of the PCA9685 device.
  * @param  Enable: Specifies whether to enable (1) or disable (0) sleep mode.
  *         This parameter can be one of the following values:
  *             @arg 0: Disable sleep mode (normal operation).
  *             @arg 1: Enable sleep mode.
  * @retval PCA9685_STATUS: PCA9685_OK if the sleep mode setting is successful, PCA9685_ERROR otherwise.
  */
PCA9685_STATUS PCA9685_SleepMode(uint8_t Enable)
{
    // Call PCA9685_SetBit function to set or clear the sleep mode bit in MODE1 register
	return PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, Enable);
}

/**
  * @fn     PCA9685_RestartMode
  * @brief  Sets the restart mode of the PCA9685 device.
  * @param  Enable: Specifies whether to enable (1) or disable (0) the restart mode.
  *         This parameter can be one of the following values:
  *             @arg 0: Disable restart mode (PWM outputs are not restarted immediately).
  *             @arg 1: Enable restart mode (PWM outputs are restarted immediately upon receiving new values in LEDn_ON registers).
  * @retval PCA9685_STATUS: PCA9685_OK if the restart mode setting is successful, PCA9685_ERROR otherwise.
  */
PCA9685_STATUS PCA9685_RestartMode(uint8_t Enable)
{
    // Call PCA9685_SetBit function to set or clear the restart mode bit in MODE1 register
	return PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_RESTART_BIT, Enable);
}

/**
  * @fn     PCA9685_AutoIncrement
  * @brief  Sets the auto-increment mode of the PCA9685 device.
  * @param  Enable: Specifies whether to enable (1) or disable (0) auto-increment mode.
  *         This parameter can be one of the following values:
  *             @arg 0: Disable auto-increment mode (consecutive register accesses are not automatically incremented).
  *             @arg 1: Enable auto-increment mode (consecutive register accesses are automatically incremented).
  * @retval PCA9685_STATUS: PCA9685_OK if the auto-increment mode setting is successful, PCA9685_ERROR otherwise.
  */
PCA9685_STATUS PCA9685_AutoIncrement(uint8_t Enable)
{
    // Call PCA9685_SetBit function to set or clear the auto-increment mode bit in MODE1 register
	return PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_AI_BIT, Enable);
}


/**
  * @fn     PCA9685_SubaddressRespond
  * @brief  Sets the response to a specific subaddress of the PCA9685 device.
  * @param  Subaddress: Specifies the subaddress bit to set or clear.
  *         This parameter can be one of the following values:
  *             @arg PCA9685_SUBADR1: Subaddress bit 1.
  *             @arg PCA9685_SUBADR2: Subaddress bit 2.
  *             @arg PCA9685_SUBADR3: Subaddress bit 3.
  * @param  Enable: Specifies whether to enable (1) or disable (0) response to the specified subaddress.
  *         This parameter can be one of the following values:
  *             @arg 0: Disable response to the specified subaddress.
  *             @arg 1: Enable response to the specified subaddress.
  * @retval PCA9685_STATUS: PCA9685_OK if the subaddress response setting is successful, PCA9685_ERROR otherwise.
  */
PCA9685_STATUS PCA9685_SubaddressRespond(SubaddressBit Subaddress, uint8_t Enable)
{
    // Call PCA9685_SetBit function to set or clear the specified subaddress bit in MODE1 register
	return PCA9685_SetBit(PCA9685_MODE1, Subaddress, Enable);
}

/**
  * @fn     PCA9685_AllCallRespond
  * @brief  Sets the response to the All Call address of the PCA9685 device.
  * @param  Enable: Specifies whether to enable (1) or disable (0) response to the All Call address.
  *         This parameter can be one of the following values:
  *             @arg 0: Disable response to the All Call address.
  *             @arg 1: Enable response to the All Call address.
  * @retval PCA9685_STATUS: PCA9685_OK if the All Call address response setting is successful, PCA9685_ERROR otherwise.
  */
PCA9685_STATUS PCA9685_AllCallRespond(uint8_t Enable)
{
    // Call PCA9685_SetBit function to set or clear the All Call bit in MODE1 register
	return PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_ALCALL_BIT, Enable);
}

//
//	Frequency - Hz value
//
/**
  * @fn     PCA9685_SetPwmFrequency
  * @brief  Sets the PWM frequency of the PCA9685 device.
  * @param  Frequency: The desired PWM frequency to be set (in Hz).
  * @note   The PWM frequency is set by configuring the prescaler value in the PCA9685 device.
  * @retval PCA9685_STATUS: PCA9685_OK if the PWM frequency setting is successful, PCA9685_ERROR otherwise.
  */
PCA9685_STATUS PCA9685_SetPwmFrequency(uint16_t Frequency)
{
	float PrescalerVal;
	uint8_t Prescale;

    // Determine the prescale value based on the desired PWM frequency
	if(Frequency >= 1526)
	{
		Prescale = 0x03;
	}
	else if(Frequency <= 24)
	{
		Prescale = 0xFF;
	}
	else
	{
		PrescalerVal = (25000000 / (4096.0 * (float)Frequency)) - 1;
		Prescale = floor(PrescalerVal + 0.5);
	}

	//	To change the frequency, PCA9685 have to be in Sleep mode.
	PCA9685_SleepMode(1);
    // Write the prescale value to the PCA9685 device
	HAL_I2C_Mem_Write(pca9685_i2c, PCA9685_ADDRESS, PCA9685_PRESCALE, 1, &Prescale, 1, 10); // Write Prescale value
	// Set the PCA9685 device back to Awake mode
	PCA9685_SleepMode(0);
    // Enable restart mode to apply the frequency change immediately
	PCA9685_RestartMode(1);
	return PCA9685_OK;
}

/**
  * @fn     PCA9685_SetPwm
  * @brief  Sets the PWM duty cycle for a specific channel of the PCA9685 device.
  * @param  Channel: The channel number (0 to 15) for which to set the PWM duty cycle.
  * @param  OnTime: The ON time (pulse start) value for the PWM signal (in ticks).
  * @param  OffTime: The OFF time (pulse end) value for the PWM signal (in ticks).
  * @note   The ON time and OFF time values determine the duty cycle of the PWM signal.
  *			Ex: the duty cycle is = (off time - on time) / 4096 = (2048 - 0) / 4096 = 50%
  *
  *         The tick value represents a fraction of the PWM period, with 4096 ticks per period.
  * @retval PCA9685_STATUS: PCA9685_OK if the PWM setting is successful, PCA9685_ERROR otherwise.
  */
PCA9685_STATUS PCA9685_SetPwm(uint8_t Channel, uint16_t OnTime, uint16_t OffTime)
{
	uint8_t RegisterAddress;
	uint8_t Message[4];

    // Calculate the register address for the specified channel
	RegisterAddress = PCA9685_LED0_ON_L + (4 * Channel);
    // Split the OnTime and OffTime values into individual bytes
	Message[0] = OnTime & 0xFF;
	Message[1] = OnTime>>8;
	Message[2] = OffTime & 0xFF;
	Message[3] = OffTime>>8;
    // Write the OnTime and OffTime values to the corresponding LED registers via I2C
	if(HAL_OK != HAL_I2C_Mem_Write(pca9685_i2c, PCA9685_ADDRESS, RegisterAddress, 1, Message, 4, 10))
	{
		return PCA9685_ERROR;
	}

	return PCA9685_OK;
}

/**
  * @fn 	PCA9685_SetPin
  * @brief  Sets the PWM duty cycle for a specific channel of the PCA9685 device based on the provided value.
  * @param  Channel: The channel number (0 to 15) for which to set the PWM duty cycle.
  * @param  Value: The value representing the desired PWM duty cycle for the channel (0 to 4095).
  * @param  Invert: Specifies whether to invert the PWM signal for the channel.
  *         This parameter can be one of the following values:
  *             @arg 0: Do not invert the PWM signal.
  *             @arg 1: Invert the PWM signal.
  * @note   The Value parameter represents the ON time for the PWM signal when not inverted, and the OFF time when inverted.
  * @retval PCA9685_STATUS: PCA9685_OK if the PWM setting is successful, PCA9685_ERROR otherwise.
  */
PCA9685_STATUS PCA9685_SetPin(uint8_t Channel, uint16_t Value, uint8_t Invert)
{

  // Ensure Value is within the valid range (0 to 4095)
  if(Value > 4095) Value = 4095;

  // Determine the PWM duty cycle based on the provided Value and Invert settings
  if (Invert) {
    if (Value == 0) {
      // Special value for signal fully on.
      return PCA9685_SetPwm(Channel, 4096, 0);
    }
    else if (Value == 4095) {
      // Special value for signal fully off.
    	return PCA9685_SetPwm(Channel, 0, 4096);
    }
    else {
    	return PCA9685_SetPwm(Channel, 0, 4095-Value);
    }
  }
  else {
    if (Value == 4095) {
      // Special value for signal fully on.
    	return PCA9685_SetPwm(Channel, 4096, 0);
    }
    else if (Value == 0) {
      // Special value for signal fully off.
    	return PCA9685_SetPwm(Channel, 0, 4096);
    }
    else {
    	return PCA9685_SetPwm(Channel, 0, Value);
    }
  }
}

#ifdef PCA9685_SERVO_MODE
/**
  * @fn     PCA9685_SetServoAngle
  * @brief  Sets the angle for a servo connected to a specific channel of the PCA9685 device.
  * @param  Channel: The channel number (0 to 15) for which to set the servo angle.
  * @param  Angle: The desired angle (in degrees) for the servo (range: MIN_ANGLE to MAX_ANGLE).
  * @note   This function is applicable only in servo mode (PCA9685_SERVO_MODE).
  *         The angle is converted to a corresponding PWM value and then set for the channel.
  * @retval PCA9685_STATUS: PCA9685_OK if the servo angle setting is successful, PCA9685_ERROR otherwise.
  */
PCA9685_STATUS PCA9685_SetServoAngle(uint8_t Channel, float Angle)
{
	float Value;
    // Ensure the Angle parameter is within the valid range
	if(Angle < MIN_ANGLE) Angle = MIN_ANGLE;
	if(Angle > MAX_ANGLE) Angle = MAX_ANGLE;

    // Convert the angle to the corresponding PWM value
	Value = (Angle - MIN_ANGLE) * ((float)SERVO_MAX - (float)SERVO_MIN) / (MAX_ANGLE - MIN_ANGLE) + (float)SERVO_MIN;
    // Set the PWM value for the specified channel
	return PCA9685_SetPin(Channel, (uint16_t)Value, 0);
}
#endif

/**
  * @fn     PCA9685_Init
  * @brief  Initializes the PCA9685 device.
  * @param  hi2c: Pointer to the I2C handle structure.
  * @note   This function initializes the PCA9685 device by performing the following steps:
  *           1. Sets the I2C handle for communication with the PCA9685 device.
  *           2. Performs a software reset on the PCA9685 device.
  *           3. Sets the PWM frequency based on the mode (servo mode or default mode).
  *           4. Enables auto-increment mode for consecutive register accesses.
  * @retval PCA9685_STATUS: PCA9685_OK if the initialization is successful, PCA9685_ERROR otherwise.
  */
PCA9685_STATUS PCA9685_Init(I2C_HandleTypeDef *hi2c)
{
    // Set the I2C handle for communication with the PCA9685 device
	pca9685_i2c = hi2c;

    // Perform a software reset on the PCA9685 device
	PCA9685_SoftwareReset();
    // Set the PWM frequency based on the mode (servo mode or default mode)
#ifdef PCA9685_SERVO_MODE
	PCA9685_SetPwmFrequency(48);
#else
	PCA9685_SetPwmFrequency(1000);
#endif
    // Enable auto-increment mode for consecutive register accesses
	PCA9685_AutoIncrement(1);

	return PCA9685_OK;
}


/**
  * @fn     OpenHand
  * @brief  Opens the hand by setting the angles of servo motors to simulate an open hand gesture.
  * @note   This function sets the angles of servo motors connected to the PCA9685 device to simulate an open hand gesture.
  *         Each servo motor corresponds to a finger of the hand, and the angles are adjusted accordingly to open the hand.
  */
