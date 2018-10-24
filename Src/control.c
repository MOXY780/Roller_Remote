/******************************************************************************
* File Name          : control.c
* Description        : control program body
******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"
#include "control.h" 
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
#ifdef SIMPLE_BUTTON
	void State_OnOff(TRemote *remote, TButton *button, GPIO_PinState *pinOnOff);
	void State_Vibr(TRemote *remote, TButton *button, GPIO_PinState *pinVibr);
	void State_Speed(TRemote *remote, TButton *button, GPIO_PinState *pinSpeed);
#endif // SIMPLE_BUTTON
/* Functions -----------------------------------------------------------------*/
/**
  * @brief  This function init Struct of Remote.
  * @param  Struct of Remote
  * @retval None
  */
void Remote_Init(TRemote *remote)
{
	remote->Ign = B_OFF;
	remote->Drivespeed = B_SLOW;
	remote->Drivemode = B_NORMAL;
	remote->Joystick = ST_HALT;
	
	return;
}

/**
  * @brief  This function init Struct of Button.
  * @param  Struct of Button
  * @retval None
  */
void Button_Init(TButton *button)
{
	button->OnOff.currState = ST_NOT_PRESS;
	button->OnOff.nextState = ST_PRESS_ON;
	button->Speed.currState = ST_NOT_PRESS;
	button->Speed.nextState = ST_PRESS_ON;
	button->Vibr.currState	= ST_NOT_PRESS;
	button->Vibr.nextState	= ST_PRESS_ON;
	
	return;
}

/**
  * @brief  This function read Buttons of Remote.
  * @param  Struct of Remote
	* @param  Struct of States of single Buttons
  * @retval None
  */
void Read_Buttons(TRemote *remote, TButton *button)
{
	GPIO_PinState pinOnOff, pinSpeed, pinVibr;
	
//	pinOnOff 		= HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);			// for debugging or if no ON-OF switch is available 
	pinOnOff 	= HAL_GPIO_ReadPin(B_ON_GPIO_Port, B_ON_Pin);
	pinSpeed 	= HAL_GPIO_ReadPin(B_SPEED_GPIO_Port, B_SPEED_Pin);
	pinVibr 	= HAL_GPIO_ReadPin(B_VIBR_GPIO_Port, B_VIBR_Pin);

	#ifdef SIMPLE_BUTTON
		State_OnOff(remote, button, &pinOnOff);
		State_Vibr(remote, button, &pinVibr);
		State_Speed(remote, button, &pinSpeed);
	#else									// use if ON-ON switches are used 
		if(pinOnOff == GPIO_PIN_SET)
		{
			remote->Ign = B_ON;
			button->OnOff.currState = ST_PRESS_ON;
			button->OnOff.nextState = ST_PRESS_OFF;
		}
		else
		{
			remote->Ign = B_OFF;
			button->OnOff.currState = ST_PRESS_OFF;
			button->OnOff.nextState = ST_PRESS_ON;
		}

		if(pinSpeed == GPIO_PIN_SET)
		{
			remote->Drivespeed = B_FAST;
			button->Speed.currState = ST_PRESS_ON;
			button->Speed.nextState = ST_PRESS_OFF;
		}
		else
		{
			remote->Drivespeed = B_SLOW;
			button->Speed.currState = ST_PRESS_OFF;
			button->Speed.nextState = ST_PRESS_ON;
		}
			
		if(pinVibr == GPIO_PIN_SET)
		{
			remote->Drivemode = B_VIBR;
			button->Vibr.currState = ST_PRESS_ON;
			button->Vibr.nextState = ST_PRESS_OFF;
		}
		else
		{
			remote->Drivemode = B_NORMAL;
			button->Vibr.currState = ST_PRESS_OFF;
			button->Vibr.nextState = ST_PRESS_ON;
		}
		
		return;
	#endif // SIMPLE_BUTTON
}


/**
  * @brief  This function read GPIOs of Joystick.
  * @param  Struct of Remote
  * @retval Direction of Joystick (N, NE, E, SE, S, SW, W, HALT)
  */
void Read_Joystick(TRemote *remote)
{
	GPIO_PinState pinN, pinE, pinS, pinW;
	
	pinN = HAL_GPIO_ReadPin(DI_S_GPIO_Port, DI_S_Pin);	// all Joystick Pins are mirrored
	pinE = HAL_GPIO_ReadPin(DI_W_GPIO_Port, DI_W_Pin);	// because contact switches are at
	pinS = HAL_GPIO_ReadPin(DI_N_GPIO_Port, DI_N_Pin);	// respective invers side
	pinW = HAL_GPIO_ReadPin(DI_E_GPIO_Port, DI_E_Pin);
	
	if(pinN || pinE || pinS || pinW)
	{
		if(pinN && !pinE && !pinS && !pinW)
			remote->Joystick = ST_FORW;
		else if (pinN && pinE && !pinS && !pinW)
			remote->Joystick = ST_FORWRIGHT;
		else if (!pinN && pinE && !pinS && !pinW)
			remote->Joystick = ST_RIGHT;
		else if (!pinN && pinE && pinS && !pinW)
			remote->Joystick = ST_BACKRIGHT;
		else if (!pinN && !pinE && pinS && !pinW)
			remote->Joystick = ST_BACK;
		else if (!pinN && !pinE && pinS && pinW)
			remote->Joystick = ST_BACKLEFT;
		else if (!pinN && !pinE && !pinS && pinW)
			remote->Joystick = ST_LEFT;
		else if (pinN && !pinE && !pinS && pinW)
			remote->Joystick = ST_FORWLEFT;
		else
			remote->Joystick = ST_HALT;
	}		
	else
		remote->Joystick = ST_HALT;
	
	return;
}

#ifdef SIMPLE_BUTTON
	void State_OnOff(TRemote *remote, TButton *button, GPIO_PinState *pinOnOff)
	{
		switch(button->OnOff.currState)				// use if simple buttons are used instead of ON-ON switches
		{
			case ST_NOT_PRESS:
				if(*pinOnOff == GPIO_PIN_SET)
				{
					button->OnOff.currState = button->OnOff.nextState;
					button->OnOff.nextState = ST_NOT_PRESS;
				}
				return;
			case ST_PRESS_ON:
				if(*pinOnOff == GPIO_PIN_RESET)
				{
					remote->Ign = B_ON;
					button->OnOff.currState = button->OnOff.nextState;
					button->OnOff.nextState = ST_PRESS_OFF;
				}
				return;
			case ST_PRESS_OFF:
				if(*pinOnOff == GPIO_PIN_RESET)
				{
					remote->Ign = B_OFF;
					button->OnOff.currState = button->OnOff.nextState;
					button->OnOff.nextState = ST_PRESS_ON;
				}
				return;
			default:																	// Error Case
				remote->Ign = B_OFF;
				button->OnOff.currState = ST_NOT_PRESS;
				button->OnOff.nextState = ST_PRESS_ON;
				return;
		}
	}
	
	void State_Vibr(TRemote *remote, TButton *button, GPIO_PinState *pinVibr)
	{
		switch(button->Vibr.currState)			
		{
			case ST_NOT_PRESS:
				if(*pinVibr == GPIO_PIN_SET)
				{
					button->Vibr.currState = button->Vibr.nextState;
					button->Vibr.nextState = ST_NOT_PRESS;
				}
				return;
			case ST_PRESS_ON:
				if(*pinVibr == GPIO_PIN_RESET)
				{
					remote->Drivemode = B_VIBR;
					button->Vibr.currState = button->Vibr.nextState;
					button->Vibr.nextState = ST_PRESS_OFF;
				}
				return;
			case ST_PRESS_OFF:
				if(*pinVibr == GPIO_PIN_RESET)
				{
					remote->Drivemode = B_NORMAL;
					button->Vibr.currState = button->Vibr.nextState;
					button->Vibr.nextState = ST_PRESS_ON;
				}
				return;
			default:																	// Error Case
				remote->Drivemode = B_NORMAL;
				button->Vibr.currState = ST_NOT_PRESS;
				button->Vibr.nextState = ST_PRESS_ON;
				return;
		}
	}
	
	void State_Speed(TRemote *remote, TButton *button, GPIO_PinState *pinSpeed)
	{
		switch(button->Speed.currState)			
		{
			case ST_NOT_PRESS:
				if(*pinSpeed == GPIO_PIN_SET)
				{
					button->Speed.currState = button->Speed.nextState;
					button->Speed.nextState = ST_NOT_PRESS;
				}
				return;
			case ST_PRESS_ON:
				if(*pinSpeed == GPIO_PIN_RESET)
				{
					remote->Drivespeed = B_FAST;
					button->Speed.currState = button->Speed.nextState;
					button->Speed.nextState = ST_PRESS_OFF;
				}
				return;
			case ST_PRESS_OFF:
				if(*pinSpeed == GPIO_PIN_RESET)
				{
					remote->Drivespeed = B_SLOW;
					button->Speed.currState = button->Speed.nextState;
					button->Speed.nextState = ST_PRESS_ON;
				}
				return;
			default:																	// Error Case
				remote->Drivespeed = B_SLOW;
				button->Speed.currState = ST_NOT_PRESS;
				button->Speed.nextState = ST_PRESS_ON;
				return;
		}
	}
#endif // SIMPLE_BUTTON
