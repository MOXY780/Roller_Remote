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
	GPIO_PinState pinOnOff=GPIO_PIN_RESET, pinSpeed=GPIO_PIN_RESET;
	GPIO_PinState pinVibr=GPIO_PIN_RESET, pinVibr2=GPIO_PIN_RESET;
	
//	pinOnOff 		= HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);			// for debugging or if no ON-OF switch is available 
	pinOnOff 	= HAL_GPIO_ReadPin(B_ON_GPIO_Port, B_ON_Pin);
	pinSpeed 	= HAL_GPIO_ReadPin(B_SPEED_GPIO_Port, B_SPEED_Pin);
	pinVibr 	= HAL_GPIO_ReadPin(B_VIBR_GPIO_Port, B_VIBR_Pin);
	pinVibr2 	= HAL_GPIO_ReadPin(B_VIBR2_GPIO_Port, B_VIBR2_Pin);

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
			
		if(pinVibr == GPIO_PIN_SET && pinVibr2 == GPIO_PIN_RESET)
		{
			remote->Drivemode = B_VIBR;
			button->Vibr.currState = ST_PRESS_ON;
			button->Vibr.nextState = ST_PRESS_OFF;
		}
		else if(pinVibr2 == GPIO_PIN_SET && pinVibr == GPIO_PIN_RESET)
		{
			remote->Drivemode = B_VIBR_ALWAYS;
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
	
	#ifdef DEBOUNCE
		static GPIO_PinState tmpN[TEMPLENGTH], tmpE[TEMPLENGTH], tmpS[TEMPLENGTH], tmpW[TEMPLENGTH];
		GPIO_PinState sameN=GPIO_PIN_RESET, sameE=GPIO_PIN_RESET, sameS=GPIO_PIN_RESET, sameW=GPIO_PIN_RESET;
		int i;

		for(i=TEMPLENGTH-1; i>0; i--)		// shift temp arrays to left
		{
			tmpN[i]=tmpN[i-1];
			tmpE[i]=tmpE[i-1];
			tmpS[i]=tmpS[i-1];
			tmpW[i]=tmpW[i-1];
		}
		
		tmpN[0] = HAL_GPIO_ReadPin(DI_S_GPIO_Port, DI_S_Pin);	// all Joystick Pins are mirrored
		tmpE[0] = HAL_GPIO_ReadPin(DI_W_GPIO_Port, DI_W_Pin);	// because contact switches are at
		tmpS[0] = HAL_GPIO_ReadPin(DI_N_GPIO_Port, DI_N_Pin);	// respective invers side
		tmpW[0] = HAL_GPIO_ReadPin(DI_E_GPIO_Port, DI_E_Pin);
		
		for(i=TEMPLENGTH-1; i>0; i--)
		{
			if(tmpN[i]==tmpN[i-1])
				sameN = GPIO_PIN_SET;
			else
				sameN = GPIO_PIN_RESET;
			
			if(tmpE[i]==tmpE[i-1])
				sameE = GPIO_PIN_SET;
			else
				sameE = GPIO_PIN_RESET;
			
			if(tmpS[i]==tmpS[i-1])
				sameS = GPIO_PIN_SET;
			else
				sameS = GPIO_PIN_RESET;
			
			if(tmpW[i]==tmpW[i-1])
				sameW = GPIO_PIN_SET;
			else
				sameW = GPIO_PIN_RESET;
		}
		
		if(sameN==GPIO_PIN_SET)
		{
			if(tmpN[0]==GPIO_PIN_SET)
				pinN = GPIO_PIN_SET;
			else
				pinN = GPIO_PIN_RESET;
		}
		
		if(sameE==GPIO_PIN_SET)
		{
			if(tmpE[0]==GPIO_PIN_SET)
				pinE = GPIO_PIN_SET;
			else
				pinE = GPIO_PIN_RESET;
		}
		
		if(sameS==GPIO_PIN_SET)
		{
			if(tmpS[0]==GPIO_PIN_SET)
				pinS = GPIO_PIN_SET;
			else
				pinS = GPIO_PIN_RESET;
		}
		
		if(sameW==GPIO_PIN_SET)
		{
			if(tmpW[0]==GPIO_PIN_SET)
				pinW = GPIO_PIN_SET;
			else
				pinW = GPIO_PIN_RESET;
		}		
	
	#else
		pinN = HAL_GPIO_ReadPin(DI_S_GPIO_Port, DI_S_Pin);	// all Joystick Pins are mirrored
		pinE = HAL_GPIO_ReadPin(DI_W_GPIO_Port, DI_W_Pin);	// because contact switches are at
		pinS = HAL_GPIO_ReadPin(DI_N_GPIO_Port, DI_N_Pin);	// respective invers side
		pinW = HAL_GPIO_ReadPin(DI_E_GPIO_Port, DI_E_Pin);
	#endif
		
//	if(pinN || pinE || pinS || pinW)
//	{
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
//	}		
//	else
//		remote->Joystick = ST_HALT;
	
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
