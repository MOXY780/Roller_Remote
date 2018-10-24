/******************************************************************************
* File Name          : control.h
* Description        : This file contains the common defines of control
******************************************************************************/
	
#ifndef __CONTROL_H
#define __CONTROL_H

/* Includes ------------------------------------------------------------------*/


/* Define --------------------------------------------------------------------*/
#define FORW		0x01
#define FORWR		0x02
#define RIGHT		0x03
#define BACKR		0x04
#define BACK		0x05
#define BACKL		0x06
#define LEFT		0x07
#define FORWL		0x08
#define HALT		0x09

#define NORMAL	0x1F
#define VIBR		0x20
#define SLOW		0x3F
#define FAST		0x40
#define OFF			0x7F
#define ON			0x80

typedef enum {
		ST_HALT,
		ST_FORW,
		ST_FORWRIGHT,
		ST_RIGHT,
		ST_BACKRIGHT,
		ST_BACK,
		ST_BACKLEFT,
		ST_LEFT,
		ST_FORWLEFT
} TState;

typedef enum {
		B_OFF = 0x00U,
		B_ON  = 0x01U,
} TButtonOnOff;

typedef enum {
		B_SLOW = 0x00U,
		B_FAST = 0x01U,
} TButtonFastSlow;

typedef enum {
		B_NORMAL = 0x00U,
		B_VIBR   = 0x01U,
} TButtonVibr;

typedef struct {
		TState 					Joystick;
		TButtonOnOff 		Ign;
		TButtonFastSlow Drivespeed;
		TButtonVibr			Drivemode;
} TRemote;

typedef enum {
		ST_PRESS_ON,
		ST_PRESS_OFF,
		ST_NOT_PRESS
}	TStateButton; 

typedef struct {
		TStateButton currState;
		TStateButton nextState;
} TStateMachine;

typedef struct {
		TStateMachine	OnOff;
		TStateMachine	Vibr;
		TStateMachine	Speed;
} TButton;

/* Function prototypes -------------------------------------------------------*/
void Remote_Init(TRemote *remote);
void Button_Init(TButton *button);
void Read_Buttons(TRemote *remote, TButton *button);
void Read_Joystick(TRemote *remote);


#endif /* __CONTROL_H */
