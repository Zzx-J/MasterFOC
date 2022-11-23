/** 
* @file		
* @brief 		
* @author 		Alfred John (Zzx)
* @date 		
* @version 	1.0
* @par Copyright(c): 	Alfred John (Zzx)
* @par History:         
*	version: 1.0	Alfred John (Zzx)	\n
*/

#ifndef __CONTROL_H
#define __CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif


#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "tim.h"
#include "stdio.h"

#define pi 3.14159265359f
#define SQRT_3 1.7320508f
#define UnitPulse 4096 //pulses from encoder per cycle(2pi)
//#define BLOCK_SIZE 1        
//#define NUM_TAPS 6

/** 
* @brief	Coordinates components structure
* @details	abc, alpha/beta, d/q
*/
typedef struct Components
{
	int16_t a, b, c;		/*!< abc components*/
	float alpha, beta;	/*!< alpha beta components*/
	float d, q;					/*!< dq components*/
}Coordinates;

/**
* @brief	PID parameter structure
* @details	pid parameters and target, error, previous error, and sum of error
*/
typedef struct PID_para{
	float kp, ki, kd;											/*!< pid parameters*/
	float target, err, err_pre, err_sum;	/*!< target, error, previous error, and sum of error*/
}PID_para;

/**
* @brief	ControlMode enumerator
* @details	to set the operation mode of the FOC driver
*/
typedef enum ControlMode
{	Idle = 0, Speed, Position, Torque }ControlMode;

/**
* @brief	UserAPP enumerator
* @details	to set different user applications of the FOC driver
*/
typedef enum UserAPP
{	Drag = 1, ForceFeedback, Switch, SpaceWalk }UserAPP;

/**
* @brief	DRV8302 amplification gain enumerator
* @details	10V/V or 40V/V
*/
typedef enum DRV_GAIN
{	DRV_GAIN_10 = 0, DRV_GAIN_40}DRV_GAIN;
/**
* @brief	Motor parameter structure
* @details	pid parameters and target, error, previous error, and sum of error
*/
typedef struct
{
	ADC_HandleTypeDef hMotorADC;		/*!< ADC handler for motor instance*/
	TIM_HandleTypeDef hTimDriver;		/*!< Driver timer handler for motor instance*/
	TIM_HandleTypeDef hTimEncoder;	/*!< PWM Encoder timer handler for motor instance*/
	I2C_HandleTypeDef hI2CEncoder;	/*!< I2C encoder handler for motor instance*/
	
	ControlMode Mode;								/*!< current control mode for motor instance*/
	UserAPP APP;										/*!< current user application for motor instance*/
	
	Coordinates Voltage;						/*!< voltage components for motor instance*/
	Coordinates Current;						/*!< current components for motor instance*/
	
	PID_para idPID;									/*!< Id PID parameters for motor instance*/
	PID_para iqPID;									/*!< Iq PID parameters for motor instance*/
	PID_para speedPID;							/*!< speed PID parameters for motor instance*/
	PID_para anglePID;							/*!< angle PID parameters for motor instance*/
	
	uint8_t nCalibrationFlag;				/*!< flag to record motor calibration status*/
	uint8_t Vdc;										/*!< DC power supply for motor instance*/
	uint8_t nPhase;									/*!< number of phase of the motor*/
	uint8_t nPolePairs;							/*!< number of pole pairs of the motor*/

	int16_t SpeedCNT;								/*!< Counter to record mechanical angle difference to calculate speed*/
	uint16_t M_angle;								/*!< Current mechanical angle*/
	uint16_t M_angle_pre;						/*!< Previous mechanical angle*/
	int16_t E_angle;								/*!< Current electrical angle*/
	int16_t E_angle_pre;						/*!< Previous electrical angle*/
	uint16_t Angle_offset;					/*!< Difference between encoder 0 deg and motor electrical 0 deg(phase a 100%)*/
	
	float speed, speed_pre;					/*!< Current and previous motor speed*/		
	float CurrentLimit, VoltageLimit, SpeedLimit;	/*!< Current, Volatge, Speed limit of the motor*/		
	int32_t lapCNT,lapCNT_pre;			/*!< Current and previous lap counter (count the lap of the motor)*/
	uint8_t sector;									/*!< sector of space voltage vector*/
	float Uref1, Uref2, Uref3;			/*!< sector of space voltage vector*/
	float X, Y, Z;									/*!< intermediate variables of dwelling time calculation*/
	float T1,T2,Tx,Ty,Tz;						/*!< intermediate variables of dwelling time calculation*/
	float Ta, Tb, Tc;								/*!< intermediate variables of dwelling time calculation*/

	uint16_t ADC_DMA_RAW[3];				/*!< raw data of phase current sample*/
	uint16_t DC_offset[2];					/*!< DC offset of the current shunt amplifiler, used for DC calibration*/
	int16_t ADC_Filtered[3];				/*!< filtered data of phase current sample*/
	//int16_t ADC_lastValue[3];			
	int16_t Current_lastValue[5];		/*!< [0,1,2] previous current sample of phase a,b,c [3,4] previous id,iq sample*/
	
}Motor;


/*! \fn void MotorInit(Motor* mHandle,ADC_HandleTypeDef hadc,TIM_HandleTypeDef hDriver, TIM_HandleTypeDef hEncoder, I2C_HandleTypeDef hIICEncoder, ControlMode mode)
* @brief			:	Motor initialization function
* @param[in]	:	mHandle	motor instance handler
* @param[in]	:	hadc	current sampling adc handler
* @param[in]	:	hDriver	driver timer handler (set the control frequency)
* @param[in]	:	hEncoder	PWM encoder timer handler
* @param[in]	:	hIICEncoder	I2C encoder handler
* @param[in]	:	mode	initial operation mode
* @return 		:	void
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void MotorInit(Motor* mHandle,ADC_HandleTypeDef hadc,TIM_HandleTypeDef hDriver, TIM_HandleTypeDef hEncoder, I2C_HandleTypeDef hIICEncoder, ControlMode mode);

/*! \fn void ModeSelection(Motor* mHandle,ControlMode mode)
* @brief			:	Mode selection function
* @param[in]	:	mHandle	motor instance handler
* @param[in]	:	mode	operation mode
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void ModeSelection(Motor* mHandle,ControlMode mode);


/*! \fn void UserAPPSelection(Motor* mHandle,UserAPP app);
* @brief			:	user application selection function
* @param[in]	:	mHandle	motor instance handler
* @param[in]	:	app	user application
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void UserAPPSelection(Motor* mHandle,UserAPP app);

/*! \fn void EnableGate(void);
* @brief			:	Pull up the ENGATE pin of the DRV8302 to activate the gate
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void EnableGate(void);

/*! \fn void GainSelection(DRV_GAIN gain);
* @brief			:	Select the gain of the buit-in amplifier of DRV8302
								If GAIN = LOW, the internal current shunt amplifiers have a gain of 10V/V. If GAIN = HIGH, the current shunt amplifiers have a gain of 40V/V
* @param[in]	:	gain	DRV_GAIN_10, DRV_GAIN_40		
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void GainSelection(DRV_GAIN gain);

/*! \fn void DC_Calibration(Motor* mHandle);
* @brief			:	Calibrate the DC offset of the current shunt amplifier
								**should be called after EnableGate()
* @param[in]	:	mHandle	motor instance handler
* @param[out]	:	
* @return 		: electric angle [range 0-4096]
* @par 	Change logs
*	Created by Alfred John (Zzx) on 04-Sept-2022
*/
void DC_Calibration(Motor* mHandle);

/*! \fn void SetMotorParam(Motor* mHandle,uint8_t DC_supply, uint8_t phase, uint8_t polePairs, float MaxCurrent, float MaxSpeed);
* @brief			:	Set motor parameters
* @param[in]	:	mHandle	motor instance handler
* @param[in]	:	V	DC supply voltage
* @param[in]	:	phase	number of phase 
* @param[in]	: polePairs	number of pole pairs
* @param[in]	: MaxCurrent	maximum current -> maximum phase current
* @param[in]	: MaxSpeed	maximum speed
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void SetMotorParam(Motor* mHandle,uint8_t DC_supply, uint8_t phase, uint8_t polePairs, float MaxCurrent, float MaxSpeed);

/*! \fn int calcAngle(Motor* mHandle);
* @brief			:	[Auxiliary Debug Function] calculate electric angle from Valpha and Vbeta
* @param[in]	:	mHandle	motor instance handler
* @param[out]	:	
* @return 		: electric angle [range 0-4096]
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
int calcAngle(Motor* mHandle);

/*! \fn void MotorCalibration(Motor* mHandle);
* @brief			:	Calibrate motor when power on
								Measure the angle offset between encoder 0 deg and motor electrical 0 deg(phase a 100%) 
								Angle_E  = (Angle_M  - Offset)*nPolePairs
* @param[in]	:	mHandle	motor instance handler
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void MotorCalibration(Motor* mHandle);

//void AngelAlignment(Motor* mHandle);

/*! \fn void MotorOpenLoop(Motor* mHandle);
* @brief			:	openloop drag control
* @param[in]	:	mHandle	motor instance handler
* @param[out]	:	
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void MotorOpenLoop(Motor* mHandle);

/*! \fn void ADCSample(Motor* mHandle);
* @brief			:	Current sampling function, once called, retrieve one set of sampled data from DMA 
								Data stored in mHandle->Current
								**called in timer callback function every control period
* @param[in]	:	mHandle	motor instance handler
* @param[out]	:	
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void ADCSample(Motor* mHandle);

/*! \fn void ADCFilter(Motor* mHandle);
* @brief			:	Filter the sampled current
* @param[in]	:	mHandle	motor instance handler
* @param[out]	:	
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void ADCFilter(Motor* mHandle);

/*! \fn void CurrentReconstruction(Motor* mHandle);
* @brief			:	Reconstruct the sampled current from 2 phases to three phases
* @param[in]	:	mHandle	motor instance handler
* @param[out]	:	
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void CurrentReconstruction(Motor* mHandle);

/*! \fn void getEangle(Motor* mHandle);
* @brief			:	Calculate electric angle from mechanical angle, store in mHandle->E_angle
* @param[in]	:	mHandle	motor instance handler 
* @param[out]	:	mHandle->E_angle
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void getEangle(Motor* mHandle);

/*! \fn void getSpeed(Motor* mHandle);
* @brief			:	Calculate the motor speed in rpm, store in mHandle->speed
* @param[in]	:	mHandle	motor instance handler
* @param[out]	:	mHandle->speed
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void getSpeed(Motor* mHandle);
//void setDQ(Motor mHandle, double d, double q);

/*! \fn void ClarkeTransform(Coordinates* pComponents);
* @brief			:	Clarke Transformation function, convert abc to alpha beta
* @param[in]	:	pComponents coordinates pointer (pComponents->a, pComponents->b, pComponents->c)
* @param[out]	: pComponents coordinates pointer (pComponents->alpha, pComponents->beta)
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void ClarkeTransform(Coordinates* pComponents);

/*! \fn void ParkTransform(Coordinates* pComponents, int theta);
* @brief			:	Park Transformation function, convert alpha, beta to d, q
								Original park: q leads d axis by 90 deg
								Modified park: d leads q axis by 90 deg
* @param[in]	:	pComponents coordinates pointer (pComponents->alpha, pComponents->beta)
* @param[out]	:	pComponents coordinates pointer (pComponents->d, pComponents->q)
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void ParkTransform(Coordinates* pComponents, int theta);

/*! \fn void InvPark(Coordinates* pComponents, int theta);
* @brief			:	Inverse Park Transformation function, convert d, q to alpha, beta
								Original park: q leads d axis by 90 deg
								Modified park: d leads q axis by 90 deg
* @param[in]	:	pComponents coordinates pointer (pComponents->d, pComponents->q)
* @param[out]	:	pComponents coordinates pointer (pComponents->alpha, pComponents->beta)
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void InvPark(Coordinates* pComponents, int theta);

/*! \fn void GetSector(Motor* mHandle);
* @brief			:	Calculate the sector of the target space voltage vector
* @param[in]	:	mHandle->Voltage.alpha, mHandle->Voltage.beta
* @param[out]	:	mHandle->sector
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void GetSector(Motor* mHandle);

/*! \fn void ModulationLimit(Motor* mHandle);
* @brief			:	Overmodulation limiting function, prevent overmodulation
* @param[in]	:	mHandle->T1, mHandle->T2
* @param[out]	:	mHandle->T1, mHandle->T2
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void ModulationLimit(Motor* mHandle);

/*! \fn void SetSVPWM(Motor* mHandle);
* @brief			:	MOSFET dwelling time calculation function, to generate SVPWM
* @param[in]	:	mHandle	motor instance handler
* @param[out]	:	mHandle->Ta, mHandle->Tb, mHandle->Tc
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void SetSVPWM(Motor* mHandle);

/*! \fn void dq2SVPWM(Motor* mHandle);
* @brief			:	Generate SVPWM directly from dq voltage
* @param[in]	:	mHandle	motor instance handler
* @param[out]	:	
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void dq2SVPWM(Motor* mHandle);

/*! \fn void SetCurrentPID(Motor* mHandle, float kp, float ki, float kd);
* @brief			:	Set PID parameters of both d,q current loop
* @param[in]	:	mHandle	motor instance handler
* @param[in]	:	kp, ki, kd
* @param[out]	:	mHandle->iqPID, mHandle->idPID
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void SetCurrentPID(Motor* mHandle, float kp, float ki, float kd);

/*! \fn void SetSpeedPID(Motor* mHandle, float kp, float ki, float kd, float outMax);
* @brief			:	Set speed loop PID parameters and the speed limit
* @param[in]	:	mHandle	motor instance handler
* @param[in]	:	kp, ki, kd, outMax
* @param[out]	:	mHandle->speedPID, mHandle->SpeedLimit
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void SetSpeedPID(Motor* mHandle, float kp, float ki, float kd, float outMax);

/*! \fn void SetCurrentTarget(Motor* mHandle, float idTarget, float iqTarget);
* @brief			:	Set target id, iq
* @param[in]	:	mHandle	motor instance handler
* @param[in]	:	idTarget, iqTarget
* @param[out]	:	mHandle->idPID.target, mHandle->iqPID.target
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void SetCurrentTarget(Motor* mHandle, float idTarget, float iqTarget);

/*! \fn void SetSpeedTarget(Motor* mHandle, float Target);
* @brief			:	Set target motor speed, with over speed limit
* @param[in]	:	mHandle	motor instance handler
* @param[in]	:	Target
* @param[out]	:	mHandle->speedPID.target
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
* Bug fixed by Alfred John (Zzx) on 04-Sept-2022
		added negative over speed judgement 
*/
void SetSpeedTarget(Motor* mHandle, float Target);

/*! \fn void IdPID(Motor* mHandle);
* @brief			:	Current loop Id PID calculation, calculate target Vd from Id
								with current kalman filter, integral limiter, and over voltage limiter
* @param[in]	:	mHandle	motor instance handler
* @param[out]	:	mHandle->Voltage.d
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void IdPID(Motor* mHandle);

/*! \fn void IqPID(Motor* mHandle);
* @brief			:	Current loop Iq PID calculation, calculate target Vq from Iq
								with current kalman filter, integral limiter, and over voltage limiter
* @param[in]	:	mHandle	motor instance handler
* @param[out]	:	mHandle->Voltage.q
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void IqPID(Motor* mHandle);

/*! \fn void SpeedPID(Motor* mHandle);
* @brief			:	Speed loop PID calculation, set target Id = 0, calculate target Iq from speed
								with integral limiter, and over current limiter
* @param[in]	:	mHandle	motor instance handler
* @param[out]	:	mHandle->iqPID.target
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void SpeedPID(Motor* mHandle);

/*! \fn void AnglePID(Motor* mHandle);
* @brief			:	Angle loop PID calculation, calculate target speed
								with integral limiter
* @param[in]	:	mHandle	motor instance handler
* @param[out]	:	SetSpeedTarget(mHandle, target)
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
* Bug fixed by Alfred John (Zzx) on 04-Sept-2022 
		used 'SetSpeedTarget(mHandle, target);' instead of directly assign target speed to prevent over speed.
*/
void AnglePID(Motor* mHandle);

/*! \fn void SetAnglePID(Motor* mHandle, float kp, float ki, float kd);
* @brief			:	Set angle loop PID parameters
* @param[in]	:	mHandle	motor instance handler
* @param[in]	:	kp, ki, kd
* @param[out]	:	mHandle->anglePID
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void SetAnglePID(Motor* mHandle, float kp, float ki, float kd);

/*! \fn void SetAngleTarget(Motor* mHandle, float Target);
* @brief			:	Set absolute target angle, only single-turn (0-360 deg)
* @param[in]	:	mHandle	motor instance handler
* @param[in]	:	Target
* @param[out]	:	mHandle->anglePID.target
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void SetAngleTarget(Motor* mHandle, float Target);

/*! \fn void SetIncrementAngle(Motor* mHandle, float Target);
* @brief			:	Set increment target angle, the incremental angle is also restricted in (0-360 deg)
* @param[in]	:	mHandle	motor instance handler
* @param[out]	:	SetAngleTarget(mHandle,Target);
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void SetIncrementAngle(Motor* mHandle, float Target);

/*! \fn void FOC(Motor* mHandle);
* @brief			:	FOC loop funtion, including electric angle conversion, phase current filtering, 
								Clarke/Park transformation, PID calculation, and SVPWM generation
								**Called in ADC_DMA callback every control period
* @param[in]	:	mHandle	motor instance handler
* @param[out]	:	
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void FOC(Motor* mHandle);

/*! \fn void FOCcheck(Motor* mHandle);
* @brief			:	[Auxiliary Debug Function] call to print FOC PID parameters
* @param[in]	:	mHandle	motor instance handler
* @param[out]	:	mHandle->idPID, mHandle->iqPID, mHandle->speedPID, mHandle->anglePID
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void FOCcheck(Motor* mHandle);

/*! \fn void GetSVPWM(Motor* mHandle);
* @brief			:	Original SVPWM calculation function using complex numbers, slow
* @param[in]	:	
* @param[out]	:	
* @return 		:
* @par 	Change logs
*	Created by Alfred John (Zzx) on 03-Sept-2022
*/
void GetSVPWM(Motor* mHandle);

/*! \mainpage MasterFOC
 *
 * \section intro_sec Introduction
 *
 * This library is designed to support the MsterFOC Dual Motor control board. The whole project is the Final Year Project of Zexing Zhao (Alfred John).
 
 * Functions provided including:
	- Encoder angle feedback functions of Photoelectric encoders, PWM encoders, and I2C encoders.
	- ADC phase current sampling, filtering, and current reconstruction function.
	- Rotor angle calibration function.
	- Coordinate Transformation functions consisting Clarke, Park transformation and their inverse forms.
	- SVPWM calculation function.
	- PID functions with anti-integral saturation and output limiting features.
	- FOC open loop functions.
	- FOC close loop functions including current, speed and position mode.
	- Serial port data feedback functions.
	- Control mode selection function.
	- User application functions.
	- Specifically, double motor operation functions including dual motor drag control function, force feedback function, and multiple gear switching function.
 *
 * \section toc_sec Table of Contents
 *
	- @ref Library Sructure 
			- see control.h
	- @ref Users Guide
 *		- The overall logic was demonstrated as,
 *			@image html hierarchy.png width=500px
 *		- Updating...
 */




#endif /* __CONTROL_H */


