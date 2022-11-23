/** 
* @file		
* @brief 		
* @author 		Alfred John
* @date 		
* @version 	1.0
* @par Copyright(c): 	Alfred John
* @par History:         
*	version: 1.0	Alfred John	\n
*/

#include "control.h"

#define CurrentGain 496.0f //2048/4.125    (4.125 = 1.65/40/0.01)
#define FILTER_KP 0.07f
#define FILTER_DQ 0.01f
//#define FILTER_ANGLE 0.9f
#define FILTER_SPEED 0.05f
#define Vector_coef (SQRT_3/mHandle->Vdc)
#define mod(x) ((x)>0?(x):-(x))

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

float Vmabs;
float Vout_para_abs;
float T2;

//filter coefficient
/*const float FilterCoeff[6] = {
  1.284488738e-17,   0.1729719192,   0.3270280957,   0.3270280957,   0.1729719192,
  1.284488738e-17
};
static float FIRcacheA[BLOCK_SIZE + NUM_TAPS - 1];
static float FIRcacheB[BLOCK_SIZE + NUM_TAPS - 1];
static float FIRcacheC[BLOCK_SIZE + NUM_TAPS - 1];
	arm_fir_instance_f32 PhaseA;
	arm_fir_instance_f32 PhaseB;
	arm_fir_instance_f32 PhaseC;*/
	
void MotorInit(Motor* mHandle, ADC_HandleTypeDef hadc, TIM_HandleTypeDef hDriver, TIM_HandleTypeDef hEncoder, I2C_HandleTypeDef hIICEncoder, ControlMode mode){
	
	mHandle->hMotorADC = hadc;
	mHandle->hTimDriver = hDriver;
	mHandle->hTimEncoder = hEncoder;
	mHandle->hI2CEncoder = hIICEncoder;
	mHandle->Mode = mode;
	//HAL_GPIO_WritePin(EN_Gate_GPIO_Port,EN_Gate_Pin,GPIO_PIN_SET);
	//HAL_GPIO_WritePin(EN_Gate_GPIO_Port,EN_Gate_Pin,GPIO_PIN_RESET);
	HAL_TIM_PWM_Start(&hDriver,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&hDriver,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&hDriver,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&hDriver,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&hDriver,TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&hDriver,TIM_CHANNEL_3);
	
}

void ModeSelection(Motor* mHandle,ControlMode mode){
	mHandle->Mode = mode;
}

void UserAPPSelection(Motor* mHandle,UserAPP app){
	mHandle->APP = app;
}

void GainSelection(DRV_GAIN gain){
	if(gain == DRV_GAIN_10){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
	}else{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
	}
}

void EnableGate(){
	HAL_GPIO_WritePin(EN_Gate_GPIO_Port,EN_Gate_Pin, GPIO_PIN_SET);
}
//called after gate enable
void DC_Calibration(Motor* mHandle){
	//pull up the DC_CAL pins
	HAL_GPIO_WritePin(DC_CAL1_GPIO_Port,DC_CAL1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DC_CAL2_GPIO_Port,DC_CAL2_Pin, GPIO_PIN_SET);
	printf("DC_offset calibrating...\n");
	HAL_Delay(1000);
	//sample the DC offset 10 times and take the average
	mHandle->DC_offset[0] = 0;
	mHandle->DC_offset[1] = 0;
	/*for(int i = 0; i<10; i++){
		HAL_ADC_Start_DMA(&mHandle->hMotorADC, (uint32_t*)mHandle->ADC_DMA_RAW, 2);
		//printf("Sampling: %d, %d\n", mHandle->ADC_DMA_RAW[0], mHandle->ADC_DMA_RAW[1]);
	}*/
	for(int i = 0; i<10; i++){
		HAL_ADC_Start_DMA(&mHandle->hMotorADC, (uint32_t*)mHandle->ADC_DMA_RAW, 2);
		mHandle->DC_offset[0] += mHandle->ADC_DMA_RAW[0];
		mHandle->DC_offset[1] += mHandle->ADC_DMA_RAW[1];
		printf("DC_offset calibrating: %d, %d\n", mHandle->ADC_DMA_RAW[0], mHandle->ADC_DMA_RAW[1]);
	}
	mHandle->DC_offset[0] /= 9;
	mHandle->DC_offset[1] /= 9;
	printf("DC_offset calibration finished: %d, %d\n", mHandle->DC_offset[0], mHandle->DC_offset[1]);
	//pull down the DC_CAL pins
	HAL_GPIO_WritePin(DC_CAL1_GPIO_Port,DC_CAL1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DC_CAL2_GPIO_Port,DC_CAL2_Pin, GPIO_PIN_RESET);
}

void SetMotorParam(Motor* mHandle, uint8_t DC_supply, uint8_t phase, uint8_t polePairs, float MaxCurrent, float MaxSpeed){
	mHandle->Vdc = DC_supply;
	mHandle->nPhase = phase;
	mHandle->nPolePairs = polePairs;
	mHandle->CurrentLimit = CurrentGain*MaxCurrent;
	mHandle->VoltageLimit = DC_supply/2*SQRT_3;
	mHandle->SpeedLimit = MaxSpeed;
}

void MotorCalibration(Motor* mHandle){
	//Calibration initialization
	mHandle->nCalibrationFlag = 0;
	mHandle->Voltage.d = 3;
	mHandle->Voltage.q = 0;
	__HAL_TIM_SetCompare(&mHandle->hTimDriver, TIM_CHANNEL_1, 0.7*mHandle->hTimDriver.Instance->ARR);
	__HAL_TIM_SetCompare(&mHandle->hTimDriver, TIM_CHANNEL_2, 0.2*mHandle->hTimDriver.Instance->ARR);
	__HAL_TIM_SetCompare(&mHandle->hTimDriver, TIM_CHANNEL_3, 0.2*mHandle->hTimDriver.Instance->ARR);
	for(int i=0;i<1000;i++){
		mHandle->Angle_offset = mHandle->M_angle;
		getEangle(mHandle);
		printf("Calibrated angle:%d,%d,%d\n",mHandle->M_angle,mHandle->E_angle,mHandle->Angle_offset);
		//
	}	
	/*__HAL_TIM_SetCompare(&mHandle->hTimDriver, TIM_CHANNEL_1, 0.2*mHandle->hTimDriver.Instance->ARR);
	__HAL_TIM_SetCompare(&mHandle->hTimDriver, TIM_CHANNEL_2, 0.7*mHandle->hTimDriver.Instance->ARR);
	__HAL_TIM_SetCompare(&mHandle->hTimDriver, TIM_CHANNEL_3, 0.2*mHandle->hTimDriver.Instance->ARR);
for(int i=0;i<10000;i++){
	getEangle(mHandle);
	printf("Calibrated angle:%d,%.2f,%d\n",mHandle->M_angle,(float)mHandle->E_angle/4096*360,mHandle->Angle_offset);
	//
}	__HAL_TIM_SetCompare(&mHandle->hTimDriver, TIM_CHANNEL_1, 0.2*mHandle->hTimDriver.Instance->ARR);
	__HAL_TIM_SetCompare(&mHandle->hTimDriver, TIM_CHANNEL_2, 0.2*mHandle->hTimDriver.Instance->ARR);
	__HAL_TIM_SetCompare(&mHandle->hTimDriver, TIM_CHANNEL_3, 0.7*mHandle->hTimDriver.Instance->ARR);
for(int i=0;i<10000;i++){
	getEangle(mHandle);
	printf("Calibrated angle:%d,%.2f,%d\n",mHandle->M_angle,(float)mHandle->E_angle/4096*360,mHandle->Angle_offset);
	//
}*/
	mHandle->nCalibrationFlag = 1;
	mHandle->Voltage.d = 0;
	
	/////Alpha and Z offset: 4.568707, 10.848739, 17.14451, 23.424261
	//filter initialization

	//arm_fir_init_f32(&PhaseA,NUM_TAPS,(float32_t*)&FilterCoeff[0],FIRcacheA,BLOCK_SIZE);
	//arm_fir_init_f32(&PhaseB,NUM_TAPS,(float32_t*)&FilterCoeff[0],FIRcacheB,BLOCK_SIZE);
	//arm_fir_init_f32(&PhaseC,NUM_TAPS,(float32_t*)&FilterCoeff[0],FIRcacheC,BLOCK_SIZE);
}
//Calibrated record M_E Angel difference
//void AngelAlignment(Motor* mHandle){
//	//stop the timer, reset the M_angle counter
//	HAL_TIM_Base_Stop_IT(&htim1);
//	__HAL_TIM_SET_COUNTER(&htim3,0);
//	mHandle->nCalibrationFlag = 1;
//	//mHandle->Angle_offset = mHandle->E_angle;
//	/*InvPark(&mHandle->Voltage, mHandle->E_angle);
//	SetSVPWM(mHandle);*/
//	//stop the motor
//	//__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, mHandle->Ta*htim1.Instance->ARR);
//	//__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, mHandle->Tb*htim1.Instance->ARR);
//	//__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, mHandle->Tc*htim1.Instance->ARR);
//	//__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0.7*htim1.Instance->ARR);
//	//__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0.2*htim1.Instance->ARR);
//	//__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0.2*htim1.Instance->ARR);
//	//getAngle(mHandle);
//	//mHandle->Angle_offset = __HAL_TIM_GET_COUNTER(&htim3);// can not read??
//	//printf("E_angle:%f\n",mTest.E_angle);
//	mHandle->Angle_offset = 77;
//	//for(int i = 0;i<10000000;i++){
//		//mHandle->Angle_offset = mHandle->M_angle;
//	//}

//	printf("Calibrated angle:%d,%d,%d\n",mHandle->M_angle,mHandle->E_angle,mHandle->Angle_offset);

//	//restart the timer
//	HAL_TIM_Base_Start_IT(&htim1);
//}
//openloop rotate for calibration
void MotorOpenLoop(Motor* mHandle){
	mHandle->E_angle = mHandle->E_angle + 1;//0.0157f*UnitPulse;
	/*if(mHandle->E_angle>=8*pi){
		mHandle->E_angle=0;
	}*/
	//mHandle->E_angle = 0;
	//printf("E_angle:%d,%d\n",mHandle->E_angle,n);
	//getAngle(mHandle);
	if(mHandle->E_angle>=UnitPulse+mHandle->Angle_offset){//mHandle->E_angle>=8*pi+mHandle->Angle_offset
			mHandle->E_angle=mHandle->Angle_offset;
	}
	InvPark(&mHandle->Voltage, mHandle->E_angle);
	SetSVPWM(mHandle);
	
}

//start adc sample once
void ADCSample(Motor* mHandle){
	HAL_ADC_Start_DMA(&mHandle->hMotorADC, (uint32_t*)mHandle->ADC_DMA_RAW, 2);
	mHandle->Current.a = mHandle->ADC_DMA_RAW[0] - mHandle->DC_offset[0];
	mHandle->Current.b = mHandle->ADC_DMA_RAW[1] - mHandle->DC_offset[1];
	mHandle->Current.c = 0 - mHandle->Current.a - mHandle->Current.b;
	//printf("%d,%d,%d\n",mHandle->Current.a,mHandle->Current.b,mHandle->Current.c);
}
//filter the adc value
void ADCFilter(Motor* mHandle){
	mHandle->ADC_Filtered[0] = mHandle->ADC_DMA_RAW[0] - 2048;
	mHandle->ADC_Filtered[1] = mHandle->ADC_DMA_RAW[1] - 2048;
	mHandle->ADC_Filtered[2] = 0 - mHandle->ADC_Filtered[0] - mHandle->ADC_Filtered[1];
	//printf("ADC:%d,%d,%d\n",mHandle->ADC_DMA_RAW[0],mHandle->ADC_DMA_RAW[1],mHandle->ADC_DMA_RAW[2]);
	//printf("%d,%d,%d\n",mHandle->ADC_Filtered[0],mHandle->ADC_Filtered[1],mHandle->ADC_Filtered[2]);
	/*mHandle->ADC_Filtered[0] = (int16_t)(FILTER_KP * (float)mHandle->ADC_Filtered[0] + (1-FILTER_KP) * (float)mHandle->ADC_lastValue[0]);
	mHandle->ADC_lastValue[0] = mHandle->ADC_Filtered[0];
	mHandle->ADC_Filtered[1] = (int16_t)(FILTER_KP * (float)mHandle->ADC_Filtered[1] + (1-FILTER_KP) * (float)mHandle->ADC_lastValue[1]);
	mHandle->ADC_lastValue[1] = mHandle->ADC_Filtered[1];
	mHandle->ADC_Filtered[2] = (int16_t)(FILTER_KP * (float)mHandle->ADC_Filtered[2] + (1-FILTER_KP) * (float)mHandle->ADC_lastValue[2]);
	mHandle->ADC_lastValue[2] = mHandle->ADC_Filtered[2];*/
	//printf("ADC:%d,%d,%d,%d,%d,%d\n",mHandle->ADC_DMA_RAW[0],mHandle->ADC_DMA_RAW[1],mHandle->ADC_DMA_RAW[2],mHandle->ADC_Filtered[0],mHandle->ADC_Filtered[1],mHandle->ADC_Filtered[2]); 
	//printf("ADC:%d,%d,%d\n",mHandle->ADC_Filtered[0],mHandle->ADC_Filtered[1],mHandle->ADC_Filtered[2]); 
}

void CurrentReconstruction(Motor* mHandle){

	
	mHandle->Current.a = (FILTER_KP * mHandle->Current.a + (1-FILTER_KP) * mHandle->Current_lastValue[0]);
	mHandle->Current_lastValue[0] = mHandle->Current.a;
	mHandle->Current.b = (FILTER_KP * mHandle->Current.b + (1-FILTER_KP) * mHandle->Current_lastValue[1]);
	mHandle->Current_lastValue[1] = mHandle->Current.b;
	mHandle->Current.c = (FILTER_KP * mHandle->Current.c + (1-FILTER_KP) * mHandle->Current_lastValue[2]);
	mHandle->Current_lastValue[2] = mHandle->Current.c;
	
	
	//printf("ADC:%f,%f,%f,%f,%f,%f\n",mHandle->ADC_Filtered[0] * 0.0100207f+8,mHandle->ADC_Filtered[1] * 0.0100207f+8,mHandle->ADC_Filtered[2] * 0.0100207f+8,mHandle->Current.a,mHandle->Current.b,mHandle->Current.c); 
	//printf("%d,%d,%d\n",mHandle->Current.a,mHandle->Current.b,mHandle->Current.c);
}

void getEangle(Motor* mHandle){
	mHandle->E_angle = ((mHandle->M_angle - mHandle->Angle_offset+UnitPulse+1)*mHandle->nPolePairs)%(UnitPulse+1);
	//mHandle->E_angle = FILTER_ANGLE * mHandle->E_angle + (1-FILTER_ANGLE) * mHandle->E_angle_pre;
	//mHandle->E_angle_pre = mHandle->E_angle;
}

void getSpeed(Motor* mHandle){

	if(mHandle->M_angle - mHandle->M_angle_pre > 2000){
		mHandle->lapCNT --;
	}else if(mHandle->M_angle - mHandle->M_angle_pre < -2000){
		mHandle->lapCNT ++;
	}
	mHandle->SpeedCNT = (mHandle->lapCNT - mHandle->lapCNT_pre)*4095 + mHandle->M_angle - mHandle->M_angle_pre;
	mHandle->lapCNT_pre = mHandle->lapCNT;
	//mHandle->speed = mHandle->SpeedCNT;
	mHandle->speed = 2*(float)mHandle->SpeedCNT*(float)HAL_RCC_GetPCLK1Freq()/(float)(htim6.Init.Period+1)/(float)(htim6.Init.Prescaler+1)/(float)(UnitPulse)*60.0f;	//rpm
	mHandle->speed = FILTER_SPEED * mHandle->speed + (1-FILTER_SPEED) * mHandle->speed_pre;
	mHandle->speed_pre = mHandle->speed;
	//printf("speed:%f\n",mHandle->speed);
	mHandle->M_angle_pre = mHandle->M_angle;
}


//set dq
//void setDQ(Motor mHandle, double d, double q){
//	mHandle.Voltage.d = d;
//	mHandle.Voltage.q = q;
//}
//abc to alpha beta
void ClarkeTransform(Coordinates* pComponents){
	pComponents->alpha = pComponents->a - 0.5f*pComponents->b-0.5f*pComponents->c;
	pComponents->beta = 0.5f*SQRT_3*pComponents->b - 0.5f*SQRT_3*pComponents->c;
}

//alpha beta to dq
void ParkTransform(Coordinates* pComponents, int theta){
	float rad = (float)theta/(UnitPulse+1)*2*pi;
	//original
	//pComponents->d = pComponents->alpha*arm_cos_f32(rad) + pComponents->beta*arm_sin_f32(rad);
	//pComponents->q = -pComponents->alpha*arm_sin_f32(rad) + pComponents->beta*arm_cos_f32(rad);
	//modified
	pComponents->d = pComponents->alpha*arm_cos_f32(rad) + pComponents->beta*arm_sin_f32(rad);
	pComponents->q = pComponents->alpha*arm_sin_f32(rad) - pComponents->beta*arm_cos_f32(rad);
	
}

//dq to alpha beta
void InvPark(Coordinates* pComponents, int theta){
	float rad = (float)theta/(UnitPulse+1)*2*pi;
	//original
	//pComponents->alpha = pComponents->d*arm_cos_f32(rad) - pComponents->q*arm_sin_f32(rad);
	//pComponents->beta = pComponents->d*arm_sin_f32(rad) + pComponents->q*arm_cos_f32(rad);
	//modified
	pComponents->alpha = pComponents->d*arm_cos_f32(rad) + pComponents->q*arm_sin_f32(rad);
	pComponents->beta = pComponents->d*arm_sin_f32(rad) - pComponents->q*arm_cos_f32(rad);
	
	
	//printf("alpha:%f,%f\n",pComponents->alpha,pComponents->beta);
}

//GetSector
void GetSector(Motor* mHandle){
	uint8_t a, b, c, N;
	
	mHandle->Uref1 = mHandle->Voltage.beta;
	mHandle->Uref2 = (SQRT_3*mHandle->Voltage.alpha - mHandle->Voltage.beta)*0.5f;
	mHandle->Uref3 = (-SQRT_3*mHandle->Voltage.alpha - mHandle->Voltage.beta)*0.5f;
	
	if(mHandle->Uref1 > 0){
		a = 1;
	}else{
		a = 0;
	}
	if(mHandle->Uref2 > 0){
		b = 1;
	}else{
		b = 0;
	}
	if(mHandle->Uref3 > 0){
		c = 1;
	}else{
		c = 0;
	}
	
	N = 4*c + 2*b + a;
	
	switch (N) {
        case 3:
            mHandle->sector = 1;
            break;
        case 1:
            mHandle->sector = 2;
            break;
        case 5:
            mHandle->sector = 3;
            break;
        case 4:
            mHandle->sector = 4;
            break;
        case 6:
            mHandle->sector = 5;
            break;
        case 2:
            mHandle->sector = 6;
            break;
    }
}
//
int calcAngle(Motor* mHandle){
	float angle = atan2(mHandle->Voltage.beta,mHandle->Voltage.alpha);
	if(mHandle->Voltage.beta<0){
		angle = 2*pi + angle;
	}
	angle = angle/(2*pi)*4096;
	return angle;
}
//
//modulation limit
void ModulationLimit(Motor* mHandle){
	if(mHandle->T1+mHandle->T2 > 1){
		mHandle->T1 = mHandle->T1/(mHandle->T1+mHandle->T2);
		mHandle->T2 = mHandle->T2/(mHandle->T1+mHandle->T2);
	}
}
void SetSVPWM(Motor* mHandle){
	GetSector(mHandle);
	float X,Y,Z;
	X = Vector_coef*mHandle->Uref1;
	Y = -Vector_coef*mHandle->Uref3;
	Z = -Vector_coef*mHandle->Uref2;
	switch(mHandle->sector){
		case 1:
			mHandle->T1 = Z; //Z
			mHandle->T2 = Y;	//Y
			break;
		case 2:
			mHandle->T1 = Y;
			mHandle->T2 = -X;
			break;
		case 3:
			mHandle->T1 = -Z;
			mHandle->T2 = X;
			break;
		case 4:
			mHandle->T1 = -X;
			mHandle->T2 = Z;
			break;
		case 5:
			mHandle->T1 = X;
			mHandle->T2 = -Y;
			break;
		case 6:
			mHandle->T1 = -Y;
			mHandle->T2 = -Z;
			break;
	}
	//modulation limit
	ModulationLimit(mHandle);
	mHandle->Tx = (1 - mHandle->T1 - mHandle->T2)/4.0f;
	mHandle->Ty = mHandle->Tx + mHandle->T1*0.5f;
	mHandle->Tz = mHandle->Ty + mHandle->T2*0.5f;
	switch(mHandle->sector){
		case 1:
			mHandle->Ta = mHandle->Ty;
			mHandle->Tb = mHandle->Tx;
			mHandle->Tc = mHandle->Tz;
			break;
		case 2:
			mHandle->Ta = mHandle->Tx;
			mHandle->Tb = mHandle->Tz;
			mHandle->Tc = mHandle->Ty;
			break;
		case 3:
			mHandle->Ta = mHandle->Tx;
			mHandle->Tb = mHandle->Ty;
			mHandle->Tc = mHandle->Tz;
			break;
		case 4:
			mHandle->Ta = mHandle->Tz;
			mHandle->Tb = mHandle->Ty;
			mHandle->Tc = mHandle->Tx;
			break;
		case 5:
			mHandle->Ta = mHandle->Tz;
			mHandle->Tb = mHandle->Tx;
			mHandle->Tc = mHandle->Ty;
			break;
		case 6:
			mHandle->Ta = mHandle->Ty;
			mHandle->Tb = mHandle->Tz;
			mHandle->Tc = mHandle->Tx;
			break;
	}
			
	//printf("Ta=%f,Tb=%f,Tc=%f\n",mHandle->Ta,mHandle->Tb,mHandle->Tc);
	//printf("%f,%f,%f\n",mHandle->Ta,mHandle->Tb,mHandle->Tc);
	//printf("%f,%f,%f,%d,%d,%d\n",mHandle->Ta,mHandle->Tb,mHandle->Tc,mHandle->sector,calcAngle(mHandle),mHandle->E_angle/100);
	//printf("%f,%f,%f\n",mHandle->T1,mHandle->T2,(1 - mHandle->T1 - mHandle->T2));

	__HAL_TIM_SetCompare(&mHandle->hTimDriver, TIM_CHANNEL_1, mHandle->Ta*mHandle->hTimDriver.Instance->ARR);
	__HAL_TIM_SetCompare(&mHandle->hTimDriver, TIM_CHANNEL_2, mHandle->Tb*mHandle->hTimDriver.Instance->ARR);
	__HAL_TIM_SetCompare(&mHandle->hTimDriver, TIM_CHANNEL_3, mHandle->Tc*mHandle->hTimDriver.Instance->ARR);
	
	
}
//
float a,b,c;
void GetSVPWM(Motor* mHandle){
	//calculate sector
	mHandle->Voltage.alpha = mHandle->Voltage.alpha/mHandle->Vdc;
	mHandle->Voltage.beta = mHandle->Voltage.beta/mHandle->Vdc;//Normallize
  
	GetSector(mHandle);
	//printf("sector %d\n", mHandle->sector);
	float realVi = arm_cos_f32((mHandle->sector-1)*pi/mHandle->nPhase);
	float imagVi = arm_sin_f32((mHandle->sector-1)*pi/mHandle->nPhase);
	float Vi[2] = {realVi, imagVi};
	float realVj = arm_cos_f32((mHandle->sector)*pi/mHandle->nPhase);
	float imagVj = arm_sin_f32((mHandle->sector)*pi/mHandle->nPhase);
	float Vj[2] = {realVj, imagVj};
	
	//printf("Vi %f,%f | Vj %f,%f\n", realVi,imagVi,realVj,imagVj);
	
	float Vm[2] = {(realVi+realVj)/2, (imagVi+imagVj)/2};
	//complex double Vm = (Vi+Vj)/2;
	
	arm_cmplx_mag_f32(Vm, &Vmabs, 1);
	//printf("Vm %f,%f Vmabs %f\n",Vm[0],Vm[1],Vmabs);
	//double Vmabs = cabs(Vm);
	float Vout[2] = {mHandle->Voltage.alpha, mHandle->Voltage.beta};
	//complex double Vout = mHandle->Voltage.alpha + _Complex_I*mHandle->Voltage.beta;
	float Vout_para[2] = {Vm[0], -Vm[1]};
	arm_cmplx_mult_cmplx_f32(Vout, Vout_para, Vout_para, 1);
	
	//arm_cmplx_mag_f32(Vout_para, &Vout_para_abs, 1);
	Vout_para_abs = Vout_para[0]/Vmabs;
	//printf("Vout %f,%f Vout_para %f\n",Vout[0],Vout[1],Vout_para_abs);
	//float Vout_para = creal(Vout * conj(Vm))/Vmabs;
	if (Vout_para_abs > Vmabs){
    //Vout = Vout/Vout_para *Vmabs;
    //Vout_para  = Vmabs;
		Vout[0] = Vout[0]/Vout_para_abs*Vmabs;
		Vout[1] = Vout[1]/Vout_para_abs*Vmabs;
		Vout_para_abs = Vmabs;
	}
	//printf("Vout %f,%f Vout_para %f\n",Vout[0],Vout[1],Vout_para_abs);
	float T0 = 1- Vout_para_abs / Vmabs;
	//complex double Vom = Vout / Vout_para *Vmabs - Vm;
	float Vom[2] = {Vout[0]/Vout_para_abs*Vmabs - Vm[0], Vout[1]/Vout_para_abs*Vmabs - Vm[1]};
	//double T2 = (creal(Vom/(Vj-Vi))+0.5) * (1-T0);
	float Vtemp[2] = {Vj[0] - Vi[0], Vj[1] - Vi[1]};
	Vtemp[0] = (Vom[0]*Vtemp[0] + Vom[1]*Vtemp[1])/(Vtemp[0]*Vtemp[0] + Vtemp[1]*Vtemp[1]);
	//Vtemp[1] = (Vom[1]*Vtemp[0] - Vom[0]*Vtemp[1])/(Vtemp[0]*Vtemp[0] + Vtemp[1]*Vtemp[1]);
	
	//arm_cmplx_mag_f32(Vtemp, &T2, 1);
	T2 = (Vtemp[0] + 0.5f) * (1.0f - T0);
	float T1 = 1-T0-T2;
	uint8_t N = (mHandle->sector-1)%(2*mHandle->nPhase);
	//printf("Vom %f,%f\n",Vom[0],Vom[1]);
	//printf("T0 %f,T2 %f,T1 %f,N %d\n",T0,T2,T1,N);
	float Tall[4] = {T0/4, T1/2+T0/4, T2/2+T0/4, T1/2+T2/2+T0/4};
	switch (N){
		case 0:
			mHandle->Ta = Tall[0];
			mHandle->Tb = Tall[1];
			mHandle->Tc = Tall[3];
			break;
		case 1:
			mHandle->Ta = Tall[2];
			mHandle->Tb = Tall[0];
			mHandle->Tc = Tall[3];
			break;
		case 2:
			mHandle->Ta = Tall[3];
			mHandle->Tb = Tall[0];
			mHandle->Tc = Tall[1];
		break;
	case 3:
			mHandle->Ta = Tall[3];
			mHandle->Tb = Tall[2];
			mHandle->Tc = Tall[0];
		break;
	case 4:
			mHandle->Ta = Tall[1];
			mHandle->Tb = Tall[3];
			mHandle->Tc = Tall[0];
		break;
	case 5:
			mHandle->Ta = Tall[0];
			mHandle->Tb = Tall[3];
			mHandle->Tc = Tall[2];
		break;
	default:
			mHandle->Ta = 0;
			mHandle->Tb = 0;
			mHandle->Tc = 0;
	}
	//printf("Ts=%f,Tm=%f,Tt=%f\n",mHandle->Ta,mHandle->Tb,mHandle->Tc);
	//float sum = mHandle->Ta+mHandle->Tb+mHandle->Tc;
	//float avg = sum/3.0f;
	
	//a = mHandle->Ta - avg;
	//b = mHandle->Tb - avg;
	//c = mHandle->Tc - avg;
	//printf("%f,%f,%f\n",mHandle->Ta*2.0f,mHandle->Tb*2.0f,mHandle->Tc*2.0f);
	//printf("%d,%f,%f\n",N,(float)mHandle->E_angle/4096*2*pi,(float)calcAngle(mHandle)/4096*2*pi);
	//printf("%f,%f,%f\n",T1,T2,T0);
	//printf("%f,%f,%f\n",a,b,c);
	
	__HAL_TIM_SetCompare(&mHandle->hTimDriver, TIM_CHANNEL_1, mHandle->Ta*2.0f*mHandle->hTimDriver.Instance->ARR);
	__HAL_TIM_SetCompare(&mHandle->hTimDriver, TIM_CHANNEL_2, mHandle->Tb*2.0f*mHandle->hTimDriver.Instance->ARR);
	__HAL_TIM_SetCompare(&mHandle->hTimDriver, TIM_CHANNEL_3, mHandle->Tc*2.0f*mHandle->hTimDriver.Instance->ARR);
	
	
}
//你所不知的船新版本
//
void dq2SVPWM(Motor* mHandle){
	getEangle(mHandle);
	
	InvPark(&mHandle->Voltage, mHandle->E_angle);
	
	SetSVPWM(mHandle);
}

void SetCurrentPID(Motor* mHandle, float kp, float ki, float kd){
	mHandle->iqPID.kp = kp;
	mHandle->iqPID.ki = ki;
	mHandle->iqPID.kd = kd;
	
	mHandle->idPID.kp = kp;
	mHandle->idPID.ki = ki;
	mHandle->idPID.kd = kd;

}

void SetCurrentTarget(Motor* mHandle, float idTarget, float iqTarget){
	mHandle->idPID.target = idTarget;
	mHandle->iqPID.target = iqTarget;
}

void IdPID(Motor* mHandle){
	//filter
	mHandle->Current.d = (FILTER_DQ * mHandle->Current.d + (1-FILTER_DQ) * mHandle->Current_lastValue[3]);
	mHandle->Current_lastValue[3] = mHandle->Current.d;
	//pid calc
	mHandle->idPID.err = mHandle->idPID.target - mHandle->Current.d;
	mHandle->idPID.err_sum += mHandle->idPID.err * mHandle->idPID.ki;
	//integral limiter
	if(mHandle->idPID.err_sum > mHandle->VoltageLimit){
		mHandle->idPID.err_sum = mHandle->VoltageLimit;
	}else if(mHandle->idPID.err_sum < -mHandle->VoltageLimit){
		mHandle->idPID.err_sum = -mHandle->VoltageLimit;
	}
	//mHandle->Voltage.d = mHandle->idPID.target;
	mHandle->Voltage.d = mHandle->idPID.kp * mHandle->idPID.err + mHandle->idPID.err_sum;
	
	if(mHandle->Voltage.d > mHandle->VoltageLimit){
		mHandle->Voltage.d = mHandle->VoltageLimit;
	}else if(mHandle->Voltage.d < -mHandle->VoltageLimit){
		mHandle->Voltage.d = -mHandle->VoltageLimit;
	}
}

void IqPID(Motor* mHandle){
	mHandle->Current.q = (FILTER_DQ * mHandle->Current.q + (1-FILTER_DQ) * mHandle->Current_lastValue[4]);
	mHandle->Current_lastValue[4] = mHandle->Current.q;
	mHandle->iqPID.err = mHandle->iqPID.target - mHandle->Current.q;
	mHandle->iqPID.err_sum += mHandle->iqPID.err * mHandle->iqPID.ki;
	//integral limiter
	if(mHandle->iqPID.err_sum > mHandle->VoltageLimit){
		mHandle->iqPID.err_sum = mHandle->VoltageLimit;
	}else if(mHandle->iqPID.err_sum < -mHandle->VoltageLimit){
		mHandle->iqPID.err_sum = -mHandle->VoltageLimit;
	}
	
	//mHandle->Voltage.q = mHandle->iqPID.target;
	mHandle->Voltage.q = mHandle->iqPID.kp * mHandle->iqPID.err + mHandle->iqPID.err_sum;


	if(mHandle->Voltage.q > mHandle->VoltageLimit){
		mHandle->Voltage.q = mHandle->VoltageLimit;
	}else if(mHandle->Voltage.q < -mHandle->VoltageLimit){
		mHandle->Voltage.q = -mHandle->VoltageLimit;
	}
	
}
void SetSpeedPID(Motor* mHandle, float kp, float ki, float kd, float outMax){
	mHandle->speedPID.kp = kp;
	mHandle->speedPID.ki = ki;
	mHandle->speedPID.kd = kd;
	mHandle->SpeedLimit = outMax;
}

void SetSpeedTarget(Motor* mHandle, float Target){
	if(Target > mHandle->SpeedLimit){
		mHandle->speedPID.target = mHandle->SpeedLimit;
	}
	else if(Target < -mHandle->SpeedLimit){
		mHandle->speedPID.target = -mHandle->SpeedLimit;
	}else
		mHandle->speedPID.target = Target;
}

void SpeedPID(Motor* mHandle){
	mHandle->speedPID.err = mHandle->speedPID.target - mHandle->speed;
	mHandle->speedPID.err_sum += mHandle->speedPID.err * mHandle->speedPID.ki;
	
	//integral limiter
	if(mHandle->speedPID.err_sum > mHandle->CurrentLimit){
		mHandle->speedPID.err_sum = mHandle->CurrentLimit;
	}else if(mHandle->speedPID.err_sum < -mHandle->CurrentLimit){
		mHandle->speedPID.err_sum = -mHandle->CurrentLimit;
	}
	
	mHandle->speedPID.err_pre = mHandle->speedPID.err;
	
	mHandle->iqPID.target = mHandle->speedPID.kp*mHandle->speedPID.err + mHandle->speedPID.err_sum + mHandle->speedPID.kd*(mHandle->speedPID.err - mHandle->speedPID.err_pre);
	mHandle->idPID.target = 0;
	

	if(mHandle->iqPID.target > mHandle->CurrentLimit){
		mHandle->iqPID.target = mHandle->CurrentLimit;
	}else if(mHandle->iqPID.target < -mHandle->CurrentLimit){
		mHandle->iqPID.target = -mHandle->CurrentLimit;
	}
	
}

void AnglePID(Motor* mHandle){
	mHandle->anglePID.err = mHandle->anglePID.target - mHandle->M_angle;
	if(mHandle->anglePID.err > UnitPulse/2){
		mHandle->anglePID.err -= UnitPulse;
	}
	if(mHandle->anglePID.err < -UnitPulse/2){
		mHandle->anglePID.err += UnitPulse;
	}
	mHandle->anglePID.err_sum += mHandle->anglePID.err * mHandle->anglePID.ki;
	
	//integral limiter
	if(mHandle->anglePID.err_sum > mHandle->CurrentLimit){
		mHandle->anglePID.err_sum = mHandle->CurrentLimit;
	}else if(mHandle->anglePID.err_sum < -mHandle->CurrentLimit){
		mHandle->anglePID.err_sum = -mHandle->CurrentLimit;
	}
	
	mHandle->anglePID.err_pre = mHandle->anglePID.err;
	
	//mHandle->iqPID.target = mHandle->anglePID.kp * mHandle->anglePID.err + mHandle->anglePID.err_sum;
	//mHandle->idPID.target = 0;
	//mHandle->speedPID.target = mHandle->anglePID.kp * mHandle->anglePID.err + mHandle->anglePID.err_sum + mHandle->anglePID.kd*(mHandle->anglePID.err - mHandle->anglePID.err_pre);
	float target = mHandle->anglePID.kp * mHandle->anglePID.err + mHandle->anglePID.err_sum + mHandle->anglePID.kd*(mHandle->anglePID.err - mHandle->anglePID.err_pre);

	SetSpeedTarget(mHandle, target);
//	if(mHandle->iqPID.target > mHandle->CurrentLimit){
//		mHandle->iqPID.target = mHandle->CurrentLimit;
//	}else if(mHandle->iqPID.target < -mHandle->CurrentLimit){
//		mHandle->iqPID.target = -mHandle->CurrentLimit;
//	}
}

void SetAnglePID(Motor* mHandle, float kp, float ki, float kd){
	mHandle->anglePID.kp = kp;
	mHandle->anglePID.ki = ki;
	mHandle->anglePID.kd = kd;
}

void SetAngleTarget(Motor* mHandle, float Target){
	if(Target < 0){
		Target = - Target;
	}
	if(Target>4096){
		Target = (int)Target %4096;
	}
	mHandle->anglePID.target = Target;
}

void SetIncrementAngle(Motor* mHandle, float Target){
	Target += mHandle->M_angle;
	Target = (int)Target%UnitPulse;
	SetAngleTarget(mHandle,Target);
}

//float ab;
void FOC(Motor* mHandle){
	getEangle(mHandle);
	
	CurrentReconstruction(mHandle);
	//abc to alpha beta
	ClarkeTransform(&mHandle->Current);
	//mHandle->E_angle = calcAngle(mHandle);
	//alpha beta to dq
	ParkTransform(&mHandle->Current,mHandle->E_angle);
	//printf("%f,%f,%d,%d,%d\n",mHandle->Current.alpha, mHandle->Current.beta,calcAngle(mHandle),mHandle->M_angle,mHandle->E_angle);
	//PID
	switch (mHandle->Mode){
		case Speed:
			SpeedPID(mHandle);
			break;
		case Position:
			AnglePID(mHandle);
			SpeedPID(mHandle);
			break;
		case Torque:
			//AnglePID(mHandle);
			//SpeedPID(mHandle);
			dq2SVPWM(mHandle);
			return;
		default:
			//SpeedPID(mHandle);
			break;
	}
	
	IdPID(mHandle);
	IqPID(mHandle);
	//printf("%.2f\n",mHandle->speed);
	//printf("%d,%f\n",mHandle->M_angle,mHandle->anglePID.target);
	//printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",mHandle->Current.d,mHandle->idPID.target,mHandle->Current.q,mHandle->iqPID.target,mHandle->speed,mHandle->speedPID.err,mHandle->Voltage.d,mHandle->Voltage.q);
	//printf("%.2f,%.2f,%.2f,%.2f,%d,%.2f,%.2f,%.2f\n",mHandle->Current.d,mHandle->idPID.target,mHandle->Current.q,mHandle->iqPID.target,mHandle->M_angle,mHandle->anglePID.target,mHandle->Voltage.d,mHandle->Voltage.q);
	//printf("%f,%f,%f,%f,%d,%d,%f,%f\n",mHandle->Current.d,mHandle->idPID.target,mHandle->Current.q,mHandle->iqPID.target,mHandle->speed, mHandle->E_angle/1000 + 30,mHandle->Voltage.d,mHandle->Voltage.q);
	//printf("%f,%f,%f,%f,%f,%d,%f,%f,%f,%f\n",mHandle->Current.d,mHandle->idPID.target,mHandle->Current.q,mHandle->iqPID.target,mHandle->speed/100+1, mHandle->E_angle,mHandle->Voltage.d,mHandle->Voltage.q,
	//ab,atanf(mHandle->Current.alpha/mHandle->Current.beta) );
	//printf("%f,%f,%f,%f\n",mHandle->idPID.err,mHandle->iqPID.err,mHandle->Voltage.d,mHandle->Voltage.q);
	//dq to alpha beta
	//int theta = 1333;
	InvPark(&mHandle->Voltage,mHandle->E_angle);
	//mHandle->Voltage.alpha = 10;
	//mHandle->Voltage.beta = 0;
	//int theta = calcAngle(mHandle);
	//ParkTransform(&mHandle->Voltage,mHandle->E_angle);
	//printf("%d,%d,%f,%f\n",mHandle->M_angle,mHandle->E_angle,mHandle->Voltage.d,mHandle->Voltage.q);
	SetSVPWM(mHandle);
	//printf("%f\n",mHandle->speed);
	//mHandle->Voltage.d = 0;
	//mHandle->Voltage.q = 1;
	//for(int i = 0;i<4000;i++){
		//mTest.Voltage.d = i;
		//mTest.Voltage.q = i;
		//float theta = i;
		//InvPark(&mHandle->Voltage,i);
		//mHandle->Voltage.alpha = arm_cos_f32((float)i/4000*(2*pi));
		//mHandle->Voltage.beta = arm_sin_f32((float)i/4000*(2*pi));
		//ParkTransform(&mTest.Voltage,i);
		//GetSVPWM(mHandle);
		//printf("%f,%f,%f,%f\n",a,b,c,(float)i/40000);
		//GetSVPWM(mHandle);
	//}

	//__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0.1*htim1.Instance->ARR);
	//__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0.7*htim1.Instance->ARR);
	//__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0.7*htim1.Instance->ARR);
	
}

void FOCcheck(Motor* mHandle){
	printf("idPID kp %.4f,ki %.4f, target %.4f\n", mHandle->idPID.kp, mHandle->idPID.ki, mHandle->idPID.target);
	printf("iqPID kp %.4f,ki %.4f, target %.4f\n", mHandle->iqPID.kp, mHandle->iqPID.ki, mHandle->idPID.target);
	printf("speedPID kp %.4f,ki %.4f, target %.4f\n", mHandle->speedPID.kp, mHandle->speedPID.ki, mHandle->speedPID.target);
	printf("anglePID kp %.4f,ki %.4f, target %.4f\n", mHandle->anglePID.kp, mHandle->anglePID.ki, mHandle->anglePID.target);
}



