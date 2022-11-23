/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "control.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern Motor M1;
extern Motor M2;

float k=1;
float kq=0.01;
float Kspeed = 0.5;
float kpre = 0.5;

int stages1 = 5;
int stages2 = 5;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UART_COMM */
osThreadId_t UART_COMMHandle;
const osThreadAttr_t UART_COMM_attributes = {
  .name = "UART_COMM",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for USER_APP */
osThreadId_t USER_APPHandle;
const osThreadAttr_t USER_APP_attributes = {
  .name = "USER_APP",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of UART_COMM */
  UART_COMMHandle = osThreadNew(StartTask02, NULL, &UART_COMM_attributes);

  /* creation of USER_APP */
  USER_APPHandle = osThreadNew(StartTask03, NULL, &USER_APP_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
		//printf("%f,%f\n",M1.duty, M2.duty);
		//printf("%.2f,%.2f\n",M1.speed,M1.speedPID.target);
		//printf("%.2f,%.2f,%.2f,%.2f\n",M1.speed,M1.speedPID.target,M2.speed,M2.speedPID.target);
		//printf("%d,%f\n",M1.M_angle, M1.anglePID.target);
		//printf("%d,%f,%d,%f\n",M1.M_angle, M1.anglePID.target,M2.M_angle, M2.anglePID.target);
		//printf("%d,%d\n",M1.M_angle, M2.M_angle);
		//printf("%d,%d\n",M2.M_angle, M2.E_angle);
		//printf("%d,%f\n",M1.M_angle, M1.speed);
		printf("%.2f,%.2f,%.2f,%.2f,%d\n",M1.Current.d,M1.idPID.target,M1.Current.q, M1.iqPID.target,M1.M_angle/10);
		//printf("%d,%d,%d\n",M1.Current.a,M1.Current.b,M1.Current.c);
		//printf("%d,%d,%d\n",M2.Current.a,M2.Current.b,M2.Current.c);
		//printf("%.2f,%d,%d\n",M1.speed,M1.M_angle, M1.E_angle);
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the UART_COMM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
    if(M1.APP == Drag){
			ModeSelection(&M1,Idle);
			ModeSelection(&M2,Position);
			SetAngleTarget(&M2,M1.M_angle);
			printf("%d,%d\n",M1.M_angle, M2.M_angle);
			//SetAngleTarget(&M1,1*(M2.M_angle - M1.M_angle));
			//SetAngleTarget(&M2,1*(M1.M_angle - M2.M_angle));
		}
		if(M1.APP == ForceFeedback){
			//method 1
			/*
			ModeSelection(&M1,Position);
			ModeSelection(&M2,Position);
			
			SetAngleTarget(&M1,M2.M_angle);
			SetAngleTarget(&M2,M1.M_angle);
			*/
			//method 2
			/*int angle_diff = M2.M_angle - M1.M_angle;
			if(angle_diff > 4000){
				angle_diff -= UnitPulse;
			}
			if(angle_diff < -4000){
				angle_diff += UnitPulse;
			}

			ModeSelection(&M1,Position);
			ModeSelection(&M2,Position);
			SetIncrementAngle(&M1,k*angle_diff);
			SetIncrementAngle(&M2,M1.M_angle);
			printf("%.2f,%.2f,%.2f,%.2f\n",M1.idPID.target,M1.iqPID.target,M2.idPID.target,M2.iqPID.target);*/
			//printf("%d,%d,%d,%d,%d,%d\n",M1.Current.a,M1.Current.b,M1.Current.c,M2.Current.a,M2.Current.b,M2.Current.c);
			//int a = (M2.M_angle - M1.M_angle);
			//int v = (M2.M_angle - M1.M_angle);
			//method 3
			//ture open loop force feedback
			ModeSelection(&M1,Torque);
			ModeSelection(&M2,Position);
			int angle_diff = M2.M_angle - M1.M_angle;
			if(angle_diff > 2048){
				angle_diff -= UnitPulse;
			}
			if(angle_diff < -2048){
				angle_diff += UnitPulse;
			}
		
			M1.Voltage.q = kq*angle_diff;
			SetAngleTarget(&M2,M1.M_angle);
			printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",M1.Current.d,M1.Current.q,M2.Current.d,M2.Current.q, (float)M1.M_angle/10+500, (float)M2.M_angle/10+500);
			//method 4
			/*ModeSelection(&M1,Torque);
			ModeSelection(&M2,Position);
			M1.iqPID = 
			SetIncrementAngle(&M2,M1.M_angle);*/
		}
		if(M1.APP == Switch){
			
			ModeSelection(&M1,Position);

			int gap1 = 4096/stages1;
			int target1 = M1.M_angle/gap1;
			target1 = target1*gap1;
			if( M1.M_angle%gap1 >gap1/2){
				target1 += gap1;
			}
			
			SetAngleTarget(&M1,target1);
		}
		if(M2.APP == Switch){
			
			ModeSelection(&M2,Position);
			
			int gap2 = 4096/stages2;
			int target2 = M2.M_angle/gap2;
			target2 = target2*gap2;
			if( M2.M_angle%gap2 >gap2/2){
				target2 += gap2;
			}
			
			SetAngleTarget(&M2,target2);
		}	

		osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the USER_APP thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

