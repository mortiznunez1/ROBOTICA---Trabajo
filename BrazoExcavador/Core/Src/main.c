/* USER CODE BEGIN Header */
/**
  **************************
  * @file           : main.c
  * @brief          : Main program body
  **************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Cinematica.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "i2c-lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFERSIZE 32
#define ADCResolution 4096
#define ServoRange 120
#define PWM_MAX __HAL_TIM_GET_AUTORELOAD(&htim1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
Position p = {15.0f, 0.0f, 12.0f};
float orientacion = 45.0f;                 // Ejemplo
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



volatile int32_t lectura1_adc;
volatile int32_t lectura1_grados;
volatile int32_t lectura1_volt;
volatile int32_t lectura2_adc;
volatile int32_t lectura2_grados;
volatile int32_t lectura2_volt;

volatile float error1;
volatile float error_previo1;
volatile float derivada1;
volatile float salida1;
volatile float error2;
volatile float error_previo2;
volatile float derivada2;
volatile float salida2;

volatile float dt=0.01;

volatile float Posicion_deseada_motor_1;
volatile float Posicion_deseada_motor_2;


volatile float Kp1=1;
volatile float Kd1=0;
volatile float Kp2=1;
volatile float Kd2=0;

char readBuf[BUFFERSIZE];  // Buffer para recibir la cadena completa
volatile uint8_t flag = 0; // Indica cuándo se ha recibido una cadena completa
         // Caracter recibido
//uint8_t step_index = 0;
//uint8_t step_state = 0;
volatile uint8_t step_pulse_state = 0; // 0 = inactivo, 1 = STEP HIGH esperando STEP LOW
volatile float angle[4] = {};
volatile float angle_buf[4] = {};
volatile uint8_t angle_ready[4] = {};
volatile uint8_t moving = 0;
volatile float angle_difference = 0;

volatile uint8_t bluetoothSetter = 0; // Iniciar la transmisión bluetooth
volatile uint8_t x,y,z = 0;

Position DesiredPosition = {};

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {

	static uint8_t index = 1; // Posición en el buffer
	static char readChar;
	if (UartHandle->Instance == USART6) {
		HAL_UART_Receive_IT(&huart6, (uint8_t *)&readChar, 1); // Recibir próximo carácter

		if (readChar == '%'||readChar == '\000') { // Indicador de fin de mensaje
			readBuf[index] = '\0'; // Terminar el string
			flag = 1;              // Indicar que el mensaje está listo
			index = 0;             // Reiniciar el índice
			HAL_UART_Receive_IT(&huart6, (uint8_t *)&readBuf, 1); // Iniciar recepción
		} else if (index < BUFFERSIZE - 1) {
			readBuf[index++] = readChar; // Guardar carácter en el buffer
		}
	}
}

void step_once_non_blocking(void) {
    step_pulse_state = 1;
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); // STEP HIGH

    // Programamos evento de 1 ms después
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,__HAL_TIM_GET_COUNTER(&htim4) + 1000); // 1000 ticks = 1ms si timer a 1MHz
    HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);
}


void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        if (step_pulse_state == 1) {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); // STEP LOW
            step_pulse_state = 0;
            HAL_TIM_OC_Stop_IT(&htim4, TIM_CHANNEL_1); // Paramos el timer hasta el siguiente paso
        }
    }
}

void SetPosition(TIM_HandleTypeDef *htim,uint16_t PulseWidth){
		if(htim->Instance == TIM1)
			__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, PulseWidth);
	}

//------------------------------------DC Motor Code--------------------------------------//
void SetSpeed(TIM_HandleTypeDef *htim,uint16_t PulseWidth){
		if(htim->Instance == TIM2)
			__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, PulseWidth);
		else if(htim->Instance == TIM2)
		    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, PulseWidth);
	}
//----------------------------------End DC Motor Code--------------------------------------//

void ParameterRegister (void){
	//	Otra posible opción es que el panel informe de operación a realizar
	// 	Por ejemplo: recogiendo arena o descargando arena. Se decidirá posteriormente
	lcd_enviar("Voltaje:",0,0);
	lcd_enviar("Intensidad:",1,0);
}

void set_stepper(void){ //Para ajustar a 1/16 de paso
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);   // HIGH
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);   // HIGH
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // HIGH
}

void step_once(void) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);   // STEP HIGH
    HAL_Delay(1);                                          // 1 ms (ajustable)
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); // STEP LOW
    HAL_Delay(1);                                          // Delay entre pasos
}

void move_stepper_degrees(float angle, uint8_t dir) {
    int steps = (int)((angle / 360.0) * 3200 * 91.0/35.0);  // calcular pasos
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, dir); // Dirección: 0 = CW, 1 = CCW
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET); // ENABLE_N: LOW para habilitar

    for (int i = 0; i < steps; i++) {
        step_once();
    }
}

int strtoint(char readBuf[]) {

    int i = 0;
    int angle = 0;

    if((readBuf[0]='!'||'$'||'&'||'/')){
    	// Iteración hasta encontrar el final de la cadena
    	    while (readBuf[i] != '\0' || readBuf[i] != '\000') {  // Finaliza al llegar a '\0' o '\000'

    			// Solo se procesan los caracteres numéricos
    				if (readBuf[i] >= '0' && readBuf[i] <= '9') {
    					angle = angle * 10 + (readBuf[i] - '0');
    				}

    	        i++;  // Se avanza al siguiente carácter
    	    }

    }

    return angle;
}

void BufferCleaner(void){

	for(uint8_t j=0; j<BUFFERSIZE; j++) readBuf[j];
}

void PositionManager(void) {

	uint8_t i = 0;
	int temp = 0;	// Valor temporal de la posición enviada
	char axis = 0;	// Eje a estudio

	    while (i < BUFFERSIZE && readBuf[i] != '\0') {
	        char c = readBuf[i];

	        // Detectar letras de ejs coordenados
	        if (c == 'X' || c == 'Y' || c == 'Z') {
	            axis = c;
	            temp = 0;
	            i++; // avanzar al siguiente carácter

	            // Acumular números mientras sean dígitos válidos
	            while (i < BUFFERSIZE && readBuf[i] >= '0' && readBuf[i] <= '9') {
	                temp = temp * 10 + (readBuf[i] - '0');
	                i++;
	            }

	            // Almacenar en el eje correspondiente
	            if (axis == 'X') x = temp;
	            else if (axis == 'Y') y = temp;
	            else if (axis == 'Z') z = temp;
	        } else {
	            i++;
	        }
	    }
}


void GeneralMoveMotor(uint8_t n)
{
	float angle_aux = 0;

	if((angle_aux = strtoint(readBuf)) != angle[n]){
		angle_buf[n] = angle[n];
		angle[n] = angle_aux;
		angle_ready[n] = 1;
	}
}

void BluetoothAction(void){
	if(flag){

		flag = 0;

		switch(readBuf[0]){
				case '!':
					if(!moving){

						GeneralMoveMotor(0);

						if(angle_ready[0]){

							  angle_ready[0] = 0;
							  angle_difference = angle[0] - angle_buf[0];

							  if(angle[0] >= angle_buf[0]) {
									move_stepper_degrees(angle_difference, 1);
								}

							else {
									move_stepper_degrees(-angle_difference, 0);
								}
					  }
					}

					break;
				case '$':
					if(!moving){

						GeneralMoveMotor(1);

						if(angle_ready[1]){

							  angle_ready[1] = 0;
							  angle_difference = angle[1] - angle_buf[1];

							  if(angle[1] >= angle_buf[1]) {
								  SetPosition(&htim1,angle[1]);
							  }

							else {
								  SetPosition(&htim1,angle[1]);
								}
					  }
					}
						break;
				case '&':
					if(!moving){

						GeneralMoveMotor(2);

						if(angle_ready[2]){

							  angle_ready[2] = 0;
							  angle_difference = angle[2] - angle_buf[2];

							  if(angle[2] >= angle_buf[2]) {
								  //Poner la del motor
							  }

							else {
								  //Poner la del motor
								}
					  }
					}
						break;
				case '/':
					if(!moving){

						GeneralMoveMotor(3);

						if(angle_ready[3]){

							  angle_ready[3] = 0;
							  angle_difference = angle[3] - angle_buf[3];

							  if(angle[3] >= angle_buf[3]) {
								  //Poner la del motor
							  }

							else {
								  //Poner la del motor
								}
					  }
					}
						break;
				case '?':
					if(!moving){
						//Angles inverse_kinematics(Position p, float desired_orientation) {
						PositionManager();
					}
						break;
			}

		BufferCleaner();
	}

}

void BluetoothSetter(void){

	if(readBuf[24] =='t' && readBuf[25] == 'h' )
		BufferCleaner();

}


void ReadADC(ADC_HandleTypeDef *hadc, volatile uint32_t *variable){

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	*variable=HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
}

void Change_VoltGrados(uint32_t *adc_val){
	lectura1_grados = (240.0f *(*adc_val))/4095.0f;
}

void Change_GradosVolt(uint32_t *Posicion){
	lectura1_volt=(3.3f *(*Posicion))/240.0f;
}

void Change_DigVolt(volatile uint32_t *adc_val, volatile  uint32_t *lectura){
	*lectura=(3.3f*(float)(*adc_val))/4095.0f;
}

void ControlPD(volatile float setpoint,volatile float medida, volatile float *error,
		volatile float *derivada,volatile float *salida, volatile float *error_previo,uint8_t Motor){

	*error=setpoint-medida;
	*derivada=(*error- *error_previo)/dt;

	if (Motor==1){

		*salida =Kp1 * *error + Kd1 * *derivada;

		if (*salida > 0){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);
			*salida= -*salida;  //Hacer PWM positivo
			if(*salida >1.0f) *salida=1.0f;
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, *salida *PWM_MAX);
		}}

	if (Motor==2){

			*salida =Kp2 * *error + Kd2 * *derivada;

			if (*salida > 0){
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);
			} else {
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);
				*salida= -*salida;  //Hacer PWM positivo
				if(*salida >1.0f) *salida=1.0f;
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, *salida *PWM_MAX);
			}}
	*error_previo= *error;

	}


void FeedBackMotor(){

	ReadADC(&hadc1,&lectura1_adc);
	ReadADC(&hadc1,&lectura2_adc);

	Change_DigVolt(&lectura1_adc, &lectura1_volt);
	Change_DigVolt(&lectura2_adc, &lectura2_volt);

	ControlPD(Posicion_deseada_motor_1,lectura1_volt,&error1,&derivada1,&salida1,&error_previo1,1);
	ControlPD(Posicion_deseada_motor_2,lectura2_volt,&error2,&derivada2,&salida2,&error_previo2,2);
}

uint16_t angle_to_pwm(float angle_deg, float min_deg, float max_deg, uint16_t pwm_min, uint16_t pwm_max) {
    if (angle_deg < min_deg) angle_deg = min_deg;
    if (angle_deg > max_deg) angle_deg = max_deg;
    return pwm_min + (uint16_t)((angle_deg - min_deg) * (pwm_max - pwm_min) / (max_deg - min_deg));
}

void EjecutarCinematica(Position posicion_deseada, float orientacion_deseada) {
    // Calcula los ángulos deseados usando cinemática inversa
    Angles a = inverse_kinematics(posicion_deseada, orientacion_deseada);

    // --- θ1: motor paso a paso (base)
        static float theta1_actual = 0.0f;
        float delta_theta1 = a.theta1 - theta1_actual;
        if (fabs(delta_theta1) > 1.0f) {
            uint8_t dir = (delta_theta1 > 0) ? 1 : 0;
            move_stepper_degrees(fabs(delta_theta1), dir);
            theta1_actual = a.theta1;
        }

    // --- θ2: servo motor (hombro)
    uint16_t pwm_theta2 = angle_to_pwm(a.theta2, 0, 180, 500, 2500);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_theta2);

    // --- θ3: motor DC (codo)
    Posicion_deseada_motor_1 = (3.3f * a.theta3) / 240.0f;  // Ajusta si tu escala es distinta

    // --- θ4: motor DC (efector)
    Posicion_deseada_motor_2 = (3.3f * a.theta4) / 240.0f;

    // Ejecuta control PD en ambos motores DC
    FeedBackMotor();

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART6_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();
  set_stepper();
  HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_UART_Receive_IT(&huart6, (uint8_t *)readBuf, 1); // Iniciar recepción

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //	  ParameterRegister();

	  	  BluetoothSetter();

	  	  BluetoothAction();

	  	  //EjecutarCinematica(p, orientacion);

	  //	  SetSpeed(&htim2, 1000);
	  //
	  //	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	  //	  HAL_GPI223.0O_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	  //
	  //	  HAL_Delay(2000);
	  //	  SetSpeed(&htim2, 100);
	  //
	  //	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	  //      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	  //
	  //      HAL_Delay(2000);
	  //
	  //      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	  //      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	  //
	  //      HAL_Delay(2000);


	  //	  move_stepper_degrees(270,1);
	  //	  HAL_Delay(5000);
	  //	  move_stepper_degrees(180,0);
	  //	  HAL_Delay(5000);

	  //	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET); // Dirección: 0 ó 1
	  //	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET); // ENABLE_N: LOW para activar
	  //
	  //	  for (int i = 0; i < 3200; i++) { // 3200 pasos = 1 vuelta (con 1/16 microstepping)
	  //
	  //	      step_once();
	  //	  }

	    }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 160-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 12;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|MS3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, MS2_Pin|MS1_Pin|ENABLE_N_Pin|DIR_Pin
                          |STEP_Pin|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 MS3_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|MS3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : MS2_Pin MS1_Pin ENABLE_N_Pin DIR_Pin
                           STEP_Pin PD15 */
  GPIO_InitStruct.Pin = MS2_Pin|MS1_Pin|ENABLE_N_Pin|DIR_Pin
                          |STEP_Pin|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
