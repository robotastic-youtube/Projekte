// Include STM32 HAL headers for hardware abstraction
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_rcc.h"

// USB CDC (virtual COM port) interface
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"

// SimpleFOC library and drivers
#include <SimpleFOC.h>
#include <SPI.h>
#include <Wire.h>
#include "SimpleFOCDrivers.h"
#include "encoders/as5047/MagneticSensorAS5047.h"

// Redefine external clock source to 8 MHz (ceramic resonator)
#undef HSE_VALUE
#define HSE_VALUE ((uint32_t)8000000)  

// Motor phase pin configuration
#define PHASE_A_PIN1 PA3
#define PHASE_A_PIN2 PA2
#define PHASE_B_PIN1 PA1
#define PHASE_B_PIN2 PA0
#define ENABLE_A_PIN PA4
#define ENABLE_B_PIN PA6

// AS5047 encoder SPI pins
#define AS5047_CS PA15 
#define AS5047_MOSI PB5 
#define AS5047_MISO PB4 
#define AS5047_SCK PB3 

// I2C address definition
#define I2C_ADDRESS 0x42

// Peripheral handle declarations
ADC_HandleTypeDef hadc1;
PCD_HandleTypeDef hpcd_USB_FS;
I2C_HandleTypeDef hi2c2;

// SPI instance for AS5047
SPIClass SPI3Instance(AS5047_MOSI, AS5047_MISO, AS5047_SCK);

// Motor and driver setup
StepperMotor motor = StepperMotor(50); // Stepper motor with 50 pole pairs
StepperDriver4PWM driver = StepperDriver4PWM(PHASE_A_PIN1, PHASE_A_PIN2, PHASE_B_PIN1, PHASE_B_PIN2, ENABLE_A_PIN, ENABLE_B_PIN);
MagneticSensorAS5047 sensor = MagneticSensorAS5047(AS5047_CS, 1);
LowsideCurrentSense current_sense = LowsideCurrentSense(910.0, PB0, PB1); // 910 mV/A, ADC pins PB0 and PB1

// Forward declarations for system configuration
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);

// Timer variables for control loop
unsigned long previousMillis = 0;
unsigned long previousPreviousMillis = 0;
const unsigned long interval = 10;

float motor_target = 0.0;

void setup() {
  HAL_Init();                      // Initialize the HAL library
  SystemClock_Config();           // Configure system clock
  MX_GPIO_Init();                 // Initialize GPIO
  MX_USB_PCD_Init();             // Initialize USB peripheral
  MX_ADC1_Init();                 // Initialize ADC1

  // Configure I2C pins and initialize
  Wire.setSCL(PA9);
  Wire.setSDA(PA8);
  Wire.begin();

  // Configure H-bridge mode pins
  pinMode(PA7, INPUT); // Set H-bridge 1 mode (independent half-bridge, high-Z)
  pinMode(PA5, INPUT); // Set H-bridge 2 mode
  pinMode(PB11, OUTPUT); // General purpose output pin

  sensor.init(&SPI3Instance); // Initialize magnetic sensor

  // Motor driver settings
  driver.voltage_power_supply = 20;
  driver.voltage_limit = 20;
  driver.pwm_frequency = 30000;
  driver.init();  // Initialize motor driver

  // Link and initialize current sensing
  current_sense.linkDriver(&driver);
  current_sense.init();

  // Connect components to motor object
  motor.linkSensor(&sensor);
  motor.linkDriver(&driver);
  motor.linkCurrentSense(&current_sense);

  // Motor control settings
  motor.voltage_limit = driver.voltage_limit / 2;
  motor.voltage_sensor_align = 5;

  // Select motor control modes
  // motor.controller = MotionControlType::velocity;
  motor.controller = MotionControlType::angle;
  // motor.torque_controller = TorqueControlType::voltage;
  // motor.torque_controller = TorqueControlType::foc_current;
  motor.foc_modulation = FOCModulationType::SinePWM; // SinePWM modulation
  // motor.controller = MotionControlType::torque;

  // Current control PID parameters
  motor.PID_current_q.P = 5;
  motor.PID_current_q.I = 1000;
  motor.PID_current_d.P = 5;
  motor.PID_current_d.I = 1000;
  motor.LPF_current_q.Tf = 0.002f;
  motor.LPF_current_d.Tf = 0.002f;
  motor.current_limit = 2;

  // Velocity control PID parameters
  motor.PID_velocity.P = 0.5;
  motor.PID_velocity.I = 10;
  motor.PID_velocity.D = 0;
  motor.LPF_velocity.Tf = 0.05f;

  // Angle (position) control PID parameters
  motor.P_angle.P = 20;
  motor.P_angle.I = 5;
  motor.P_angle.D = 0.001;
  motor.P_angle.output_ramp = 10000;
  motor.velocity_limit = 30;

  // Initialize motor and FOC algorithm
  motor.init();
  motor.initFOC();

  motor_target = 2; // Set target angle
}

void loop() {
  motor.loopFOC(); // Execute FOC algorithm

  unsigned long currentMillis = millis();

  // Run motor control loop every 10 ms
  if (currentMillis - previousMillis >= interval){
    motor.move(motor_target); // Move to target position
    previousMillis = currentMillis;
  }
}

// System clock configuration
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  // Configure internal oscillators and PLL
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 18;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  // Configure system clocks
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

// USB peripheral initialization
static void MX_USB_PCD_Init(void)
{
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;

  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    // Error_Handler();
  }

  HAL_PCD_Start(&hpcd_USB_FS); // Start USB device
}

// GPIO initialization
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
}

// ADC1 initialization
static void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;

  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    // Error_Handler();
  }

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    // Error_Handler();
  }

  // Configure ADC channel
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    // Error_Handler();
  }
}
