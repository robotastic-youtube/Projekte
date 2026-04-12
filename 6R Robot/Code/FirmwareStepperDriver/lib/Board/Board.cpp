/*
================================================================================
File: Board.cpp
Author: Robotastic (YouTube)
Copyright (c) 2026 Robotastic

Description: 
Implementation file for the Board class. Contains the logic for initializing 
the STM32 HAL, configuring the system clock, setting up GPIO pins, and 
managing hardware timers and their associated interrupts.

License:
This software is provided for educational and personal use only. 
You may freely use, modify, and distribute this code for non-commercial 
purposes. Any commercial use, sale, or incorporation into commercial 
products is strictly prohibited without prior written permission from 
the author.
================================================================================
*/

#include "Board.h"

// A static pointer to the Board instance, used by C-style callback functions.
Board* BoardInstance = nullptr;

/**
 * @brief Constructor implementation.
 */
Board::Board() {
    // Set the static instance pointer so C-style functions can find this object.
    BoardInstance = this;
    // Initialize the STM32 HAL library.
    HAL_Init();
}

/**
 * @brief Configures the main system clock of the microcontroller.
 */
void Board::initSystemClock() {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Configure the main internal regulator output voltage.
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    // Initialize the RCC Oscillators according to the specified parameters.
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN = 18;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        if (_debug) Serial.println("HAL_RCC_OscConfig failed.");
        while(true); // Halt on failure
    }

    // Initialize the CPU, AHB and APB buses clocks.
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        if (_debug) Serial.println("HAL_RCC_ClockConfig failed.");
        while(true); // Halt on failure
    }
}

/**
 * @brief Adds a new GPIO pin configuration to the list.
 * @param port The GPIO port (e.g., GPIOA).
 * @param initStruct The HAL initialization structure for the pin.
 */
void Board::addGPIOPin(GPIO_TypeDef* port, GPIO_InitTypeDef initStruct) {
    GPIOPinConfig config = {port, initStruct};
    _gpioConfig.pinConfigs.push_back(config);
}

/**
 * @brief Initializes all GPIO pins defined in the GPIOConfig struct.
 */
void Board::initGPIOPins() {
    // Enable clocks for all used GPIO ports.
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    // Iterate through the configuration vector and initialize each pin.
    for (auto& cfg : _gpioConfig.pinConfigs) {
        HAL_GPIO_Init(cfg.port, &cfg.init);
    }
}

/**
 * @brief Writes a digital state to a GPIO pin.
 * @param port The GPIO port.
 * @param pin The pin number.
 * @param state The desired state (true for HIGH, false for LOW).
 */
void Board::writeGPIOPin(GPIO_TypeDef* port, uint16_t pin, bool state) {
    HAL_GPIO_WritePin(port, pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief Starts and configures a hardware timer for periodic interrupts.
 * * @param timerInstance Pointer to the timer peripheral (e.g., TIM2, TIM3).
 * @param usInterval    The timer overflow interval in microseconds.
 * @param callback      The C++ std::function to be executed when the interrupt occurs.
 */
void Board::startHardwareTimer(TIM_TypeDef* timerInstance, uint32_t usInterval, std::function<void()> callback) {
    if (_debug) Serial.println("Initialize Hardware Timer...");
    
    // Create a new HardwareTimer object if it doesn't exist for this instance.
    if (_timers.count(timerInstance) == 0) {
        _timers[timerInstance] = new HardwareTimer(timerInstance);
    }

    // Get the timer object and store the callback function.
    HardwareTimer* timer = _timers[timerInstance];
    _timerCallbacks[timerInstance] = callback;

    // Configure the timer's overflow period and attach the static interrupt handler.
    timer->pause();
    timer->setOverflow(usInterval, MICROSEC_FORMAT);
    timer->attachInterrupt([=]() {
        Board::handleStaticInterrupt(timerInstance);
    });
    timer->resume();
    if (_debug) Serial.println("Timer started.");
}

/**
 * @brief Static interrupt handler that calls the appropriate C++ member function callback.
 * * @param instance Pointer to the timer peripheral that triggered the interrupt.
 */
void Board::handleStaticInterrupt(TIM_TypeDef* instance) {
    if (BoardInstance && BoardInstance->_timerCallbacks.count(instance)) {
        BoardInstance->_timerCallbacks[instance]();
    }
}

/**
 * @brief Activates debug prints.
 */
void Board::activateDebug() {
    _debug = true;
}

/**
 * @brief Sets the CAN ID this board will listen for.
 * * @param canReceiveID The 8-bit ID to be used for filtering incoming CAN messages.
 */
void Board::setCANReceiveID(uint8_t canReceiveID) {
    _canReceiveID = canReceiveID;
}

/**
 * @brief Sets the CAN ID this board will use for sending messages.
 * * @param canSendID The 8-bit ID to be attached to outgoing CAN messages.
 */
void Board::setCANSendID(uint8_t canSendID) {
    _canSendID = canSendID;
}

/**
 * @brief Configures which GPIO pin is used as the endstop.
 * * @param port Pointer to the GPIO port (e.g., GPIOA, GPIOB).
 * @param pin  The specific pin number (e.g., GPIO_PIN_12).
 */
void Board::setEndstopPin(GPIO_TypeDef* port, uint16_t pin) {
    _endstopPin.port = port;
    _endstopPin.pin = pin;
}

/**
 * @brief Sets the CAN-Bus baudrate.
 * * @param canBaudrate The desired baudrate in bits per second (e.g., 125000, 500000).
 */
void Board::setCANBaudrate(uint32_t canBaudrate) {
    _canBaudrate = canBaudrate;
}

/**
 * @brief Reads the digital state of a GPIO pin.
 * * @param port Pointer to the GPIO port (e.g., GPIOA).
 * @param pin  The specific pin number to read.
 * @return true  If the pin state is HIGH.
 * @return false If the pin state is LOW.
 */
bool Board::readGPIOPin(GPIO_TypeDef* port, uint16_t pin) {
    return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET;
}

// --- Implementation of Getters ---

/**
 * @brief Retrieves the configured CAN receive ID.
 * * @return uint8_t The ID used for filtering incoming CAN messages.
 */
uint8_t Board::getCANReceiveID() { return _canReceiveID; }

/**
 * @brief Retrieves the configured CAN send ID.
 * * @return uint8_t The ID attached to outgoing CAN messages.
 */
uint8_t Board::getCANSendID() { return _canSendID; }

/**
 * @brief Retrieves the configured CAN-Bus baudrate.
 * * @return uint32_t The baudrate in bits per second.
 */
uint32_t Board::getCANBaudrate() { return _canBaudrate; }

/**
 * @brief Retrieves the entire GPIO configuration.
 * * @return GPIOConfig& A reference to the struct containing all pin definitions.
 */
GPIOConfig& Board::getGPIOConfig() { return _gpioConfig; }

/**
 * @brief Retrieves the configured endstop pin.
 * * @return GPIOPin A struct containing the port and pin number of the endstop.
 */
GPIOPin Board::getEndstopPin() { return _endstopPin; }

/**
 * @brief C-style function to be called by the startup code to configure the system clock.
 * * @details This acts as a bridge to the C++ class method. The STM32 HAL requires 
 * a C-linkage function for the system clock configuration, which is 
 * routed here to the Board instance.
 */
extern "C" void SystemClock_Config(void) {
    if (BoardInstance) {
        BoardInstance->initSystemClock();
    }
}