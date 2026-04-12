/*
================================================================================
File: Board.h
Author: Robotastic (YouTube)
Copyright (c) 2026 Robotastic

Description: 
Header file for the Board class. This class serves as a Hardware Abstraction 
Layer (HAL) for the specific microcontroller board. It handles low-level 
initializations like the system clock, GPIO pins, and hardware timers, 
providing a clean, centralized interface for the main application logic.

License:
This software is provided for educational and personal use only. 
You may freely use, modify, and distribute this code for non-commercial 
purposes. Any commercial use, sale, or incorporation into commercial 
products is strictly prohibited without prior written permission from 
the author.
================================================================================
*/

#pragma once

#include <Arduino.h>
#include <vector>
#include <cstdlib>
#include <functional>
#include <HardwareTimer.h>
#include <map>

#include "stm32g4xx_hal.h"

/**
 * @struct GPIOPinConfig
 * @brief Associates a GPIO port with its corresponding HAL initialization structure.
 */
struct GPIOPinConfig {
    GPIO_TypeDef* port;     //!< Pointer to the GPIO port (e.g., GPIOA, GPIOB).
    GPIO_InitTypeDef init;  //!< HAL structure containing pin mode, pull, speed, etc.
};

/**
 * @struct GPIOConfig
 * @brief Contains a vector of all GPIO pin configurations for the board.
 * This provides a centralized place to define the setup for all required pins.
 */
struct GPIOConfig {
    std::vector<GPIOPinConfig> pinConfigs = {
        // --- Analog Inputs for Current Sensing ---
        {
            GPIOB,
            {
                .Pin = GPIO_PIN_0,
                .Mode = GPIO_MODE_ANALOG,
                .Pull = GPIO_NOPULL
            }
        },
        {
            GPIOB,
            {
                .Pin = GPIO_PIN_1,
                .Mode = GPIO_MODE_ANALOG,
                .Pull = GPIO_NOPULL
            }
        },

        // --- FDCAN Pins ---
        {
            GPIOB,
            {
                .Pin = GPIO_PIN_8, // CAN RX
                .Mode = GPIO_MODE_AF_PP,
                .Pull = GPIO_NOPULL,
                .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
                .Alternate = GPIO_AF9_FDCAN1
            }
        },
        {
            GPIOB,
            {
                .Pin = GPIO_PIN_9, // CAN TX
                .Mode = GPIO_MODE_AF_PP,
                .Pull = GPIO_NOPULL,
                .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
                .Alternate = GPIO_AF9_FDCAN1
            }
        },

        // --- Input Pins (Example) ---
        {
            GPIOA,
            {
                .Pin = GPIO_PIN_7,
                .Mode = GPIO_MODE_INPUT,
                .Pull = GPIO_NOPULL,
                .Speed = GPIO_SPEED_FREQ_LOW
            }
        },
        {
            GPIOA,
            {
                .Pin = GPIO_PIN_5,
                .Mode = GPIO_MODE_INPUT,
                .Pull = GPIO_NOPULL,
                .Speed = GPIO_SPEED_FREQ_LOW
            }
        },

        // --- Output Pins (Example) ---
        {
            GPIOA,
            {
                .Pin = GPIO_PIN_10,
                .Mode = GPIO_MODE_OUTPUT_PP,
                .Pull = GPIO_NOPULL,
                .Speed = GPIO_SPEED_LOW
            }
        },
        {
            GPIOB,
            {
                .Pin = GPIO_PIN_11,
                .Mode = GPIO_MODE_OUTPUT_PP,
                .Pull = GPIO_NOPULL,
                .Speed = GPIO_SPEED_FREQ_LOW
            }
        },

        // --- Endstop Input Pin ---
        {
            GPIOB,
            {
                .Pin = GPIO_PIN_12,
                .Mode = GPIO_MODE_INPUT,
                .Pull = GPIO_NOPULL,
                .Speed = GPIO_SPEED_FREQ_LOW
            }
        }
    };
};

/**
 * @struct GPIOPin
 * @brief A simple struct to hold a GPIO port and pin number.
 */
struct GPIOPin {
    GPIO_TypeDef* port; //!< Pointer to the GPIO port.
    uint16_t pin;       //!< The pin number (e.g., GPIO_PIN_12).
};

/**
 * @class Board
 * @brief Hardware Abstraction Layer for the custom board.
 * This class manages low-level hardware initialization and provides
 * simplified access to peripherals like GPIO and timers.
 */
class Board {
public:
    // --- Constructor. Initializes HAL and sets up the static instance pointer ---
    Board();

    // --- Gets a reference to the GPIO configuration struct ---
    GPIOConfig& getGPIOConfig();

    // --- Initialization Functions ---
    void initSystemClock();
    void initGPIOPins();
    void activateDebug();

    // --- Configuration Setters ---
    void setCANSendID(uint8_t canSendID);
    void setCANReceiveID(uint8_t canReceiveId);
    void setCANBaudrate(uint32_t canBaudrate);
    void setEndstopPin(GPIO_TypeDef* port, uint16_t pin);
    
    // --- GPIO Control ---
    void addGPIOPin(GPIO_TypeDef* port, GPIO_InitTypeDef initStruct);
    void writeGPIOPin(GPIO_TypeDef* port, uint16_t pin, bool state);
    bool readGPIOPin(GPIO_TypeDef* port, uint16_t pin);

    // --- Starts a hardware timer to call a function at a specified interval ---
    void startHardwareTimer(TIM_TypeDef* timerInstance, uint32_t usInterval, std::function<void()> callback);
    
    // --- Configuration Getters ---
    uint8_t getCANSendID();
    uint8_t getCANReceiveID();
    uint32_t getCANBaudrate();
    GPIOPin getEndstopPin();

private:
    // --- Board-specific Properties ---
    uint8_t _canSendID;      //!< The CAN ID used for sending messages from this board.
    uint8_t _canReceiveID;   //!< The CAN ID this board listens to.
    uint32_t _canBaudrate = 125000;   //!< The CAN Baudrate for this board.

    GPIOConfig _gpioConfig;  //!< Struct holding all GPIO pin configurations.
    GPIOPin _endstopPin;     //!< Struct holding the configured endstop pin.

    bool _debug = false;     //!< Flag to enable/disable debug prints.

    // --- Timer Management ---
    std::map<TIM_TypeDef*, HardwareTimer*> _timers; // A map to store HardwareTimer objects, indexed by the timer peripheral instance.
    std::map<TIM_TypeDef*, std::function<void()>> _timerCallbacks; // A map to store callback functions for each timer.

    // --- A static bridge function to handle timer interrupts ---
    static void handleStaticInterrupt(TIM_TypeDef* instance);
};