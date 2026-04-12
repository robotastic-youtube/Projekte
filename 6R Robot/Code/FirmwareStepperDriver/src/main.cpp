/*
================================================================================
File: main.cpp
Author: Robotastic (YouTube)
Copyright (c) 2026 Robotastic

Description: 
Main application entry point. Instantiates all hardware interfaces, configures 
the system components (Board, CANbus, StepperFOC), and links them together 
via the BoardControler. Contains the primary setup and continuous execution 
loops.

License:
This software is provided for educational and personal use only. 
You may freely use, modify, and distribute this code for non-commercial 
purposes. Any commercial use, sale, or incorporation into commercial 
products is strictly prohibited without prior written permission from 
the author.
================================================================================
*/

#include <Arduino.h>
#include "Board.h"
#include "stm32g4xx_hal.h"
#include "CANbus.h"
#include "StepperFOC.h"
#include "BoardControler.h"

// Redefine the High-Speed External (HSE) oscillator value.
// This is necessary for the STM32 HAL library to calculate the correct clock frequencies.
#undef HSE_VALUE
#define HSE_VALUE ((uint32_t)8000000)

// --- Global Objects ---
// These objects represent the main components of the system.

Board board; // Instance of the Board class, which acts as the Hardware Abstraction Layer (HAL).
FDCAN_HandleTypeDef hfdcan1; // STM32 HAL handle for the FDCAN1 peripheral.
CANbus customCAN(hfdcan1); // Custom CANbus wrapper instance, passing the HAL handle.
StepperFOC stepper(50, 24.0, 24.0, 5.0); // Instance for the FOC motor control with parameters:
                                         // - 50 pole pairs (common for 1.8-degree steppers)
                                         // - 24.0V power supply voltage
                                         // - 24.0V driver voltage limit
                                         // - 5.0V sensor alignment voltage
BoardControler boardControler; // Instance of the main application controller that orchestrates everything.


// --- Configuration ---
bool activate_debug = false; // Global flag to enable or disable serial debug output.
uint8_t canReceiveID = 0x014; // ID for incomming messages which get received
uint8_t canSendID = 0x024; // ID for sending messages
uint32_t canBaudrate = 125000; // Baudrate of the CAN-Bus -> set to 125 kBit/s

/**
 * @brief The standard Arduino setup function, which runs once on startup.
 */
void setup() {

    // If debug mode is active, initialize the Serial port and enable debug prints
    // for SimpleFOC and the custom controller class.
    if (activate_debug) {
        Serial.begin(115200);
        while (!Serial); // Wait for the serial port to connect.
        SimpleFOCDebug::enable(&Serial);
        boardControler.activateDebug();
        Serial.println("Debug Mode activated.");
    }

    // Initialize all GPIO pins as defined in the Board class configuration.
    board.initGPIOPins();
    // Set a specific GPIO pin to a known state.
    board.writeGPIOPin(GPIOA, GPIO_PIN_10, false);

    // Configure board-specific parameters like CAN IDs and the endstop pin location.
    board.setCANReceiveID(canReceiveID); 
    board.setCANSendID(canSendID);
    board.setCANBaudrate(canBaudrate);
    board.setEndstopPin(GPIOB, GPIO_PIN_12);

    // A small delay can be useful to let hardware components stabilize.
    delay(10);
    
    // Link all the created objects (Board, CAN, Stepper) to the main controller.
    // This uses dependency injection to connect the different parts of the application.
    boardControler.linkBoard(&board);
    boardControler.linkCANBus(&customCAN);
    boardControler.linkStepperFOC(&stepper);

    // Set up and enable the interrupt for receiving CAN messages.
    boardControler.activateCANInterrupt();
    // Initialize all linked components (this will call initFOC, initCANbus, etc.).
    boardControler.initAll();
    // Start the high-frequency hardware timer for the motion control loop.
    // The interval is 1000 microseconds, resulting in a 1kHz control loop frequency.
    boardControler.activateMoveStepperInterrupt(1000);

}

/**
 * @brief The standard Arduino loop function, which runs repeatedly after setup.
 */
void loop() {
    // Execute the main loop of the board controller. This function is responsible for
    // calling motor.loopFOC(), which executes the core FOC algorithm calculations.
    boardControler.loop();
}