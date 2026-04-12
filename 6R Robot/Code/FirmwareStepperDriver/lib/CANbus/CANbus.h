/*
================================================================================
File: CANbus.h
Author: Robotastic (YouTube)
Copyright (c) 2026 Robotastic

Description: 
Header file for the CANbus class, which provides a high-level interface for 
FDCAN communication on STM32 microcontrollers. It encapsulates the STM32 
HAL FDCAN library, simplifying initialization, configuration, message 
filtering, and transmission.

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
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"
#include <cstdlib>
#include <vector>
#include <map>
#include <functional>

/**
 * @struct CANConfig
 * @brief Holds all general configuration parameters for the FDCAN peripheral.
 */
struct CANConfig {
    // General FDCAN settings
    uint32_t ClockDivider = FDCAN_CLOCK_DIV1;        //!< Clock divider for the FDCAN peripheral clock.
    uint32_t FrameFormat = FDCAN_FRAME_CLASSIC;      //!< Frame format (e.g., Classic CAN, CAN-FD).
    uint32_t Mode = FDCAN_MODE_NORMAL;               //!< Operating mode (e.g., Normal, Loopback).

    // Behavior settings
    FunctionalState AutoRetransmission = DISABLE;    //!< Enable/disable automatic retransmission of failed messages.
    FunctionalState TransmitPause = DISABLE;         //!< Enable/disable transmit pause feature.
    FunctionalState ProtocolException = DISABLE;     //!< Enable/disable protocol exception handling.

    // Nominal (Arbitration Phase) Bit Timing
    uint32_t NominalPrescaler = 16;                  //!< Prescaler for nominal bit rate timing.
    uint32_t NominalSyncJumpWidth = 1;               //!< Nominal synchronization jump width.
    uint32_t NominalTimeSeg1 = 13;                   //!< Nominal time segment 1.
    uint32_t NominalTimeSeg2 = 4;                    //!< Nominal time segment 2.

    // Data (Data Phase) Bit Timing - for CAN-FD
    uint32_t DataPrescaler = 1;                      //!< Prescaler for data phase bit rate timing.
    uint32_t DataSyncJumpWidth = 1;                  //!< Data phase synchronization jump width.
    uint32_t DataTimeSeg1 = 1;                       //!< Data phase time segment 1.
    uint32_t DataTimeSeg2 = 1;                       //!< Data phase time segment 2.

    // Filter and FIFO settings
    uint32_t StdFiltersNbr = 0;                      //!< Number of standard ID filters to configure.
    uint32_t ExtFiltersNbr = 0;                      //!< Number of extended ID filters to configure.
    uint32_t TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION; //!< Transmit FIFO/Queue mode.
    uint32_t CLKSource = RCC_FDCANCLKSOURCE_PCLK1;   //!< Clock source for the FDCAN peripheral.
};

/**
 * @struct TXHeaderConfig
 * @brief Holds default configuration parameters for outgoing CAN message headers.
 */
struct TXHeaderConfig {
    uint32_t Identifier = 0x124;                     //!< Default CAN identifier for outgoing messages.
    uint32_t IdType = FDCAN_STANDARD_ID;             //!< Default ID type (Standard or Extended).
    uint32_t TxFrameType = FDCAN_DATA_FRAME;         //!< Default frame type (Data or Remote).
    uint32_t BitRateSwitch = FDCAN_BRS_OFF;          //!< Bit rate switching for CAN-FD (On or Off).
    uint32_t ErrorStateIndicator = FDCAN_ESI_ACTIVE; //!< Error state indicator.
    uint32_t MessageMarker = 0;                      //!< An optional message marker for application use.
};

/**
 * @struct FilterConfig
 * @brief Holds configuration parameters for a single CAN message filter.
 */
struct FilterConfig {
    uint32_t IdType = FDCAN_STANDARD_ID;             //!< ID type for this filter (Standard or Extended).
    uint32_t FilterIndex = 0;                        //!< Index of the filter (0 to StdFiltersNbr-1 or ExtFiltersNbr-1).
    uint32_t FilterType = FDCAN_FILTER_MASK;         //!< Filter type (e.g., Mask, Range).
    uint32_t FilterID1 = 0x000;                      //!< First ID for the filter configuration (e.g., the ID to match).
    uint32_t FilterID2 = 0x000;                      //!< Second ID for the filter configuration (e.g., the mask).
    uint32_t FilterConfig = FDCAN_FILTER_TO_RXFIFO0; //!< Action to take on a filter match (e.g., store in FIFO0).
};

/**
 * @class CANbus
 * @brief A wrapper class for the STM32 HAL FDCAN library.
 */
class CANbus {
public:
    CANbus() = default;
    CANbus(FDCAN_HandleTypeDef& hfdcan);

    // --- Getters for configuration structs & Handles ---
    FilterConfig& getFilterConfig();
    CANConfig& getCANConfig();
    TXHeaderConfig& getTXHeaderConfig();
    FDCAN_HandleTypeDef* getHandle();

    // --- Initialization and Configuration ---
    void initCANbus(uint32_t baudrate = 0, FDCAN_GlobalTypeDef* instance = FDCAN1);
    void addFilter(uint32_t id1, uint32_t id2, bool extendedId = false);
    void activateDebug();

    // --- Interrupt Management ---
    void enableInterrupt();
    void setReceiveCallback(const std::function<void(FDCAN_RxHeaderTypeDef, uint8_t*)>& callback);
    std::function<void(FDCAN_RxHeaderTypeDef, uint8_t*)> getReceiveCallback();

    // --- Message Transmission and Reception ---
    bool sendMessage(uint32_t id, const uint8_t* data, uint8_t length);
    bool receiveMessage();
    std::pair<FDCAN_RxHeaderTypeDef, std::vector<uint8_t>> getLastMessage(uint32_t id);

    // --- Utility Functions ---
    uint32_t getDLCFromLength(uint8_t length);
    uint8_t getLengthFromDLC(uint32_t dlc);

private:
    // --- Private Member Variables ---
    bool _debug = false;
    bool _enableInterrupt = false;

    uint32_t _stdIdFilterNum = 0;
    uint32_t _extIdFilterNum = 0;

    std::vector<FilterConfig> _filters;

    FDCAN_HandleTypeDef _hfdcan;
    FDCAN_FilterTypeDef _sFilterConfig;
    FDCAN_RxHeaderTypeDef _rxHeader;
    FDCAN_TxHeaderTypeDef _txHeader;

    CANConfig _canConfig;
    TXHeaderConfig _txHeaderConfig;
    FilterConfig _filterConfig;

    std::map<uint32_t, std::pair<FDCAN_RxHeaderTypeDef, std::vector<uint8_t>>> _messages;
    std::function<void(FDCAN_RxHeaderTypeDef, uint8_t*)> _receiveCallback = nullptr;

    // --- Private Methods ---
    uint32_t getCANClockFrequency();
    uint32_t calcPrescaler(uint32_t desiredBaudRate);
};