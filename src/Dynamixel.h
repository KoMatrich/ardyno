/**
 * \file Dynamixel.h
 * \brief Define classes for dynamixel protocol
 */

#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include <stdint.h>
#include <stdlib.h>

/** \brief Type of dynamixel device ID */
typedef uint8_t DynamixelID;
/** \brief Type of dynamixel status code */
typedef uint8_t DynamixelStatus;
/** \brief Type of dynamixel instructions */
typedef uint8_t DynamixelInstruction;

/** \brief ID for broadcast */
#define BROADCAST_ID 0xFE

/**
 * \brief Dynamixel instruction values
 */
enum DynInstruction
{
    PING = 0x01,
    READ = 0x02,
    WRITE = 0x03,
    REG_WRITE = 0x04,
    ACTION = 0x05,
    RESET = 0x06,
    SYNC_WRITE = 0x83
};

/**
 * \brief Dynamixel status values
 *
 * How to interpret status value :
 *
 * If (status&COM_ERROR)==0 , the value is the
 * the status returned by the motor, describing its internal
 * error.
 * If (status&COM_ERROR)==1, there was an error during
 * communication, and the value describe that error.
 *
 * CHECKSUM_ERROR may appear in both cases, in the first
 * case, it means there was an error in the checksum of the
 * instruction packet, in second case in the response packet.
 *
 *
 */
enum DynStatus
{
    OK = 0,

    INPUT_VOLTAGE_ERROR = 1,
    ANGLE_LIMIT_ERROR = 2,
    OVERHEATING_ERROR = 4,
    RANGE_ERROR = 8,
    CHECKSUM_ERROR = 16,
    OVERLOAD_ERROR = 32,
    INSTRUCTION_ERROR = 64,

    TIMEOUT = 1,

    COM_ERROR = 128,
    INTERNAL_ERROR = 255
};

/**
 * \struct DynamixelPacket
 * \brief Struct of a dynamixel packet (instruction or status)
 */
struct DynamixelPacket
{
    DynamixelPacket() {}
    // note : allow to construct from const data, but const_cast it (constness should be respected if code is correct)
    DynamixelPacket(uint8_t aID, uint8_t aInstruction, uint8_t aLength, const uint8_t *aData, uint8_t aAddress = 255, uint8_t aDataLength = 255, uint8_t aIDListSize = 0, const uint8_t *aIDList = NULL) : mID(aID), mIDListSize(aIDListSize), mIDList(const_cast<uint8_t *>(aIDList)), mLength(aLength), mInstruction(aInstruction), mAddress(aAddress), mDataLength(aDataLength), mData(const_cast<uint8_t *>(aData))
    {
        mCheckSum = checkSum();
    }

    /** \brief Packet ID */
    DynamixelID mID;
    /** \brief ID list, used for sync write, set to 0 if not used */
    uint8_t mIDListSize;
    DynamixelID *mIDList;
    /** \brief Packet length (full length)*/
    uint8_t mLength;
    /** \brief Packet instruction or status */
    union
    {
        DynamixelInstruction mInstruction;
        DynamixelStatus mStatus;
    };
    /** \brief Address to read/write, set to 255 if not used */
    uint8_t mAddress;
    /** \brief Length of data to read/write, only needed for read and sync write, set to 255 if not used */
    uint8_t mDataLength;
    /** \brief Pointer to packet parameter (or NULL if no parameter) */
    uint8_t *mData;
    /** \brief Packet checksum */
    uint8_t mCheckSum;

    /**
     * \brief Compute checksum of the packet
     * \return Checksum value
     */
    uint8_t checkSum();
};

/**
 * \brief Dynamixel control table addresses (only addresses used by all models)
 */
enum DynCommonAddress
{
    /** \brief Model number, uint16_t , read only */
    MODEL = 0x00,
    /** \brief Firmware version, uint8_t, read only */
    FIRMWARE = 0x02,
    /** \brief Device ID, uint8_t, writable */
    ID = 0x03,
    /** \brief Communication baudrate, uint8_t, writable */
    BAUDRATE = 0x04,
    /** \brief Return Delay Time , uint8_t, writable */
    RDT = 0x05,
    /** \brief Status Return Level , uint8_t, writable
     *
     * Define when the device will send back a status packet :
     * 0 : Ping only
     * 1 : Read and ping
     * 2 : All instructions
     */
    SRL = 0x10
};

/**
 * \brief Dynamixel motor control table addresses (only addresses used by all motor models)
 */
enum DynMotorAddress
{
    /** \brief Clockwise angle limit, uint16_t , writable */
    CW_LIMIT = 0x06,
    /** \brief Counter clockwise angle limit, uint16_t , writable */
    CCW_LIMIT = 0x08,
    /** \brief Maximum torque that will trigger alarm! , uint16_t , writable */
    MAX_TORQUE = 0x0E,
    /** \brief Enable torque, uint8_t , writable */
    ENABLE_TORQUE = 0x18,
    /** \brief LED state, uint8_t , writable */
    LED = 0x19,
    /** \brief CW compliance margin, uint8_t , writable */
    CW_COMP_MARGIN = 0x1A,
    /** \brief CCW compliance margin, uint8_t , writable */
    CCW_COMP_MARGIN = 0x1B,
    /** \brief CW compliance slope, uint8_t , writable */
    CW_COMP_SLOPE = 0x1C,
    /** \brief CCW compliance slope, uint8_t , writable */
    CCW_COMP_SLOPE = 0x1D,
    /** \brief Goal position, uint16_t , writable */
    GOAL_POSITION = 0x1E,
    /** \brief Goal speed, uint16_t , writable */
    GOAL_SPEED = 0x20,
    /** \brief Maximum torque that will not trigger alarm! must be smaller than MAX_TORQUE , uint16_t , writable */
    TORQUE_LIMIT = 0x22,
    /** \brief Current position, uint16_t , readable */
    CURRENT_POSITION = 0x24,
    /** \brief Nonzero if any movement, uint8_t, readable */
    MOVING = 0x2E
};

enum DynMotorAdditionalAddress
{
    /** \brief Temperature threshold that will trigger alarm! , uint16_t , writable */
    TEMP_LIMIT = 0x0B,
    /** \brief Minimal voltage threshold that will trigger alarm! , uint16_t , writable */
    MIN_VOLTAGE_LIMIT = 0x0C,
    /** \brief Maximal voltage threshold that will trigger alarm! , uint16_t , writable */
    MAX_VOLTAGE_LIMIT = 0x0D,
    /** \brief Alarm led state , uint8_t , writable
     *
     *  7.bit Not used
     *  6.bit Instruction error
     *  5.bit Overload error
     *  4.bit CheckSum error
     *  3.bit Range error
     *  2.bit Overheating error
     *  1.bit Angle Limit error
     *  0.bit Input voltage error
     */
    ALARM_LED = 0x11,
    /** \brief Shutdown reason , uint8_t , writable
     *
     *  7.bit Not used
     *  6.bit Instruction error
     *  5.bit Overload error
     *  4.bit CheckSum error
     *  3.bit Range error
     *  2.bit Overheating error
     *  1.bit Angle Limit error
     *  0.bit Input voltage error
     */
    SHUTDOWN = 0x12,
    /** \brief Current speed of servo , uint16_t , writable */
    CURRENT_SPEED = 0x26,
    /** \brief Current load/torque of servo , uint16_t , writable */
    CURRENT_TORQUE = 0x28,
    /** \brief Current input voltage , uint16_t , writable */
    CURRENT_VOLTAGE = 0x2A,
    /** \brief Current temperature , uint16_t , writable */
    CURRENT_TEMP = 0x2B,
    /** \brief State of EEPROM lock* , bool , writable */
    EEPROM_LOCK = 0x2F,
    /** \brief Minimum current , uint16_t , writable */
    MIN_CURRENT = 0x30
};

/**
 * \brief Dynamixel model number values
 */
enum DynModel
{
    AX12A = 0x0C,
    AX12W = 0x2C,
    AX18A = 0x12,

    DX113 = 0x71,
    DX114 = 0x74,
    DX117 = 0x75,

    RX10 = 0x0A,
    RX24F = 0x18,
    RX28 = 0x1C,
    RX64 = 0x40,

    EX106 = 0x6B,

    MX12W = 0x68,
    MX28T = 0x1D,
    MX28R = 0x1D,
    MX64T = 0x36,
    MX64R = 0x36,
    MX106T = 0x40,
    MX106R = 0x40,

    AXS1 = 0x0D
};

#endif
