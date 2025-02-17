
#include "DynamixelInterface.h"

void DynamixelInterface::transaction(bool aExpectStatus, uint8_t answerSize)
{
    sendPacket(mPacket);
    if (aExpectStatus)
    {
        receivePacket(mPacket, answerSize);
    }
    else
    {
        mPacket.mStatus = DynStatus::OK;
    }
}

DynamixelStatus DynamixelInterface::read(uint8_t aID, uint8_t aAddress, uint8_t aSize, uint8_t *aPtr, uint8_t aStatusReturnLevel)
{
    mPacket = DynamixelPacket(aID, DynInstruction::READ, 4, aPtr, aAddress, aSize);
    transaction(aStatusReturnLevel > 0 && aID != BROADCAST_ID, aSize);
    return mPacket.mStatus;
}

DynamixelStatus DynamixelInterface::write(uint8_t aID, uint8_t aAddress, uint8_t aSize, const uint8_t *aPtr, uint8_t aStatusReturnLevel)
{
    mPacket = DynamixelPacket(aID, DynInstruction::WRITE, aSize + 3, aPtr, aAddress);
    transaction(aStatusReturnLevel > 1 && aID != BROADCAST_ID);
    return mPacket.mStatus;
}

DynamixelStatus DynamixelInterface::regWrite(uint8_t aID, uint8_t aAddress, uint8_t aSize, const uint8_t *aPtr, uint8_t aStatusReturnLevel)
{
    mPacket = DynamixelPacket(aID, DynInstruction::REG_WRITE, aSize + 3, aPtr, aAddress);
    transaction(aStatusReturnLevel > 1 && aID != BROADCAST_ID);
    return mPacket.mStatus;
}

DynamixelStatus DynamixelInterface::syncWrite(uint8_t nID, const uint8_t *aID, uint8_t aAddress, uint8_t aSize, const uint8_t *aPtr, uint8_t aStatusReturnLevel)
{
    mPacket = DynamixelPacket(BROADCAST_ID, DynInstruction::SYNC_WRITE, (aSize + 1) * nID + 4, aPtr, aAddress, aSize, nID, aID);
    transaction(false);
    return mPacket.mStatus;
}

DynamixelStatus DynamixelInterface::ping(uint8_t aID)
{
    mPacket = DynamixelPacket(aID, DynInstruction::PING, 2, NULL);
    transaction(true);
    return mPacket.mStatus;
}

DynamixelStatus DynamixelInterface::action(uint8_t aID, uint8_t aStatusReturnLevel)
{
    mPacket = DynamixelPacket(aID, DynInstruction::ACTION, 2, NULL);
    transaction(aStatusReturnLevel > 1 && aID != BROADCAST_ID);
    return mPacket.mStatus;
}

DynamixelStatus DynamixelInterface::reset(uint8_t aID, uint8_t aStatusReturnLevel)
{
    mPacket = DynamixelPacket(aID, DynInstruction::RESET, 2, NULL);
    transaction(aStatusReturnLevel > 1 && aID != BROADCAST_ID);
    return mPacket.mStatus;
}
