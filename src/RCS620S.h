#ifndef RCS620S_Header
#define RCS620S_Header

#include "inttypes.h"
#include "string.h"

#define RCS620S_MAX_CARD_RESPONSE_LEN    254
#define RCS620S_MAX_RW_RESPONSE_LEN      265

class RCS620S{
public:
    RCS620S();

    int initDevice();
    int polling(uint16_t systemCode = 0xffff);
    int cardCommand(
        const uint8_t* command,
        uint8_t commandLen,
        uint8_t response[RCS620S_MAX_CARD_RESPONSE_LEN],
        uint8_t* responseLen);
    int rfOff();

    int push(
        const uint8_t* data,
        uint8_t dataLen);

private:
    int rwCommand(
        const uint8_t* command,
        uint16_t commandLen,
        uint8_t response[RCS620S_MAX_RW_RESPONSE_LEN],
        uint16_t* responseLen);
    void cancel();
    uint8_t calcDCS(
        const uint8_t* data,
        uint16_t len);

    void writeSerial(
        const uint8_t* data,
        uint16_t len);
    int readSerial(
        uint8_t* data,
        uint16_t len);
    void flushSerial();

    int checkTimeout(unsigned long t0);

public:
    unsigned long timeout;
    uint8_t idm[8];
    uint8_t pmm[8];
};

#endif