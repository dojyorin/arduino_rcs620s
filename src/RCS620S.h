#ifndef RCS620S_Header
#define RCS620S_Header

#include "Arduino.h"

class RCS620S{
public:
    static const size_t responseLengthMax = 265;
    static const size_t responseLengthOffset = 11;

    uint8_t idm[8] = {0};
    uint8_t pmm[8] = {0};


    RCS620S(Uart* uart);

    bool begin(uint16_t timeout);
    bool rfOff();
    bool polling();
    bool push(const uint8_t *data, size_t dataLength);
    size_t cardCommand(const uint8_t *command, size_t commandLength, uint8_t* response);

private:
    Uart* io = nullptr;
    uint16_t timeout = 0;

    static uint8_t calcDCS(const uint8_t *data, size_t dataLength);

    size_t rwCommand(const uint8_t *command, size_t commandLength, uint8_t* response);
    bool writeSerial(const uint8_t *data, size_t dataLength);
    bool readSerial(uint8_t *data, size_t dataLength);
};

#endif