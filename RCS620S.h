#ifndef RCS620S_H_
#define RCS620S_H_

#define CardResponseMax 254
#define RwResponseMax 265

class RCS620S{
public:
    uint32_t timeout;
    uint8_t idm[8];
    uint8_t pmm[8];

    RCS620S();

    bool begin(HardwareSerial* uart);
    bool polling(uint16_t code = 0xFFFF);
    bool close();

    bool cardCommand(const uint8_t* cmd, uint8_t cmdLen, uint8_t res[CardResponseMax], uint8_t* resLen);
    bool push(const uint8_t* data, uint8_t dataLen);

private:
    HardwareSerial* serial;

    bool rwCommand(const uint8_t* cmd, uint16_t cmdLen, uint8_t res[RwResponseMax], uint16_t* resLen);
    uint8_t calcDcs(const uint8_t* data, uint16_t dataLen);

    bool serialWrite(uint8_t* data, uint16_t dataLen);
    bool serialRead(uint8_t* data, uint16_t dataLen);
    void serialCancel();
};

#endif