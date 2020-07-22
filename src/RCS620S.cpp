#include "Arduino.h"

#include "RCS620S.h"

namespace{
    struct rw_response_t{
        uint8_t* data;
        uint16_t len;
    };
}

RCS620S::RCS620S(){}

bool RCS620S::begin(HardwareSerial* uart){
    this->serial = uart;
    this->timeout = 1000;

    bool result;
    uint8_t res[RwResponseMax];
    uint16_t resLen;

    // ----- RF Configuration -----
    // Timings
    result = rwCommand((uint8_t[]){0xD4, 0x32, 0x02, 0x00, 0x00, 0x00}, 6, res, &resLen);
    if(!result || (resLen != 2) || (memcmp(res, (const uint8_t[]){0xD5, 0x33}, 2) != 0)){
        return false;
    }

    // Retries
    result = rwCommand((uint8_t[]){0xD4, 0x32, 0x05, 0x00, 0x00, 0x00}, 6, res, &resLen);
    if(!result || (resLen != 2) || (memcmp(res, (const uint8_t[]){0xD5, 0x33}, 2) != 0)){
        return false;
    }

    // Additional wait time (24ms)
    result = rwCommand((uint8_t[]){0xD4, 0x32, 0x81, 0xB7}, 4, res, &resLen);
    if(!result || (resLen != 2) || (memcmp(res, (const uint8_t[]){0xD5, 0x33}, 2) != 0)){
        return false;
    }

    return true;
}

bool RCS620S::polling(uint16_t code){
    bool result;
    uint8_t buf[9];
    uint8_t res[RwResponseMax];
    uint16_t resLen;

    // In list passive target
    memcpy(buf, (const uint8_t[]){0xD4, 0x4A, 0x01, 0x01, 0x00, 0xFF, 0xFF, 0x00, 0x00}, 9);
    buf[5] = (uint8_t)((code >> 8) & 0xFF);
    buf[6] = (uint8_t)((code >> 0) & 0xFF);

    result = rwCommand(buf, 9, res, &resLen);
    if(!result || (resLen != 22) || (memcmp(res, (const uint8_t[]){0xD5, 0x4B, 0x01, 0x01, 0x12, 0x01}, 6) != 0)){
        return false;
    }

    memcpy(this->idm, res + 6, 8);
    memcpy(this->pmm, res + 14, 8);

    return true;
}

bool RCS620S::close(){
    bool result;
    uint8_t res[RwResponseMax];
    uint16_t resLen;

    result = rwCommand((uint8_t[]){0xD4, 0x32, 0x01, 0x00}, 4, res, &resLen);
    if(!result || (resLen != 2) || (memcmp(res, (const uint8_t[]){0xD5, 0x33}, 2) != 0)){
        return false;
    }

    return true;
}

bool RCS620S::cardCommand(uint8_t* cmd, uint16_t cmdLen, uint8_t* res, uint16_t* resLen){
    bool result;
    uint16_t cmdTimeout;
    uint8_t buf[RwResponseMax];
    uint16_t bufLen;

    if(this->timeout >= (0x10000 / 2)){
        cmdTimeout = 0xFFFF;
    }
    else{
        cmdTimeout = (uint16_t)(this->timeout * 2);
    }

    // Communicate thru ex
    buf[0] = 0xD4;
    buf[1] = 0xA0;
    buf[2] = (uint8_t)((cmdTimeout >> 0) & 0xFF);
    buf[3] = (uint8_t)((cmdTimeout >> 8) & 0xFF);
    buf[4] = (uint8_t)(cmdLen + 1);
    memcpy(buf + 5, cmd, cmdLen);

    result = rwCommand(buf, 5 + cmdLen, buf, &bufLen);
    if(!result || (bufLen < 4) || (buf[0] != 0xD5) || (buf[1] != 0xA1) || (buf[2] != 0x00) || (bufLen != (3 + buf[3]))){
        return false;
    }

    *resLen = (uint8_t)(buf[3] - 1);
    memcpy(res, buf + 4, *resLen);

    return true;
}

bool RCS620S::push(uint8_t* data, uint16_t dataLen){
    bool result;
    uint8_t buf[CardResponseMax];
    uint8_t resLen;

    if(dataLen > 224){
        return false;
    }

    // Push
    buf[0] = 0xB0;
    memcpy(buf + 1, this->idm, 8);
    buf[9] = dataLen;
    memcpy(buf + 10, data, dataLen);

    result = cardCommand(buf, 10 + dataLen, buf, &resLen);
    if(!result || (resLen != 10) || (buf[0] != 0xB1) || (memcmp(buf + 1, this->idm, 8) != 0) || (buf[9] != dataLen)){
        return false;
    }

    buf[0] = 0xA4;
    memcpy(buf + 1, this->idm, 8);
    buf[9] = 0x00;

    result = cardCommand(buf, 10, buf, &resLen);
    if(!result || (resLen != 10) || (buf[0] != 0xA5) || (memcmp(buf + 1, this->idm, 8) != 0) || (buf[9] != 0x00)){
        return false;
    }

    return true;
}

bool RCS620S::rwCommand(uint8_t* cmd, uint16_t cmdLen, uint8_t* res, uint16_t* resLen){
    bool result;
    uint8_t buf[9];

    uint8_t dcs = calcDcs(cmd, cmdLen);

    // Transmit the cmd
    buf[0] = 0x00;
    buf[1] = 0x00;
    buf[2] = 0xFF;

    if(cmdLen <= 255){
        // Normal frame
        buf[3] = cmdLen;
        buf[4] = (uint8_t)-buf[3];
        serialWrite(buf, 5);
    }
    else{
        // Extended frame
        buf[3] = 0xFF;
        buf[4] = 0xFF;
        buf[5] = (uint8_t)((cmdLen >> 8) & 0xFF);
        buf[6] = (uint8_t)((cmdLen >> 0) & 0xFF);
        buf[7] = (uint8_t)-(buf[5] + buf[6]);
        serialWrite(buf, 8);
    }

    serialWrite(cmd, cmdLen);
    buf[0] = dcs;
    buf[1] = 0x00;
    serialWrite(buf, 2);

    // Receive an ACK
    result = serialRead(buf, 6);
    if(!result || (memcmp(buf, (const uint8_t[]){0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00}, 6) != 0)){
        serialWrite((uint8_t[]){0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00}, 6);
        return false;
    }

    // Receive a response
    result = serialRead(buf, 5);
    if(!result){
            serialWrite((uint8_t[]){0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00}, 6);
        return false;
    }
    else if(memcmp(buf, (const uint8_t[]){0x00, 0x00, 0xFF}, 3) != 0){
        return false;
    }

    if((buf[3] == 0xFF) && (buf[4] == 0xFF)){
        result = serialRead(buf + 5, 3);

        if(!result || (((buf[5] + buf[6] + buf[7]) & 0xFF) != 0)){
            return false;
        }

        *resLen = (((uint16_t)buf[5] << 8) | ((uint16_t)buf[6] << 0));
    }
    else{
        if(((buf[3] + buf[4]) & 0xFF) != 0){
            return false;
        }

        *resLen = buf[3];
    }

    if(*resLen > RwResponseMax){
        return false;
    }

    result = serialRead(res, *resLen);
    if(!result){
        serialWrite((uint8_t[]){0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00}, 6);
        return false;
    }

    dcs = calcDcs(res, *resLen);

    result = serialRead(buf, 2);
    if(!result || (buf[0] != dcs) || (buf[1] != 0x00)){
        serialWrite((uint8_t[]){0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00}, 6);
        return false;
    }

    return true;
}

uint8_t RCS620S::calcDcs(uint8_t* data, uint16_t dataLen){
    uint8_t sum = 0;

    for(uint16_t i = 0; i < dataLen; i++){
        sum += data[i];
    }

    return -(sum & 0xFF);
}

bool RCS620S::serialWrite(uint8_t* data, uint16_t dataLen){
    this->serial->write(data, dataLen);
    return true;
}

bool RCS620S::serialRead(uint8_t* data, uint16_t dataLen){
    uint16_t bytes = 0;
    uint32_t t0 = millis();

    while(bytes < dataLen){
        if((millis() - t0) >= this->timeout){
            return false;
        }

        if(this->serial->available()){
            data[bytes++] = this->serial->read();
        }
    }

    return true;
}