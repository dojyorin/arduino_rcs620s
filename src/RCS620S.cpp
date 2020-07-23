#include "RCS620S.h"

RCS620S::RCS620S(Uart* uart):io(uart){}

bool RCS620S::begin(uint16_t timeout){
    this->io->begin(115200);
    while(!this->io);

    this->timeout = timeout;

    uint8_t response[responseLengthMax];

    if(rwCommand((const uint8_t[]){0xD4, 0x32, 0x02, 0x00, 0x00, 0x00}, 6, response) != 2 || !!memcmp(response, (const uint8_t[]){0xD5, 0x33}, 2)){
        return false;
    }

    if(rwCommand((const uint8_t[]){0xD4, 0x32, 0x05, 0x00, 0x00, 0x00}, 6, response) != 2 || !!memcmp(response, (const uint8_t[]){0xD5, 0x33}, 2)){
        return false;
    }

    if(rwCommand((const uint8_t[]){0xD4, 0x32, 0x81, 0xB7}, 4, response) != 2 || !!memcmp(response, (const uint8_t[]){0xD5, 0x33}, 2)){
        return false;
    }

    return true;
}

bool RCS620S::polling(){
    uint8_t response[responseLengthMax];

    if(rwCommand((const uint8_t[]){0xD4, 0x4A, 0x01, 0x01, 0x00, 0xFF, 0xFF, 0x00, 0x00}, 9, response) != 22 || !!memcmp(response, (const uint8_t[]){0xD5, 0x4B, 0x01, 0x01, 0x12, 0x01}, 6)){
        return false;
    }

    memcpy(this->idm, response + 6, 8);
    memcpy(this->pmm, response + 14, 8);

    return true;
}

size_t RCS620S::cardCommand(const uint8_t *command, size_t commandLength, uint8_t* response){
    uint8_t bufferTx[responseLengthMax] = {0};
    uint8_t bufferRx[responseLengthMax] = {0};
    size_t responseLength = 0;

    bufferTx[0] = 0xD4;
    bufferTx[1] = 0xA0;
    bufferTx[2] = this->timeout & 0xFF;
    bufferTx[3] = (this->timeout >> 8) & 0xFF;
    bufferTx[4] = commandLength + 1;
    memcpy(bufferTx + 5, command, commandLength);

    if(rwCommand(bufferTx, commandLength + 5, bufferRx) < (3 + bufferRx[3]) || bufferRx[0] != 0xD5 || bufferRx[1] != 0xA1 || bufferRx[2] != 0x00){
        return 0;
    }

    responseLength = bufferRx[3] - 1;
    memcpy(response, bufferRx + 4, responseLength);

    return responseLength;
}

bool RCS620S::rfOff(){
    uint8_t response[responseLengthMax] = {0};

    return rwCommand((const uint8_t[]){0xD4, 0x32, 0x01, 0x00}, 4, response) != 2 || !!memcmp(response, (const uint8_t[]){0xD5, 0x33}, 2);
}

bool RCS620S::push(const uint8_t* data, size_t dataLength){
    uint8_t bufferTx[responseLengthMax - responseLengthOffset] = {0};
    uint8_t bufferRx[responseLengthMax - responseLengthOffset] = {0};

    if(dataLength > 224){
        return false;
    }

    bufferTx[0] = 0xB0;
    memcpy(bufferTx + 1, this->idm, 8);
    bufferTx[9] = dataLength;
    memcpy(bufferTx + 10, data, dataLength);

    if(cardCommand(bufferTx, dataLength + 10, bufferRx) != 10 || bufferRx[0] != 0xB1 || !!memcmp(bufferRx + 1, this->idm, 8) || bufferRx[9] != dataLength){
        return false;
    }

    memset(bufferTx, 0, sizeof(bufferTx));
    bufferTx[0] = 0xA4;
    memcpy(bufferTx + 1, this->idm, 8);
    bufferTx[9] = 0x00;

    memset(bufferRx, 0, sizeof(bufferRx));
    if(cardCommand(bufferTx, 10, bufferRx) != 10 || bufferRx[0] != 0xA5 || !!memcmp(bufferRx + 1, this->idm, 8) || bufferRx[9] != 0x00){
        return false;
    }

    return true;
}

uint8_t RCS620S::calcDCS(const uint8_t* data, size_t dataLength){
    uint8_t sum = 0;

    while(dataLength--){
        sum += *data++;
    }

    return -(sum & 0xFF);
}

size_t RCS620S::rwCommand(const uint8_t *command, size_t commandLength, uint8_t* response){
    uint8_t bufferTx[8] = {0};
    uint8_t bufferRx[6] = {0};
    size_t responseLength = 0;

    bufferTx[0] = 0x00;
    bufferTx[1] = 0x00;
    bufferTx[2] = 0xFF;

    if(commandLength <= 0xFF){
        bufferTx[3] = commandLength;
        bufferTx[4] = -bufferTx[3];

        writeSerial(bufferTx, 5);
    }
    else{
        bufferTx[3] = 0xFF;
        bufferTx[4] = 0xFF;
        bufferTx[5] = (commandLength >> 8) & 0xFF;
        bufferTx[6] = commandLength & 0xFF;
        bufferTx[7] = -(bufferTx[5] + bufferTx[6]);

        writeSerial(bufferTx, 8);
    }

    writeSerial(command, commandLength);

    memset(bufferTx, 0, sizeof(bufferTx));
    bufferTx[0] = calcDCS(command, commandLength);
    bufferTx[1] = 0x00;

    writeSerial(bufferTx, 2);

    if(!readSerial(bufferRx, 6) || !!memcmp(bufferRx, (const uint8_t[]){0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00}, 6)){
        writeSerial((const uint8_t[]){0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00}, 6);
        return 0;
    }

    memset(bufferRx, 0, sizeof(bufferRx));
    if(!readSerial(bufferRx, 5)){
        writeSerial((const uint8_t[]){0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00}, 6);
        return 0;
    }
    else if(!!memcmp(bufferRx, (const uint8_t[]){0x00, 0x00, 0xFF}, 3)){
        return 0;
    }

    if(bufferRx[3] == 0xFF && bufferRx[4] == 0xFF){
        memset(bufferRx, 0, sizeof(bufferRx));
        if(!readSerial(bufferRx, 3) || !!((bufferRx[0] + bufferRx[1] + bufferRx[2]) & 0xFF)){
            return 0;
        }

        responseLength = (bufferRx[0] << 8) | bufferRx[1];
    }
    else{
        if(!!((bufferRx[3] + bufferRx[4]) & 0xFF)){
            return 0;
        }

        responseLength = bufferRx[3];
    }

    if(responseLength > responseLengthMax){
        return 0;
    }

    if(!readSerial(response, responseLength)){
        writeSerial((const uint8_t[]){0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00}, 6);
        return 0;
    }

    memset(bufferRx, 0, sizeof(bufferRx));
    if(!readSerial(bufferRx, 2) || bufferRx[0] != calcDCS(response, responseLength) || bufferRx[1] != 0x00){
        writeSerial((const uint8_t[]){0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00}, 6);
        return 0;
    }

    return responseLength;
}

bool RCS620S::writeSerial(const uint8_t *data, size_t dataLength){
    size_t bytes = this->io->write(data, dataLength);
    this->io->flush();

    return bytes == dataLength;
}

bool RCS620S::readSerial(uint8_t *data, size_t dataLength){
    size_t bytes = 0;
    uint32_t t0 = millis();

    while(bytes < dataLength){
        if((millis() - t0) >= this->timeout){
            return false;
        }

        if(this->io->available() > 0){
            data[bytes++] = this->io->read();
        }
    }

    return true;
}