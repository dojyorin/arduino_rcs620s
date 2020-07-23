#include "Arduino.h"

#include "RCS620S.h"

RCS620S felica(&Serial1);

void setup(){
    Serial.begin(115200);
    while(!Serial);

    felica.begin(1000);
}

void loop(){
    if(felica.polling()){
        Serial.print("IDm:");
        for(uint8_t i = 0; i < 8; i++){
            if(felica.idm[i] / 0x10 == 0){
                Serial.print(0);
            }
            Serial.print(felica.idm[i], HEX);
        }
        Serial.println("");
    }
    else{
        Serial.println("Polling...");
    }

    felica.rfOff();
    delay(1000);
}