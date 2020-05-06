#include "Arduino.h"

#include "RCS620S.h"

RCS620S nfc;

void setup(){
    Serial.begin(115200);
    Serial.setTimeout(60000);
    while(!Serial);

    Serial1.begin(115200);
    while(!Serial1);

    nfc.begin(&Serial1);
}

void loop(){
    nfc.polling();

    nfc.close();

    delay(1000);
}