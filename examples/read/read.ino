#include "Arduino.h"

#include "RCS620S.h"

RCS620S felica;

void setup(){
    Serial.begin(115200);
    Serial.setTimeout(60000);
    while(!Serial);

    Serial1.begin(115200);
    while(!Serial1);

    felica.begin(&Serial1);
}

void loop(){
    felica.polling();

    felica.close();

    delay(1000);
}