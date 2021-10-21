
#include <SPI.h>

int chipSelectPin = 7;

void setup() {
   pinMode(chipSelectPin, OUTPUT);
   digitalWrite(chipSelectPin, HIGH);
   Serial.begin(9600);
   SPI.begin();
   
}

void writeRegister(uint32_t thisRegister, uint32_t thisValue) {
   uint8_t data[4];

   SPI.beginTransaction(SPISettings(100000,MSBFIRST,SPI_MODE0));
   uint32_t value = 0x00000000;
   value |= ((thisRegister << 20) & 0x7FF00000);
   value |= (thisValue & 0xFFFFF);

   data[0] = (value >> 24) & 0xFF;
   data[1] = (value >> 16) & 0xFF;
   data[2] = (value >>  8) & 0xFF;
   data[3] = (value     )  & 0xFF;

   Serial.print("Write Control: ");
   Serial.println(value,HEX);
   
   digitalWrite(chipSelectPin, LOW);
   SPI.transfer(data,4);
   digitalWrite(chipSelectPin, HIGH);
   delay(10);
   SPI.endTransaction();
}


void subRead(uint32_t valIn, uint32_t &valOut) {
   uint8_t data[4];
   
   SPI.beginTransaction(SPISettings(100000,MSBFIRST,SPI_MODE0));
   //Serial.print("Read Control: ");
   //Serial.println(valIn,HEX);

   data[0] = (valIn >> 24) & 0xFF;
   data[1] = (valIn >> 16) & 0xFF;
   data[2] = (valIn >>  8) & 0xFF;
   data[3] = (valIn     )  & 0xFF;

   digitalWrite(chipSelectPin, LOW);
   //delay(10);
   SPI.transfer(data,4);
   digitalWrite(chipSelectPin, HIGH);
   delay(10);
   
   valOut = uint32_t(data[0]) << 24;
   valOut |= uint32_t(data[1]) << 16;
   valOut |= uint32_t(data[2]) << 8;
   valOut |= uint32_t(data[3]);

   //Serial.print("Read Value: ");
   //Serial.println(valOut,HEX);
   SPI.endTransaction();
}

uint32_t readRegister(uint32_t thisRegister) {
   uint32_t valIn;
   uint32_t valOut;
   
   valIn = 0x80000000;
   valIn |= ((thisRegister << 20) & 0x7FF00000);

   subRead(valIn, valOut);
   subRead(0x80000000, valOut);

   return valOut & 0xFFFFF;
}

void loop() {
    delay(3000);
    int x;
    uint32_t val;
    writeRegister(7,0x000F);

    Serial.println("--------------------------------------");
    for (x=0; x < 13; x ++) {
       val = readRegister(x);
       Serial.print("Value "); Serial.print(x); Serial.print(": "); Serial.println(val, HEX);
    }
}
