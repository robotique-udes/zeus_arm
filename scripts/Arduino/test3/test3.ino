
/********************** Includes **********************/
/*
 * No need to include them since its only class specific objects
  #include "CytronMotorDriver.h"
  #include <ams_as5048b.h>
*/

#include <ams_as5048b.h>
/********************** Includes **********************/


/********************** Constants **********************/


AMS_AS5048B enc = AMS_AS5048B(0x42);

void setup() {
  // Use same baud as rosserial_arduino
  Serial.begin(57600);

  while (!Serial) {}

  Serial.println("Setup");

  enc.toggleDebug();

  enc.begin();

  Serial.println("Setup2");
}

void loop() {
  
}
