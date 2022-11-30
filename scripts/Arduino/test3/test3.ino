
/********************** Includes **********************/
/*
 * No need to include them since its only class specific objects
  #include "CytronMotorDriver.h"
  #include <ams_as5048b.h>
*/

#include "Motor.h"
/********************** Includes **********************/


/********************** Constants **********************/

Motor_talon* mot = new Motor_talon(9);

void setup() {
  // Use same baud as rosserial_arduino
  Serial.begin(57600);

  while (!Serial) {}

  Serial.println("Setup");

  mot->setup();

}

void loop() {
  mot->set_speed(0.3);
}
