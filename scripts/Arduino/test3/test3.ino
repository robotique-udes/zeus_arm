
/********************** Includes **********************/
/*
 * No need to include them since its only class specific objects
  #include "CytronMotorDriver.h"
  #include <ams_as5048b.h>
*/

#include "Limitswitch.h"
#include "Motor.h"
/********************** Includes **********************/


/********************** Constants **********************/
Limitswitch* s1 = new Limitswitch(27, false);
Limitswitch* s2 = new Limitswitch(30, false);
Limitswitch* s3 = new Limitswitch(28, false);
Limitswitch* s4 = new Limitswitch(29, false);


void setup() {
  // Use same baud as rosserial_arduino
  Serial.begin(57600);

  while (!Serial) {}

  Serial.println("Setup");

}

void loop() {
  Serial.print("S1 : ");
  Serial.print(s1->get());
  Serial.print(",S2 : ");
  Serial.print(s2->get());
  Serial.print(", S3 : ");
  Serial.print(s3->get());
  Serial.print(", S4 : ");
  Serial.println(s4->get());
  
  delay(10);
}
