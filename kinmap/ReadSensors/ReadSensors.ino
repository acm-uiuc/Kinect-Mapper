#include <RotaryEncoder.h>
#include <SimpleRotaryEncoder.h>

SimpleRotaryEncoder rightWheel (2, 250);
RotaryEncoder leftWheel  (6, 7, 250);

void setup() {
  Serial.begin(9600);
}

void loop() {
  int leftCount = leftWheel.getPulses();
  int rightCount = rightWheel.getPulses();
  
  Serial.print("Right Count = ");
  Serial.print(rightCount);
  Serial.print("  Left Count = ");
  Serial.println(leftCount);
  
  delay(1000); // wait for 1 second
}
