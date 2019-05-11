#include <wiringPi.h>

int main() {

  wiringPiSetup();
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);

  for(;;){
    digitalWrite(0, HIGH);
    digitalWrite(1, LOW);
    delay(500);
    digitalWrite(0, LOW);
    digitalWrite(1, HIGH);
    delay(500);
  }

  return 0;
}
