#include <SoftwareSerial.h>

const int btnPin = 2;
int btnState = 0;
int preBtnState = 0;
int active = 0;

void setup() {
  pinMode(btnPin, INPUT);
  Serial.begin(9600);
  preBtnState = digitalRead(btnPin);
}

void loop() {
  btnState = digitalRead(btnPin);

  if (btnState != preBtnState && btnState == LOW) {
    active = !active;
  }

  if(Serial.available() > 0) {

    int byte = Serial.read();

    // Prints Serial Line of the statis active
    if(byte == 1) {
      delay(10);
      Serial.println(active);
    }

    // active is set to 1 if int 2 is received
    if(byte == 2) {
      delay(10);
      active = 1;
      Serial.println(active);
    }

    // active is set to 1 if int 3 is received
    if(byte == 3) {
      delay(10);
      active = 0;
      Serial.println(active);
    }

  }

  preBtnState = btnState;
  delay(50);

}
