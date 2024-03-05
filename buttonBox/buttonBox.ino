#include <Joystick.h>
Joystick_ Joystick;
void setup() {
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);

  Joystick.begin();
}
int lastButtonState[10] = {0,0,0,0,0,0,0,0,0,0};
const int pinToButtonMap = 4;
void loop() {
  for(int i = 0; i < 10; i++){
    int currentButtonState = !digitalRead(i + pinToButtonMap);
    if(currentButtonState != lastButtonState[i]){
      Joystick.setButton(i, currentButtonState);
      lastButtonState[i] = currentButtonState;
    }
  }
  delay(50);
}
