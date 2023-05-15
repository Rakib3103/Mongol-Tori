#include <Keyboard.h> // The main library for sending keystrokes.
const int ACT_1_UP = 1;
const int ACT_1_DOWN = 2;//assign relay INx pin to arduino pin

const int ACT_2_UP = 3;
const int ACT_2_DOWN = 4;//assign relay INx pin to arduino pin

const int ACT_3_UP = 5;
const int ACT_3_DOWN = 6;//assign relay INx pin to arduino pin

const int WHEEL_FORWARD = 7;
const int WHEEL_BACKWARD = 8;
void setup() 
{
 Keyboard.begin();  // Initialise the library.
}

// Loop around waiting for a button press on pin 2.
// When the button is pressed, go to the function triggerAutomation.
void loop() 
{
  if(digitalRead(r) == HIGH)
  {
    triggerAutomation();    
  }
}

void triggerAutomation()
{
  Keyboard.press(KEY_LEFT_GUI);     // Press and hold the Windows key.
  Keyboard.press('r');              // Press and hold the 'r' key.
  delay(100);                       // Wait for the computer to register the press.
  Keyboard.releaseAll();            // Release both of the above keys.
  delay(1000);                      // Wait for the Windows Run Dialog to open.
  Keyboard.print("Notepad");        // Type "Notepad".
  Keyboard.press(KEY_RETURN);       // Press the Enter key.
  delay(100);                       // Wait for the computer to register the press.
  Keyboard.releaseAll();            // Release the Enter key.
}