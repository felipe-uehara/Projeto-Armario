#include <Bluepad32.h>

/* teste feito em cima do código-exemplo "Controller" da lib ESP32_Bluepad

Fonte: https://racheldebarros.com/esp32-projects/connect-your-game-controller-to-an-esp32/
Fonte 2: https://racheldebarros.com/esp32-projects/control-leds-motors-servos-with-a-game-controller-and-esp32/

Board: DOIT ESP32 DEVKIT V1
*/

const int LED_state = 2; //LED that indicates the current state of the conexion between the ESP32 and the Dualshock controller

//Lists of Drawers
const int D1 = 13, D2 = 14, D3 = 26;
/*27
19
33

D11
D12
D21
D22
D31
D32
EN1
EN2
EN3

*/

//List of Endstop sensors
const int E1 = 32, E2 = 35, E3 = 34; /*E4, E5, E6*/
/*note: I'm considering this sensor as being '0' for 'not pressed' and '1' for 'pressed'. 
That may be changed if necessarily futurely.*/

/*note2: maybe using the internal rtc as a "failsafe" is a good idea*/

//LED strip
const int LED_strip = 33;

//Microphone
const int mic = 25;

//Locomotion Pins
const int Left_Forward = 4, Left_Backward = 18, /*Left_Enable = 12*/, Right_Forward = 21, Right_Backward = 22, /*Right_Enable = 23*/;



ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      digitalWrite(LED_state, HIGH);
      break;
      }
    }

    if (!foundEmptySlot) {
      Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      digitalWrite(LED_state, HIGH);
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      digitalWrite(LED_state, LOW);
      break;
    }
  }

    if (!foundController) {
      Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

// ========= SEE CONTROLLER VALUES IN SERIAL MONITOR ========= //

void dumpGamepad(ControllerPtr ctl) {
  Serial.printf(
  "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
  "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
  ctl->index(),        // Controller Index
  ctl->dpad(),         // D-pad
  ctl->buttons(),      // bitmask of pressed buttons
  ctl->axisX(),        // (-511 - 512) left X Axis
  ctl->axisY(),        // (-511 - 512) left Y axis
  ctl->axisRX(),       // (-511 - 512) right X axis
  ctl->axisRY(),       // (-511 - 512) right Y axis
  ctl->brake(),        // (0 - 1023): brake button
  ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
  ctl->miscButtons(),  // bitmask of pressed "misc" buttons
  ctl->gyroX(),        // Gyro X
  ctl->gyroY(),        // Gyro Y
  ctl->gyroZ(),        // Gyro Z
  ctl->accelX(),       // Accelerometer X
  ctl->accelY(),       // Accelerometer Y
  ctl->accelZ()        // Accelerometer Z
  );
}

// ========= GAME CONTROLLER ACTIONS SECTION ========= //

void processGamepad(ControllerPtr ctl) {
  // There are different ways to query whether a button is pressed.
  // By query each button individually:
  //  a(), b(), x(), y(), l1(), etc...


  //Gaveta 3//
  if (ctl->buttons() == 0x0001) {
    // code for when "X" button is pushed
    Serial.println("Gaveta 3 abrindo....");
    analogWrite(D3, 127);
    while(digitalRead(E3) == LOW){}

    if (digitalRead(E3) == HIGH){
      analogWrite(D3, 0);
      Serial.println("Gaveta 3 aberta!");
    }
  }
  if (ctl->buttons() != 0x0001) {
    // code for when X button is released
  }


  //LED strip - turn ON//
  if (ctl->buttons() == 0x0004) {
    // code for when SQUARE button is pushed
      digitalWrite(LED_strip, HIGH);  
      Serial.println("Fita de LED aceso!");
    }
      
  
  if (ctl->buttons() != 0x0004) {
  // code for when square button is released
  }

  //Drawer 1 - OPEN//
  if (ctl->buttons() == 0x0008) {
    // code for when TRIANGLE button is pushed
    Serial.println("Gaveta 1 abrindo....");
    analogWrite(D1, 127);
    while(!digitalRead(E1)){}

    if (digitalRead(E1)){
      analogWrite(D1, 0);
      Serial.println("Gaveta 1 aberta!");
    }
    
  } 
  if (ctl->buttons() != 0x0008) {
    // code for when triangle button is released
  }

  //Gaveta 2//
  if (ctl->buttons() == 0x0002) {
    // code for when CIRCLE button is pushed
    Serial.println("Gaveta 2 abrindo....");
    analogWrite(D2, 127);
    while(digitalRead(E2) == LOW){}

    if (digitalRead(E2) == HIGH){
      analogWrite(D2, 0);
      Serial.println("Gaveta 2 aberta!");
    }
  }
  if (ctl->buttons() != 0x0002) {
    // code for when circle button is released
  }

  //== PS4 Dpad UP button = 0x01 ==//
  if (ctl->buttons() == 0x01) {
    // code for when dpad up button is pushed
  }
  if (ctl->buttons() != 0x01) {
    // code for when dpad up button is released
  }

  //==PS4 Dpad DOWN button = 0x02==//
  if (ctl->buttons() == 0x02) {
    // code for when dpad down button is pushed
  }
  if (ctl->buttons() != 0x02) {
    // code for when dpad down button is released
  }

  //== PS4 Dpad LEFT button = 0x08 ==//
  if (ctl->buttons() == 0x08) {
    // code for when dpad left button is pushed
  }
  if (ctl->buttons() != 0x08) {
    // code for when dpad left button is released
  }

  //== PS4 Dpad RIGHT button = 0x04 ==//
  if (ctl->buttons() == 0x04) {
    // code for when dpad right button is pushed
  }
  if (ctl->buttons() != 0x04) {
    // code for when dpad right button is released
  }

  //LED Strip - turn OFF//
  if (ctl->buttons() == 0x0020) {
    // code for when R1 button is pushed
    digitalWrite(LED_strip, LOW);  
    Serial.println("Fita de LED apagada!");
  }
  if (ctl->buttons() != 0x0020) {
    // code for when R1 button is released
  }

  //== PS4 R2 trigger button = 0x0080 ==//
  if (ctl->buttons() == 0x0080) {
    // code for when R2 button is pushed
  }
  if (ctl->buttons() != 0x0080) {
    // code for when R2 button is released
  }

  //== PS4 L1 trigger button = 0x0010 ==//
  if (ctl->buttons() == 0x0010) {
    // code for when L1 button is pushed
  }
  if (ctl->buttons() != 0x0010) {
    // code for when L1 button is released
  }

  //== PS4 L2 trigger button = 0x0040 ==//
  if (ctl->buttons() == 0x0040) {
    // code for when L2 button is pushed
  }
  if (ctl->buttons() != 0x0040) {
    // code for when L2 button is released
  }



  //== LEFT JOYSTICK - UP ==//
  if (ctl->axisY() <= -25) {
    // code for when left joystick is pushed up
    }

  //== LEFT JOYSTICK - DOWN ==//
  if (ctl->axisY() >= 25) {
    // code for when left joystick is pushed down
  }

  //== LEFT JOYSTICK - LEFT ==//
  if (ctl->axisX() <= -25) {
    // code for when left joystick is pushed left
  }

  //== LEFT JOYSTICK - RIGHT ==//
  if (ctl->axisX() >= 25) {
    // code for when left joystick is pushed right
  }

  //== LEFT JOYSTICK DEADZONE ==//
  if (ctl->axisY() > -25 && ctl->axisY() < 25 && ctl->axisX() > -25 && ctl->axisX() < 25) {
    // code for when left joystick is at idle
  }

  //== RIGHT JOYSTICK - X AXIS ==//
  if (ctl->axisRX()) {
    // code for when right joystick moves along x-axis
  }

  //== RIGHT JOYSTICK - Y AXIS ==//
  if (ctl->axisRY()) {
  // code for when right joystick moves along y-axis
  }
  //dumpGamepad(ctl);
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
         processGamepad(myController);
      }
      else {
        Serial.println("Unsupported controller");
      }
    }
  }
}






// Arduino setup function. Runs in CPU 1
void setup() {
  Serial.begin(115200);

  //Definition of the pinage
  pinMode(LED_state, OUTPUT);
  pinMode(LED_strip, OUTPUT);
  pinMode(mic, INPUT);

  pinMode(E1, INPUT_PULLDOWN);
  pinMode(E2, INPUT_PULLDOWN);
  pinMode(E3, INPUT_PULLDOWN);

  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);

  pinMode(Left_Forward, OUTPUT);
  pinMode(Left_Backward, OUTPUT);
  pinMode(Left_Enable, OUTPUT);
  pinMode(Right_Forward, OUTPUT);
  pinMode(Right_Backward, OUTPUT);
  pinMode(Right_Enable, OUTPUT);

  //Initialize with all components in zero
  digitalWrite(LED_state, LOW);
  digitalWrite(LED_strip, LOW);
  analogWrite(D1, 0);
  analogWrite(D2, 0);
  analogWrite(D3, 0);
  analogWrite(Left_Forward, 0);
  analogWrite(Left_Backward, 0);
  analogWrite(Left_Enable, 0);
  analogWrite(Right_Forward, 0);
  analogWrite(Right_Backward, 0);
  analogWrite(Right_Enable, 0);

  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.

  //bool LED_strip_last_state = LOW;
  bool dataUpdated = BP32.update();
  if (dataUpdated)
    processControllers();

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    // vTaskDelay(1);
  delay(150);
}