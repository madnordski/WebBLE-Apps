/*** ServoCar

     This is a servo control script intended to be used with Web BLE
     interface. See ServoCar.html for information on the
     interface. It controls two 360 degree servo motors, one
     representing X movement and another representing Y axis
     movement. Keep in mind that with 360 degree servos you control
     speed and direction but not position.  We also control a red LED
     which was useful in proving our Bluetooth LE was working. A green
     LED is used to indicate when a client is connected.  The intended
     client ServoCar.html and runs in a Chrome browser on Android.
     You can see BLE characteristics and toggle the red LED state
     using a BLE tool such as nRF Connect.  Read the comments in this
     file and especially in ServoCar.html to understand what's going
     on here as well as a few important lessons regarding the use of
     Arduino ESP32 BLE library and Web BLE in Chrome.

     This code runs on an XIAO ESP32-C3 but should run on other ESP32
     boards. There was a change made to the type returned by
     BLECharacteristic::getValue() that caused compiled failures on
     the S3 board I tried.  You can easily work around this because
     the first thing this program does with the returned String value
     is convert it to a C string which was what the S3 returned.

     Android phones assume this is a headphone device so when it
     disconnects, you'll get a notification that headphone modes is off.

     NB: the word ASPECT is used to represent CHARACTERISTICS in many
     places only because characteristics cumbersome to type. It has no
     special meaning beyond representing characteristics.
     
     Uses BLE read/write/indicate. 

     Parts of this code was based on the work of Rui Santos.
     Find his project details at
     https://RandomNerdTutorials.com/esp32-web-bluetooth/

     Copyright (c) 2024, Joseph J. King, PhD

     Permission is hereby granted, free of charge, for educational
     purposes. Use of the this software for commercial purposes is
     strictly prohibited.  The above copyright notice and this
     permission notice shall be included in all copies or substantial
     portions of the Software.

     Generate UUIDs here:
     // https://www.uuidgenerator.net/

***/

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLE2901.h>
#include <ESP32Servo.h>
#include <Arduino.h>

#include <esp_system.h>
#include <esp_heap_caps.h>
#include <esp_flash.h> // For flash memory APIs

//#define DEBUG (timings are wrong when debug is enabled)
#ifdef DEBUG
#define sp1(x)   Serial.println(x)
#define sp2(x,y) Serial.print(x);Serial.println(y)
#else
#define sp1(x)
#define sp2(x,y)
#endif

// BLE instance pointers
BLEServer *pServer = NULL;
BLECharacteristic *pStatusAspect = NULL;
BLECharacteristic *pLedAspect = NULL;
BLECharacteristic *pServoAspect = NULL;
BLECharacteristic *pControlAspect = NULL;
BLE2901 *statusD2901 = NULL;
BLE2901 *controlD2901 = NULL;
BLE2901 *ledD2901 = NULL;

// state variables
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

// we connect our LEDs and Servos to these pins on the ESP32-C3
// NB: the C3 requires D9 instead of just 9 -- change this as needed
const int redPin = D9; // Use the appropriate GPIO pin for your setup
const int grnPin = D10; // connected
const int servo1Pin = D7; // must use D notation for digital pins on this board
const int servo2Pin = D6; // must use D notation for digital pins on this board

// we will need servo class instances
Servo servo1;   // X
Servo servo2;   // Y

// server
#define DEVICENAME "ServoCar"

// UUIDs - derevative works should replace these with unique ones
#define SERVICE_UUID        "4339e3f5-3dbc-4c95-a7a2-0282b5c65a42"
#define LED_ASPECT_UUID "a3ca6555-03a3-4c85-9bdd-b13a8008ef25"
#define STATUS_ASPECT_UUID "3211cb2c-9cc3-46be-9f1d-9dfcf734c4fc"
#define SERVO_ASPECT_UUID "51d584bf-1d90-49f2-a31e-5dc6442050f8"
#define CONTROL_ASPECT_UUID "d5301086-6975-48c3-976a-2847eb453eaf"

// servo status codes
#define STOPPED 2
#define MOVING  1
#define READY   0

// servo movement commands
#define CW  -1
#define CCW  1
#define STOP 0

// servo status and speed settings
uint8_t gServoStatus = STOPPED;

// record
bool gRecording = false;
bool gPlaying = false;
unsigned long gRecordStart = 0;

// NB: we use a heartbeat method to obtain client proof of life.
// This allows us to restart the ESP32 when the client goes away for
// any reason. We did it this way because the client can end without
// ever triggering the onDisconnect callback. ESP.restart() is our
// standard way to get ready for the next connection.

bool gACK = true;    // we start with this true for each connection because
                     // we are checking on the previous status update

// We have a linked list that will record all of moves when requested.

typedef struct listNode {
  Servo *servo;
  int speed;
  int direction;
  unsigned long time;
  struct listNode *next;
} Node;

Node *rootNode = NULL;
Node *thisNode = NULL;

/** startList

    Free the old list, if any and start a new one.
*/
void startList() {
  if ( rootNode != NULL ) {
    // free this list
    freeList();
    rootNode = NULL;
  }
  rootNode = (Node *) malloc(sizeof(Node));
  thisNode = rootNode;
  thisNode->next = NULL;
  gRecordStart = millis();
  gRecording = true;
}

/** addToList

    Add a move to the list. Notice, the last node will be
    empty. Relies on thisNode!!!
    
    @param servo The servo that got this move command.
    @param speed The speed setting for this servo.
    @param direction The direction of movement.
    @return nothing.
*/

void addToList(Servo *servo, int speed, int direction) {
  if ( thisNode == NULL ) {
    sp1("ERROR in addToList: Attempt to add to a null list failed!");
  }
  else {
    thisNode->servo = servo;
    thisNode->speed = speed;
    thisNode->direction = direction;
    thisNode->time = millis() - gRecordStart;
    thisNode->next = (Node *) malloc(sizeof(Node));
    thisNode = thisNode->next;
    thisNode->next = NULL;
  }
}

/** freeList

    Free the list.  We iterate instead of recurse to save memory
    (i.e. less stack use).  How do we go backward?

    @return nothing
*/

void freeList() {
  Node *tmp, *node;
  node = rootNode;
  rootNode = NULL;
  while ( node != NULL ) {
    tmp = node;
    node = node->next;
    // free the node
    free(tmp);
  }
}

/** endList

    Not sure what we do here except reset the recording flag.
*/

void endList() {
  gRecording = false;
}

/** playList

    Play back the recorded moves.  NB: we have to add start and stop.

*/

void playList() {
  Node *node = rootNode;
  unsigned long start;
  gServoStatus = READY; // required to obey
  gRecording = false;

  pStatusAspect->setValue("Playing back");
  pStatusAspect->indicate();
  
  start = millis();
  
  while ( gPlaying && node != NULL && node->next != NULL ) {
    // each node has includes a delay from the start
    while (node->time > millis() - start) {}
    // now we move
    servoMove(node->servo, node->speed, node->direction);
    node = node->next;
  }

  playStop();
}

/** playStop

    stop the playback
*/

void playStop() {
  servoStop(&servo1);
  servoStop(&servo2);
  gServoStatus = STOPPED;
  gPlaying = false;
  pStatusAspect->setValue("Playback Ended");
  pStatusAspect->indicate();
}

void printList() {
  Node *node = rootNode;
  while ( node != NULL && node->next != NULL ) {
    Serial.print("speed "); Serial.println(node->speed);
    Serial.print("direction "); Serial.println(node->direction);
    Serial.print("time "); Serial.println(node->time);
    node = node->next;
  }
}

/** servoMove

    Tell the servo to move forward or back at the specified speed. We
    do some math to convert speed and direction to 0 to 180 degrees
    which is what the servo library expects. The speed of 10 is
    fastest regardless of direction.
    
    @param servo Pointer to a servo object.
    @param speed An integer value between 0 and 10, with 10 being
    fastest.
    @param direction The direction of movement. -1 for CW, 1 for CCW.
    @return nothing
*/

void servoMove(Servo *servo, int speed, int direction) {
  // speed = 0 to 10, direction = -1 for CW, 1 for CCW
  if ( speed < 0 || speed > 10 ) speed = 5; // default speed if out of bounds
  int code = 90 + direction * speed * 9; // d=-1: 90 to 0, d=1: 90 to 180
  
  // NB: speed: 0 = 90 degrees = stop
  servo->write(code);
  delay(15);
  gServoStatus = MOVING;
  if ( gRecording ) 
    addToList(servo, speed, direction);
}

/** servoStop

    Stop the servo. Passing zero for the direction is enough but we'll
    pass zero for the speed as well.

    @param servo Servo object for the servo we want to stop.
    @return nothing
*/

void servoStop(Servo *servo) {
  servoMove(servo, STOP, STOP);
  servo->release();
  delay(15);
  gServoStatus = STOPPED;
}

/** ServerCallbacks Method Override

    The BLE subsystem will call these methods when a connection to a BLE
    client is established or when it goes away.  Here we are
    overriding the class methods onConnect() and onDisconnect().
 */

class ServerCallbacks: public BLEServerCallbacks {
  /** onConnect
      Set the connection flag, turn the green led on and set gACK to
      true.
  */
  
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    digitalWrite(grnPin, HIGH);
    gACK = true; // set this true for the first proof of life check
  };

  /** onDisconnect
      Set the connection flag and turn off the green LED.
  */
  
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    digitalWrite(grnPin, LOW);
  }
};

/** LEDAspectCallbacks Method Override

    The BLE subsystem calls the onWrite() method when the client
    writes a message to the server (limited to 20 bytes).
*/
   
class LEDAspectCallbacks : public BLECharacteristicCallbacks {
  /** onWrite
      
      Get the value the client has sent. The version of the ESP32-C3
      BLE Arduino library we are using returns a C++ String. Some
      versions returned a C-string pointer instead (October 2024).

      @param pLedAspect The Charactistic object containing the value.
      @return nothing
  */
  
  void onWrite(BLECharacteristic* pLedAspect) {
    String value = pLedAspect->getValue();
    if (value.length() > 0) {
      char cstr[21];
      strncpy(cstr, value.c_str(), sizeof(cstr));
      sp2("Value length: ", value.length());
      sp2("LED Aspect: ", cstr); // Print the value (debugging)

      // act on the command with no error checking
      if ( strstr("off", cstr) ) {
        digitalWrite(redPin, LOW);
      } else {
        digitalWrite(redPin, HIGH);
      }
    }
  }
};

/** ControlAspectCallbacks Method Override

    This override gives the client the ability to control the ESP32
    server. The strings sent by the client have the following
    purposes.

    String    Purpose
    DIE       This restarts the server. We do this when we disconnect
              to ensure the next connection starts fresh. We
              discovered there are situations when the onDisconnect
	      callback cannot be relied upon.
    ACK       This is used to acknowledge the client recieved our status
              update.  We use this as proof of life for the client. If
	      the client shutdown, we want to restart so that we can
	      accept a new connection.
    STOP      Stop the servos. Using a separate characteristic for STOP
              and START insures these commands don't get lost in the
	      stream of move commands the client sends when a finger is
	      moving on the touch screen.
    START     Prepare for movement. Sent when the touch screen if first
              touched.
    SPEED a s Sets the servo speed.  For example, "SPEED 0 6" sets the X
              axis to speed zero and "SPEED 1 9" sets the Y servo to 9.
              The speed setting is 0 to 10 with 0 for slowest, 10 for fastest.
    RECORD    Start recording the moves, if true.
    
*/

class ControlAspectCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pControlAspect){
    char *p, *pAxis, *pSpeed, *tok;
    String value = pControlAspect->getValue();
    if ( value.length() > 0 ) {
      char cstr[21];
      strncpy(cstr, value.c_str(), sizeof(cstr));
      sp2("Control called: ", cstr);
      if ( strstr(cstr, "DIE") ) {
	// the cleanest way to be ready for next connection is to restart
	ESP.restart();
      }
      else if ( strstr(cstr, "ACK") ) {
	// proof of life acknowledgment from client
	gACK = true;
      }
      else if ( strstr(cstr, "REC_BEGIN") ) {
	startList();
      }
      else if ( strstr(cstr, "REC_END") ) {
	endList();
      }
      else if ( strstr(cstr, "REC_PLAY") ) {
	gPlaying = true; // play in the background
      }
      else if ( strstr(cstr, "STOP") ) {
	// STOP THE ENGINES!
	if ( gPlaying ) {
	  playStop();
	}
	else {
	  servoStop(&servo1);
	  servoStop(&servo2);
	  gServoStatus = STOPPED;
	  sp1("Stop completed.");
	}
      }
      else if ( strstr(cstr, "START") ) {
	// Prepare to move the servos.
	if ( gPlaying ) {
	  playStop();
	  spin(100);
	}
	gServoStatus = READY;
	sp1("Ready completed.");
      }
    }
  }
};

/** ServoAspectCallbacks Method Override

    This override supports the move servo command from the
    client. NB: this does not include starting or stopping the servos.

    The client sends two numbers (X,Y) as a text message. These numbers
    tell us which motor to turn on and which direction it should turn.

    Acceptible values for X and Y are only -1 and 1, zero is ignored
    so use the STOP command to stop the servos. The -1 is for
    clockwise and 1 if for CCW.  The speed of the motors is set with
    the SPEED command.  This will only ever start or reverse servo
    motion at the user set speed.
*/

class ServoAspectCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pServoAspect) {
    String value = pServoAspect->getValue();
    sp1("Entered servo callback");

    // our value is a string representing two integers
    
    if (value.length() > 0) {
      char cstr[21];           // max message length is 20 + 1 for null
      char *pX, *pY, *tok;

      // stop any playback in progress
      if ( gPlaying ) playStop();
      
      strncpy(cstr, value.c_str(), sizeof(cstr));
      
      // Allow the STOP command to shut us down, no matter how many
      // move commands remain in the queue.
      // nb: the strtok_r result in the if statement gets the x
      
      if ( gServoStatus != STOPPED
	   && (pX = strtok_r(cstr, " ", &tok)) != NULL ) {
	pY = strtok_r(NULL, " ", &tok);  // pull out the y
	int x = atoi(pX);
	int y = atoi(pY);
	sp2("x=", x); sp2("y=", y);

	
	// move clockwise or counter clockwise
	// NB: the motors get mounted upside down with respect to
	// each other so CCW for one is CW for the other
	if ( x )
	  servoMove(&servo1, abs(x), x > 0 ? CCW : CW);
	if ( y )
	  servoMove(&servo2, abs(y), y > 0 ? CW : CCW);
      }
    }
  }
};

void setup() {
  // serial output for debugging only, see #debug above
  Serial.begin(115200);

  // set up the red and green LEDs
  pinMode(redPin, OUTPUT);
  pinMode(grnPin, OUTPUT);
  digitalWrite(grnPin, HIGH);
  digitalWrite(redPin, HIGH);

  // Create the BLE Device
  BLEDevice::init(DEVICENAME);

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create the Status Characteristic (herein called Aspects)
  // This is used to communicate the status of servos but is also used
  // as a heartbeat - the client responds to the heartbeat to prove it's alive
  pStatusAspect = pService->createCharacteristic(
		      STATUS_ASPECT_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // Create the RED LED Characteristic
  // This toggles the red led
  pLedAspect = pService->createCharacteristic(
                      LED_ASPECT_UUID,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  // Create the Servo Characteristic
  // The client writes to this to move the servos
  pServoAspect = pService->createCharacteristic(
		      SERVO_ASPECT_UUID,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  // Create the control Characteristic
  // It turned out to be essential to have a characteristic write the
  // client could use to stop the servos. Other uses include restart
  // and acknowledge, see handler for details.
  pControlAspect = pService->createCharacteristic(
		      CONTROL_ASPECT_UUID,
		      BLECharacteristic::PROPERTY_WRITE);

  // Create a BLE Descriptor for the status aspect because it uses INDICATE
  pStatusAspect->addDescriptor(new BLE2902());

  // Optionally, be friendly - to see this use the LightBlue app.
  // It doesn't show up in nRF Connect & our Web BLE App does not use these.
  // In other words, delete all of these lines and everything still works.
  statusD2901 = new BLE2901();
  statusD2901->setValue("Servo Status");
  statusD2901->setAccessPermissions(ESP_GATT_PERM_READ);
  pStatusAspect->addDescriptor(statusD2901);
  controlD2901 = new BLE2901();
  controlD2901->setValue("Control Aspect");
  controlD2901->setAccessPermissions(ESP_GATT_PERM_READ);
  pControlAspect->addDescriptor(controlD2901);
  ledD2901 = new BLE2901();
  ledD2901->setValue("LED ON/OFF Toggle");
  ledD2901->setAccessPermissions(ESP_GATT_PERM_READ);
  pLedAspect->addDescriptor(ledD2901);
  

  // Set callbacks for our characteristics (aka aspects)
  // nb: no overrides for the status aspect
  
  // Set the RED LED callback
  pLedAspect->setCallbacks(new LEDAspectCallbacks());

  // Set the Servo callback
  pServoAspect->setCallbacks(new ServoAspectCallbacks());

  // Set the Control callback
  pControlAspect->setCallbacks(new ControlAspectCallbacks());
  
  // Start the service
  pService->start();

  // get the servos ready before advertising our service
  // attach and start the servos
  startServos();

  // *** START BROADCASTING OUR SERVICE ***
  
  // get a pointer to the BLE advertising instance
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();

  // our custom service UUID
  pAdvertising->addServiceUUID(SERVICE_UUID);

  // don't respond to scans (saves battery)
  pAdvertising->setScanResponse(false);

  // this leaves the connection interval up to the client
  pAdvertising->setMinPreferred(0x0);

  // make it happen
  BLEDevice::startAdvertising();

  sp1("Waiting a client connection...");

  // all done with setup, turn off our LEDs
  digitalWrite(redPin, LOW);
  digitalWrite(grnPin, LOW);

  // optionally do a little dance at startup
  //servoTest(&servo1);
  // system info

#ifdef DEBUG
  printSysInfo();
  sp1("*******************************************************");
  sp2("size of node: ", sizeof(listNode));
  unsigned long h = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
  sp2("size of free heap: ", h);
  sp2("number of nodes possible: ", h / sizeof(listNode));

  sp1("--------------------------------------------------------");
  sp1("simple test of linked list...");
  startList();
  for (int i=0; i <= 10; i++) {
    addToList(&servo1, i, 1);
    addToList(&servo2, i, 1);
    spin(50 * i);
  }
  sp2("free heap after mallocs: ", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
  printList();
  freeList();
  sp2("free heap after free: ", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
  
#endif
  
}

void startBLE() {
  // it's possible to do all the BLE setup here and then we
  // could deinit() ble to disconnect all clients
  // but, why not just restart???
}

// these global variables support our loop

int gLastStatus = READY; // set to ready so that we write immediately

// when connected, check on the client reqularly by indicating status
unsigned long gHeartbeat = millis();     // timer
const unsigned long gHBI = 1000;         // heartbeat interval
const unsigned long gProofOfLife = 2000; // client must respond in this time

void loop() {
  char msg[21];

  // play the list

  if ( gPlaying ) {
    // we stay here until the playlist is complete
    playList();
    playStop();
  }
  
  // BLE indicate is used to tell the client we have a status update.
  // Status updates come every second or when the status changes.
  // We only do updates when we're connected of course.
  // nb: Notify or Indicate -- not sure it matters when using the ESP
  // BLE library for Arduino because we don't have access to error
  // callbacks (that's why we do our own using gACK).
  
  if ( deviceConnected
       && (gLastStatus != gServoStatus || millis() - gHeartbeat > gHBI) ) {
    gLastStatus = gServoStatus;
    if ( gLastStatus == MOVING  ) strcpy(msg, "Servos Moving");
    if ( gLastStatus == STOPPED ) strcpy(msg, "Servos Stopped");
    if ( gLastStatus == READY   ) strcpy(msg, "Servos Ready");
    if ( gRecording )             strcpy(msg, "Recording...");

    // Before indicating a status update, we check to see if the
    // previous update was acknowledged by the client.
    // This proof of life must be received in gProofOfLifeTime or we
    // reboot so that we are ready for the next connection.
    // NB: gACK is set to true when a client connects
    
    unsigned long start = millis();   // start the proof of life timer
    while ( !gACK ) {
      if ( millis() - start > gProofOfLife ) {
	ESP.restart();
      }
      sp1("waiting for ACK...");
      spin(50);
    }
    sp1("ACK recieved");

    // set gACK to false for this update - the client will set it true
    gACK = false;

    // send our status update (which is also a heartbeat, aka ping-pong)
    pStatusAspect->setValue(msg);
    pStatusAspect->indicate();

    // reset the heartbeat timer
    gHeartbeat = millis();
    sp2("Indication sent: ", msg);
  }
  
  // Our onDisconnect callback has been triggered.
  // This might not ever happen because the client tells us to restart
  // when it goes away.
  if (!deviceConnected && oldDeviceConnected) {
    sp1("Device disconnected.");
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    sp1("Start advertising");
    oldDeviceConnected = deviceConnected;
  }
  
  // Our onConnect callback has been triggered.
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
    sp1("Device Connected");
  }

  // blink when not connected
  if ( !deviceConnected ) {
    digitalWrite(redPin, HIGH);
    spin(500);
    digitalWrite(redPin, LOW);
    spin(500);
  }
}

void startServos() {

  // test the servo before continuing
  // Allow allocation of all timers
  // OKAY now we understand this
  // 0 to 90 (fast to slow) CW, front view)
  // 90 to 180 (slow to fast) CCW, front view)
  // 90 = stop
  // can't read position but can read speed and direction

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo1.setPeriodHertz(50);    // standard 50 hz servo
  servo2.setPeriodHertz(50);
  
  delay(300);
  
  servo1.attach(servo1Pin, 1000, 2000);
  servo2.attach(servo2Pin, 1000, 2000);
  delay(300);
}

void servoTest(Servo *servo) {
  int speed = 5;
  Serial.println("start servo test");
  // zero to 90 = 1.0 ms to 1.5 ms
  // 90 to 180 = 1.5 ms to 2.0 ms
  Serial.println("Spin CW");
  servoMove(servo, speed, CW);
  for (unsigned long start = millis(); millis() - start < 2000; ) {
  }
  servoStop(servo);
  delay(1000);
  Serial.println("Spin CCW");
  servoMove(servo, speed, CCW);
  for (unsigned long start = millis(); millis() - start < 2000; ) {
  }
  servoStop(servo);
  delay(1000);
  Serial.println("Bonus! Spin both CCW");
  servoMove(&servo1, speed, CCW);
  servoMove(&servo2, speed, CCW);
  for (unsigned long start = millis(); millis() - start < 2000; ) {
  }
  servoStop(&servo1);
  servoStop(&servo2);
  Serial.println("Servo test complete.");
}

void spin(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) { }
  return;
}

void printSysInfo() {
  printChipInfo();
  printMemoryInfo();
  printFlashInfo();
}

void printChipInfo() {
  Serial.println("ESP32-C3 Chip Information:");
  Serial.printf("Chip Model: %s\n", ESP.getChipModel());
  Serial.printf("Chip Revision: %d\n", ESP.getChipRevision());
  Serial.printf("Cores: %d\n", ESP.getChipCores());
}

void printMemoryInfo() {
  // Heap memory information
  Serial.println("\nHeap Memory Information:");
  Serial.printf("Total heap size: %d bytes\n",
		heap_caps_get_total_size(MALLOC_CAP_DEFAULT));
  Serial.printf("Free heap size: %d bytes\n",
		heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
  Serial.printf("Minimum free heap ever: %d bytes\n",
		heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT));

  // Internal memory breakdown
  Serial.println("\nInternal Memory Breakdown:");
  Serial.printf("Internal Total: %d bytes\n",
		heap_caps_get_total_size(MALLOC_CAP_INTERNAL));
  Serial.printf("Internal Free: %d bytes\n",
		heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
}

void printFlashInfo() {
  Serial.println("\nFlash Memory Information:");

  uint32_t flash_chip_size = 0;
  if (esp_flash_get_size(nullptr, &flash_chip_size) == ESP_OK) {
    Serial.printf("Flash chip size: %d bytes\n", flash_chip_size);
  } else {
    Serial.println("Failed to get flash chip size.");
  }

  extern uint8_t _data_start, _data_end;
  size_t used_program_memory = (size_t)&_data_end - (size_t)&_data_start;
  size_t available_program_memory = flash_chip_size - used_program_memory;

  Serial.printf("Program memory used: %d bytes\n", used_program_memory);
  Serial.printf("Program memory available: %d bytes\n", available_program_memory);
}

