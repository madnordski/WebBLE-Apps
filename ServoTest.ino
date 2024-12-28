/*** ServoTest

     This is a servo control script intended to be used with a Web BLE
     interface. See ServoTest.html for information on the
     interface. It controls two 360 degree servo motors, one
     representing X movement and another representing Y axis
     movement. Keep in mind that with 360 degree servos you control
     speed and direction but not position.  We also control a red LED
     which was useful in proving the Bluetooth LE connection was
     working. A green LED is used to indicate when a client is
     connected.  The intended client is ServoTest.html and runs in a
     Chrome browser on Android.  You can see BLE characteristics and
     toggle the red LED state using a BLE tool such as nRF Connect or
     LightBlue.  Read the comments in this file and especially in
     ServoTest.html to understand what's going on here as well as a
     few important lessons regarding the use of Arduino ESP32 BLE
     library, touch events, Web BLE in Chrome and moving motors with
     BLE.

     This code has been run on an XIAO ESP32-C3 and an XIAO ESP32-S3
     but should run on other ESP32 boards. If your Arduino BLE library
     is out of date or you still have the old one installed, you'll
     get a complie error due to a change to the return type of
     BLECharacteristic::getValue().  Be sure to uninstall
     older versions. I had to do this even after installing the new
     version.

     NB: the word ASPECT is used to represent CHARACTERISTICS in
     variable names only because characteristics was cumbersome to
     type. It has no special meaning beyond representing
     characteristics. Just in case you didn't know, NB is latin for
     <i>nota bene</i> which translates to note well.
     
     Uses BLE read/write/notify. 

     Parts of this code was based on the work of Rui Santos.
     Find his project details at
     https://RandomNerdTutorials.com/esp32-web-bluetooth/

     Things to learn from this file that are not included in the RNT
     example:

     **1** Client Characteristic Configuration Descriptors (CCCD) meanings
     and use for 2902 and 2901.

     **2** When using a WebBLE client, what is the preferred interval
     and does it make a difference?

     **3** Optimizing the handling of motor movement or other possibly
     time consuming requests by using PROPERTY_WRITE_NR and FreeRTOS
     tasks.

     Copyright (c) 2024, Joseph J. King, PhD

     Permission is hereby granted, free of charge, for educational
     purposes. Use of the this software for commercial purposes is
     strictly prohibited.  The above copyright notice and this
     permission notice shall be included in all copies or substantial
     portions of the Software.

***/

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLE2901.h>
#include <ESP32Servo.h>

// uncomment to turn on debugging but also be aware, printing to the
// serial monitor impacts timing (i.e. some ble callbacks might take
// too long)

//#define DEBUG

#ifdef DEBUG
#define sp1(x)   Serial.println(x)
#define sp2(x,y) Serial.print(x);Serial.println(y)
#else
#define sp1(x)
#define sp2(x,y)
#endif

// simplify serial print when full debugging isn't desired
#define sp2s(x,y) Serial.print(x);Serial.print(" ");Serial.println(y)
#define sp2x(x,y) Serial.print(x);Serial.println(y)
#define sp1x(x)   Serial.println(x)

// BLE instance pointers
BLEServer *pServer = NULL;
BLECharacteristic *pStatusAspect = NULL;
BLECharacteristic *pLedAspect = NULL;
BLECharacteristic *pServoAspect = NULL;
BLECharacteristic *pControlAspect = NULL;
BLE2901 *statusD2901 = NULL;
BLE2901 *controlD2901 = NULL;
BLE2901 *ledD2901 = NULL;

// The connection interval in BLE interval units is 1.25 ms/unit.
// NB: 7.5 ms is the Android minimum
int minIntervalNA = 0; // don't set the minInterval, it's up to the client
int minIntervalAndroid = 6; // 7.5ms (lowest setting for Android)
int minInterval10  = 8;   //  10ms
int minInterval20  = 16;   //  20ms
int minInterval40  = 32;  //  40ms
int minInterval80  = 64;  //  80ms
int minInterval160 = 128; // 160ms
int minInterval180 = 155; // 180ms (time for 6 20ms packets)
int minInterval200 = 160; // 200ms (6 packet w/headroom)
int minInterval320 = 256; // 320ms
int minInterval640 = 512; // 640ms
int minInterval1280 = 1024; // 1280ms
// set it here
int gIntervalSetting = minIntervalNA;

// state variables
bool deviceConnected = false;
bool oldDeviceConnected = false;

// we connect our LEDs and Servos to these pins on the ESP32-C3
// NB: the C3 requires D9 instead of just 9 -- change this as needed

#ifdef D9
const int redPin = D9; // Use the appropriate GPIO pin for your setup
const int grnPin = D10; // connected
const int servo1Pin = D7; // must use D notation for digital pins on this board
const int servo2Pin = D4; // must use D notation for digital pins on this board
#else
const int redPin = 9; // Use the appropriate GPIO pin for your setup
const int grnPin = 10; // connected
const int servo1Pin = 7; // must use D notation for digital pins on this board
const int servo2Pin = 4; // must use D notation for digital pins on this board
#endif

// we will need servo class instances
Servo servo1;   // X
Servo servo2;   // Y

// **3** Create a queue for servo movements. This is used to keep
// things from piling up on the BLE stack. It reduces errors
// (slightly). NB: errors were largely by use of WRITE_NR instead of
// WRITE.

// our servo queue handle
QueueHandle_t servoCommandQueue;

// our move command for two servos, packaged for task queue
struct ServoCommand {
  int x;
  int y;
};
  
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
// If you modify this file to create your own project, generate new
// UUIDs to replace these.

#define SERVICE_UUID        "75f29868-d8a4-452c-a78b-1f698ee6ab2b"
#define STATUS_ASPECT_UUID "1a476625-3aae-4ab4-a1a7-0c9e1f5c97e0"
#define LED_ASPECT_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"
#define SERVO_ASPECT_UUID "99819a8b-59cc-4174-beee-fcc74b02fc5b"
#define CONTROL_ASPECT_UUID "05ca6ae0-6810-4dc0-9f00-424d9b06261b"

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
uint8_t gXSpeed = 5;
uint8_t gYSpeed = 5;
unsigned long gMoveCount = 0;
int gLastN = 0;
unsigned long gStartTime = 0;

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
  //spin(15);
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
  //spin(15);
  gServoStatus = STOPPED;
}

/** ServerCallbacks Method Override

    The BLE subsystem will call these methods when a connection to a BLE
    client is established or when it goes away.  Here we are
    overriding the class methods onConnect() and onDisconnect().
 */

class ServerCallbacks: public BLEServerCallbacks {
  /** onConnect
      Set the connection flag and turn the green led on.  The loop()
      takes action when the connection status changes.
  */
  
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    digitalWrite(grnPin, HIGH);
  };

  /** onDisconnect
      Set the connection flag and turn off the green LED. The loop()
      takes action when the deviceConnected status changes.
      Gets called when the WebBLE client:
      1. Intentionally disconnects
      2. Browser is closed
      3. Turns Bluetooth off
  */
  
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    digitalWrite(grnPin, LOW);
    sp1x("onDisconnect called.");
  }
};

/** LEDAspectCallbacks Method Override

    The BLE subsystem calls the onWrite() method when the client
    writes a message to the server (limited to 20 bytes).
*/
   
class LEDAspectCallbacks : public BLECharacteristicCallbacks {
  /** onWrite
      
      Get the value the client has sent. Some older verions of the BLE
      Arduino library returned a stanard C string. Newer versions
      return a C-string pointer instead (October 2024).

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
      }
      else { // we could insist on "on" but we don't
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
	      callback cannot be relied upon. (obsolete, not used)
    ACK       This is used to acknowledge the client recieved our status
              update.  We use this as proof of life for the client. If
	      the client shutdown, we want to restart so that we can
	      accept a new connection. (obsolete, not used)
    STOP      Stop the servos. Using a separate characteristic for STOP
              and START insures these commands don't get lost in the
	      stream of move commands the client sends when a finger is
	      moving on the touch screen.  NB: these also circumvent
              move commands in the queue (when used).
    START     Prepare for movement. Sent when the touch screen if first
              touched.  Needed to make STOP work properly.
    SPEED a s Sets the servo speed.  a is for axis (0=x, 1=y) and s is
              for speed (0 to 10). For example, "SPEED 0 6" sets the X
              axis to speed zero and "SPEED 1 9" sets the Y servo to
              9.  The speed setting is 0 to 10 with 0 for slowest, 10
              for fastest.
*/

bool gACK = true; // was for proof of life, this is no longer used

class ControlAspectCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pControlAspect){
    char *p, *pAxis, *pSpeed, *tok;
    String value = pControlAspect->getValue();
    if ( value.length() > 0 ) {
      char cstr[21];
      strncpy(cstr, value.c_str(), sizeof(cstr));
      //sp2("Control called: ", cstr);
      if ( strstr(cstr, "DIE") ) {
	// the cleanest way to be ready for next connection is to restart
	ESP.restart();
      }
      else if ( strstr(cstr, "ACK") ) {
	// proof of life acknowledgment from client
	gACK = true;
      }
      else if ( strstr(cstr, "STOP") ) {
	unsigned long elapsedTime;
	elapsedTime =  millis() - gStartTime;
	// STOP THE ENGINES!
	gServoStatus = STOPPED; // do this first because moves could be queued
	servoStop(&servo1);
	servoStop(&servo2);
	sp1("Stop completed.");
	spin(20);
	sp2x("elapsed time: ", elapsedTime);
	spin(20);
	if ( gMoveCount > 0 ) {
	  sp2x("time per move: ", elapsedTime / gMoveCount);
	}
      }
      else if ( strstr(cstr, "START") ) {
	// Prepare to move the servos.
	gServoStatus = READY;
	gMoveCount = 0;
	gLastN = 0;
	gStartTime = millis();
	sp1("Ready completed.");
      }
      else if ( strstr(cstr, "SPEED")
		&& (p = strtok_r(cstr, " ", &tok)) != NULL ) {
	// the full command is SPEED a s
	//   a is for axis, 0=X, 1=Y
	//   s is for speed, 0 to 10 where 10 is the fastest
	pAxis = strtok_r(NULL, " ", &tok);
	pSpeed = strtok_r(NULL, " ", &tok);
	if ( atoi(pAxis) == 0 )    // x axis
	  gXSpeed = atoi(pSpeed);
	else                       // y axis
	  gYSpeed = atoi(pSpeed);
      }
    }
  }
};

/** ServoAspectCallbacks Method Override

    This override supports the move servo command from the
    client. NB: this does not include starting or stopping the servos.

    The client sends two numbers (X,Y) as a text message. These numbers
    tell us which motor to move and which direction it should turn.

    Acceptible values for X and Y are only -1 and 1, zero is ignored
    so use the STOP command to stop the servos. The -1 is for
    clockwise and 1 if for CCW.  The speed of the motors is set with
    the SPEED command.  This will only ever start or reverse servo
    motion at the user set speed.

    The command string also contains a move number. We used this to
    ensure we are performing moves in the intended order.  NB: when
    using client retry without also using a client queue, move
    commands sometimes arrived in the incorrect order.
*/

class ServoAspectCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pServoAspect) {
    String value = pServoAspect->getValue();
    
    // our value is a string representing two integers
    
    if (value.length() > 0 && gServoStatus != STOPPED ) {
      char cstr[21];           // max message length is 20 + 1 for null
      char *pX, *pY, *pN, *tok;
      strncpy(cstr, value.c_str(), sizeof(cstr));

      // Allow the STOP command to shut us down, no matter how many
      // move commands remain in the queue.
      // nb: the strtok_r result in the if statement gets the x
      
      if ( gServoStatus != STOPPED
	   && (pX = strtok_r(cstr, " ", &tok)) != NULL ) {
	gMoveCount++;
	pY = strtok_r(NULL, " ", &tok);  // pull out the y
	pN = strtok_r(NULL, " ", &tok);  // pull out the move number
	int x = atoi(pX);
	int y = atoi(pY);
	int n = atoi(pN);
	//if ( gMoveCount != n+1 ) {
	if ( n < gLastN ) {
	  sp1x("Bad order");
	  sp2s(gLastN, n);
	}
	gLastN = n;
	ServoCommand command;
	sp2("x=", x);
	sp2("y=", y);
	command.x = x;
	command.y = y;

	if ( true ) { // TRUE = use queue, FALSE = don't use queue
	  // queue this move and return
	  if ( xQueueSend(servoCommandQueue, &command, 0) != pdTRUE ) {
	    Serial.println("Servo queue is full!");
	  }
	  else if ( x || y ) {
	    gServoStatus = MOVING;
	  }
	}
	else if ( gServoStatus != STOPPED ) {
	  if ( command.x )
	    servoMove(&servo1, gXSpeed, command.x > 0 ? CCW : CW);
	  if ( command.y )
	    servoMove(&servo2, gYSpeed, command.y > 0 ? CCW : CW);
	  if ( x || y ) gServoStatus= MOVING;
	}
      }
    }
  }
};

/** servoTask

 **3** Moving motors gets it's own thread because it takes
 time. Servicing it in the BLE callback causes the BLE stack to get
 overwhelmed under certain test conditions resulting in an increased
 number of errors that occur when the client writes to this server to
 move servos.

 NB: When we get the stop command, this task will drain the queue.
 
 @param param Passed to us from FreeRTOS.
 
*/

void servoTask(void *param) { 
  ServoCommand command;
  while ( true ) {
    if ( xQueueReceive(servoCommandQueue, &command, portMAX_DELAY) ) {
      if ( gServoStatus != STOPPED ) { // control command stop, drains
				       // the queue
	if ( command.x )
	  servoMove(&servo1, gXSpeed, command.x > 0 ? CCW : CW);
	if ( command.y )
	  servoMove(&servo2, gYSpeed, command.y > 0 ? CCW : CW);
      }
    }
  }
}

// SETUP

void setup() {
  // serial output for debugging only, see #debug above
  Serial.begin(115200);

  // set up the red and green LEDs
  pinMode(redPin, OUTPUT);
  pinMode(grnPin, OUTPUT);
  digitalWrite(grnPin, HIGH);
  digitalWrite(redPin, HIGH);

  // Create the BLE Device
  BLEDevice::init("ServoTest");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create the Status Characteristic (herein called Aspects)
  // This is used to communicate the status of servos
  
  /* NB: Tested and rejected:

     1. Use of this for a heartbeat to detect client disconnection.  We
     tested using this plus gACK acknowledgement as a way for the
     client to prove it was alive.  Due to a bug in the RNT example,
     client disconnects were not being handled properly.  Once we
     found and corrected the bug in ServoTest.html, proof of life was
     no longer necessary.
     
     2. Use of INDICATE instead of NOTIFY. By using INDICATE we
     determined it has no functional benefit when using WebBLE and the
     Arduino BLE library.  It does increase overhead in the BLE layer,
     however, by requiring replies for each INDICATE sent.  When using
     the Arduino Library for BLE, INDICATE should not be used.
  */
  
  pStatusAspect = pService->createCharacteristic(
		      STATUS_ASPECT_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  // **3** No need to make the BLE Layer wait for a response.
  // Create the RED LED Characteristic
  // This toggles the red led
  pLedAspect = pService->createCharacteristic(
                      LED_ASPECT_UUID,
                      BLECharacteristic::PROPERTY_WRITE_NR
                    );

  // **3** It is very advantageous to keep the BLE Layer from waiting
  // for a response by using PROPERTY_WRITE_NR instead of
  // PROPERTY_WRITE.  With no client or server side queues or retries
  // the error rate was reduced in half when WRITE_NR was used as
  // compared to WRITE (with response).  We learned that the BLE Layer
  // expects a response when WRITE and INDICATE are used and that this
  // takes time and resources. When using the Espressif IDF one can
  // make full use of the response but when using the Arduino BLE
  // library, the response callbacks are not available.  Functionally,
  // this makes INDICATE no different than NOTIFY except that INDICATE
  // adds overhead.  For WRITE versus WRITE_NR we inferred that when
  // the WebBLE client does not get the reply in time, a Network error
  // is thrown as observed by the Javascript debugger. Therefore, use
  // WRITE_NR because there is less overhead which makes it faster. In
  // our experiments, no messages were silently lost when WRITE_NR was
  // used.
  
  // Create the Servo Characteristic
  // The client writes to this to move the servos
  
  pServoAspect = pService->createCharacteristic(
		      SERVO_ASPECT_UUID,
                      BLECharacteristic::PROPERTY_WRITE_NR
                    );

  // **3** For the Control Characteristic, use WRITE_NR.
  // While WRITE might seem like a way to allow us to retry all failures
  // on the client (javascript) side -- it screws up subsequent writes
  // to the ServoAspect..
  
  // Create the control Characteristic -- It turned out to be essential
  // to have a characteristic-write_nr so that the client could stop the
  // servos immediately. When stop was within the stream of move
  // commands, the servos would keep going until the backlog of move
  // commands completed.  By having this control aspect we can stop
  // the motors immediately.  Other uses include start and speed
  // setting, see handler for details.
  
  pControlAspect = pService->createCharacteristic(
		      CONTROL_ASPECT_UUID,
		      BLECharacteristic::PROPERTY_WRITE_NR);

  // **1** Client Characteristic Configuration Descriptors (CCCD),
  // 2901 & 2902.

  // Create a BLE Descriptor for the status aspect because
  // it uses NOTIFY
  pStatusAspect->addDescriptor(new BLE2902());

  // these are definitely not needed here (only needed for NOTIFY & INDICATE)
  //pLedAspect->addDescriptor(new BLE2902());
  //pServoAspect->addDescriptor(new BLE2902());
  //pControlAspect->addDescriptor(new BLE2902());

  // **1** Optionally, be friendly - to see this, use the LightBlue
  // app.  It doesn't show up in nRF Connect & our Web BLE App does
  // not use these.  In other words, delete all of these lines and
  // everything still works.
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
  
  // **1** What we learned here: we need a CCCD for each characteristic
  // with which we want to perform notify or indicate operations.  The
  // 2902 CCCD is needed because notify & indicate are actually
  // enabled by the client.  The client sets a value in the 2902 CCCD
  // that tells the BLE layer that it's okay for the server to send
  // these. Not that we need to know for this application, a value of
  // 0x0001 set by the client enables notify, 0x0002 enables indicate
  // and 0x0003 enables both.

  // **1** We also learned that we can set human readable strings to
  // go with the characteristics we define by including the 2901
  // CCCD. Our investigation revealed that WebBLE does not support a
  // mechanism for reading these.  We did verify we set the properly
  // by using the LightBlue app.
  
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

  // **3**
  // Create the task & queue so that when the client asks to move the
  // motors, the BLE stack can return right away. This task will then
  // move the motors in the background.

  // create the queue
  servoCommandQueue = xQueueCreate(100, sizeof(ServoCommand));
  if ( servoCommandQueue == NULL ) {
    Serial.println("Failed to create servo queue!!!");
    return;
  }

  // create & start the servo task
  xTaskCreate(servoTask, "ServoTask", 2048, NULL, 1, NULL);
  
  // *** START BROADCASTING OUR SERVICE ***
  
  // get a pointer to the BLE advertising instance
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();

  // our custom service UUID
  pAdvertising->addServiceUUID(SERVICE_UUID);

  // don't respond to scans (saves battery)
  pAdvertising->setScanResponse(false);

  // **2** We tried using different intervals here to see if that had
  // any impact on our app.  We arranged for the javascript to send
  // servo commands via WRITE with no queue or retry.  The we compiled
  // this code with each interval setting. We also created a BURST
  // button on the client that sent move commands at a fixed interval
  // using javascript. The shortest usable burst interval was around
  // 20ms. We recorded the number of errors during the burst and
  // verified the timing on the ESP32 side.  We repeated bursts for
  // each minPreferred setting 3 times.  While there was some
  // variation, there were no differences that could be associated
  // with any particular value of minPreferred.  We concluded that
  // WebBLE ignores this preference which is not surprising and
  // allowed by the BLE protocol.
  
  // 0x0 leaves the connection interval up to the client
  pAdvertising->setMinPreferred(gIntervalSetting);

  // make it happen
  BLEDevice::startAdvertising();

  sp1("Waiting a client connection...");

  // all done with setup, turn off our LEDs
  digitalWrite(redPin, LOW);
  digitalWrite(grnPin, LOW);

  // optionally do a little dance at startup
  servoTest(&servo1);
}

void startBLE() {
  // it's possible to do all the BLE setup here and then we
  // could deinit() ble to disconnect all clients
  // but, why not just restart??? Is deinit() ever necessary???
}

// these global variables support our loop

int gLastStatus = READY; // set to ready so that we write immediately

// only print the number of moves when it changes
unsigned long gLastCnt = 0;

void loop() {
  char msg[21];
  char moves[10];
  
  // BLE notify is used to tell the client we have a status update.
  // Status updates are sent when the status changes.
  // We only do updates when we're connected of course.
  // nb: Notify or Indicate -- functionally the same, Indicate is more
  // overhead.
  
  if ( deviceConnected && gLastStatus != gServoStatus ) {
    gLastStatus = gServoStatus;
    if ( gLastStatus == MOVING  ) strcpy(msg, "Servos Moving");
    if ( gLastStatus == STOPPED ) {
      // include the number of moves performed when stopped
      strcpy(msg, "Stopped ");
      itoa(gMoveCount, moves, 10);
      strcat(msg, moves);
      if ( gMoveCount != gLastCnt ) {
	sp1x(msg);
	gLastCnt = gMoveCount;
      }
    }
    if ( gLastStatus == READY   ) strcpy(msg, "Servos Ready");

    // send our status update
    pStatusAspect->setValue(msg);
    pStatusAspect->notify();

  }
  
  // Our onDisconnect callback has been triggered.
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
  double x = 1.111;
  while (millis() - start < ms) { x = x * 2.0; }
  return;
}

