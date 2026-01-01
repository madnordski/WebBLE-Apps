/*** TrackVoltageCar

    This voltage reader is mounted in a train car or engine and used
    to identify track sections that are either dirty or have faulty
    wiring.  It was designed to work on Lionel Convensional Control
    A/C layouts (variable voltage A/C transformer for controlling
    trains).  The circuit and other details can be found on my GitHub
    page, https://github.com/madnordski?tab=repositories.

    This code has only ever been complied and run on the XIAO ESP32-C3
    but it should run on any ESP32 with BLE. Minor modifications may
    be necessary (NB: the C3 uses D9 for pin 9).  Be sure you BLE
    library is up to date (indicated by return type errors).  It
    should work fine on other MCUs as well with modification of
    course. NB: this does not work with the NIMBLE for BLE.

    Analog A/C voltage accuracy is achieved by use of the ADS1115 ADC
    and calibration.  The calibration results are hard coded below.
    To perform calibration turn debugging on, set SCALE_FACTOR below
    to 1.0, compile and run.  Set your transformer to about 10 volts
    and connect your DMM to the track to measure the volts.  Then
    change the scale factor to <measured volts> / <reported volts> and
    recompile.  You'll want to turn debugging off (see #debug below)
    before deployment for performance reasons.
    
    It is intended to be used with a WebBLE interface. To use it, pull
    the car or dummy engine (avoid putting in with a motor) behind an
    engine.  Stop, acquire readings and save a reference.  Move to a
    new location. The app will show you the difference in voltage.
    When you move and stop, do not touch the transformer throttle.
    The APP can be found and run from here:
    https://madnordski.github.io/WebBLE-Apps/TrackVoltageCar.html (it
    must be run using https (secure) or localhost).
    When a BLE connection is closed, whether by the user, browser
    exit or page reload -- the ESP is restarted.  This resets the BLE
    layer for the next connnection.
      
    NB: the word ASPECT is used to represent CHARACTERISTICS in
    variable names only because characteristics was cumbersome to
    type. It has no special meaning beyond representing a
    characteristics. Just in case you didn't know, NB is latin for
    <i>nota bene</i> which translates to note well.

    @author J. King
    @date 27 Dec 2025
    @copy Limited rights.
    @brief Track Voltage Car reads the track voltage anywhere on your layout.

    Copyright (c) 2025, 2026, Joseph J. King, PhD

    Permission is hereby granted, free of charge, for educational
    purposes. Use of the this software for commercial purposes is
    strictly prohibited.  The above copyright notice and this
    permission notice shall be included in all copies or substantial
    portions of the Software.

    $Revision: 1.7 $
    $Date: 2026/01/01 16:13:11 $
*/

#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// BLE standard libraries, do not include NIMBLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLE2901.h>

// uncomment DEBUG for calibration or debugging

//#define DEBUG
// (timings are wrong when debug is enabled)
#ifdef DEBUG
#define sp1(x)   Serial.println(x)
#define sp2(x,y) Serial.print(x);Serial.println(y)
#define spxy(x,y) Serial.print(x); Serial.print(" "); Serial.println(y);
#else
#define sp1(x)
#define sp2(x,y)
#define spxy(x,y)
#endif
#define sp1x(x)   Serial.println(x)
#define sp2x(x,y) Serial.print(x);Serial.println(y)
#define spxyx(x,y) Serial.print(x); Serial.print(" "); Serial.println(y);

// BLE Defines

#define SERVICE_UUID "1357632d-42e8-44f5-a5c8-6ff6103bf0ea"
#define ASPECT_UUID  "c96ff38c-f327-4dbb-a1e2-41c4219c3c59"


// we used the ADC from Adafruit to get more accurate measurements
Adafruit_ADS1115 ads;

// prototypes
void GetRMSVolts(void *param);

// ---- Configuration ----
static const float ADC_FULL_SCALE = 4.096;     // volts
static const int   ADC_COUNTS     = 32768;
static const float LSB_VOLTS      = ADC_FULL_SCALE / ADC_COUNTS;

static const int SAMPLE_RATE_SPS  = 860;
static const int SAMPLE_WINDOW_MS = 1000;      // 1 second

// *********** CALIBRATION HERE ********************
// sf = known voltage / reported vRMS
// experimented with offset volts and it was to determined to be unhelpful
// NB: see notes above on calibration

static const float SCALE_FACTOR = 9.19 / 0.683;  // calibration
static const float ZERO_VOLTS = 0;               // offset, not helpful

// we use a task that continually updates this value
//  and just read when we want it

volatile float Volts = 0.0;

BLECharacteristic *pNotifyAspect;

#define WHITE_LED D10
#define RED_LED   D9

unsigned long zeroTime;
bool deviceConnected = false;
bool ledToggle = false;

/** ServerCallbacks Method Override

    The BLE subsystem will call these methods when a connection to a BLE
    client is established or when it goes away.  Here we are
    overriding the class methods onConnect() and onDisconnect().
 */

class ServerCallbacks: public BLEServerCallbacks {
  /** onConnect
      Set the connection flag and turn the white led on.
  */
  
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    digitalWrite(WHITE_LED, HIGH);
  }

  void onDisconnect(BLEServer* pServer) {
    /** onDisconnect
	Restart the ESP32 so that we are ready for next connection.
    */
    digitalWrite(RED_LED, LOW);
    digitalWrite(WHITE_LED, LOW);
    ESP.restart();
  }
};

// SETUP

void setup() {
  // for debugging only, see #debug above
  Serial.begin(115200);

  // we use I2C to communicate with the ADS1115
  Wire.begin();

  // RED blinks when not connected, RED is solid when reading >10 volts
  // WHITE comes on when connected
  pinMode(RED_LED, OUTPUT);
  pinMode(WHITE_LED, OUTPUT);

  // start the ADC, ADS1115 on I2C
  if (!ads.begin()) {
    Serial.println("ADS1115 not found!");
    while (1);
  }

  digitalWrite(WHITE_LED, LOW);
  digitalWrite(RED_LED, HIGH);

  // default gain and highest data rate
  ads.setGain(GAIN_ONE);      // ±4.096 V
  ads.setDataRate(RATE_ADS1115_860SPS);

  // x,y -> time,volts
  zeroTime = millis();
  xTaskCreate(GetRMSVolts, "GetRMSVolts", 2048, NULL, 1, NULL);

  // NOW LETS GET BLE GOING

  sp1("Start BLE!");

  BLEDevice::init("TrackVoltageCar");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pNotifyAspect
    = pService->createCharacteristic(ASPECT_UUID,
				     BLECharacteristic::PROPERTY_NOTIFY |
				     BLECharacteristic::PROPERTY_WRITE |
				     BLECharacteristic::PROPERTY_READ);

  pNotifyAspect->addDescriptor(new BLE2902());
  pNotifyAspect->setValue("--"); // nothing to report until we start reading
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  
  BLEDevice::startAdvertising();
  sp1("Ready.");
}

// LOOP

bool lastConnected = false;

void loop() {
  char valueStr[80];

  // a separate task keeps volts up to date
  float v = Volts;
  
  if ( deviceConnected ) {
    // report to our listener
    if ( ! lastConnected )
      sp1x("Connected.");
    digitalWrite(RED_LED, v > 10.0 ? HIGH : LOW);
    sprintf(valueStr, "%0.2f Volts", v);
    pNotifyAspect->setValue(valueStr);
    pNotifyAspect->notify();
    lastConnected = true;
  }
  else {
    // just toggle the red led to indicate we have no listeners
    ledToggle = ! ledToggle;
    digitalWrite(RED_LED, ledToggle ? HIGH : LOW);
    lastConnected = false;
  }
  vTaskDelay(1000);
}

// GET VOLTS TASK
// This is the main thread that reads and reports voltage.
// It's always doing it, even when not connected which makes debugging
// easy. We could probably save battery by idling?  

void GetRMSVolts(void *param) {

  while ( true ) {
    uint32_t start_ms = millis();

    double sum_sq = 0.0;
    uint32_t samples = 0;
    //unsigned long sum = 0;
  
    //digitalWrite(RED_LED, LOW);
  
    while (millis() - start_ms < SAMPLE_WINDOW_MS) {
      int16_t raw = ads.readADC_SingleEnded(0);
      //sum += raw;
      // Convert counts to volts
      float v = raw * LSB_VOLTS;
      
      // Half-wave rectified input: negative values should not occur,
      // but guard anyway.
      // Offset was set to zero when we determined that Lionel
      // transformers are leaky. It's not a math problem, it's
      // electrical reality.  It's left here as a reminder/option.
      v = v - ZERO_VOLTS;
      if (v < 0.0f) v = 0.0f;

      // sum squares for RMS calc.
      sum_sq += (double)v * (double)v;
      samples++;
    }
  
    if (samples > 0) {
      // vRMS is what we measure at ADC IN which is half wave
      // rectified after a voltage divider. We could do all the
      // threoretical math for estimating the missing half wave and
      // taking into account voltage drops from the divider and diode
      // or -- as we are doing here -- just calibrate which takes
      // these plus other variable into account (so long as we do it
      // carefully). One big variable is the contact of the wheels and
      // pickup to the track so for a careful calibration hook up the
      // transformer to the circuit inputs directly (was I did).
      
      float vRMS = sqrt(sum_sq / samples);
      float vCorrected = vRMS * SCALE_FACTOR;
      // new reading available, atomic operation so no semaphore needed
      Volts = vCorrected;
      sp2("vRMS: ", vRMS);
      sp2("Volts: ", Volts);
      sp2("Scale factor: ", SCALE_FACTOR);
    }
  }
}

