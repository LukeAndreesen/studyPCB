#include <Wire.h>
#include "VCNL3040.h"
#include "si7210.h"  
/*
SOURCES:
  [1] https://www.youtube.com/watch?v=QI1IJLB42G8 - used for all shift register logic!
  [2] https://chatgpt.com/share/68372485-d47c-8005-8657-77b860ad0506 - used chatgpt to help me learn about receiving/parsing serialusb 
  [3] https://github.com/FARLY7/si7210-driver - si7210 driver library
*/

VCNL3040 proximitySensor;

// timer tracking ()
bool TIMER_STARTED = false;


// SI7210 device structure
si7210_dev_t hallSensor;
bool si7210_working = false;  // Track if SI7210 is working

const int histSize = 10;
const float MAG_THRESH = 0.40;
const int PROX_THRESH = 3; // 7 of the last 10 readings should be nonzero
const int POT = A0;
const int CODE_LED = 40;
const int HALL_LED = 41;  // Add a second LED for hall sensor indication
const int BUZZER = 38;
const int DS = 6;
const int SHCP = 4;
const int STCP = 3;
const int BT1 = 1;
const int BT2 = 0;

uint16_t prevProx[histSize] = {0};
int proxIdx = 0;
bool userPresent = true;
bool phoneDetected = true;

// Source: [1]
// decimal representation of 7 segment display values
int digits[10] = {126, 48, 109, 121, 51, 91, 95, 112, 127, 123}; // ideally used for display ... didn't work :(

void displayTest(){
  // SOURCE: [1]
  // Ideally ths would iterate through all numbers on each screen ... but doesn't work
  for (int k = 0; k < 10; k++) {       // Left digit
    for (int j = 0; j < 10; j++) {     // middle digit
      for (int i = 0; i < 10; i++) {   // Right digit
        digitalWrite(STCP, LOW);
        shiftOut(DS, SHCP, LSBFIRST, digits[i]);  // Right
        shiftOut(DS, SHCP, LSBFIRST, digits[j]);  // Middle
        shiftOut(DS, SHCP, LSBFIRST, digits[k]);  // Lef
        digitalWrite(STCP, HIGH);
        delay(200);
      }
    }
  }
}

int getPotVal(int pot_reading) {
  // read potentiometer
  float range_val = 1023.0 / 10; // potentonmeter goes up to 1023, want one of 10 values
  int val = pot_reading / 102.4;  
  if (val > 9){
    val = 9; // just in case it somehow exceeds 9
  }
  return val;
}

void checkSerialCommands() {
  // Listens for serial commands from our application to start/stop timer
  // Source: [2]
  if (SerialUSB.available()) {
    String cmd = SerialUSB.readStringUntil('\n');
    cmd.trim();

    if (cmd == "TIMER_START") {
      SerialUSB.println("Timer started!");
      // Single beep - timer started
      tone(BUZZER, 2000);   
      delay(200);
      noTone(BUZZER);
      TIMER_STARTED = true;    
    } 
    else if (cmd == "TIMER_END") {
      // Double beep - timer done
      SerialUSB.println("Timer ended!");
      TIMER_STARTED = false; 
      tone(BUZZER, 1500);   
      delay(400);
      noTone(BUZZER); 
      delay(300);
      tone(BUZZER, 1500);   
      delay(400);
      noTone(BUZZER); 
    }
  }
}


// I2C communication functions for SI7210
si7210_status_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
  // Source: [3] - taken straight from examples
  Wire.beginTransmission(dev_id >> 1);  // converts to copmatible adress form (7-bit)
  Wire.write(reg_addr);
  if (Wire.endTransmission() != 0) {
    return SI7210_E_IO;
  }
  
  Wire.requestFrom((uint8_t)(dev_id >> 1), len);
  for (uint16_t i = 0; i < len; i++) {
    if (Wire.available()) {
      data[i] = Wire.read();
    } else {
      return SI7210_E_IO;
    }
  }
  return SI7210_OK;
}

si7210_status_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
  // Source: [3] - taken straight from examples
  Wire.beginTransmission(dev_id >> 1);  // converts to copmatible adress form (7-bit)
  Wire.write(reg_addr);
  for (uint16_t i = 0; i < len; i++) {
    Wire.write(data[i]);
  }
  if (Wire.endTransmission() != 0) {
    return SI7210_E_IO;
  }
  return SI7210_OK;
}

void delay_ms(uint32_t period) {
  // Used for si7210 setup (via examples [3])
  delay(period);
}

void setup_proximity()
{
  // Initialize VCNL3040 proximity sensor 
  if (!proximitySensor.begin()) {
    SerialUSB.println("VCNL3040 not found");
    while (1);
  }
  SerialUSB.println("VCNL3040 initialized.");
  proximitySensor.startReading(); // now let's start reading proximty data
}

void setup_hall()
{
  // Initialize SI7210 hall effect sensor - via [3]
  hallSensor.dev_id = 0x33 << 1;  // Convert to 8-bit address for library
  // reference functions setup as per [3]
  hallSensor.read = i2c_read;
  hallSensor.write = i2c_write;
  hallSensor.delay_ms = delay_ms;
  hallSensor.threshold_callback = NULL; // not needed
  
  // Set sensor settings - as per [3]
  hallSensor.settings.range = SI7210_200mT;  // 200mT range
  hallSensor.settings.compensation = SI7210_COMPENSATION_NONE;
  hallSensor.settings.output_pin = SI7210_OUTPUT_PIN_LOW;

  // Initialize the SI7210 sensor [3]
  SerialUSB.println("Initializing SI7210 hall effect sensor");
  si7210_status_t result = si7210_init(&hallSensor);
  SerialUSB.print("Initialization result: ");
  SerialUSB.println(result);
    
  // build in redundancy - if it's not working, or detached, we should continue
  if (result != SI7210_OK) {
    SerialUSB.print("SI7210 initialization failed. Error code:");
    SerialUSB.println(result);
    SerialUSB.println("We will proceed without the hall effect sensor");
    si7210_working = false;
  } 
  else {
    SerialUSB.println("SI7210 initialized successfully!");
    si7210_working = true;
  }
}

void setup() {
  // LED's for debuggin
  pinMode(CODE_LED, OUTPUT);
  pinMode(HALL_LED, OUTPUT); 
  // pins for shift registers for display
  pinMode(DS, OUTPUT);
  pinMode(SHCP, OUTPUT);
  pinMode(STCP, OUTPUT);
  // button pins .. didn't end up using though
  pinMode(BT1, INPUT);
  pinMode(BT2, INPUT);
  
  SerialUSB.begin(9600);
  while (!SerialUSB); // let serial usb setup first

  // Initialize I2C
  Wire.begin();

  // setup sensors
  setup_proximity();
  setup_hall();

  SerialUSB.println("Setup complete. Starting main loop...");
  delay(5000); // a delay to get set up
}

void loop() {
  checkSerialCommands(); // check for timer start/stop via web app
  // Read proximity sensor
  uint16_t proximity = proximitySensor.readPSData();
  
  // Read potentiometer
  int pot_reading = analogRead(POT);
  int pot_val = getPotVal(pot_reading);
  SerialUSB.print("Dial Value: ");
  SerialUSB.print(pot_val);
  SerialUSB.print(" | ");


  
  // Only try to read hall sensor if it's working - suggested in [3]
  float magneticField = 0.0;
  si7210_status_t result = SI7210_E_DEV_NOT_FOUND;
  
  // result will be SI7210_OK if reading works and saved to magneticField, else something went wrong
  if (si7210_working) {
    result = si7210_get_field_strength(&hallSensor, &magneticField);
  }
  
  if (result != SI7210_OK) {
    SerialUSB.println("N/A (SI7210 not available)");
  
  }
  // any singnificant change in magnetic field (+ or -) should be detected (phone is next to sensor)
  if (abs(magneticField) < MAG_THRESH)
  {
    SerialUSB.print("Phone: False |");
    phoneDetected = false;
  }
  else
  {
     SerialUSB.print("Phone: True | ");
     phoneDetected = true;
  }

  // If we have started timer and we move away from desk or move phone ...  BUZZ!
  if (TIMER_STARTED)
  {
    (phoneDetected & (proximity >= 5)) ? noTone(BUZZER) : tone(BUZZER, 4000);
  }
  
  // unused logic -- ideally we'd make the proxmity software a bit more robust to account for sensor sensitivity
  prevProx[proxIdx] = proximity;
  proxIdx = (proxIdx + 1) % 10; // we want to look at previous 10 readings

  // Count how many of the last 10 readings are zero
  int num_zeros = 0;
  for (int i = 0; i < histSize; i++) {
    if ((prevProx[i] == 0) or (prevProx[i] == 1))  {
      num_zeros++;
    }
  }

  // we have moved away from desk 
  if (proximity < 5){
    userPresent = false;
    SerialUSB.println(" At desk: False");
  }
  else
  {
    userPresent = true;
    SerialUSB.println(" At desk: True");
  }


  delay(100); // give a slight timing gap
}