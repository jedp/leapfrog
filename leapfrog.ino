#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

#define FACTORYRESET_ENABLE         1

// LeapFrog matrix: Digital inputs for rows.
#define ROW0 6
#define ROW1 5
#define ROW2 3
#define ROW3 2

#define COL0 A1
#define COL1 A2
#define COL2 A3
#define COL3 A4
#define COL4 A5

#define BUTTON_PRESS_LED 10

// COMMON SETTINGS
// ----------------------------------------------------------------------------------------------
// These settings are used in both SW UART, HW UART and SPI mode
// ----------------------------------------------------------------------------------------------
#define BUFSIZE                        160   // Size of the read buffer for incoming data
#define VERBOSE_MODE                   true  // If set to 'true' enables debug output

// SHARED SPI SETTINGS
// ----------------------------------------------------------------------------------------------
// The following macros declare the pins to use for HW and SW SPI communication.
// SCK, MISO and MOSI should be connected to the HW SPI pins on the Uno when
// using HW SPI.  This should be used with nRF51822 based Bluefruit LE modules
// that use SPI (Bluefruit LE SPI Friend).
// ----------------------------------------------------------------------------------------------
#define BLUEFRUIT_SPI_CS               8
#define BLUEFRUIT_SPI_IRQ              7
#define BLUEFRUIT_SPI_RST              4    // Optional but recommended, set to -1 if unused

// SOFTWARE SPI SETTINGS
// ----------------------------------------------------------------------------------------------
// The following macros declare the pins to use for SW SPI communication.
// This should be used with nRF51822 based Bluefruit LE modules that use SPI
// (Bluefruit LE SPI Friend).
// ----------------------------------------------------------------------------------------------
#define BLUEFRUIT_SPI_SCK              13
#define BLUEFRUIT_SPI_MISO             12
#define BLUEFRUIT_SPI_MOSI             11

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

int32_t hrmServiceId;
int32_t hrmMeasureCharId;
int32_t hrmLocationCharId;

void setup() {
  pinMode(ROW0, INPUT);
  pinMode(ROW1, INPUT);
  pinMode(ROW2, INPUT);
  pinMode(ROW3, INPUT);

  pinMode(COL0, INPUT);
  pinMode(COL1, INPUT);
  pinMode(COL2, INPUT);
  pinMode(COL3, INPUT);
  pinMode(COL4, INPUT);

  pinMode(BUTTON_PRESS_LED, OUTPUT);
  digitalWrite(BUTTON_PRESS_LED, LOW);

  boolean success;
    
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit AT Command Example"));
  Serial.println(F("-------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if (FACTORYRESET_ENABLE) {
    Serial.println(F("Performing a factory reset: "));
    if (!ble.factoryReset()){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Setting device name to 'LeapFrog': "));

  if (!ble.sendCommandCheckOK(F("AT+GAPDEVNAME=Leapfrog")) ) {
    error(F("Could not set device name?"));
  }



  /* Reset the device for the new service setting changes to take effect */
  Serial.print(F("Performing a SW reset (service changes require a reset): "));
  ble.reset();

}

void dontloop() {
  // Display command prompt
  Serial.print(F("AT > "));

  // Check for user input and echo it back if anything was found
  char command[BUFSIZE+1];
  getUserInput(command, BUFSIZE);

  // Send command
  ble.println(command);

  // Check response status
  ble.waitForOK();
}

uint32_t right_now = micros();
uint32_t last_read_row[4] = { right_now, right_now, right_now, right_now };
uint32_t this_read_row[4] = { right_now, right_now, right_now, right_now };
uint32_t last_gap_row[4] = { 32000, 32000, 32000, 32000 };
uint32_t this_gap_row[4] = { 0, 0, 0, 0 };
uint32_t rows[4] = { ROW0, ROW1, ROW2, ROW3 };
uint32_t reading_row = -1;

uint32_t debounce_micros = 5000;

uint32_t last_read_row3 = 0;
uint32_t this_read_row3 = right_now; 
uint32_t last_gap_row3 = 32000;
uint32_t this_gap_row3 = 32000;
uint32_t reading_row3 = 0;

uint32_t last_read_row2 = 0;
uint32_t this_read_row2 = right_now; 
uint32_t last_gap_row2 = 32000;
uint32_t this_gap_row2 = 32000;
uint32_t reading_row2 = 0;

const uint32_t led_timer_max = 10000;
uint32_t led_timer = 0;

void loop() {

  right_now = micros();

  // No loops or arrays. Slight gain by unrolling everything.
  if ((right_now > last_read_row3 + debounce_micros) && (digitalRead(ROW3) == HIGH)) {
    last_read_row3 = this_read_row3;
    this_read_row3 = right_now;
    last_gap_row3 = this_gap_row3;
    this_gap_row3 = this_read_row3 - last_read_row3;
    if (this_gap_row3 > 1000 && this_gap_row3 < 9000 && last_gap_row3 > 28000) {
      reading_row3 = 1;
    }
  }

  // No loops or arrays. Slight gain by unrolling everything.
  if ((right_now > last_read_row2 + debounce_micros) && (digitalRead(ROW2) == HIGH)) {
    last_read_row2 = this_read_row2;
    this_read_row2 = right_now;
    last_gap_row2 = this_gap_row2;
    this_gap_row2 = this_read_row2 - last_read_row2;
    if (this_gap_row2 > 1000 && this_gap_row2 < 9000 && last_gap_row2 > 28000) {
      reading_row2 = 1;
    }
  }

    if (reading_row3) {
      if (digitalRead(COL1) == HIGH) {
        ble.println("AT+BLEUARTTX=White Milk");
        reading_row3 = 0;
      } else if (digitalRead(COL2) == HIGH) {
        ble.println("AT+BLEUARTTX=Orange Carrots");
        reading_row3 = 0;
      } else if (digitalRead(COL3) == HIGH) {
        ble.println("AT+BLEUARTTX=Red Strawberries");
        reading_row3 = 0;
      } else if (digitalRead(COL4) == HIGH) {
        ble.println("AT+BLEUARTTX=Brown Bread");
        reading_row3 = 0;
      }
    }

    if (reading_row2) {
      if (digitalRead(COL0) == HIGH) {
        ble.println("AT+BLEUARTTX=Charge");
        reading_row2 = 0;
      } if (digitalRead(COL1) == HIGH) {
        ble.println("AT+BLEUARTTX=Green Peas");
        reading_row2 = 0;
      } else if (digitalRead(COL2) == HIGH) {
        ble.println("AT+BLEUARTTX=Purple Grapes");
        reading_row2 = 0;
      } else if (digitalRead(COL3) == HIGH) {
        ble.println("AT+BLEUARTTX=Yellow Cheese");
        reading_row2 = 0;
      } else if (digitalRead(COL4) == HIGH) {
        ble.println("AT+BLEUARTTX=White Eggs");
        reading_row2 = 0;
      }
    }
}

void getUserInput(char buffer[], uint8_t maxSize)
{
  memset(buffer, 0, maxSize);
  while( Serial.available() == 0 ) {
    delay(1);
  }

  uint8_t count=0;

  do
  {
    count += Serial.readBytes(buffer+count, maxSize);
    delay(2);
  } while( (count < maxSize) && !(Serial.available() == 0) );
}

void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}
