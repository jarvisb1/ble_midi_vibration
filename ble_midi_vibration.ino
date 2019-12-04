#include <Arduino.h>

#include <Wire.h>
#include <SPI.h>

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BLEMIDI.h"

#include "BluefruitConfig.h"
#include "notes.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

/****************************************
 * CONFIGURATION SETTINGS
 ***************************************/
#define SET_BLE_NAME (0)
String ble_name = "Your name here"; // The name that gets advertised by the BLE chip

#define NUM_MUX_PORTS (16)
#define HISTORY_DEPTH (10)  // The number of readings to average together before deciding which note to play. Lower numbers make the notes more responsive, but also more erratic. Higher numbers are more smooth, but slower to react to motion.
#define READ_SLEEP_MS (100) // This limits how frequently you read from the sensor (milliseconds)
#define SEND_SLEEP_MS (100) // This limits how frequently you send output (milliseconds)
#define NOTE_ON_DURATION_MS (50) // How long notes are played for

#define MUX_SIG (A1)
#define MUX_ADDR_0 (11)
#define MUX_ADDR_1 (10)
#define MUX_ADDR_2 (9)
#define MUX_ADDR_3 (6)

/* END OF CONFIGURATION - DON'T CHANGE STUFF BELOW HERE */

//BLE SETTINGS
#define BLE_FACTORY_RESET_ENABLE  0 //If 1, the BLE device will factory reset and will revert any custom device name you've programmed into it. Avoid this unless something's really gone wrong in the BLE chip.
#define MINIMUM_FIRMWARE_VERSION  "0.7.0"

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_BLEMIDI midi(ble);

bool isConnected = false;

void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void connected(void)
{
  isConnected = true;
  Serial.println(F("BLE connected."));
  delay(1000);
}

void disconnected(void)
{
  Serial.println("BLE disconnected");
  isConnected = false;
}

void BleMidiRX(uint16_t timestamp, uint8_t status, uint8_t byte1, uint8_t byte2)
{
  Serial.print("[MIDI ");
  Serial.print(timestamp);
  Serial.print(" ] ");

  Serial.print(status, HEX); Serial.print(" ");
  Serial.print(byte1 , HEX); Serial.print(" ");
  Serial.print(byte2 , HEX); Serial.print(" ");

  Serial.println();
}

int latest[NUM_MUX_PORTS];
int history[NUM_MUX_PORTS][HISTORY_DEPTH];
int history_index = 0;
unsigned long total_measurements = 0;

//A running average of the last HISTORY_DEPTH values
float avg[NUM_MUX_PORTS];
bool movement[NUM_MUX_PORTS];

void setup_ble_midi()
{
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("Bluefruit found.") );

  if ( BLE_FACTORY_RESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  //ble.sendCommandCheckOK(F("AT+uartflow=off"));
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  //Set the BLE name if SET_BLE_NAME is 1
  if (SET_BLE_NAME)
  {
    String ble_name_command = String("AT+GAPDEVNAME=" + ble_name);
    
    // Send command
    ble.println(ble_name_command.c_str());

    // Check response status
    ble.waitForOK();
  }

  /* Set BLE callbacks */
  ble.setConnectCallback(connected);
  ble.setDisconnectCallback(disconnected);

  // Set MIDI RX callback
  midi.setRxCallback(BleMidiRX);

  Serial.println(F("Enabling MIDI."));
  if ( ! midi.begin(true) )
  {
    error(F("Could not enable MIDI"));
  }

  ble.verbose(false);
  Serial.println(F("Waiting for BLE connection..."));
}

void setup() 
{
  pinMode(MUX_SIG, INPUT);
  
  delay(500);
  Serial.begin(115200);
  delay(500);

  setup_ble_midi();
}

void noteOn(byte channel, byte pitch, byte velocity) {
  midi.send(0x90 | channel, pitch, velocity);
}

void noteOff(byte channel, byte pitch, byte velocity) {
  midi.send(0x80 | channel, pitch, velocity);
}

//val should be a 14-bit value
void pitchBend(uint16_t val) {
  byte lsb = val & 0x007F;
  byte msb = (val & 0x3F10) >> 7;
  midi.send(0xE0, lsb, msb);
}

//Maintain the BLE connection. Call this each loop.
void ble_tick()
{
  ble.update(500);
}

void print_debug()
{
  Serial.println("\nLatest Values:");
  for (int i = 0; i < NUM_MUX_PORTS; i++)
  {
    Serial.print(latest[i]); Serial.print(" ");
  }

  Serial.println("\nAvg Values:");
  for (int i = 0; i < NUM_MUX_PORTS; i++)
  {
    Serial.print(avg[i]); Serial.print(" ");
  }

  Serial.println("\nMovement?:");
  for (int i = 0; i < NUM_MUX_PORTS; i++)
  {
    Serial.print(movement[i] ? "1" : "0"); Serial.print(" ");
  }
  Serial.println("");
}

void read_vibrations()
{
  for (int i = 0; i < NUM_MUX_PORTS; i+= 2)
  {
    digitalWrite(MUX_ADDR_0, bitRead(i, 0));
    digitalWrite(MUX_ADDR_1, bitRead(i, 1));
    digitalWrite(MUX_ADDR_2, bitRead(i, 2));
    digitalWrite(MUX_ADDR_3, bitRead(i, 3));
    delay(10);

    latest[i] = analogRead(MUX_SIG);
    history[i][history_index] = latest[i];
    delay(10);
  }
  history_index = (history_index + 1) % HISTORY_DEPTH;

  for (int i = 0; i < NUM_MUX_PORTS; i++)
  {
    for (int j = 0; j < HISTORY_DEPTH; j++)
      avg[i] += history[i][j];
    avg[i] /= HISTORY_DEPTH;
  }
  total_measurements++;
}

//Logic for if there is movement: if the most recent measurement is 15% above or below the running average.
//That may be a naive way to determine it, but let's see how it goes.
void calc_is_movement()
{
  for (int i = 0; i < NUM_MUX_PORTS; i++)
  {
    movement[i] = (latest[i] > (avg[i]*2)) || (latest[i] < (avg[i]*0.3));
  }
}

unsigned long last_read_time = 0;
unsigned long last_send_time = 0;
void loop()
{
  ble_tick();
  
  unsigned long curr_time = millis();
  if ( (curr_time - last_read_time) > READ_SLEEP_MS) 
  {
    read_vibrations();
    last_read_time = curr_time;
  }

  if ((total_measurements > HISTORY_DEPTH) && ((curr_time - last_send_time) > SEND_SLEEP_MS))
  {
    calc_is_movement();
    print_debug();

    for (int i = 0; i < NUM_MUX_PORTS; i++)
    {
      if (movement[i])
      {
        noteOn(0, note_pitches[i], 100);
      }
    }
    
    delay(NOTE_ON_DURATION_MS);

    for (int i = 0; i < NUM_MUX_PORTS; i++)
    {
      if (movement[i])
      {
        noteOff(0, note_pitches[i], 100);
      }
    }

    last_send_time = curr_time;
  }
}
