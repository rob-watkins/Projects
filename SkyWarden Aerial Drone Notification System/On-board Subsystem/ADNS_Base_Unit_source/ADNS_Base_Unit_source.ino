/* @file:         ADNS_Base_source.ino
 * @author:       Rony Calderon
 *                C.S.E. UNR Student 
 * @author:       Robert Watkins
 *                C.S.E. UNR Student
 * @author:       Bryan Kline
 *                C.S.E. UNR Student
 * @author:       Jia Li
 *                C.S.E. UNR Student
 *              
 * @date:		  April 05, 2018
 * 
 * @brief:        Implementation file for the ADNS Base Unit system that captures the pertinent On-board transmitted data
 *
 * @compiler:     Compiled using the Arduino IDE 1.8.5
 * 
 * @dev board:    Arduino Nano
 *
 * @notes:      Please read through the system notes & configurations to understand the ADNS Base Unit system development.
 */

/*********************************** ADNS Base Unit System Description, Features, and Configuration Notes *****************************
 *  The Aerial Drone Notification System is composed of two separate system that communicate via radio frequency (RF) wireless using
 *  the NRF24L01 transceiver modules. The On-board specifications and configurations are covered in the ADNS_Onboard_Prototype.ino
 *  source code file and this file contains the specifications and configurations for the Base Unit.
 *  
 *  The Base Unit for the ANDS system works by capturing the sensor data sent by the On-board system. This is performed using the NRF24L01
 *  transceiver modules and both transceiver modules must be configured with matching data rate and RF channel. The base unit uses only the
 *  NRF24L01 module at the moment. Therefore, only the Nano's SPI pins are used in the Base Unit's configuration. The Nano microcontroller
 *  is only to be used as an intermediary between the On-board system and the notification system at the moment. Below is the configuration
 *  for the Nano MCU and the NRF24L01 unit.
 * 
 *                  *********************************
 *                  ****    Arduino Nano MCU     ****
 *                   -------------------------------
 *                  [D0                          VIN]
 *                  [D1                          GND]
 *                  [RESET                     RESET]
 *                  [D2                           5V]
 *                  [D3                           A7]
 *                  [D4                           A6]
 *                  [D5                      D19(A5)]-- I2C Serial Clock (SCL) 
 *                  [D6                      D18(A4)]-- I2C Serial Data (SDA)
 *                  [D7                      D17(A3)]
 *                  [D8                      D16(A2)]
 * Chip Enable(CE)--[D9                      D15(A1)]
 * Slave Select   --[D10                     D14(A0)]
 *           MOSI --[D11                        3.3V]
 *           MISO --[D12                         D13]-- SPI Clock (SCK)
 *                   -------------------------------
 */
// NOTE: The chip enable (CE) pin can be any digital IO pin but the slave select (SS)
// pin must be the SS pin mapped to the board such as the following, where A(x) is the
// alternate pin (if available) and x is the pin number:
// NOTE: If A(x) is listed, then any digital IO pin may be used.
/*
 *                    ATmega      Teensy 3.2      Nano    
 *  SPI Function        Pin #         Pin #       Pin #     
 * ------------        -----        -------      --------
 * Chip Enable (CE)    8 A(x)         9 A(x)      8 A(x)   
 *     MISO           50             12 A(8)     12        
 *     M0SI           51             11 A(7)     11       
 * SPI Clock (SCK)    52             13 A(14)    13        
 * Slave Select (SS)  53             10          10         
 * 
 * --------------------------------- NRF24L01 Transceiver unit with antenna ---------------------------
 *  
 *                                |-------------------
 *                                |Vcc (3.3V)| Gnd   |
 *                                |-------------------
 *  /-----------------------------| SS (CSN) | CE    |
 * <|    Antenna                  |------------------|
    \-----------------------------| MOSI     | SCK   |
 *                                |------------------|
 *                                | IRQ      | MISO  |
 *                                |-------------------
 * IRQ pin is used to trigger an interrupt in an MCU                                
 */

 //
// Preprocessor Directives
// NOTE: RH_NRF24.h is the radioHead wireless library for microcontrollers
//
#include <RH_NRF24.h>
#include <SPI.h>
#include <String.h>

 //
 // Macro definitions
 //
#define KILL_SWITCH_BUTTON_PIN  2
#define LED_ACTIVITY_PIN        8
#define CHIP_ENABLE_PIN         9
#define SS_PIN                  10
#define RF_CHANNEL              115
#define RF_DATA_RATE_250kbps    DataRate250kbps
#define RF_DATA_RATE_1Mbps      DataRate1Mbps
#define RF_DATA_RATE_2Mbps      DataRate2Mbps
#define BAUD_RATE               115200


#define RECEIVER_DATA_RATE      RF_DATA_RATE_1Mbps

//
// Declare and initialize NRF24 transceiver
//
RH_NRF24 base_unit_transceiver(CHIP_ENABLE_PIN, SS_PIN);

// 
// Character array & string used to hold messages for RF transmission.
//
//uint8_t in_buffer[RH_NRF24_MAX_MESSAGE_LEN];
uint8_t in_buffer[7];
uint8_t buffer_length = sizeof(in_buffer);

//
// variable used to hold the readings received
//
unsigned long int reading = 0;

//
// bool used to poll kill switch button
//
bool kill_button_not_pressed = true;

void setup() 
{
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // SETUP SERIAL DATA BAUD RATE ////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.begin(BAUD_RATE);

  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // SETUP NRF24L01 BASE UNIT TRANSCEIVER ///////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Executes if base unit transceiver fails to be initialized.
  if (!base_unit_transceiver.init())
  {
    Serial.println("base_unit_transceiver initialization failed");
  }

  // Set up the RF channel, data rate, and transmit power for the base_unit_transceiver.
  // NOTE: Transmitter must have identical RF channel and data rate values
  base_unit_transceiver.setChannel(RF_CHANNEL);
  // Executes if setting channel fails.
  if (!base_unit_transceiver.setChannel(RF_CHANNEL))
  {
    Serial.println("base_unit_transceiver setChannel failed");
  }
  
  base_unit_transceiver.setRF(RH_NRF24::RECEIVER_DATA_RATE, RH_NRF24::TransmitPower0dBm);
  // Executes if RF settings failed to be initialized.
  if (!base_unit_transceiver.setRF(RH_NRF24::RECEIVER_DATA_RATE, RH_NRF24::TransmitPower0dBm))
  {
    Serial.println("base_unit_transceiver setRF failed");  
  }

  // Set the digital pin settings 
  pinMode(KILL_SWITCH_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_ACTIVITY_PIN,       OUTPUT);
  

}// End setup()


void loop() 
{
  // Check to see if there is any data available.
  if(base_unit_transceiver.available())
  {
    // Data is available so check to see if the buffer is not empty.
    if(base_unit_transceiver.recv(in_buffer, &buffer_length))
    {
      // Turn on activity LED and increment the reading for debugging purposes
      digitalWrite(LED_ACTIVITY_PIN, HIGH);
      reading++;

      // Print out reading and value
     // Serial.print("Reading: ");
      //Serial.print(reading);
      //Serial.print("            Received: ");
      Serial.println((char*) in_buffer);

      // Turn off activity LED
      digitalWrite(LED_ACTIVITY_PIN, LOW);
    }
    else
    {
      Serial.println("Received failed");
    } 
  }

  // Poll the kill switch button state
  kill_button_not_pressed = digitalRead(KILL_SWITCH_BUTTON_PIN);

  // Executes if the kill button has been pressed
  if(kill_button_not_pressed == false)
  {    

  
    // Send a kill command to the on-board system
    uint8_t kill_command_array[] = "KILL";

    base_unit_transceiver.send(kill_command_array, sizeof(kill_command_array));
    base_unit_transceiver.waitPacketSent();

    

    // Print a message to base unit system
    Serial.println("KILL COMMAND SENT");
    delay(500);
  }
}





