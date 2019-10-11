/* @file:         ADNS_Onboard_source.ino
 * @author:       Rony Calderon
 *                C.S.E. UNR Student 
 * @author:       Robert Watkins
 *                C.S.E. UNR Student
 * @author:       Bryan Kline
 *                C.S.E. UNR Student
 * @author:       Jia Li
 *                C.S.E. UNR Student
 *              
 *                   
 * @version:      v1.01 - Added two ToF functionality performing at 24Hz with independent addressing.
 * @version:      v1.02 - Added four ToF sensors performing at 24Hz with independent addressing.
 * @version:      v1.03 - Added nRF24L01 transceiver unit to transmit ToF sensor data.
 * @version:      v1.04 - Refined nRF24L01 transmission rate to be at 1Mbps and channel change to 115.
 * @version:      v1.05 - Sped up ToF sensors to perform at 56Hz. Transmission rate stayed as before.
 * @version:      v1.06 - On-board system now supports 8 ToF sensors running at 56Hz.
 * @version:      v1.07 - Added LED alert functionality
 * @version:      v1.08 - Added software reset exclusive for the Teensy 3.2
 * @version:      v1.09 - Added INA219 battery sensor & increased ToF sensors to 14 total
 * @version:      v1.10 - Optimized transmission rate and added extra debugging output messages
 * @v1.10 date:   April 19, 2018
 * 
 * @brief:        Implementation file for the VL53L0X flight-of-time (ToF) sensors with RF
 *                wireless transmission using the NRF24L01 transceiver module.
 *
 * @compiler:    Compiled using the Arduino IDE 1.8.5
 * 
 * @dev board:    Teensy 3.2
 *
 * @notes:      Please read through the system notes & configurations to understand the ADNS on-board system development.
 */

/******************************** ADNS System Description, Features, and Configuration Notes *****************************************
 *  The Aerial Drone Notification System is composed of two separate systems that communicate via radio frequency (RF) wireless using
 *  the NRF24L01 transceiver modules. The two systems considered are the on-board unit mounted to an aerial drone and a ground base
 *  unit that is used to capture the data from the on-board system and process the data accordingly. The system is designed to accomodate 
 *  up to 20+ VL53L0X Time of Flight (ToF) sensors using independent addressing through the I2C bus. The ADNS system currently uses 14 ToF 
 *  sensors and a INA219 voltage/current sensor to grab voltage levels from the aerial drone's battery pack.
 *  
 *  The sensors are sequentially polled and transmitted wirelessly from the on-board system to the ground base system in a stream of 
 *  data that is, unfortunately, unreliable (think UDP). To combat the unreliable datagrams, the ADNS team chose to send each data
 *  stream independently. This is done to reduce the amount of bad or corrupted datagrams as well as increase the throughput of the 
 *  wireless system. Please read the documentation at the website listed below for more information:
 *  
 *                          http://www.airspayce.com/mikem/arduino/RadioHead/classRH__NRF24.html
 *                          
 *  Features for the ADNS system include independent I2C addressing and high speed polling rates of the ToF sensors. Currently and
 *  collectively, the ToF sensors poll at about 63 Hz. The wireless data rate has been set to the lowest setting possible at 250 kbps
 *  to extend the possible ranges of the NRF24L01 transceivers and due to the bottleneck of the system which are the ToF sensors.
 *  Additionally, the on-board system has a software reset feature that will attempt to restart the system if a sensor has timed out.
 *  This presents a good robust and recovery method but also introduces the possibility of a single sensor triggering a constant reboot
 *  of the on-board system. The estimated reboot time is around 500 ms (half a second).
 *  
 *  Another feature implemented is the auto restart of the system in the event of system initialization failure. The ADNS system begins
 *  by trying to poll a set amount of readings for the system upon boot and will trigger a red LED alarm if even one sensor fails to
 *  initialize properly. The set amount of readings or polls can be set by the user and default is 5 readings. Additionally, if the 
 *  ADNS system successfully initializes, a green LED is turned on constant state to indicate successful initialization. Moreover, if a
 *  sensor is not seated properly, the user will see the triggered alarm and can re-seat the faulty sensor to re-initialize the system.
 *  
 *  WARNING: Please use the system files included with the ADNS including its source code as the source code for several libraries
 *           has been changed to implement some of the features.
 *           
 *  WARNING: Different breakout boards using the VL53L0X ToF sensor may be used interchangeably but the breakout boards must have the
 *           XSHUT pin. The XSHUT pin is used for independent addressing when interacting with the I2C bus. 
 * 
 ******************************** ADNS On-board Transmitter System Notes & Pin Configurations ****************************************
 *  Since the on-board system for the Aerial Drone Notification System (ADNS) utilizes a Teensy 3.2 microcontroller, we will define
 *  the setup of the on-board system specific to the Teensy 3.2 MCU. Other MCU's work as well but the Teensy 3.2 gave the ADNS team
 *  enough output digital pins to connect the large amount of sensors placed on the on-board system (20+ sensors will be used). Thus,
 *  the pin mapping may not be identical to other boards so please use this precaution to ensure proper connection to the various
 *  sensors utilized.
 *  
 *  NRF24L01 Transceiver module NOTE: The NRF24L01 transceiver unit has a in-chip voltage regulator and the Vcc pin is 5 volt tolerant.
 *  However, 3.3 volts works on some models and does not work in other models. Therefore, if the 3.3V pin is utilized and wireless
 *  transmission is not available, please consider switching to the 5V pin. The team encountered this issue with on transceiver module
 *  and lessons were learned.
 *  
 *  Additionally, the NRF24L01 module works with the 2.4 GHz wireless ISM band and WiFi conflicts & contention are possible. Therefore,
 *  the ADNS development team decided to have the NRF24L01 transceiver modules work above the WiFi known operating ranges of 2.401 GHz to 
 *  2.495 GHz in accordance to 802.11 legacy ISM protocols and referencing:
 *  
 *        http://www.radio-electronics.com/info/wireless/wi-fi/80211-channels-number-frequencies-bandwidth.php
 *        https://www.intel.com/content/www/us/en/support/articles/000005725/network-and-i-o/wireless-networking.html
 *  
 *  Due to the possible 2.4 GHz conflicts and contention, the recommended range of channels for the NRF24L01 transceiver is 100 to 125.
 *  Any values below channel recommendations can lead to wireless communication issues and any channel values above 125 will default to
 *  channel 1.
 *
 *  The wireless RF transmission rates are defined in the macros section with the possible rates of 250 kbps, 1 Mbps, and 2 Mbps. To
 *  simplify changing of transmission data rate, an additional macro (TRANSMIT_DATA_RATE) is defined and stores the numerical constant
 *  value needed by the NH_NRF24.h library. Please only consider these valid macro values when setting transmission rates.
 *
 *  WARNING: The channel and RF wireless transmission rates must match on both the transceiver sides. Namely, the on-board transceiver
 *           unit and the base unit must have these parameters matching. 
 *           
 *  
 *  ********************************** ADNS Teensy 3.2 Pin Mapping & Configuration *************************************************
 *  NOTE: The XSHUT pins are interchangeable for each VL53L0X sensor used but must be configured properly in the setup() function to
 *  accommodate the new changes. The SPI pins are not interchangeable (MOSI, MISO, & SCK) and must be connected in the specified
 *  manner for the NRF24L01 transceiver to work properly. However, the chip enable (CE) and slave select (SS) pins can be changed to
 *  any other available digital IO pin but must be redefined in the macros definitions.
 *
 *
 *    D = Digital pin
 *   (A)= Analog pin
 *
 *                         ------------------------------------
 *                        |******  Teensy 3.2 (Top View) ******|
 *                         ------------------------------------
 *                             [GND                  VIN]
 *                             [D0                  AGND]
 *     VL53L0X Sensor1 XSHUT --[D1                  3.3V]
 *     VL53L0X Sensor2 XSHUT --[D2               D23(A9)]-- System Init LED (Green)
 *     VL53L0X Sensor3 XSHUT --[D3               D22(A8)]-- System Init LED (Red)
 *     VL53L0X Sensor4 XSHUT --[D4               D21(A7)]-- VL53L0X Sensor14 XSHUT
 *     VL53L0X Sensor5 XSHUT --[D5               D20(A6)]-- VL53L0X Sensor13 XSHUT
 *     VL53L0X Sensor6 XSHUT --[D6               D19(A5)]-- I2C Serial Clock (SCL)
 *     VL53L0X Sensor7 XSHUT --[D7               D18(A4)]-- I2C Serial Data (SDA)
 *     VL53L0X Sensor8 XSHUT --[D8               D17(A3)]-- VL53L0X Sensor12 XSHUT
 *     NRF24L01 Chip Enable  --[D9               D16(A2)]-- VL53L0X Sensor11 XSHUT
 *     NRF24L01 Slave Select --[D10              D15(A1)]-- VL53L0X Sensor10 XSHUT
 *                  SPI MOSI --[D11              D14(A0)]-- VL53L0X Sensor9 XSHUT
 *                  SPI MISO --[D12                  D13]-- SPI Clock (SCK)
 *                              ------------------------
 * 
 */
 /********************************* NRF24L01 Pin Mapping & Configuration **************************************************************/
// NOTE: The chip enable (CE) and slave select (SS) pins can be any digital IO pin but the SPI pins must be mapped to the board
// such as the following, where A(x) is the alternate pin (if available) and x is the pin number:
// NOTE: If A(x) is listed, then any digital IO pin may be used.
/*
 *                    ATmega      Teensy 3.2      Nano   
 *  SPI Function        Pin #         Pin #       Pin #  
 * ------------        -----        -------      ------- 
 * Chip Enable (CE)    8 A(x)         9 A(x)      8 A(x) 
 *     MISO           50             12 A(8)     12      
 *     M0SI           51             11 A(7)     11      
 * SPI Clock (SCK)    52             13 A(14)    13      
 * Slave Select (SS)  53             10          10      
 * 
 * --------------------------------- NRF24L01 Transceiver unit with antenna ---------------------------
 *  
 *                                |-------------------
 *                                | Vcc (5V) | Gnd   |
 *                                |-------------------
 *  /-----------------------------| SS (CSN) | CE    |
 * <|    Antenna                  |------------------|
    \-----------------------------| MOSI     | SCK   |
 *                                |------------------|
 *                                | IRQ      | MISO  |
 *                                |-------------------
 * IRQ pin is used to trigger an interrupt in an MCU but not needed as a stead stream of sensor data is desired.                             
 */

 /***************** INA219 DC Current & High-Side Voltage Sensor Notes and Configuration ************************
 *  The ADNS development team chose to use the INA219 Current Sensor breakout board to measure the operating
 *  current and voltages of the aerial drone which the on-board ADNS  system interacts with. The module can be
 *  used to measure voltages up to 26V DC and currents of up to 3.2A. 
 *  
 *  The battery's positive and negative terminals are connected via the Vin- and Vin+ pins or Vin- and Vin+ screw
 *  ports as depicted below. If current measurements exceed 3.2A, the datasheet recommends reducing the 0.1 ohm
 *  current sense resistor and replace it with a 0.01 ohm to reach current measurements of up 32A with a resolution 
 *  of 8mA. The current sense resistor is also depicted below.
 *  
 *  I2C Address NOTE: The INA219 breakout broad used for the on-board ADNS system uses the default I2C address
 *  associated with the module. Namely, 0x40 is used as the default address but can be changed in the event of I2C
 *  address confliction or if multiple INA219 modules are in use. To change the address of any INA219 module, you have
 *  to simply solder together the two contact points on A0, A1, or both. Below, is the following combinaitions allowed:
 *  
 *  
 *                                  I2C Address             A0 Soldered?        A1 Soldered?
 *                                  -----------             ------------        ------------
 *                                    0x40                      No                   No
 *                                    0x41                      Yes                  No
 *                                    0x44                      No                   Yes
 *                                    0x45                      Yes                  Yes    <---- Both A0 & A1 are soldered and bridged together.
 *                                    
 *                                    
 ********************************** INA219 Current & High-Side Voltage Sensor Pin Configuration **************************************************
 *************************************************************************************************************************************************
 *
 *
 *            X = Screw terminals
 *            1 = Vcc (3V - 5V)
 *            2 = Gnd
 *            3 = I2C Serial Data   (SDA)
 *            4 = I2C Serial Clock  (SCL)
 *            5 = Vin- (Alternate from screw terminal Vin-)
 *            6 = Vin+ (Alternate from screw terminal Vin+)
 *            
 *                                      ---------- INA219 Sensor -----------
 *                                      ------------------------------------
 *                                     |          X              X          |
 *                                     |         Vin-           Vin+        |
 *                                     |                                    |
 *                                     |            [0.1 Ohm ]              | <------ Replace current sense resistor with lower value if applicable
 *                                     |              R100         [ ][ ] A1|
 *                                     |                           [ ][ ] A0|
 *                                     |                                    |
 *                                     |                                    |
 *                                     |   1     2     3     4     5     6  |
 *                                      ------------------------------------
 *                                      
 * The battery's positive lead will be either connected to pin 6 or to the Vin+ screw terminal.
 * The battery's negative lead is connected to ground.
 * 
 */


//
// Preprocessor Directives
// NOTE: RH_NRF24.h is the radioHead wireless library for microcontrollers
//
#include <Wire.h>
#include <VL53L0X.h>
#include <RH_NRF24.h>
#include <SPI.h>
#include <String.h>
#include <Adafruit_INA219.h>

//
// System Restart Macros (Teensy 3.2 Exclusive (MK20DX256VLH7 Cortex-M4 Processor))
//
#define CPU_RESTART_ADDRESS (uint32_t *)0xE000ED0C
#define CPU_RESTART_VALUE 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDRESS = CPU_RESTART_VALUE);

//
// Macro Definitions for VL53L0X ToF sensors
//
#define INIT_DELAY                          25
#define LED_ALARM_PULSE_TIME                125
#define TIMING_BUDGET                       17195
#define TIMEOUT_DELAY                       100
#define LASER_PRE_RANGE                     18
#define LASER_FINAL_RANGE                   14
#define TIMEOUT_VALUE                       65535
#define NUMBER_OF_SENSORS                   14

//
// Macros used for the system status LED's
//
#define SYSTEM_INITIALIZED_UNSUCCESSFUL_LED_PIN 22
#define SYSTEM_INITIALIZED_SUCCESSFUL_LED_PIN   23

//
// Macro Definitions for NRF24L01 Transceiver Unit
//
#define CHIP_ENABLE_PIN         9
#define SS_PIN                  10
#define RF_CHANNEL              115
#define RF_DATA_RATE_250kbps    DataRate250kbps
#define RF_DATA_RATE_1Mbps      DataRate1Mbps
#define RF_DATA_RATE_2Mbps      DataRate2Mbps
#define BAUD_RATE               1000000
#define SYSTEM_INIT_POLLS       250

//
// Define the transceiver data rate
// WARNING: Must match base-unit data rate
//
#define TRANSMIT_DATA_RATE    RF_DATA_RATE_1Mbps

//
// Declare and initialize NRF24 transceiver
//
RH_NRF24 on_board_transceiver(CHIP_ENABLE_PIN, SS_PIN);

// Array used to store the ToF sensors attached to each arm on the aerial quadcopter drone.
VL53L0X ToF_sensors[NUMBER_OF_SENSORS];

//
// ToF sensor XSHUT pin numbers
//
uint8_t sensor_pin_number[NUMBER_OF_SENSORS] = {1, 2, 3, 4, 5, 6, 7, 8,
                                               14, 15, 16, 17, 20, 21};

// Sensor addresses used in the program for each sensor.
// WARNING: Do not start with address 41 as it causes a timeout error.
const uint16_t sensor_addresses[NUMBER_OF_SENSORS] = {42, 43, 44, 45 ,46 ,47, 48,
                                                      49, 50, 51, 52, 53, 54, 55};

// Sensor char's used to identify each sensor reading
const char sensor_ID[15] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h',
                            'i', 'j', 'k', 'l', 'm', 'n', 'v'};

// Declare the battery sensor module object
Adafruit_INA219 battery_sensor;

//
// Unsigned integers used throughout the program to gather data. (Subject to change).
//
unsigned int sensor_value;
unsigned long int reading = 0;
float battery_voltage     = 0.0;

// 
// Character array & string used to hold messages for RF transmission.
//
const int TRANSMIT_BUFFER_SIZE              = 7;
char message_to_send[TRANSMIT_BUFFER_SIZE]  = "";
String string_value                         = "";

//
// Boolean used to trigger system status
//
bool system_initialized_properly;
bool sensor_initialized_properly;


void setup() 
{
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  //  SETUP SERIAL DATA BAUD RATE ///////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.begin(BAUD_RATE);

  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  //  SETUP NRF24L01 ON-BOARD TRANSCEIVER & INA219 BATTERY MODULE////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  // Executes if RF on_board_transceiver fails to be initialized.
  if(!on_board_transceiver.init())
  {
    Serial.println("on_board_transceiver initialization failed");
    delay(5000);
  }
  
  // Set up the RF channel, data rate, and transmit power for the on_board_transceiver.
  // NOTE: Receiver must have identical RF channel and data rate
  on_board_transceiver.setChannel(RF_CHANNEL);
  // Executes if setting channel fails.
  if(!on_board_transceiver.setChannel(RF_CHANNEL))
  {
    Serial.println("on_board_transceiver setChannel failed");
    delay(5000);
  }

  on_board_transceiver.setRF(RH_NRF24::TRANSMIT_DATA_RATE, RH_NRF24::TransmitPower0dBm);
  // Executes if RF settings failed to be initialized.
  if(!on_board_transceiver.setRF(RH_NRF24::TRANSMIT_DATA_RATE, RH_NRF24::TransmitPower0dBm))
  {
    Serial.println("on_board_transceiver setRF failed");
  }

  // Initialize and start the INA218 battery sensor module
  battery_sensor.begin();


  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  //  SETUP VL53L0X ToF PIN CONFIGURATIONS //////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////

  // Defined sensor pins are used for XSHUT pin on VL53L0X sensors. XSHUT is active HIGH and must
  // be put into LOW to suspend the sensor in order to change the internal I2C addresses.
  // Set each pin to OUTPUT mode.
  for(int index = 0; index < NUMBER_OF_SENSORS; index++)
  {
    pinMode(sensor_pin_number[index], OUTPUT);
  }

  //
  // Set the LED pins as output
  //
  pinMode(SYSTEM_INITIALIZED_UNSUCCESSFUL_LED_PIN,  OUTPUT);
  pinMode(SYSTEM_INITIALIZED_SUCCESSFUL_LED_PIN,    OUTPUT);

  // Suspend each sensor to change the addresses.
  for(int index = 0; index < NUMBER_OF_SENSORS; index++)
  {
    digitalWrite(sensor_pin_number[index], LOW);
  }

  // Initialization delay. Datasheet recommends 50ms for sensor initialization.
  delay(INIT_DELAY);
  Wire.begin();

  /* WARNING: The XSHUT pin on the VL53L0X sensors are not tolerant to 5V from the Arduino when the pin is
 *          set the active HIGH. Therefore, setting the pinMode to INPUT releases the pin and allows the 
 *          sensor board to pull the XSHUT up to operating 2.8V.
 */
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  //  SETUP I2C ADDRESSES FOR TOF SENSORS ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  
  for(int index = 0; index < NUMBER_OF_SENSORS; index++)
  {
    pinMode(sensor_pin_number[index], INPUT);
    delay(INIT_DELAY);
    sensor_initialized_properly = ToF_sensors[index].init(true);
    if(!sensor_initialized_properly)
    {
      Serial.print("Failed to initialize ToF sensor '");
      Serial.print(sensor_ID[index]);
      Serial.println("'. Please check XSHUT and power connections");
    }
    delay(INIT_DELAY);
    ToF_sensors[index].setAddress(sensor_addresses[index]);
  }


  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  //  SETUP CONFGURATION SETTINGS FOR TOF SENSORS ON EACH ARM ///////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////

  for(int index = 0; index < NUMBER_OF_SENSORS; index++)
  {
    // Set the timeout, long range limit, and pulse period (frequency) for each sensor
    ToF_sensors[index].setTimeout(TIMEOUT_DELAY);
    // lower the return signal rate limit (default is 0.25 MCPS)
    ToF_sensors[index].setSignalRateLimit(0.1);
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    ToF_sensors[index].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange,   LASER_PRE_RANGE);
    ToF_sensors[index].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, LASER_FINAL_RANGE);
    ToF_sensors[index].setMeasurementTimingBudget(TIMING_BUDGET);
    ToF_sensors[index].startContinuous(0);
  }
  
  // Set the system initialized flag to true
  system_initialized_properly = true;
}// End setup()

void loop() 
{ 
  // Increment the total cycle readings with each loop
  reading++;
  Serial.print("< ");
  Serial.print(reading);

  // Begin for loop used to poll sensor readings and transmit them to base unit
  for(int index = 0; index < NUMBER_OF_SENSORS; index++)
  {
    // Get the ToF range in mm
    sensor_value = ToF_sensors[index].readRangeContinuousMillimeters();
    // Executes if ToF sensor has timed out
    if(ToF_sensors[index].timeoutOccurred())
    {
      Serial.print("ToF sensor '");
      Serial.print(sensor_ID[index]);
      Serial.println("' timeout. Check XSHUT and power connections.");
    }

    // Executes if the sensor has timed out and system has begun initialization cycle
    if(sensor_value == TIMEOUT_VALUE && reading < SYSTEM_INIT_POLLS)
    {
      system_initialized_properly = false;
    }

    // Executes if the sensor has timed out after system initialization
    if(sensor_value == TIMEOUT_VALUE && reading > SYSTEM_INIT_POLLS)
    {
      Serial.println("Resetting system");
      delay(INIT_DELAY);
      CPU_RESTART
    }
    
    // Cast the value as a string type
    string_value = (String)sensor_ID[index] + (String)sensor_value;
    // Convert it to a char array and send it out before printing to the serial monitor
    string_value.toCharArray(message_to_send, TRANSMIT_BUFFER_SIZE);
    
    on_board_transceiver.send((uint8_t*)message_to_send, sizeof(message_to_send));
    

    Serial.print(", ");
    Serial.print(sensor_value);
  }
  // Poll the battery voltage
  battery_voltage = battery_sensor.getBusVoltage_V();
  
  string_value = (String)sensor_ID[NUMBER_OF_SENSORS] + (String)battery_voltage;
  string_value.toCharArray(message_to_send, TRANSMIT_BUFFER_SIZE);
  on_board_transceiver.send((uint8_t*)message_to_send, sizeof(message_to_send));

  Serial.print(", ");
  Serial.print(battery_voltage);
  

  // Executes if SYSTEM_INIT_POLLS macro has not been met
  if(reading == SYSTEM_INIT_POLLS)
  {
    // Executes if system was initialized properly. Else, system alarm is set
    if(system_initialized_properly)
    {
      digitalWrite(SYSTEM_INITIALIZED_SUCCESSFUL_LED_PIN, HIGH);
    }
    else
    {
      digitalWrite(SYSTEM_INITIALIZED_SUCCESSFUL_LED_PIN, LOW);
      set_off_init_alarm();
    }
  }

  Serial.println(" >");
  Serial.flush();

}// End loop()

/* @function: set_off_init_alarm
 * @brief:    Function that lights up the system error LED
 * @desc:     Triggers on and off the system initilization red LED indicating that an initilization
 *            error has occurred by either a system init trigger or a timeout issue that led to a
 *            system reset and could not be re-initialized.
 * @param:    none
 * 
 * @note:     The for loop used to pulse the LED alarm is set for 40 pulses. Default has the macro
 *            LED_ALARM_PULSE_TIME set to 125ms which gives the user 10 seconds of notice. If changed, use
 *            formula:      LED_ALARM_PULSE_TIME * 40 = Total alarm time
 */
void set_off_init_alarm()
{
  int index;
  // For loop used to indicate system initialization has failed
  for(index = 0; index < 40; index++)
  {
    digitalWrite(SYSTEM_INITIALIZED_UNSUCCESSFUL_LED_PIN, HIGH);
    delay(LED_ALARM_PULSE_TIME);
    digitalWrite(SYSTEM_INITIALIZED_UNSUCCESSFUL_LED_PIN, LOW);
    delay(LED_ALARM_PULSE_TIME);
  }
  // Force restart of system to re-attempt system initialization
  CPU_RESTART
}



void check_for_data()
{
  // Check if the base unit has a message for the on-board system
  if(on_board_transceiver.available())
  {
    // Allocate a receive buffer for the message
    uint8_t receive_buffer[RH_NRF24_MAX_MESSAGE_LEN];
    uint8_t buffer_length = sizeof(receive_buffer);

    if(on_board_transceiver.recv(receive_buffer, &buffer_length))
    {
      Serial.println((char*) receive_buffer);
      delay(500);
    }
  }
}
