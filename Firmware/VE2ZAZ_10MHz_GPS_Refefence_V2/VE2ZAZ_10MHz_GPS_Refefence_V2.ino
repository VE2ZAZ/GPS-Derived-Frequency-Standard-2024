/*
  _  _  ___  ___  ___   __   ___     _   _  _  __   ___  __  _    _ 
 ( )( )(  _)(__ \(_  ) (  ) (_  )   / ) ( )( )(  ) (__ \(  )( \/\/ )
  \\//  ) _)/ __/ / /  /__\  / /   / /   \\// /__\ / __/ )(  \    / 
  (__) (___)\___)(___)(_)(_)(___) (_/    (__)(_)(_)\___)(__)  \/\/                                                                                          

  10 MHz GPS-Referenced Frequency Standard: Uses DACx0501 12/14/16 bit DAC, and STM32 Black Pill or Blue Pill platform
  VE2ZAZ/VA2IW, V2, May 2025. https://ve2zaz.net
  Licence: CC BY-SA 4.0 (Creative Commons Attribution-ShareAlike 4.0 International). 
  Read https://creativecommons.org/licenses/by-sa/4.0/

  Version history:
  ================
  V2 (2025-05): Corrects the SPI transmission mode to the DAC80501 chip. Must use mode 2 (falling clock edge latches the data, high clock level in idle), as per the DAC datasheet.
                Using mode 0 may lead to DAC misbehavior.
  V1 (2024-03): Initial version

*/

// COMPILER DIRECTIVES
#include <SPI.h> 
#include <Wire.h>
#include <FlashStorage_STM32.h>               // Requires the installation of FlashStorage_STM32 in the Arduino IDE library manager.
#include <LiquidCrystal_PCF8574.h>            // Requires the installation of LiquidCrystal_PCF8574 in the Arduino IDE library manager.

// ********** For the STM32F103T8C6 (Blue Pill), you must eliminate or comment out the following line
#define BLACK_PILL_USED


#define addr_LCD                0x27                                    // The I2C address of the LCD
LiquidCrystal_PCF8574 lcd(addr_LCD);                                    // Sets the I2C address of the LCD to 0x27

#define PIN_PPS                 PA1                                     // Timer2, capture input (PPS Signal)
#define PIN_REF_IN              PA0                                     // Timer2, counter increment input (10MHz signal).
#define PIN_SPI_SS              PB12                                    // The output pin to the Slave Select of the DAC
#define PIN_SPI_MOSI            PB15                                    // The SPI data pin to the DAC
#define PIN_SPI_MISO            PB14                                    // The SPI data pin coming from the DAC (not used)
#define PIN_SPI_SCLK            PB13                                    // The SPI output pin to the DAC clock
#define PIN_UART_RX             PA3                                     // The data pin coming from the GPS
#define PIN_UART_TX             PA2                                     // The data pin going to the GPS (not used)
#define BUTTON_CLR_ALM          PB9                                     // Push button pin to clear previous alarms
#define FREQ_OCXO               10000000.0000                           // OCXO frequency. Express with 4 decimal places to force the precision of calculations
#define YES                     true
#define NO                      false
#define SEPAR                   "|"
#define VT_CLS                  "\x1B[2J\x1B[H"                         // Code VT-100 "clearscreen + cursorhome"
#define VT_DEFAULT_COLOR        "\x1B[49m"                              // VT-100 code for default background color
#define VT_REVERSE              "\x1B[7m"                               // VT-100 Code of Inverted Background Color
#define VT_NORMAL               "\x1B[m"                                // VT-100 code of normal background color
#define SEPAR_LINE              "================================================================================"
#define GPRMC_PREAMBLE          "$GNRMC"            
#define WELC_MESSAGE            "\n10 MHz GPS-Referenced Frequency Standard, Version 1, March 2024.\r\nBy VE2ZAZ/VA2IW  https://ve2zaz.net\r\n"    // Welcome message

HardwareSerial Serial2(PIN_UART_RX, PIN_UART_TX);                       // Instantiation of software UART to receive messages from GPS
HardwareTimer RefTimer(TIM2);                                           // Instantiating the TIM2 timer

// GLOBAL VARIABLES
unsigned short int            nominal_count;                            // The sample result of TIM2 if the frequency is exactly 10 MHz
volatile unsigned short int   new_val_capture;                          // The new sample value of TIM2
volatile unsigned short int   prev_val_capture;                         // The old sample value of TIM2
volatile unsigned short       capture_counter = 0;                      // The counter of the number of PPS during sampling
volatile unsigned int         osc_count;                                // The TIM2 sample to analyze
short int                     delta_osc_count;                          // The difference between sample and nominal count
short int                     sum_delta_osc_count = 0;                  // The sum of delta_count_osc for calculation purposes
unsigned short int            sample_counter = 0;                       // The number of samples counter
double                        avg_delta_osc_count;                      // The accumulated mean of sample-nominal differences
double                        avg_offset_hz = 0;                        // The average of the sample offset, expressed in Hz
double                        measured_freq_hz;                         // The measured 10 MHz frequency
static double                 response_lsb_per_hz;                      // The variation of the DAC corresponding to a 1 Hz change in the oscillator. Calculated value. Must be "Static" otherwise the value is lost when entering the Loop function! Unexpected behavior!
volatile bool                 sample_ready = NO;                        // The signal announcing that a sample is ready
volatile bool                 detection_pps = NO;                       // The signal indicating that a PPS pulse has been detected
bool                          end_of_cycle = NO;                        // The signal announcing the end of the measurement cycle
int                           delta_dac;                                // The correction to apply to the DAC value
unsigned short int            dac_val;                                  // The current DAC value (16 bits).
unsigned short int            new_dac_val;                              // The new calculated DAC value (16 bits).
char                          sign_delta_dac;                           // The character (+ or -) to display on the delta_dac result
unsigned long int             begin_led_millis = 0;                     // The retained value of millis for the purpose of turning off the LED
unsigned long int             begin_stabilization_millis = 0;           // The retained value of millis for FLL stabilization purposes
unsigned long int             begin_rx_command_millis;                  // The retained value of millis for the purposes of receipt of an order
unsigned long int             begin_detection_pps_millis;               // The retained value of millis for purposes of detecting the absence of PPS signal
bool                          reinitialize_counter = NO;                // The signal to advise to re-initialize the TIM2 counter
bool                          mode_rx_command_uart;                     // The signal indicating that a received order is being completed
char                          charact_received;                         // The character received from the computer's UART.
String                        charact_received_string = "";             // The character string received from the computer's UART.
String                        command;                                  // A command received from the computer
unsigned short int            index1;                                   // The position of the first parameter of the command received
unsigned short int            index2;                                   // The position of the second parameter of the received command
unsigned short int            index3;                                   // The position of the third parameter of the received command
unsigned short int            index4;                                   // The position of the fourth parameter of the received command
String                        param1;                                   // The first parameter of the received command
String                        param2;                                   // The second parameter of the received command
String                        param3;                                   // The third parameter of the received command
bool                          fll_active;                               // Reports the active or inactive state of the FLL
unsigned char                 hrs;                                      // Numerical value of time
unsigned char                 mins;                                     // Numerical value of the minute
unsigned char                 secs;                                     // Numerical value of the second
unsigned char                 day_val;                                  // Numerical value of the day
unsigned char                 month_val;                                // Numerical value of the month
unsigned char                 year_val;                                 // Numerical value of the year
String                        gps_charact_string_received;              // The character string received from the GPS UART.
String                        hour_string;                              // The character string of the time received from the GPS
String                        date_string;                              // The character string of the date received from the GPS
String                        hrs_string;                               // Text value to display of the time
String                        mins_string;                              // Text value to display of the minute
String                        secs_string;                              // Text value to display of the second
String                        day_string;                               // Text value to display of the day
String                        month_string;                             // Text value to display of the month
String                        year_string;                              // Text value to display of the year
bool                          leap_year;                                // The active or inactive state of the leap year
bool                          long_cycle = NO;                          // Indicates if the current cycle is a long cycle
bool                          medium_cycle = NO;                        // Indicates if the current cycle is an average cycle
bool                          short_cycle = YES;                        // Indicates if the current cycle is a short cycle
bool                          valid_gps_data = NO;                      // Indicates whether GPS is sending data and a valid PPS signal
double                        offset_PI_loop_hz;                        // The offset (in Hz) calculated by the P-I loop and used to adjust the DAC
double                        avg_offset_hz_array[10];                  // Warning: a double takes 8 bytes! Table of accumulation of the average of cycle shifts
int                           index_avg_offset_hz_array;                // Offset Average Accumulation Table Position Index
int                           max_index_avg_offset_hz_array;            // Maximum of the position index of the accumulation table of the average of offsets
double                        avg_avg_offset_hz_array;                  // The average of the average of the cycle shifts taken from the table
byte                          delta[8] = {                              // The definition of the uppercase delta Greek character used on the LCD
                                           B00100,
                                           B00100,
                                           B01010,
                                           B01010,
                                           B10001,
                                           B10001,
                                           B11111
                                         }; 
// System Alarms
unsigned char           current_alarms =                 B00000000;  // The alarm register saving the presence of current alarms
unsigned char           past_alarms   =                 B00000000;  // The register of previous alarms
// The alarm bit positions in the alarm and past alarm registers are as follows
#define                 ALARM_ACQUISITION_INIT              0           // In initial acquisition after start-up
#define                 ALARM_DAC_LIMIT                     1           // The DAC has reached its minimum or maximum value
#define                 ALARM_FLL_OFF                       2           // FLL is off
#define                 ALARM_PPS_MISSING                   3           // GPS PPS signal is missing
#define                 ALARM_REJECTED_SAMPLE               4           // In a long cycle, the sample is rejected because it is too far away
#define                 ALARM_FLL_UNLOCKED                  5           // The FLL is unlocked because not in long cycle
#define                 ALARM_10M_MISSING                   6           // The 10 MHz reference signal is missing
#define                 ALARM_GPS_DATA_INVALID              7           // GPS provides invalid data

// Default system variables
unsigned short int      num_pps_per_sample =                10;         // The number of PPS pulses per frequency sample
unsigned short int      num_samples_per_cycle =             1;          // The number of samples in the analysis period
unsigned char           num_dac_bits =                      16;         // The number of bits of the DAC
unsigned short int      duration_short_cycle =              1;          // The number of samples of a short cycle
unsigned short int      duration_medium_cycle =             10;         // The number of samples of an average cycle
unsigned short int      duration_long_cycle =               100;        // The number of samples of a long cycle
float                   thresh_medium_cycle =               0.101;      // The threshold (in Hz) of medium -> short cycle change
float                   thresh_long_cycle =                 0.0101;     // The threshold (in Hz) of long -> medium cycle change
float                   p_index =                           1;          // The Kp component of the P-I loop
float                   i_index =                           0;          // The Ki component of the P-I loop
unsigned char           thresh_delta_count_reject =         2;          // In long cycle, the only one forcing the rejection of the sample
bool                    mode_verbose =                      YES;        // The signal indicating that messages sent to the UART are in verbose mode
bool                    mode_verbose_vt100 =                YES;        // The option indicating whether VT-100 control characters should be sent

// Fixed system defaults. Must be measured and entered here:
#define                 DAC_VOLTAGE_MIN                     0.0123      // Minimum DAC voltage, ignoring the next gain stage. Must be a number with decimal(s)
#define                 DAC_VOLTAGE_MAX                     4.995       // Maximum voltage of the DAC, ignoring the next gain stage. Must be a number with decimal(s)
#define                 DAC_POST_GAIN                       1.0         // Gain of the op amp at the DAC output. Must be a number with decimal(s)
#define                 RESPONSE_OCXO_HZ_PER_V              1.489       // Morion MV89A: 1.489 Isotemp OCXO131-100: 2.6385 Measured response of the OCXO used. Must be a number with decimal(s). Will be negative if the OCXO has an inverted -Hz/V response.


// Function adding zeros to the front of an "integer" value to ensure constant display width
String int_stuff_zeros_str(int a)
{
  if (a >= 1000 && a < 10000) return "0" + String(a);
  else if (a >= 100 && a < 1000) return "00" + String(a);
  else if (a >= 10 && a < 100) return "000" + String(a);
  else if (a < 10) return "0000" + String(a);
  else return String(a);
}
// Function adding zeros to the front of a "double" value to ensure constant display width
String double_stuff_zeros_str(double a)
{
  if (a >= 100 && a < 1000) return String(a,3);
  else if (a >= 10 && a < 100) return String(a,4);
  else if (a < 10) return String(a,5);
}

// Function managing the interruption of timer T2 generated by the PPS signal
void interrupt_TIM2_PPS()
{
  detection_pps = YES;                                      // Notify of detection of a PPS pulse
  if (reinitialize_counter)                                 // Request to reset the counter?
  {  // Yes
    reinitialize_counter = NO;                              // Clear request
    new_val_capture = TIM2->CCR2;                           // Take a new counter value
    capture_counter = 1;                                    // Reset capture counter
  }
  else if (capture_counter++ >= num_pps_per_sample)         // Not reset. Waited for the specified duration in seconds?
    { // Yes
      if (fabs(TIM2->CCR2 - prev_val_capture) < 1000) bitSet(current_alarms, ALARM_10M_MISSING);   // Missing 10 MHz reference detection
      else bitClear(current_alarms, ALARM_10M_MISSING);     // 10 MHz reference not missing
      prev_val_capture = new_val_capture;                   // Keep the previous captured value of TIM2
      new_val_capture = TIM2->CCR2;                         // Read the new captured value from TIM2
      if (new_val_capture > prev_val_capture) osc_count = new_val_capture - prev_val_capture;    // Handle overflow of new and previous values
      else osc_count = 65536 + new_val_capture - prev_val_capture;                                  //     "
      capture_counter = 1;                                  // Reset capture counter
      sample_ready = YES;                                   // Advise that the sample is ready.
    }
}

// This function forces the restart of an acquisition cycle
void restart_acquisition()
{
  reinitialize_counter = YES;                               // Re-initialize the TIM2 counter
  sample_counter = 0;                                       // Re-initialize the different values to start a new sampling cycle.
  avg_delta_osc_count = 0;                                  //            "
  sum_delta_osc_count = 0; 
  num_samples_per_cycle = duration_short_cycle;             // Start with a short cycle.
  short_cycle = YES;                                        //            "
  medium_cycle = NO;
  long_cycle = NO;            
}

// The function sending a new value to the DACx0501
void Send_Val_Dac(unsigned short int valeur)
{
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2)); // Start of writing to one of the DACs via the SPI port (rate, bit order, SPI bus mode)
  digitalWrite(PIN_SPI_SS, LOW);                             // Lowering the DAC’s Slave Select signal
  SPI.transfer(8);                                           // Transferring the DAC value register address
  SPI.transfer16(valeur<<(16-num_dac_bits));                 // Transfer of 16 bits
  digitalWrite(PIN_SPI_SS, HIGH);                            // Raising the DAC’s Slave Select signal
  SPI.endTransaction();                                      // End of DAC writing, release of the bus.
}

// Function that retrieves various system settings from flash memory
void read_eeprom_params()
{
  unsigned long int signature;                               // Contains the signature to read
  EEPROM.get(0,signature);                                   // Read the signature of address 0
  if (signature == 0xCAFEFADE)                               // Signature match with “CAFEFADE”?
  {                                                          // Yes, recover the different parameters from Flash memory
    EEPROM.get(4, num_pps_per_sample);                       //           "
    EEPROM.get(6, duration_short_cycle );
    EEPROM.get(8, duration_medium_cycle);
    EEPROM.get(10, duration_long_cycle);
    EEPROM.get(12, thresh_medium_cycle);
    EEPROM.get(16, thresh_long_cycle);
    EEPROM.get(20, mode_verbose);
    EEPROM.get(21, mode_verbose_vt100);
    EEPROM.get(22, thresh_delta_count_reject);    
    EEPROM.get(23, p_index);                      
    EEPROM.get(27, i_index);                     
    EEPROM.get(31, num_dac_bits);                     
    Serial.println("Parametres recuperes de la memoire Flash");
  } 
  // No, no signature match detected
  else Serial.println("Paramètres par défaut utilisés");      // Default settings will be used
}   

// Function that saves the various system parameters in flash memory
// Attention! Flash memory has a guaranteed lifespan of 10,000 writes.
void save_eeprom_params()
{ 
  EEPROM.setCommitASAP(NO);                                      // Force writing to Flash only with the commit() command
  EEPROM.put(0, 0xCAFEFADE);                                     // 4 bytes. Save signature to verify that flash memory contains settings.
  EEPROM.put(4, num_pps_per_sample);                             // 2 bytes. Save other settings.
  EEPROM.put(6, duration_short_cycle);                           // 2 bytes
  EEPROM.put(8, duration_medium_cycle);                          // 2 bytes
  EEPROM.put(10, duration_long_cycle);                           // 2 bytes
  EEPROM.put(12, thresh_medium_cycle);                           // 4 bytes
  EEPROM.put(16, thresh_long_cycle);                             // 4 bytes
  EEPROM.put(20, mode_verbose);                                  // 1 byte
  EEPROM.put(21, mode_verbose_vt100);                            // 1 byte
  EEPROM.put(22, thresh_delta_count_reject);                     // 1 byte
  EEPROM.put(23, p_index);                                       // 4 bytes
  EEPROM.put(27, i_index);                                       // 4 bytes
  EEPROM.put(31, num_dac_bits);                                  // 1 byte
  EEPROM.commit();                                               // Complete memory writing
}

// Results display function called when there is a receipt PPS.
void display_results_console()
{
  if (!mode_rx_command_uart)                                                      // Command being received on the UART?
  {                                                                               // No, allowed to display results
    if (mode_verbose)                                                             // Verbose (verbose) mode active?
    { // Yes
      if (mode_verbose_vt100) Serial.print(VT_CLS);                               // If VT-100 display mode active, send screen clear code
      Serial.println();                                                           // Print all fields to serial port
      Serial.println(SEPAR_LINE);                                                 //           "
      Serial.println(date_string + "-" + hour_string + "-UTC");
      Serial.print("Alarms:                                           ");
      if ((mode_verbose_vt100) && (current_alarms)) Serial.print(VT_REVERSE);     // If VT-100 display mode active, display alarms in reverse video
      Serial.print(bitRead(current_alarms, ALARM_ACQUISITION_INIT) ? "A" : (bitRead(past_alarms, ALARM_ACQUISITION_INIT) ? "a" : "-"));
      Serial.print(bitRead(current_alarms, ALARM_FLL_UNLOCKED) ? "L" : (bitRead(past_alarms, ALARM_FLL_UNLOCKED) ? "l" : "-"));
      Serial.print(bitRead(current_alarms, ALARM_DAC_LIMIT) ? "D" : (bitRead(past_alarms, ALARM_DAC_LIMIT) ? "d" : "-"));
      Serial.print(bitRead(current_alarms, ALARM_FLL_OFF) ? "F" : (bitRead(past_alarms, ALARM_FLL_OFF) ? "f" : "-"));
      Serial.print(bitRead(current_alarms, ALARM_PPS_MISSING) ? "P" : (bitRead(past_alarms, ALARM_PPS_MISSING) ? "p" : "-"));
      Serial.print(bitRead(current_alarms, ALARM_REJECTED_SAMPLE) ? "R" : (bitRead(past_alarms, ALARM_REJECTED_SAMPLE) ? "r" : "-"));      
      Serial.print(bitRead(current_alarms, ALARM_10M_MISSING) ? "O" : (bitRead(past_alarms, ALARM_10M_MISSING) ? "o" : "-"));      
      Serial.println(bitRead(current_alarms, ALARM_GPS_DATA_INVALID) ? "G" : (bitRead(past_alarms, ALARM_GPS_DATA_INVALID) ? "g" : "-"));      
      if (mode_verbose_vt100) Serial.print(VT_NORMAL);                            // If VT-100 display mode active, display alarms in normal video
      if (!bitRead(current_alarms, ALARM_PPS_MISSING))                            // PPS missing?
      {                                                                           // No, show the rest of the info
        Serial.print("Cycle:                                            ");
        Serial.print(short_cycle ? "Short"   : "");
        Serial.print(medium_cycle ? "Medium" : "");
        Serial.println(long_cycle ? "Long"   : "");
        Serial.print("Sample:                                           ");
        Serial.print(sample_counter);
        Serial.print(" / ");
        Serial.println(num_samples_per_cycle);     
        Serial.print("Current DAC value:                                ");
        Serial.println(dac_val);
        Serial.print("Oscillator counter| Nominal Count:                ");
        Serial.print(osc_count);
        Serial.print(" | ");
        Serial.println(nominal_count);      
        Serial.print("Offset from nominal count:                        ");
        Serial.println(delta_osc_count);
        Serial.print("Count offset average:                             ");
        Serial.println(avg_delta_osc_count,6);
        Serial.print("Offset average (ppm)                              ");
        Serial.println(avg_offset_hz / FREQ_OCXO * 1e6, 4);
        Serial.print("Offset average (Hz):                              ");
        Serial.println(avg_offset_hz,6);
        Serial.print("Calculated average frequency of reference (Hz):   ");
        Serial.println(measured_freq_hz,6);
        if (end_of_cycle)                                                           // End of sampling cycle?
        {                                                                           // Yes, Show end of cycle data.
          if (mode_verbose)                                                         // Verbose (verbose) mode active?
          {                                                                         // Yes, give more details
            if (mode_verbose_vt100) Serial.print(VT_REVERSE);                       // If VT-100 display mode active, display in reverse video
            Serial.println("SAMPLING CYCLE COMPLETED");                            // Printing end of cycle results
            Serial.print("Ajustement made to DAC:                           ");     //        "
            Serial.println(-delta_dac);     
            Serial.print("New DAC value:                                    ");
            Serial.println(new_dac_val);     
            Serial.print("Offset calculation (Hz) of PI loop:               ");
            Serial.println(offset_PI_loop_hz,6);
            
            Serial.println("Pause for stabilization...");   
            if (mode_verbose_vt100) Serial.print(VT_NORMAL);                        // If VT-100 display mode active, return to normal video mode
          }            
        }
      }
      Serial.println(SEPAR_LINE);
    }
    else                                                                            // Not in verbose mode, display in condensed form
    { 
      String tempstr =  String('S') + SEPAR +                                       // Construct the string to display. Compiler bug. You must do String('S').
                        date_string + "_" + hour_string + SEPAR +
                        (bitRead(current_alarms, ALARM_ACQUISITION_INIT) ? "A" : (bitRead(past_alarms, ALARM_ACQUISITION_INIT) ? "a" : "_")) +
                        (bitRead(current_alarms, ALARM_FLL_UNLOCKED) ? "L" : (bitRead(past_alarms, ALARM_FLL_UNLOCKED) ? "l" : "_")) +
                        (bitRead(current_alarms, ALARM_DAC_LIMIT) ? "D" : (bitRead(past_alarms, ALARM_DAC_LIMIT) ? "d" : "_")) +
                        (bitRead(current_alarms, ALARM_FLL_OFF) ? "F" : (bitRead(past_alarms, ALARM_FLL_OFF) ? "f" : "_")) +
                        (bitRead(current_alarms, ALARM_PPS_MISSING) ? "P" : (bitRead(past_alarms, ALARM_PPS_MISSING) ? "p" : "_")) +
                        (bitRead(current_alarms, ALARM_REJECTED_SAMPLE) ? "R" : (bitRead(past_alarms, ALARM_REJECTED_SAMPLE) ? "r" : "_")) + 
                        (bitRead(current_alarms, ALARM_10M_MISSING) ? "O" : (bitRead(past_alarms, ALARM_10M_MISSING) ? "o" : "_")) + 
                        (bitRead(current_alarms, ALARM_GPS_DATA_INVALID) ? "G" : (bitRead(past_alarms, ALARM_GPS_DATA_INVALID) ? "g" : "_")) + SEPAR +
                        int_stuff_zeros_str(dac_val) + SEPAR; 
      if (!bitRead(current_alarms, ALARM_PPS_MISSING))
      {
        tempstr = tempstr + (short_cycle ? "C" : "") +
                            (medium_cycle ? "M" : "") +
                            (long_cycle ? "L" : "") + SEPAR +
                            int_stuff_zeros_str(sample_counter) + SEPAR + 
                            int_stuff_zeros_str(num_samples_per_cycle) + SEPAR + 
                            (delta_osc_count < 0 ? "-" : " ") + int_stuff_zeros_str(abs(delta_osc_count)) + SEPAR +
                            (avg_offset_hz < 0 ? "-" : " ") + double_stuff_zeros_str(abs(avg_offset_hz)) + SEPAR + 
                            ((end_of_cycle && long_cycle) ? ((offset_PI_loop_hz < 0 ? "-" : " ") + double_stuff_zeros_str(abs(offset_PI_loop_hz))) : "________") + SEPAR + 
                            (end_of_cycle ? ((delta_dac > 0 ? "-" : " ") + int_stuff_zeros_str(abs(delta_dac))) : "______");                       
      }
      Serial.println(tempstr);                                                      // Show channel
    }
  }
}

// LCD display function of FLL results. Called when there is a receipt PPS.
void display_results_lcd()
{
  lcd.clear();                                            // Clear screen
  lcd.setCursor(0, 0);                                    // position the cursor
  char buffer1[5];                                        // Create temporary character memories
  char buffer2[5];                                        //         "
  sprintf(buffer1, "%05d", dac_val);                      // Format the DAC value to 5 digits
  sprintf(buffer2, "%06d", -delta_dac);                   // Format the DAC gap to 6 digits
  lcd.print("D:" + String(buffer1) + "  ");               // Show DAC
  lcd.write(byte(0));                                     // display the "delta" character placed in LCD memory in the setup function
  lcd.print("D:" + String(buffer2));                      // Show DAC
  lcd.setCursor(0, 1);                                    // position the cursor
  lcd.print("C:" + String(short_cycle ? "C" : "") + String(medium_cycle ? "M" : "") + String(long_cycle ? "L" : ""));    // Show current cycle
  lcd.print("#" + String(sample_counter) + "  ");         // Show sample counter
  lcd.write(byte(0));                                     // display the "delta" character placed in LCD memory in the setup function
  lcd.print("Hz:" + String(avg_offset_hz,4));             // Show deviation in Hz
  lcd.setCursor(0, 2);                                    // position the cursor
  lcd.print("AL:");                                       // Show alarms
  lcd.print(bitRead(current_alarms, ALARM_ACQUISITION_INIT) ? "A" : (bitRead(past_alarms, ALARM_ACQUISITION_INIT) ? "a" : "-"));
  lcd.print(bitRead(current_alarms, ALARM_FLL_UNLOCKED) ? "L" : (bitRead(past_alarms, ALARM_FLL_UNLOCKED) ? "l" : "-"));
  lcd.print(bitRead(current_alarms, ALARM_DAC_LIMIT) ? "D" : (bitRead(past_alarms, ALARM_DAC_LIMIT) ? "d" : "-"));
  lcd.print(bitRead(current_alarms, ALARM_FLL_OFF) ? "F" : (bitRead(past_alarms, ALARM_FLL_OFF) ? "f" : "-"));
  lcd.print(bitRead(current_alarms, ALARM_PPS_MISSING) ? "P" : (bitRead(past_alarms, ALARM_PPS_MISSING) ? "p" : "-"));
  lcd.print(bitRead(current_alarms, ALARM_REJECTED_SAMPLE) ? "R" : (bitRead(past_alarms, ALARM_REJECTED_SAMPLE) ? "r" : "-")); 
  lcd.print(bitRead(current_alarms, ALARM_10M_MISSING) ? "O" : (bitRead(past_alarms, ALARM_10M_MISSING) ? "o" : "-")); 
  lcd.print(bitRead(current_alarms, ALARM_GPS_DATA_INVALID) ? "G" : (bitRead(past_alarms, ALARM_GPS_DATA_INVALID) ? "g" : "-")); 
}

// Function allowing you to “RESET” the execution of the program on request
void(* resetFunc) (void) = 0; 

// Initialization function, Arduino style, executed only once at program launch
void setup()
{     
  // Reservation of sufficient space for handling 'Strings'
  hrs_string.reserve(5);       
  mins_string.reserve(5);        
  secs_string.reserve(5);         
  day_string.reserve(5);            
  month_string.reserve(5);      
  year_string.reserve(5);   
  command.reserve(10);
  param1.reserve(10);
  param2.reserve(10);
  param3.reserve(10);
  hour_string.reserve(12);
  date_string.reserve(12);
  charact_received_string.reserve(30);
  gps_charact_string_received.reserve(100);

  Serial.begin(115200);                               // The serial port to read reports and results
  while (Serial.available() > 0) Serial.read();       // Flush hardware serial port buffer
  Serial2.begin(9600);                                // The “soft” serial port to receive GPS data

  pinMode(PIN_PPS, INPUT_PULLUP);                     // PPS pin input with pullup
  pinMode(PIN_REF_IN, INPUT_PULLUP);                  // Reference pin 10 MHz input with pullup
  pinMode(LED_BUILTIN, OUTPUT);                       // Integrated LED output pin
  pinMode(BUTTON_CLR_ALM, INPUT_PULLUP);              // Clear Previous Alarms Button Pin

  // SPI Port #2 Assignment
  pinMode(PIN_SPI_SS, OUTPUT);                        // Slave Select pin to DAC
  digitalWrite(PIN_SPI_SS, HIGH);                     // Disable DAC SS
  SPI.setMOSI(PIN_SPI_MOSI);
  SPI.setMISO(PIN_SPI_MISO);
  SPI.setSCLK(PIN_SPI_SCLK);

  // Retrieve system settings from EEPROM here.
//  save_params_eeprom();
  read_eeprom_params();                               // Load settings from flash memory if they exist.

  // Calculating the nominal count of a sample for a reference of exactly 10 MHz
  nominal_count = round((num_pps_per_sample * FREQ_OCXO / 65536 - trunc(num_pps_per_sample * FREQ_OCXO / 65536)) * 65536); 

  // Calculation of system parameters based on components and topology
  response_lsb_per_hz = 1 / double((DAC_VOLTAGE_MAX - DAC_VOLTAGE_MIN) * DAC_POST_GAIN / (1<<num_dac_bits) * RESPONSE_OCXO_HZ_PER_V);
  begin_stabilization_millis = millis();

  // Initialize the FLL, DAC and acquisition process
  fll_active = YES;
  restart_acquisition();
  end_of_cycle  = YES;
  bitSet(current_alarms,ALARM_ACQUISITION_INIT);  // Enable initial acquisition alarm

  // DAC setup
  SPI.begin();                                                    // Prepare the SPI bus.
  delay(100);                                                     // Pause to allow the serial port to open on the computer

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));// Start of a DAC write via the SPI port (rate, bit order, SPI bus mode)
  digitalWrite(PIN_SPI_SS, LOW);                                  // Lowering the DAC’s Slave Select signal
  SPI.transfer(4);                                                // Transferring the DAC gain register address
  SPI.transfer16(1);                                              // Transfer of 16 bits, gain of 2 on the internal reference
  digitalWrite(PIN_SPI_SS, HIGH);                                 // Raising the DAC’s Slave Select signal
  SPI.endTransaction();                                           // End of DAC writing, release of the bus.
  dac_val = (1<<(num_dac_bits-1));                                // DAC placed halfway.
  Send_Val_Dac(dac_val);                                          // Position the DAC-A

  // Redirection of PA0 and PA1 pins to the TIM2 counter. Only required with the STM32F4x1 Black Pill
  #if defined(BLACK_PILL_USED)
    GPIOA->MODER |= 0b00001010;
    GPIOA->AFR[0] |= 0b00010001;
  #endif 
  
  // Configuration of the STM32 TIM2 counter to “capture” the 10 MHz reference
  TIM2->CR1 =   0b0000000000000000;                // TIM2 disabled initially
  TIM2->CR2 =   0b0000000000000000;                // TIM2 Channel 1 is the input
  TIM2->SMCR =  0b0100000001110111;                // ETR non-inverted, External Mode 2 activated, Prescaler off, no filter. master-slave sync, ETR input will increment TIM2
  TIM2->DIER =  0b0000000000000100;                // Capture/Compare 2 interrupt enabled,
  TIM2->EGR =   0b0000000000000100;                // Capture/compare 2 generation enabled
  TIM2->CCMR1 = 0b0000000100000000;                // Channel 2 input for PPS triggering, no filter, no preselector
  TIM2->CCMR2 = 0b0000000000000000;                // Channels not used
  TIM2->CCER =  0b0000000000010000;                // Rising edge of CC2P, Capture activated
  TIM2->PSC =   0b0000000000000000;                // Prescale to 1
  TIM2->ARR =   0b1111111111111111;                // Auto-Reload value at maximum.
  TIM2->CR1 =   0b0000000000000001;                // TIM2 activated and increments

  RefTimer.attachInterrupt(2,interrupt_TIM2_PPS);     // Associate the interrupt function with the TIM2 counter, channel 2

  // Show starting message to serial port
  delay(2000);                                        // Pause to allow the serial port to open on the computer
  Serial.println(WELC_MESSAGE);                       // Welcome message
  Serial.println("Pause de stabilisation...");   

  // Configure the LCD and display the starting message
  lcd.begin(20, 4);                                   // Define and initialize the LCD as having 16 columns and 2 rows
  lcd.createChar(0, delta);
  lcd.setBacklight(255);                              // Turn on the backlight
  lcd.home();                                         // position the cursor at the first box
  lcd.clear();                                        // Clear screen
  // Ignition logo display
  lcd.setCursor(0, 0);                                // position the cursor
  lcd.print("VE2ZAZ GPS Reference");                   // Show text between apostrophes
  lcd.setCursor(0, 1);                                // position the cursor
  lcd.print(" Version 2, 2025/05");                   // Show text between apostrophes
  lcd.setCursor(0, 2);                                // position the cursor
  lcd.print("   Please Wait...");                     // Show text between apostrophes
}

// Main loop, Arduino style
void loop()
{
  if (sample_ready)                                                                       // Is a sample ready?
  { // Yes
    sample_ready = NO;                                                                    // Reset New Sample Ready Marker

    // Cumulative calculation of sampling results
    delta_osc_count = osc_count - nominal_count;                                          // Calculate the difference between the sample count and the nominal count
    if (valid_gps_data && (!long_cycle || (long_cycle && (abs(delta_osc_count) < thresh_delta_count_reject))))  // Reject invalid or too bad samples (sudden GPS deviation) in long cycle.
    {
      ++sample_counter;                                                                   // Increment sample counter

      // Calculation of the accumulated "Proportional" component of the P-I loop in all types of cycles
      sum_delta_osc_count += delta_osc_count;                                             // Accumulate these differences for calculation purposes
      avg_delta_osc_count = double(sum_delta_osc_count) / sample_counter;                 // Average these differences
      avg_offset_hz = double(avg_delta_osc_count) / num_pps_per_sample;                   // Express the average of the differences in Hz. Is the P component of the P-I loop
      measured_freq_hz = FREQ_OCXO + avg_offset_hz;                                       // Calculated the measured frequency of the OCXO
 
      // Here the OCXO correction processing is done if the number of samples in the cycle is met.
      if (sample_counter >= num_samples_per_cycle)                                        // Is it time to calculate the OCXO correction?
      {                                                                                   // Yes
        end_of_cycle = YES;                                                               // Indicate end of sampling cycle
        if (long_cycle)                                                                   // Long cycle?
        {                                                                                 // Yes
          // Calculation of the “Integral” component of the P-I loop at the end of the cycle
          // The offset averages are kept in an array. An average of the averages is taken at the end of the cycle
          avg_avg_offset_hz_array = 0;                                                    // Initialize the average of averages to zero.
          // Move existing averages in the average table by one position to make room for the new average
          for (index_avg_offset_hz_array = (sizeof(avg_offset_hz_array)/8 - 2); index_avg_offset_hz_array >= 0; index_avg_offset_hz_array--)
          {
            avg_offset_hz_array[index_avg_offset_hz_array + 1] = avg_offset_hz_array[index_avg_offset_hz_array];
          }
          avg_offset_hz_array[0] = avg_offset_hz;                                         // Save the new average in the table
          // Increment the counter by the number of averages existing in the table. Limit the value to the size of the array.
          if (max_index_avg_offset_hz_array < sizeof(avg_offset_hz_array)/8) max_index_avg_offset_hz_array++;          // 10 items in the average array
          // Calculate the mean of deviation means
          for (index_avg_offset_hz_array = 0; index_avg_offset_hz_array <= (max_index_avg_offset_hz_array-1); index_avg_offset_hz_array++)
          {
            avg_avg_offset_hz_array += avg_offset_hz_array[index_avg_offset_hz_array];    // Sum the averages
          }
          // Divide the result by the number of averages. Result: The average of averages
          avg_avg_offset_hz_array = double(avg_avg_offset_hz_array) / max_index_avg_offset_hz_array;  

          // Calculation of P-I compensation in Hz. If not in long cycle, Proportional only.
          offset_PI_loop_hz = p_index * avg_offset_hz + (long_cycle * i_index * avg_avg_offset_hz_array);  // The sum of the weighted influences P and I
        }
        // Proportional compensation only if in cycle other than long. Integral component not considered
        else offset_PI_loop_hz = avg_offset_hz;

        // Calculating the change in DAC value based on the calculated offset
        delta_dac = int(lround(double(offset_PI_loop_hz) * response_lsb_per_hz));          // The correction will be the opposite of the measured offset value.
        if (delta_dac > ((1<<num_dac_bits)-1)) delta_dac = ((1<<num_dac_bits)-1);          // Ensure that the correction fits within 16 signed bits
        if (delta_dac < -((1<<num_dac_bits)-1)) delta_dac = -((1<<num_dac_bits)-1);        //            "

        // If the FLL is active, calculate a new DAC value
        if (fll_active)                                                 // FLL active?
        {                                                               // Yes
          // Calculating the DAC value
           if (delta_dac > 0)                                           // positive shift average?
          { // Yes, make a negative correction
            if (delta_dac > dac_val)
            {
              dac_val = 0;
              bitSet(current_alarms, ALARM_DAC_LIMIT);
            }
            else 
            {
              dac_val -= delta_dac;                                     // Add the effect of delta_dac to the value of dac_val.
              bitClear(current_alarms, ALARM_DAC_LIMIT);
            }
          }
          else if (delta_dac < 0)                                       // negative shift average
          { // Make a positive correction
            if (-delta_dac > (((1<<num_dac_bits)-1) - dac_val))
            {
              dac_val = ((1<<num_dac_bits)-1);                               
              bitSet(current_alarms, ALARM_DAC_LIMIT);
            }
            else 
            {
              dac_val -= delta_dac;                                     // Add the effect of delta_dac to the value of dac_val.
              bitClear(current_alarms, ALARM_DAC_LIMIT);
            }
          }

          // Sending value to DACs
          if (delta_dac != 0)
          {
            Send_Val_Dac(dac_val);                                      // Sending the DAC value
          }
          new_dac_val = dac_val;                                        // 16-bit DAC value calculated for display
        }
        else   // FLL inactive, the DAC value does not change.
        {
          new_dac_val = dac_val;
          delta_dac = 0;
        }
      }
    }
    else  // A sample was rejected because it was too far from the stable trend. Definitely a sudden GPS deviation.
    {
      bitSet(current_alarms,ALARM_REJECTED_SAMPLE);                           // Notify of sample rejection
    }
    // Print the different calculation results to the serial port
    display_results_console();

    // Complete preparation for the next sampling cycle
    if (end_of_cycle)
    {
      // Selection of the length of the next sampling cycle based on the results of the completed cycle.
      if (long_cycle && (abs(avg_offset_hz) > thresh_long_cycle))             // Transition from long cycle to medium cycle?
      {
        num_samples_per_cycle = duration_medium_cycle;
        long_cycle = NO;
        medium_cycle = YES;
      }
      else if (medium_cycle && (abs(avg_offset_hz) > thresh_medium_cycle))    // Transition from medium cycle to short cycle?
      {
        num_samples_per_cycle = duration_short_cycle;
        medium_cycle = NO;
        short_cycle = YES;
       }
      else if (medium_cycle && (abs(avg_offset_hz) <= thresh_long_cycle))     // Transition from medium cycle to long cycle?
      {
        num_samples_per_cycle = duration_long_cycle;
        medium_cycle = NO;
        long_cycle = YES;
        bitClear(current_alarms,ALARM_ACQUISITION_INIT);                      // Clear initial acquisition alarm
        // Clear the average deviation table
        for (index_avg_offset_hz_array = 0; index_avg_offset_hz_array < sizeof(avg_offset_hz_array)/8; index_avg_offset_hz_array++)
            avg_offset_hz_array[index_avg_offset_hz_array] = 0;
        
        max_index_avg_offset_hz_array = 0;                                    // Reset the counter of the number of averages in the table
      }
      else if (short_cycle && (abs(avg_offset_hz) <= thresh_medium_cycle))    // Transition from short cycle to medium cycle?
      {
        num_samples_per_cycle = duration_medium_cycle;
        short_cycle = NO;
        medium_cycle = YES;
      }       

      display_results_lcd();                                                  // Display results on LCD
      
      // Re-initialize the different values to start a new sampling cycle.
      sample_counter = 0;                        
      avg_delta_osc_count = 0;                     
      sum_delta_osc_count = 0;
      begin_stabilization_millis = millis();
    }
    else display_results_lcd();                                               // Not at end of cycle: Display results on LCD
  }

  // Treatment of PPS pulsations (non-critical part)
  if (detection_pps)                                                          // PPS detected by interrupt function?
  {   // Yes
      detection_pps = NO;                                                     // Reset received PPS marker

      // Recovery after absence of PPS
      if (bitRead(current_alarms,ALARM_PPS_MISSING))                          // No PPS?
      {  // Yes
        bitClear(current_alarms, ALARM_PPS_MISSING);                          // Clear Missing PPS Alarm
        restart_acquisition();                                                // Restart a new acquisition cycle
      }
      digitalWrite(LED_BUILTIN, LOW);                                         // Light the LED to indicate reception of a PPS
      begin_led_millis = millis();                                            // Start of LED off cycle

      // Display the flashing character indicating receipt of a PPS, as well as the time of the PPS
      lcd.setCursor(19, 3);                                                   // position the cursor
      lcd.print('P');                                                         // Show PPS character
      lcd.setCursor(0, 3);                                                    // position the cursor
      lcd.print(date_string + "  " + hour_string);                            // Show time and date

      // Management of the display of the seconds counter during sampling in verbose mode
      if ((!end_of_cycle) && mode_verbose && (!mode_rx_command_uart))
      {
        Serial.print("_");                                          // Display PPS pulse number during sampling
        Serial.print(capture_counter);                              //            "
      }

      begin_detection_pps_millis = millis();                        // Reset PPS Detection Delay Count
  }

  // No PPS pulses received; non-critical work is completed.
  else  
  {  
    // Alarm management
    long_cycle ? bitClear(current_alarms,ALARM_FLL_UNLOCKED) : bitSet(current_alarms,ALARM_FLL_UNLOCKED);   // 
    if (!digitalRead(BUTTON_CLR_ALM)) past_alarms = 0;
    past_alarms = past_alarms | current_alarms;
    bitClear(current_alarms, ALARM_REJECTED_SAMPLE);     // Clear sample reject alarm to be ready for the next sample

  
    // PPS absence detection
    if (millis() - begin_detection_pps_millis > 1010)               // PPS delay exceeds one second?
    {                                                               // Yes
      begin_detection_pps_millis = millis();                        // Reset PPS Delay Count
      bitSet(current_alarms, ALARM_PPS_MISSING);                    // Enable Missing PPS Alarm
      display_results_console();                                    // Show results on serial port
    }
    
    // LED turns off after a short delay
    if (millis() - begin_led_millis >= 50)                          // Time to turn off the STM32 Blue Pill LED?
    {
      digitalWrite(LED_BUILTIN,HIGH);                               // Yes, turn off the LED
    }

    // Turning off the flashing marker on the LCD display
    if (millis() - begin_led_millis >= 400)                         // Time to clear the PPS indicator on the LCD display?
    {
      lcd.setCursor(19, 3);                                         // Yes, position the cursor
      lcd.print(" ");                                               // Show empty space above indicator
    }

    // End of a 5 second stabilization period at the end of the sampling cycle
    if (end_of_cycle && (millis() - begin_stabilization_millis >= 5000))  // // Time to complete the OCXO stabilization period
    {
      end_of_cycle = NO;                                            // Clear End of Cycle Marker
      reinitialize_counter = YES;                                   // Start a new cycle
    }

    // Management of the character string received from GPS to extract the UTC date and time
    while (Serial2.available() > 0)                                         // Loop as long as there are characters already received from GPS
    {
      charact_received = Serial2.read();                                   // Read a character
      gps_charact_string_received = gps_charact_string_received + charact_received;  // Add it to the character string already received
      if (charact_received == '\n')                                        // If the character is a newline, process the string
      {   // Yes
        // The expected format of the string is: $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
        if (gps_charact_string_received.substring(0, 6) == GPRMC_PREAMBLE)    // Compare the first 6 characters of the string with the desired GPS preamble...
        {
          if ((gps_charact_string_received.substring(7,8) == ",") || (gps_charact_string_received.substring(17,18) != "A"))     // ...and check that the string is not empty or invalid
          {   // Invalid data
            bitSet(current_alarms, ALARM_GPS_DATA_INVALID);                // Enable Invalid GPS Data Alarm
            valid_gps_data = NO;                                           // Lower valid data marker
            date_string = "--/--/--";                                      // Do not display date and time
            hour_string = "--:--:--";                                      //          "
          }
          else  // Otherwise, the GPS data is valid. Process the string
          {                                                                 
            valid_gps_data = YES;                                          // Notify that GPS data is valid
            bitClear(current_alarms, ALARM_GPS_DATA_INVALID);              // Clear Invalid GPS Data Alarm
            // Extracting the time
            hrs = gps_charact_string_received.substring(7,9).toInt();      // Extract the numeric value of the time
            mins = gps_charact_string_received.substring(9,11).toInt();    // Extract the numerical value of the minute
            secs = gps_charact_string_received.substring(11,13).toInt();   // Extract the numeric value of the second
            if (++secs == 60)                                              // ADD A SECOND since the GPS phrase comes after the PPS pulse. Then deal with the seconds overrun?
            {                                                              // Yes
              secs = 0;                                                    // Reset seconds
              if (++mins == 60)                                            // Then deal with the minute overrun?
              {                                                            // Yes
                mins = 0;                                                  // Reset minutes
                if (++hrs == 24)                                           // Then deal with overtime?
                {                                                          // Yes
                  hrs = 0;                                                 // Reset hours
                }
              }
            }
            hrs_string = String(hrs);                                      // Convert numeric value of hours to character string
            if (hrs < 10) hrs_string = "0" + hrs_string;                   // Add a leading zero if necessary
            mins_string = String(mins);                                    // Convert numeric value of minutes to character string
            if (mins < 10) mins_string = "0" + mins_string;                // Add a leading zero if necessary
            secs_string = String(secs);                                    // Convert numeric value of seconds to string
            if (secs < 10) secs_string = "0" + secs_string;                // Add a leading zero if necessary
            hour_string = hrs_string + ":" + mins_string + ":" + secs_string;  // Create the character string of the time to display
  
            //  Extracting the date
            unsigned short int b = 0;                                      // used as a comma counter
            unsigned short int c = 0;                                      // used as character counter
            leap_year = false;                                             // Initialize leap year marker as false.
            do                                                             // Locate the date field in the received GPS string.
            { 
              if (gps_charact_string_received[c++] == ',') b++;            // Increment the comma counter if one is detected
            } 
            while (b < 9);                                                      // Loop if necessary because the date is located after the 9th comma
            day_val = gps_charact_string_received.substring(c,c+2).toInt();     // Extract the character string representing the day
            month_val = gps_charact_string_received.substring(c+2,c+4).toInt(); // Extract the character string representing the month
            year_val = gps_charact_string_received.substring(c+4,c+6).toInt();  // Extract the string representing the year
            if (((year_val%4 == 0) && (year_val%100 != 0)) || (year_val%400 == 0)) leap_year = true;  // Check if the current year is a leap year?
            else leap_year = false;
            if ((secs == 0) && (mins == 0) && (hrs == 0))                       // Process change of day (date)?
            {                                                                   // Yes
              day_val++;                                                        // Increment day value
              if ((day_val > (28)) && !((leap_year) && (month_val == 2) && (day_val == 29))) // Process change of month? (taking into account leap years)
              {                                                                 // Yes
                month_val++;                                                    // Increment month value
                day_val = 1;                                                    // Reset day to 1
              }
              if (month_val > 12)                                               // Dealing with the change of year?
              {                                                                 // Yes
                year_val++;                                                     // Increment the year value
                month_val = 1;                                                  // Reset month to 1
                day_val = 1;                                                    // Reset day to 1
              }
            }
            day_string = String(day_val);                                       // Convert the numeric value of the day to a character string
            if (day_val < 10) day_string = "0" + day_string;                    // Add a leading zero if necessary
            month_string = String(month_val);                                   // Convert the numeric value of the month to a character string
            if (month_val < 10) month_string = "0" + month_string;              // Add a leading zero if necessary
            year_string = String(year_val);                                     // Convert numeric value of year to string
            date_string = day_string + "/" + month_string + "/" + year_string;  // Create the character string of the date to display
          }
        }
        else 
        {
          bitSet(current_alarms, ALARM_GPS_DATA_INVALID);               // Enable Invalid GPS Data Alarm
          valid_gps_data = NO;                                          // Lower valid data marker
          date_string = "--/--/--";                                     // Do not display date and time
          hour_string = "--:--:--";                                     //          "
        }
        gps_charact_string_received = "";                               // After receiving the end of sentence character and processing the string, delete the received character string
      }
    } 

    // Managing a command received on the serial port
    if (Serial.available() > 0)                                         // Character(s) received on the serial port?
    {
      mode_rx_command_uart = YES;                                       // Engage UART command reception mode
      begin_rx_command_millis = millis();                               // Mark the start of the command reception mode
      charact_received = Serial.read();                                 // Read a character from the UART
      if (charact_received_string == "") Serial.println();              // At the start of receiving an order? If yes, insert an empty line
      Serial.print(charact_received);                                   // Print received character
      if ((charact_received == 8) || (charact_received == 127))  charact_received_string.remove(charact_received_string.length()-1);          // Processing Backspace or Delete: erasing the character
      else if ((charact_received != '\n') && (charact_received != '\r')) charact_received_string += charact_received;  // Not the end of a command, append the character to the string.
      else                                                              // receipt of a complete order detected
      {
        // Separation of command and received parameters
        index1 = charact_received_string.indexOf(' ');                  // Finds location of first space,
        command = charact_received_string.substring(0, index1);         // Captures first data String
        command.toUpperCase();
        index2 = charact_received_string.indexOf(' ', index1+1 );       // Finds location of second space,
        param1 = charact_received_string.substring(index1+1, index2);   // Captures second data String
        param1.toUpperCase();
        index3 = charact_received_string.indexOf(' ', index2+1 );
        param2 = charact_received_string.substring(index2+1, index3);
        param2.toUpperCase();
        index4 = charact_received_string.indexOf(' ', index3+1 );
        param3 = charact_received_string.substring(index3+1, index4);
        param3.toUpperCase();

        // Processing the order received
        if (command == "RESET")                              // Command to restart the microcontroller program
        { 
          Serial.println("\nOK");
          NVIC_SystemReset();
        }
        else if (command == "REACQ")                         // Command to restart FLL acquisition
        { 
          Serial.println("\nOK");
          restart_acquisition();
        } 
        else if (command == "CLRALM")                        // Command to clear the log of previous alarms
        { 
          Serial.println("\nOK");
          past_alarms = 0;
        }
        else if (command == "FLL")                           // Command to enable or disable FLL
        {
          if (param1 == "ON") 
          {
            fll_active = YES;
            bitClear(current_alarms, ALARM_FLL_OFF);
            restart_acquisition();
            Serial.println("\nOK");
          }
          else if (param1 == "OFF")
          {
            fll_active = NO;     
            bitSet(current_alarms, ALARM_FLL_OFF);
            Serial.println("\nOK"); 
          }
          else Serial.println("\nERROR: invalid parameter");                   // Otherwise, parameter not recognized.
        }
        else if (command == "DAC")                                             // Command to force a FLL DAC value
        {
          if ((param1.toInt() >= 0) && (param1.toInt() < (1<<num_dac_bits)))   // Validate the value between 0 and 65535
          {
            dac_val = param1.toInt();
            Send_Val_Dac(dac_val);                                             // Sending the DAC value
            restart_acquisition();
            Serial.println("\nOK");
          }
          else Serial.println("\nERROR: invalid parameter");
        }
        else if (command == "DACBIT")                                          // Command to force a FLL DAC value
        {
          if ((param1.toInt() == 16) || (param1.toInt() == 14) || (param1.toInt() == 12))  // Validate the value between 0 and 65535
          {
            num_dac_bits = param1.toInt();
            save_eeprom_params();                 
            restart_acquisition();
            Serial.println("\nOK");
          }
          else Serial.println("\nERROR: invalid parameter");
        }
        else if (command == "CYCDUR")                                          // Command to change the duration of short, medium and long cycles (in order)
        {
          if ((param1.toInt() >= 1) && (param1.toInt() < 65536))               // Value Validation
          { 
            if ((param2.toInt() >= 1) && (param2.toInt() < 65536)) 
            {
              if ((param3.toInt() >= 1) && (param3.toInt() < 65536)) 
              {   // Acceptance of parameters only if all three values are validated
                duration_short_cycle = param1.toInt();                         // The number of samples of a short cycle
                duration_medium_cycle = param2.toInt();                        // The number of samples of an average cycle
                duration_long_cycle = param3.toInt();                          // The number of samples of a long cycle
                restart_acquisition();
                Serial.println("\nOK");
                save_eeprom_params();                 
              }
              else Serial.println("\nERROR: invalid parameters");
            }
            else Serial.println("\nERROR: invalid parameters");
          }  
          else Serial.println("\nERROR: invalid parameters");
        }
        else if (command == "NPPS")                                            // Command to change the number of PPS pulses in a sample
        {
          if ((param1.toInt() >= 1) && (param1.toInt() < 65536)) 
          {
            num_pps_per_sample = param1.toInt();
                                           
            // Calculation of the nominal count of a sample for a reference of exactly 10 MHz, since this parameter changes the residual value in the counter
            nominal_count = round((num_pps_per_sample * FREQ_OCXO / 65536 - trunc(num_pps_per_sample * FREQ_OCXO / 65536)) * 65536); 
            save_eeprom_params();
            Serial.println("\nOK");
          }
          else Serial.println("\nERROR: invalid parameter");
        }
        else if (command == "THRES")                                           // Command to change the deviation thresholds forcing the change of cycle type
        {
          if ((param1.toFloat() >= 0) && (param1.toFloat() < 100))             // Medium-short threshold
          {
            if ((param2.toFloat() >= 0) && (param2.toFloat() < 100))           // Long-medium threshold
            {
              thresh_medium_cycle = param1.toFloat();   // 
              thresh_long_cycle = param2.toFloat();   // 
              restart_acquisition();
              Serial.println("\nOK");
              save_eeprom_params();
            }
            else Serial.println("\nERROR: invalid parameters");
          }         
          else Serial.println("\nERROR: invalid parameters");
        }
        else if (command == "VERBOS")                                          // Command enabling verbosity mode on the UART serial port
        {
          if (param1 == "ON") 
          {
            mode_verbose = YES;
            Serial.println("\nOK");
            save_eeprom_params();
          }
          else if (param1 == "OFF") 
          {
            mode_verbose = NO;      
            Serial.println("\nOK");
            save_eeprom_params();    
          }
          else Serial.println("\nERROR: invalid parameter");
        }
        else if (command == "VT100")                                           // Command enabling VT-100 type display in verbosity mode on the UART serial port
        {
          if (param1 == "ON") 
          {
            mode_verbose_vt100 = YES;
            Serial.println("\nOK");
            save_eeprom_params();
          }
          else if (param1 == "OFF") 
          {
            mode_verbose_vt100 = NO;      
            Serial.println("\nOK");
            save_eeprom_params();
          }
          else Serial.println("\nERROR: invalid parameter");
        }        
        else if (command == "PARAM")                                           // Command displaying the values of adjustable system parameters
        {
          Serial.println(SEPAR_LINE);          
          Serial.println("Parameter Values");
          Serial.println("----------------");
          Serial.println("Number of DAC resolution bits:              " + String(num_dac_bits));
          Serial.println("DAC value:                                  " + String(dac_val));
          Serial.println("Vtune voltage at OCXO (calculated):         " + String((float(dac_val)/(1<<num_dac_bits)) * (DAC_VOLTAGE_MAX-DAC_VOLTAGE_MIN) * DAC_POST_GAIN,5));
          Serial.println("FLL operation:                              " + String(fll_active ? "ON" : "OFF"));
          Serial.println("Number of PPS per sample:                   " + String(num_pps_per_sample));
          Serial.println("Short cycle duration (samples):             " + String(duration_short_cycle));
          Serial.println("Medium cycle duration (samples):            " + String(duration_medium_cycle));
          Serial.println("Long cycle duration (samples):              " + String(duration_long_cycle));
          Serial.println("Medium cycle threshold (Hz):                " + String(thresh_medium_cycle,6));
          Serial.println("Long cycle threshold (Hz):                  " + String(thresh_long_cycle,6));
          Serial.println("Detailed display mode:                      " + String(mode_verbose ? "ON" : "OFF"));
          Serial.println("VT100 detailed display mode:                " + String(mode_verbose_vt100 ? "ON" : "OFF"));
          Serial.println("PI Loop Index Kp:                           " + String(p_index,2));
          Serial.println("PI Loop Index Ki:                           " + String(i_index,2));
          Serial.println(SEPAR_LINE);          
        }
        else if (command == "PI")                                             // Command allowing you to change the constants Kp and Ki of the PI ball
        {
          if ((param1.toFloat() >= 0) && (param1.toFloat() <= 1)) 
          {
            if ((param2.toFloat() >= 0) && (param2.toFloat() <= 1) && (param2.toFloat() + param2.toFloat() <= 1)) 
            {
              p_index = param1.toFloat();   // 
              i_index = param2.toFloat();   // 
              restart_acquisition();
              save_eeprom_params();
              Serial.println("\nOK");
            }
            else Serial.println("\nERROR: invalid parameters. Limits: Kp, Ki between 0 and 1   Kp + Ki <= 1");
          }         
          else Serial.println("\nERROR: invalid parameters");
        }
        else if ((command == "HELP") || (command == "?"))                     // Command displaying help with commands and alarms
        {
          Serial.println(SEPAR_LINE);          
          Serial.println("Commands:");
          Serial.println("----------");
          Serial.println("HELP or ?                            : This help.");
          Serial.println("DEFIN                                : Help for condensed display mode fields.");
          Serial.println("DAC <0-65535>                        : New forced DAC value.");
          Serial.println("DACBIT <16/14/12>                    : The number of bits used by the DACx0501.");
          Serial.println("CYCDUR <1-65535> <1-65535> <1-65535> : Number of samples per short/medium/long cycles.");
          Serial.println("CLRALM                               : Clears previous alarms shown in lowercase.");
          Serial.println("FLL <ON/OFF>                         : Enable FLL.");
          Serial.println("NPPS <1-10000>                       : Number of PPS per sample.");
          Serial.println("PARAM                                : List of adjustable parameters and their current value.");
          Serial.println("PI <0-1.0000> <0-1.0000>             : Constants Kp and Ki of the P-I loop of the FLL.");
          Serial.println("RESET                                : Restarts the microcontroller.");
          Serial.println("REACQ                                : Restarts the acquisition process.");
          Serial.println("THRES <0-100.0000> <0-100.0000>      : Thresholds (in Hz) for switching to medium and long cycles.");
          Serial.println("VERBOS <ON/OFF>                      : Detailed display mode.");
          Serial.println("VT100 <ON/OFF>                       : VT100 option of detailed display mode.");
          Serial.println("\nAlarms:");
          Serial.println("--------");
          Serial.println("Uppercase letters indicate currently active alarms.");
          Serial.println("Lowercase letters indicate previous alarms. Can be cleared with the CLRALM command.");
          Serial.println("A / a                                : In initial acquisition after startup.");
          Serial.println("L / l                                : The DAC has reached its minimum or maximum value (limit).");
          Serial.println("F / f                                : The FLL is off.");
          Serial.println("P / p                                : The GPS PPS signal is missing.");
          Serial.println("R / r                                : In long cycle, the sample is rejected because it is too far away.");
          Serial.println("V / v                                : The FLL is unlocked because it is not in a long cycle.");
          Serial.println("O / o                                : The OCXO does not provide a 10 MHz reference.");
          Serial.println("G / g                                : The GPS provides invalid data.");                
          Serial.println(WELC_MESSAGE);          
          Serial.println(SEPAR_LINE);          
        }
        else if (command == "DEFIN")                                                 // Command displaying help for condensed mode fields (non-verbose)
        {
          Serial.println("S|       a         |   b    |  c  |d| e   |  f  |  g   |    h   |    i   |  j   |\n");
          Serial.println("a: UTC date and time taken from GPS.");
          Serial.println("b: Alarm field.");
          Serial.println("c: Current value of the DAC (from 0 to 65535) controlling the OCXO frequency.");
          Serial.println("d: Current FLL cycle type (C=Short, M=Medium, L=Long).");
          Serial.println("e: Sample number in the current sample cycle.");
          Serial.println("f: Total number of samples to collect in the current cycle.");
          Serial.println("g: Average deviation in counter pulses from the nominal value.");
          Serial.println("h: Average deviation in counter pulses (g) expressed in Hertz.");
          Serial.println("i: Average frequency deviation of the P-I loop in a long cycle.");
          Serial.println("j: DAC change applied at the end of the current cycle.");          
          Serial.println(SEPAR_LINE);          
        }       
        else Serial.println("\nERROR: unrecognized command");                       // Otherwise, no command recognized
        charact_received_string = "";
        mode_rx_command_uart = NO;
        while (Serial.available()) Serial.read();
      }
    }

    // End of a command reception period on the UART port
    if (mode_rx_command_uart && ((millis() - begin_rx_command_millis) >= 10000))    // UART command mode time expired?
    {
      Serial.println("\r\nCommand Cancelled");                                      // Yes, notify on the console
      mode_rx_command_uart = NO;                                                    // Disable command mode
    }
  }
}
// End of source code
