/**
 * @file rtty-transmitter.ino
 *
 * @brief RTTY Transmitter with AT Command Parser
 *
 * @author Balazs Markus
 *
 * @details
 * This program serves as an RTTY (Radio Teletype) transmitter with an AT command parser
 * for controlling various parameters such as frequency and baud rate. It encodes text
 * into Baudot code and transmits it via RTTY modulation using a tone generator. The AT
 * command parser allows users to interact with the transmitter via serial communication,
 * enabling them to send messages, set frequency parameters, query current settings, and
 * reset to factory defaults.
 *
 * @note
 * - This program is designed to work with Arduino boards.
 * - Ensure that the DAC pins are correctly connected to the RTTY transmitter circuit.
 * - The baud rate for serial communication should match the baud rate set in the setup function.
 *
 * @par Usage:
 * - Connect the Arduino board to the computer via USB.
 * - Upload this sketch to the Arduino board.
 * - Open the serial monitor in the Arduino IDE or any other serial terminal program.
 * - Send AT commands to interact with the transmitter:
 *   - To send a message, use the AT+send command: AT+send="your_message"
 *   - To set parameters, use the AT+set command: AT+set=<parameter>,<value>
 *     Supported parameters: high, low, baud
 *   - To get parameter values, use the AT+get command: AT+get=<parameter>
 *     Supported parameters: high, low, baud
 *   - To reset to factory defaults, use the AT+reset command: AT+reset
 *   - For help with commands, use the AT+help command: AT+help
 * - Ensure that the RTTY receiver is tuned to the correct frequency to receive transmissions.
 */

#include <avr/pgmspace.h>
#include <SPI.h>
#include <avr/interrupt.h>
#include <avr/io.h>

// --- DDS / MCP4901 CONFIGURATION START ---
// These variables manage the high-speed sine wave generation
const int CS_PIN = 10;
const double SAMPLE_RATE = 80000.0;
volatile uint32_t phaseAccumulator = 0;
volatile uint32_t phaseIncrement = 0; // Controls the frequency
const byte CONFIG_HIGH = 0b00110000;  // MCP4901 Config

// Sine Lookup Table (256 values)
const uint8_t sineTable[256] = {
  128, 131, 134, 137, 140, 143, 146, 149, 152, 155, 158, 162, 165, 167, 170, 173,
  176, 179, 182, 185, 188, 190, 193, 196, 198, 201, 203, 206, 208, 211, 213, 215,
  218, 220, 222, 224, 226, 228, 230, 232, 234, 235, 237, 238, 240, 241, 243, 244,
  245, 246, 248, 249, 250, 250, 251, 252, 253, 253, 254, 254, 254, 255, 255, 255,
  255, 255, 255, 255, 254, 254, 254, 253, 253, 252, 251, 250, 250, 249, 248, 246,
  245, 244, 243, 241, 240, 238, 237, 235, 234, 232, 230, 228, 226, 224, 222, 220,
  218, 215, 213, 211, 208, 206, 203, 201, 198, 196, 193, 190, 188, 185, 182, 179,
  176, 173, 170, 167, 165, 162, 158, 155, 152, 149, 146, 143, 140, 137, 134, 131,
  128, 124, 121, 118, 115, 112, 109, 106, 103, 100, 97, 93, 90, 88, 85, 82,
  79, 76, 73, 70, 67, 65, 62, 59, 57, 54, 52, 49, 47, 44, 42, 40,
  37, 35, 33, 31, 29, 27, 25, 23, 21, 20, 18, 17, 15, 14, 12, 11,
  10, 9, 7, 6, 5, 5, 4, 3, 2, 2, 1, 1, 1, 0, 0, 0,
  0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 4, 5, 5, 6, 7, 9,
  10, 11, 12, 14, 15, 17, 18, 20, 21, 23, 25, 27, 29, 31, 33, 35,
  37, 40, 42, 44, 47, 49, 52, 54, 57, 59, 62, 65, 67, 70, 73, 76,
  79, 82, 85, 88, 90, 93, 97, 100, 103, 106, 109, 112, 115, 118, 121, 124
};
// --- DDS / MCP4901 CONFIGURATION END ---

const String baudotLTRS = "11111";
const String baudotFIGS = "11011";

/**
 * @brief Returns the Baudot code for a given character based on the mode (LTRS or FIGS). This function works similarly to a dict data structure.
 *
 * @param c The character for which Baudot code is to be retrieved.
 * @param mode The mode (either "LTRS" or "FIGS") for which the Baudot code is needed.
 * @return The Baudot code corresponding to the input character, or an empty string if the character was not found.
 */
String getBaudotCode(char c, String mode) {
    if (mode == "LTRS") {
        switch(c) {
            case 'A': return "11000";
            case 'B': return "10011";
            case 'C': return "01110";
            case 'D': return "10010";
            case 'E': return "10000";
            case 'F': return "10110";
            case 'G': return "01011";
            case 'H': return "00101";
            case 'I': return "01100";
            case 'J': return "11010";
            case 'K': return "11110";
            case 'L': return "01001";
            case 'M': return "00111";
            case 'N': return "00110";
            case 'O': return "00011";
            case 'P': return "01101";
            case 'Q': return "11101";
            case 'R': return "01010";
            case 'S': return "10100";
            case 'T': return "00001";
            case 'U': return "11100";
            case 'V': return "01111";
            case 'W': return "11001";
            case 'X': return "10111";
            case 'Y': return "10101";
            case 'Z': return "10001";
            case '\n': return "01000";
            case ' ': return "00100";
            case '\r': return "00010";
            default: return "";
        }
    } else if (mode == "FIGS") {
        switch(c) {
            case '-': return "11000";
            case '?': return "10011";
            case ':': return "01110";
            case '$': return "10010";
            case '3': return "10000";
            case '!': return "10110";
            case '&': return "01011";
            case '#': return "00101";
            case '8': return "01100";
            case '4': return "01010";
            case '(': return "11110";
            case ')': return "01001";
            case '.': return "00111";
            case ',': return "00110";
            case '9': return "00011";
            case '0': return "01101";
            case '1': return "11101";
            case '\'': return "01010";
            case '5': return "00001";
            case '7': return "11100";
            case ';': return "01111";
            case '2': return "11001";
            case '/': return "10111";
            case '6': return "10101";
            case '\"': return "10001";
            case '\n': return "01000";
            case ' ': return "00100";
            case '\r': return "00010";
            default: return "";
        }
    }
    return "";
}

/**
 * @brief Generates a RTTY-compatible 5-bit Baudot-encoded string including start and stop bits from the given text string.
 *
 * @param text The text to be encoded into Baudot format.
 * @return The complete RTTY-compatible binary string that contains 0-s and 1-s, to be used by the transmission for loop.
 */
String generateBaudotString(String text) {
    String currentMode = getBaudotCode(text[0], "LTRS") != "" ? "LTRS" : "FIGS";

    String start_bit = "0";
    String stop_bit = "11";

    String bits = "";

    // Send CR-LF
    bits += start_bit + getBaudotCode('\r', "LTRS") + stop_bit;
    bits += start_bit + getBaudotCode('\n', "LTRS") + stop_bit;

    // Send initial character set
    if (currentMode == "LTRS") {
        bits += start_bit + baudotLTRS + stop_bit;
    } else {
        bits += start_bit + baudotFIGS + stop_bit;
    }

    for (int i = 0; i < text.length(); i++) {
        char currentChar = text[i];

        if (getBaudotCode(currentChar, "LTRS") != "") {
            if (currentMode != "LTRS") {
                bits += start_bit + baudotLTRS + stop_bit; // Change to letters mode if not already
                currentMode = "LTRS";
            }
            bits += start_bit + getBaudotCode(currentChar, "LTRS") + stop_bit;
        } else if (getBaudotCode(currentChar, "FIGS") != "") {
            if (currentMode != "FIGS") {
                bits += start_bit + baudotFIGS + stop_bit; // Change to figures mode if not already
                currentMode = "FIGS";
            }
            bits += start_bit + getBaudotCode(currentChar, "FIGS") + stop_bit;
        }
    }

    return bits;
}

// AT COMMAND PARSER BEGIN

/**
 * @brief Maximum length of the buffer for incoming commands.
 */
#define BUF_LENGTH 128  /* Buffer for the incoming command. */

/**
 * @brief Flag indicating whether to echo back received characters.
 */
static bool do_echo = true;

/**
 * @brief Struct to store set parameters for RTTY communication.
 */
struct SetParams {
    int high;
    int low;
    float baud;
} setParams = {2125, 2295, 45.45};  // Initialize with default values

// --- EEPROM bit-bang definitions on PORTC (A0..A5) ---
// A0 PC0 -> EE /CS
// A3 PC3 -> EE SI (MOSI) (we will drive this for future writes)
// A4 PC4 -> EE SCK
// A5 PC5 -> EE SO (MISO)
#define EE_CS_LOW()  PORTC &= ~_BV(PC0)
#define EE_CS_HIGH() PORTC |=  _BV(PC0)
#define EE_SCK_LOW() PORTC &= ~_BV(PC4)
#define EE_SCK_HIGH() PORTC |=  _BV(PC4)
#define EE_SI_HIGH() PORTC |=  _BV(PC3)
#define EE_SI_LOW()  PORTC &= ~_BV(PC3)
#define EE_SO_READ() ((PINC & _BV(PC5))?1:0)

// EEPROM playback globals
volatile uint32_t ee_sample_len = 0;
volatile uint32_t ee_sample_count = 0;
volatile uint8_t ee_sample = 0;
volatile bool playback_active = false;
volatile uint8_t saved_TIMSK1 = 0;

// Counter for end-of-audio detection
volatile int consecutive_ff_count = 0;
// Threshold: 250 samples @ 8kHz is approx 31ms. 
// It is extremely unlikely real audio stays at 255 for 31ms.
#define FF_THRESHOLD 250 

// send 32-bit command+addr (MSB first) via bit-bang
void ee_set_addr(uint32_t cmd_addr) {
  EE_CS_LOW();
  // send 32 bits MSB first
  for (uint8_t i = 0; i < 32; i++) {
    if (cmd_addr & 0x80000000UL) EE_SI_HIGH(); else EE_SI_LOW();
    EE_SCK_HIGH();
    EE_SCK_LOW();
    cmd_addr <<= 1;
  }
}

// read next byte (MSB first) via bit-bang
uint8_t ee_read_byte_bb() {
  uint8_t d = 0;
  for (uint8_t i = 0; i < 8; i++) {
    d <<= 1;
    EE_SCK_HIGH();
    if (EE_SO_READ()) d |= 1;
    EE_SCK_LOW();
  }
  return d;
}

// Prepare read from the absolute beginning
void start_eeprom_stream() {
  uint32_t cmd_addr = (0x03UL << 24) | 0x000000UL;
  ee_set_addr(cmd_addr);
  // CS stays LOW here, ready to clock out bytes in ISR
}

/**
 * @brief Executes a complete AT command.
 *
 * @param cmdline The command line to be executed.
 */
static void exec(char *cmdline)
{
    char *command = strsep(&cmdline, "=");

    if (strcmp_P(command, PSTR("AT+send")) == 0) {
        char *message = strstr(cmdline, "\"");
        if (message != NULL) {
            Serial.print(F("Send command with message: "));
            message++;  // Skip the opening quote
            message[strlen(message) - 1] = '\0'; // Remove the closing quote
            Serial.println(message);
            // Convert to upper case
            char *s = message;
            while (*s) {
              *s = toupper((unsigned char) *s);
              s++;
            }
            String bits = generateBaudotString(message);

            // --- DDS CALCULATION START ---
            // We pre-calculate the phase increments for the requested frequencies             // This allows the high/low frequencies to be dynamic
            uint32_t incHigh = (uint32_t)((setParams.high * 4294967296.0) / SAMPLE_RATE);
            uint32_t incLow = (uint32_t)((setParams.low * 4294967296.0) / SAMPLE_RATE);
            // --- DDS CALCULATION END ---

            for (int i = 0; i < bits.length(); i++) {
              if (bits[i] == '1') {
                phaseIncrement = incHigh; // Set DDS generator to high freq
              } else {
                phaseIncrement = incLow; // Set DDS generator to low freq
              }
              delay(1000 / setParams.baud);
            }
            phaseIncrement = 0; // Stop the wave generation
            Serial.println("OK");
        } else {
            Serial.println(F("Error: Invalid syntax for AT+send command."));
        }
    } else if (strcmp_P(command, PSTR("AT+set")) == 0) {
        char *param = strsep(&cmdline, ",");
        float value = atof(cmdline);
        if (strcmp_P(param, PSTR("high")) == 0) {
            setParams.high = (int)value;
            Serial.print(F("Set high frequency to: "));
            Serial.println(setParams.high);
            Serial.println(F("OK"));
        } else if (strcmp_P(param, PSTR("low")) == 0) {
            setParams.low = (int)value;
            Serial.print(F("Set low frequency to: "));
            Serial.println(setParams.low);
            Serial.println(F("OK"));
        } else if (strcmp_P(param, PSTR("baud")) == 0) {
            setParams.baud = value;
            Serial.print(F("Set baud rate to: "));
            Serial.println(setParams.baud);
            Serial.println(F("OK"));
        } else {
            Serial.println(F("Error: Invalid parameter for AT+set command."));
        }
    } else if (strcmp_P(command, PSTR("AT+get")) == 0) {
        char *param = cmdline;
        if (strcmp_P(param, PSTR("low")) == 0) {
            Serial.print(F("Get value of parameter low: "));
            Serial.println(setParams.low);
            Serial.println(F("OK"));
        } else if (strcmp_P(param, PSTR("high")) == 0) {
            Serial.print(F("Get value of parameter high: "));
            Serial.println(setParams.high);
            Serial.println(F("OK"));
        } else if (strcmp_P(param, PSTR("baud")) == 0) {
            Serial.print(F("Get value of parameter baud: "));
            Serial.println(setParams.baud);
            Serial.println(F("OK"));
        } else {
            Serial.println(F("Error: Invalid parameter for AT+get command."));
        }
    } else if (strcmp_P(command, PSTR("AT+reset")) == 0) {
        setParams.high = 2125;
        setParams.low = 2295;
        setParams.baud = 45.45;
        Serial.println(F("OK"));
    } else if (strcmp_P(command, PSTR("AT+help")) == 0) {
        Serial.println(F(
            "AT+send=\"message\": Send message\r\n"
            "AT+set=high,<frequency>: Set high frequency\r\n"
            "AT+set=low,<frequency>: Set low frequency\r\n"
            "AT+set=baud,<baud_rate>: Set baud rate\r\n"
            "AT+get=<parameter>: Get parameter value\r\n"
            "AT+reset: Reset device to factory settings\r\n"
            "AT+help: Show this help"));
        Serial.println(F("OK"));
    } else if (strcmp_P(command, PSTR("AT+play")) == 0) {
        // 1. Initialize EEPROM read pointer to 0 immediately
        start_eeprom_stream();
        
        // 2. Pre-load first byte
        ee_sample = ee_read_byte_bb();
        consecutive_ff_count = 0;

        // 3. Save RTTY timer state and disable it
        saved_TIMSK1 = TIMSK1;
        TIMSK1 = 0;

        // 4. Configure Timer2 for 8kHz Playback
        cli();
        TCCR2A = (1 << WGM21); // CTC
        TCCR2B = (1 << CS21);  // prescaler 8
        OCR2A = 249;           // (16MHz/8)/8000 - 1 = 249
        TCNT2 = 0;
        TIMSK2 |= (1 << OCIE2A); // enable Timer2 compare interrupt
        sei();

        // 5. Block until playback finishes (detected in ISR)
        playback_active = true;
        while(playback_active);
        
        Serial.println(F("OK"));
    } else if (strcmp_P(command, PSTR("AT")) == 0) {
        Serial.println(F("OK"));
    } else {
        Serial.print(F("Error: Unknown command: "));
        Serial.println(command);
    }
}

/**
 * @brief Setup function called once when the program starts.
 *
 * @details
 * - Initializes serial communication.
 * - Initializes DAC output.
 */
void setup()
{
    Serial.begin(9600);
    // --- MCP4901 & TIMER INIT ---
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    SPI.begin();
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
    
    // Configure Timer 1 for 80kHz Interrupt
    cli(); 
    TCCR1A = 0; 
    TCCR1B = 0; 
    TCNT1  = 0; 
    OCR1A = 199; 
    TCCR1B |= (1 << WGM12); // CTC mode
    TCCR1B |= (1 << CS10);  // No prescaling
    TIMSK1 |= (1 << OCIE1A); // Enable interrupt
    sei();
    // --- END MCP4901 INIT ---
    // --- PORTC setup for EEPROM bit-bang ---
    // PC0 CS output, PC3 SI output, PC4 SCK output, PC5 SO input
    DDRC |= _BV(PC0) | _BV(PC3) | _BV(PC4);
    DDRC &= ~_BV(PC5);
    // set idle states
    PORTC |= _BV(PC0); // CS high
    PORTC &= ~_BV(PC3); // SI low
    PORTC &= ~_BV(PC4); // SCK low
    // --- END PORTC setup for EEPROM bit-bang ---
}

/**
 * @brief The main execution loop that continuously processes incoming commands.
 *
 * @details
 * - Reads incoming serial data.
 * - Processes incoming commands.
 */
void loop() {
    /* Process incoming commands. */
    while (Serial.available()) {
        static char buffer[BUF_LENGTH];
        static int length = 0;

        int data = Serial.read();
        if (data == '\b' || data == '\177') {  // BS and DEL
            if (length) {
                length--;
                if (do_echo) Serial.write("\b \b");
            }
        }
        else if (data == '\r') {
            if (do_echo) Serial.write("\r\n");    // output CRLF
            buffer[length] = '\0';
            if (length) exec(buffer);
            length = 0;
        }
        else if (length < BUF_LENGTH - 1) {
            buffer[length++] = data;
            if (do_echo) Serial.write(data);
        }
    }

    /* Whatever else needs to be done... */
}

// --- High Speed DAC Interrupt ---
ISR(TIMER1_COMPA_vect) {
  // 1. Update Phase
  phaseAccumulator += phaseIncrement;
  
  // 2. Get Sine Value
  byte sineValue = sineTable[phaseAccumulator >> 24];

  // 3. Send to MCP4901
  PORTB &= ~_BV(2); // CS Low (Pin 10)

  // Send High Byte
  SPDR = CONFIG_HIGH | (sineValue >> 4); 
  while (!(SPSR & _BV(SPIF))); 
  
  // Send Low Byte
  SPDR = (sineValue << 4); 
  while (!(SPSR & _BV(SPIF))); 

  PORTB |= _BV(2); // CS High
}

// --- Timer2 ISR for 8kHz playback (Audio) ---
ISR(TIMER2_COMPA_vect) {
  // 1. Output current sample to DAC
  uint8_t high = CONFIG_HIGH | (ee_sample >> 4);
  uint8_t low  = (ee_sample << 4);
  PORTB &= ~_BV(2); // DAC CS low
  SPDR = high; while(!(SPSR & _BV(SPIF)));
  SPDR = low;  while(!(SPSR & _BV(SPIF)));
  PORTB |= _BV(2); // DAC CS high

  // 2. Read NEXT byte from EEPROM (Bit-Bang)
  // We assume CS is held LOW by the setup function
  uint8_t next = 0;
  for (uint8_t i = 0; i < 8; i++) {
    next <<= 1;
    EE_SCK_HIGH();
    if (EE_SO_READ()) next |= 1;
    EE_SCK_LOW();
  }
  ee_sample = next;

  // 3. Check for End of Audio (Consecutive 0xFF)
  if (ee_sample == 0xFF) {
      consecutive_ff_count++;
  } else {
      consecutive_ff_count = 0;
  }

  // 4. Stop if threshold reached
  if (consecutive_ff_count >= FF_THRESHOLD) {
    EE_CS_HIGH(); // Release EEPROM
    TIMSK2 &= ~(1 << OCIE2A); // Stop Timer2
    playback_active = false; // Signal loop to continue
    TIMSK1 = saved_TIMSK1; // Restore RTTY Timer
  }
}
