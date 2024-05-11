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
 * - Ensure that the tonePin is correctly connected to the RTTY transmitter circuit.
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


// RTTY params

const int tonePin = 9;

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
            for (int i = 0; i < bits.length(); i++) {
              if (bits[i] == '1') {
                tone(tonePin, setParams.high);
              } else {
                tone(tonePin, setParams.low);
              }
              delay(1000 / setParams.baud);
            }
            noTone(tonePin);
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
 * - Sets the tone pin as an output.
 */
void setup()
{
    Serial.begin(9600);
    pinMode(tonePin, OUTPUT);
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
