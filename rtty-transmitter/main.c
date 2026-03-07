/**
 * @file        main.c
 * @brief       RTTY/AM Radio Transmitter Firmware
 * @version     v1.0.0
 * @author      Balazs Markus
 *
 * =============================================================================
 * PROJECT OVERVIEW
 * =============================================================================
 *
 * This firmware drives an ATmega328P-based RTTY (Radio Teletype) transmitter
 * card. The board generates FSK (Frequency-Shift Keying) audio tones via a
 * Direct Digital Synthesis (DDS) engine, feeding an MCP4901 8-bit SPI DAC,
 * whose output is amplified by a two-stage BJT amplifier (Q1/Q2) and fed into
 * a tank circuit / RF output stage (L1/L2, Q3). A 25LC640A SPI EEPROM (U4)
 * is used to store an 8-bit PCM audio sample for AM-mode audio playback. The
 * board communicates with a host over a CH340C USB-UART bridge (U1) at 9600
 * baud and is controlled entirely through an AT command interface.
 *
 * =============================================================================
 * HARDWARE SUMMARY (derived from KiCad netlist rtty-transmitter.net)
 * =============================================================================
 *
 *  MCU         : ATmega328P (U2), 16 MHz crystal (Y1)
 *  USB-UART    : MCP2221A (U1) -  UART RX->PD0, TX->PD1
 *  DAC         : MCP4901 (U3) -  hardware SPI (PB5=SCK, PB3=MOSI, PB2=/CS)
 *  EEPROM      : 25LC1024 (U4)-  bit-bang SPI on PORTC (131072 bytes, 17-bit addr):
 *                    PC0 = /CS   (output)
 *                    PC3 = MOSI  (output)
 *                    PC4 = SCK   (output)
 *                    PC5 = MISO  (input)
 *  Status LED  : PD7 via R6   (output, active-high)
 *  GPIO header : PD2..PD6     (J3, user-accessible outputs)
 *  ISP header  : J2            (RESET, SCK, MISO, MOSI - for USBasp)
 *
 * =============================================================================
 * SIGNAL CHAIN
 * =============================================================================
 *
 *  DDS engine (Timer1 ISR, 80 kHz) --> MCP4901 DAC (SPI)
 *      --> R12 --> op-amp buffer (U5A) --> BJT driver (Q2/Q1)
 *      --> tank circuit (L1, C10) --> RF output (J7)
 *
 *  Audio playback (Timer2 ISR, 8 kHz):
 *      25LC640A EEPROM --> bit-bang read --> MCP4901 DAC
 *
 * =============================================================================
 * AT COMMAND INTERFACE (9600 8N1)
 * =============================================================================
 *
 *  AT                        - Echo / connection test
 *  AT+send="<message>"       - Encode and transmit a Baudot/RTTY message
 *  AT+set=<param>,<value>    - Set a runtime parameter
 *                              params: high (Hz), low (Hz), baud (45.45 etc.)
 *  AT+get=<param>            - Read back a parameter value
 *                              params: high, low, baud
 *  AT+write=<pin>,<value>    - Drive a GPIO pin (pin 0-5 -> PD2-PD7, val 0/1)
 *  AT+play                   - Play 8-bit PCM audio stored in EEPROM
 *  AT+upload                 - Write raw hex bytes to EEPROM from UART stream
 *                              (stream ends on ~1 s idle timeout or EEPROM full)
 *  AT+reset                  - Restore factory defaults (high/low/baud)
 *  AT+help                   - Print command reference
 *
 * =============================================================================
 * TIMER ALLOCATION
 * =============================================================================
 *
 *  Timer1 (16-bit, CTC, OCR1A=199, no prescaler) -> 80 kHz DDS ISR
 *      Drives the MCP4901 with a sine-table sample every 12.5 µs.
 *      phaseIncrement == 0 means carrier is muted (between transmissions).
 *
 *  Timer2 (8-bit, CTC, OCR2A=249, prescaler /8)  -> 8 kHz audio playback ISR
 *      Active only during AT+play. Timer1 is suspended for the duration.
 *
 * @note
 *  - Build with avr-gcc; set F_CPU=16000000UL and target=atmega328p.
 *  - Flash with avrdude + USBasp: avrdude -c usbasp -p m328p -U flash:w:main.hex
 *  - The EEPROM upload protocol transmits pairs of ASCII hex digits (e.g.
 *    "A3 4F 00 ...") optionally separated by spaces/newlines. The session ends
 *    automatically after UPLOAD_IDLE_MS milliseconds of inactivity.
 */

/* ============================================================================
 * HARDWARE CLOCK - must come before any AVR library header that reads F_CPU
 * ============================================================================
 *
 * The board has a 16 MHz crystal (Y1) with no clock prescaler (CLKDIV8 fuse
 * unprogrammed).  Defining F_CPU here, in the source file, means the value is
 * always correct regardless of IDE or build-system settings - the hardware is
 * soldered down and will never change.  The #ifndef guard prevents a conflict
 * if the toolchain also passes -DF_CPU on the command line.
 *
 * <util/delay.h> requires F_CPU to be defined before it is #included so that
 * _delay_ms() computes the correct busy-loop counts at compile time.
 * ============================================================================ */
#ifndef F_CPU
#define F_CPU 16000000UL      /**< External 16 MHz crystal, no prescaler        */
#endif

/* ============================================================================
 * INCLUDES
 * ============================================================================ */
#include <avr/io.h>           /* AVR register and bit definitions              */
#include <avr/interrupt.h>    /* sei() / cli() and ISR() macro                 */
#include <avr/pgmspace.h>     /* PSTR() / pgm_read_byte() - keep strings in Flash */
#include <util/delay.h>       /* _delay_ms() - blocking millisecond delays     */
#include <stdint.h>           /* uint8_t, uint16_t, uint32_t                   */
#include <stdbool.h>          /* bool, true, false                              */
#include <string.h>           /* strcmp(), strtok_r(), strlen(), etc.           */
#include <stdlib.h>           /* atoi(), atof()                                 */
#include <ctype.h>            /* toupper(), isxdigit()                          */


/* ============================================================================
 * FIRMWARE METADATA
 * ============================================================================ */
#define FW_VERSION   "v1.0.0"         /**< Firmware version string             */
#define DEVICE_NAME  "RTTY-TX"        /**< Device name shown in the banner      */


/* ============================================================================
 * UART HELPERS  (9600 baud, 16 MHz crystal)
 * ============================================================================
 *
 * Register values are hardcoded as literals to match exactly what the Arduino
 * core's Serial.begin(9600) writes on an ATmega328P at 16 MHz.  This removes
 * any dependency on F_CPU being correctly defined in the build system and
 * guarantees identical UART behaviour to the original Arduino firmware.
 *
 * Arduino HardwareSerial::begin(9600) sequence (verified from Arduino source):
 *
 *   UCSR0A = 0x02   - U2X0=1 (double-speed, /8 divisor), all flags cleared
 *   UBRR0H = 0x00   - baud divisor high byte  (207 >> 8 = 0)
 *   UBRR0L = 0xCF   - baud divisor low byte   (207 = 0xCF)
 *   UCSR0B = 0x98   - RXCIE0|RXEN0|TXEN0 (Arduino uses RX interrupt)
 *   UCSR0C = 0x06   - async, no parity, 1 stop bit, 8-bit data (8N1)
 *
 * We use 0x18 for UCSR0B instead of 0x98: RXCIE0 is omitted because this
 * firmware polls RXC0 directly in the main loop.  Enabling RXCIE0 without a
 * corresponding ISR(USART_RX_vect) handler causes the AVR to jump to the
 * default (reset) vector on every received byte, making RX completely silent.
 *
 * With U2X0=1 and UBRR=207:  actual baud = 16 MHz / (8 × 208) = 9615 Bd (0.16%)
 * ============================================================================ */

/**
 * @brief Initialise USART0 for 9600 8N1 using the exact register values that
 *        Arduino's Serial.begin(9600) writes on a 16 MHz ATmega328P, except
 *        RXCIE0 which is cleared because RX is handled by polling RXC0.
 *
 * Hardcoded literals are used instead of computed constants so that the result
 * is independent of how (or whether) F_CPU is defined in the build system.
 */
static void uart_init(void)
{
    UCSR0A = 0x02;   /* U2X0=1: double-speed mode, clears TXC0/FE0/DOR0/UPE0  */
    UBRR0H = 0x00;   /* Baud divisor high byte = 0                              */
    UBRR0L = 0xCF;   /* Baud divisor low byte  = 207 -> 9600 baud @ 16 MHz      */
    UCSR0B = 0x18;   /* RXEN0=1, TXEN0=1  (no RXCIE0 - RXC0 is polled directly) */
    UCSR0C = 0x06;   /* UCSZ01=1, UCSZ00=1 -> async 8N1                         */
}

/**
 * @brief Block until UDRE0 is set, then write one character to UART.
 *
 * @param c  The byte to transmit.
 */
static void uart_putc(char c)
{
    while (!(UCSR0A & (1 << UDRE0)));            /* Wait for empty TX register  */
    UDR0 = (uint8_t)c;                           /* Load character into buffer  */
}

/**
 * @brief Transmit a NUL-terminated string to UART, one character at a time.
 *
 * @param s  Pointer to the string in RAM.
 */
static void uart_puts(const char *s)
{
    while (*s) uart_putc(*s++);                  /* Send each char until NUL    */
}

/**
 * @brief Transmit a NUL-terminated string stored in program (Flash) memory.
 *
 * @param s  PROGMEM pointer obtained via PSTR().
 */
static void uart_puts_P(const char *s)
{
    char c;
    while ((c = (char)pgm_read_byte(s++))) uart_putc(c);  /* Read from Flash   */
}

/**
 * @brief Return non-zero when a byte is available in the UART RX buffer.
 */
static inline uint8_t uart_available(void)
{
    return (UCSR0A & (1 << RXC0));               /* RXC0 set when byte arrived  */
}

/**
 * @brief Read one byte from the UART RX register (blocking).
 *
 * @return The received byte.
 */
static inline uint8_t uart_getc(void)
{
    while (!uart_available());                   /* Block until data arrives    */
    return UDR0;
}

/**
 * @brief Non-blocking attempt to read one byte.
 *
 * @param[out] out  Filled with the received byte when available.
 * @return true if a byte was read, false if the RX buffer was empty.
 */
static inline bool uart_try_getc(uint8_t *out)
{
    if (!uart_available()) return false;
    *out = UDR0;
    return true;
}

/**
 * @brief Format and print a signed 16-bit integer over UART.
 *
 * Implemented without snprintf / printf to avoid pulling in the heavyweight
 * avr-libc formatted I/O library (~1?2 kB extra Flash).
 *
 * @param v  The value to print.
 */
static void uart_print_int(int16_t v)
{
    char buf[7];                                  /* "-32768" + NUL = 7 chars    */
    uint8_t i = sizeof(buf) - 1;
    bool neg = (v < 0);
    uint16_t u = neg ? (uint16_t)(-(int32_t)v) : (uint16_t)v;
    buf[i] = '\0';
    do {
        buf[--i] = (char)('0' + u % 10);
        u /= 10;
    } while (u);
    if (neg) buf[--i] = '-';
    uart_puts(&buf[i]);
}

/**
 * @brief Format and print a float with two decimal places over UART.
 *
 * Implemented without snprintf to avoid avr-libc printf float support
 * which adds ~1.5 kB to Flash.  Only handles positive values up to 9999.99.
 *
 * @param v  The value to print (must be positive).
 */
static void uart_print_float(float v)
{
    uint16_t whole = (uint16_t)v;
    uint8_t  frac  = (uint8_t)((v - whole) * 100.0f + 0.5f);  /* 2 decimal places */
    uart_print_int((int16_t)whole);
    uart_putc('.');
    if (frac < 10) uart_putc('0');               /* Leading zero for frac < 10  */
    uart_print_int((int16_t)frac);
}

/** Convenience macro: send CRLF line ending */
#define uart_crlf()  uart_puts("\r\n")


/* ============================================================================
 * STATUS LED  (PD7, active-high, cathode to GND via R6)
 * ============================================================================ */
#define LED_DDR   DDRD
#define LED_PORT  PORTD
#define LED_PIN   PD7

/** Drive the status LED on. This overrides any user-set value on that pin. */
#define LED_ON()  (LED_PORT |=  (1 << LED_PIN))
/** Drive the status LED off. This overrides any user-set value on that pin. */
#define LED_OFF() (LED_PORT &= ~(1 << LED_PIN))


/* ============================================================================
 * DDS / MCP4901 CONFIGURATION
 * ============================================================================ */
#define SAMPLE_RATE    80000.0          /**< Timer1 ISR rate in Hz             */
#define CONFIG_HIGH    0x30U            /**< MCP4901 config: active, 1x gain   */

/** Hardware SPI chip-select for MCP4901 is PB2 (Arduino D10). */
#define DAC_CS_PORT  PORTB
#define DAC_CS_PIN   PB2

volatile uint32_t phaseAccumulator = 0;  /**< DDS phase accumulator (32-bit)  */
volatile uint32_t phaseIncrement   = 0;  /**< DDS step; 0 = carrier muted     */
volatile uint16_t rtty_bit_ticks   = 0;  /**< Countdown timer for RTTY bit period, decremented by Timer1 ISR */

/**
 * @brief Sine look-up table - 256 entries, one full cycle, unsigned 8-bit.
 *
 * Generated as: round(127.5 * (1 + sin(2*pi*i/256))) for i in 0..255.
 * Values range 0..255 with midpoint at 128.
 */
/* Stored in SRAM (no PROGMEM) so the Timer1 ISR can access it with a plain
 * array index - identical to the Arduino sketch.  Uses 256 bytes of SRAM.   */
static const uint8_t sineTable[256] = {
    128, 131, 134, 137, 140, 143, 146, 149, 152, 155, 158, 162, 165, 167, 170, 173,
    176, 179, 182, 185, 188, 190, 193, 196, 198, 201, 203, 206, 208, 211, 213, 215,
    218, 220, 222, 224, 226, 228, 230, 232, 234, 235, 237, 238, 240, 241, 243, 244,
    245, 246, 248, 249, 250, 250, 251, 252, 253, 253, 254, 254, 254, 255, 255, 255,
    255, 255, 255, 255, 254, 254, 254, 253, 253, 252, 251, 250, 250, 249, 248, 246,
    245, 244, 243, 241, 240, 238, 237, 235, 234, 232, 230, 228, 226, 224, 222, 220,
    218, 215, 213, 211, 208, 206, 203, 201, 198, 196, 193, 190, 188, 185, 182, 179,
    176, 173, 170, 167, 165, 162, 158, 155, 152, 149, 146, 143, 140, 137, 134, 131,
    128, 124, 121, 118, 115, 112, 109, 106, 103, 100,  97,  93,  90,  88,  85,  82,
     79,  76,  73,  70,  67,  65,  62,  59,  57,  54,  52,  49,  47,  44,  42,  40,
     37,  35,  33,  31,  29,  27,  25,  23,  21,  20,  18,  17,  15,  14,  12,  11,
     10,   9,   7,   6,   5,   5,   4,   3,   2,   2,   1,   1,   1,   0,   0,   0,
      0,   0,   0,   0,   1,   1,   1,   2,   2,   3,   4,   5,   5,   6,   7,   9,
     10,  11,  12,  14,  15,  17,  18,  20,  21,  23,  25,  27,  29,  31,  33,  35,
     37,  40,  42,  44,  47,  49,  52,  54,  57,  59,  62,  65,  67,  70,  73,  76,
     79,  82,  85,  88,  90,  93,  97, 100, 103, 106, 109, 112, 115, 118, 121, 124
};

/**
 * @brief Write one 12-bit word to the MCP4901 DAC over hardware SPI.
 *
 * The MCP4901 accepts a 16-bit frame: 4 config bits + 8 data bits + 4 ignored.
 * We send two bytes manually through SPDR to avoid function-call overhead inside
 * the 80 kHz ISR.
 *
 * @param sample  8-bit PCM value (0-255).
 */
static inline void dac_write(uint8_t sample)
{
    DAC_CS_PORT &= ~(1 << DAC_CS_PIN);           /* Assert /CS (active low)     */
    SPDR = CONFIG_HIGH | (sample >> 4);          /* High nibble + config bits   */
    while (!(SPSR & (1 << SPIF)));               /* Wait for SPI transfer done  */
    SPDR = (uint8_t)(sample << 4);               /* Low nibble in upper bits    */
    while (!(SPSR & (1 << SPIF)));
    DAC_CS_PORT |=  (1 << DAC_CS_PIN);           /* Deassert /CS                */
}


/* ============================================================================
 * BAUDOT CODE TABLES
 * ============================================================================ */

/** ITA-2 shift control codes (5-bit, sent as start+code+stop). */
#define BAUDOT_LTRS  "11111"   /**< Letters shift character                    */
#define BAUDOT_FIGS  "11011"   /**< Figures shift character                    */

/**
 * @brief Return the 5-bit ITA-2 Baudot code string for a character.
 *
 * The function acts as a two-mode look-up table (LTRS and FIGS).  It returns
 * a pointer to a string literal of exactly 5 '0'/'1' characters, or NULL if
 * the character has no representation in the requested mode.
 *
 * @param c     The ASCII character to look up (already upper-cased by caller).
 * @param figs  false = Letters shift table, true = Figures shift table.
 * @return      Pointer to a 5-character bit-pattern string, or NULL.
 */
static const char *baudot_code(char c, bool figs)
{
    if (!figs) {
        /* ---- Letters shift (LTRS) ---- */
        switch (c) {
            case 'A': return "11000"; case 'B': return "10011";
            case 'C': return "01110"; case 'D': return "10010";
            case 'E': return "10000"; case 'F': return "10110";
            case 'G': return "01011"; case 'H': return "00101";
            case 'I': return "01100"; case 'J': return "11010";
            case 'K': return "11110"; case 'L': return "01001";
            case 'M': return "00111"; case 'N': return "00110";
            case 'O': return "00011"; case 'P': return "01101";
            case 'Q': return "11101"; case 'R': return "01010";
            case 'S': return "10100"; case 'T': return "00001";
            case 'U': return "11100"; case 'V': return "01111";
            case 'W': return "11001"; case 'X': return "10111";
            case 'Y': return "10101"; case 'Z': return "10001";
            case '\n': return "01000"; case ' ': return "00100";
            case '\r': return "00010";
            default:   return NULL;
        }
    } else {
        /* ---- Figures shift (FIGS) ---- */
        switch (c) {
            case '-':  return "11000"; case '?':  return "10011";
            case ':':  return "01110"; case '$':  return "10010";
            case '3':  return "10000"; case '!':  return "10110";
            case '&':  return "01011"; case '#':  return "00101";
            case '8':  return "01100"; case '(':  return "11110";
            case ')':  return "01001"; case '.':  return "00111";
            case ',':  return "00110"; case '9':  return "00011";
            case '0':  return "01101"; case '1':  return "11101";
            case '\'': return "01010"; case '5':  return "00001";
            case '7':  return "11100"; case ';':  return "01111";
            case '2':  return "11001"; case '/':  return "10111";
            case '6':  return "10101"; case '"':  return "10001";
            case '4':  return "01010"; /* shared code with ' in some tables   */
            case '\n': return "01000"; case ' ':  return "00100";
            case '\r': return "00010";
            default:   return NULL;
        }
    }
}


/* ============================================================================
 * RTTY TRANSMISSION
 * ============================================================================ */

/**
 * @brief Transmit one RTTY bit and wait exactly one bit period.
 *
 * Sets phaseIncrement to select the mark or space tone, then loads
 * rtty_bit_ticks and spins until the Timer1 ISR counts it down to zero.
 * Because the counter is decremented by the same ISR that drives the DAC,
 * the timing is exact regardless of loop overhead - equivalent to Arduino's
 * timer-interrupt-based delay().
 *
 * @param bit       '1' = MARK tone, '0' = SPACE tone.
 * @param inc_high  Pre-computed DDS increment for mark frequency.
 * @param inc_low   Pre-computed DDS increment for space frequency.
 * @param ticks     Number of Timer1 ticks per bit (80000 / baud, rounded).
 */
static void rtty_send_bit(char bit,
                          uint32_t inc_high, uint32_t inc_low,
                          uint16_t ticks)
{
    phaseIncrement = (bit == '1') ? inc_high : inc_low;
    rtty_bit_ticks = ticks;
    while (rtty_bit_ticks) {}   /* ISR counts down - spin until done           */
}

/**
 * @brief Encode and transmit a NUL-terminated ASCII string as Baudot RTTY.
 *
 * The function:
 *   1. Prefixes the transmission with CR+LF (receiver alignment).
 *   2. Sends the appropriate shift character (LTRS or FIGS).
 *   3. Iterates through each character, inserting shift changes as needed.
 *   4. Wraps each 5-bit code in a start bit (0) and two stop bits (11).
 *   5. Uses delay_us() for bit timing, matching the original Arduino sketch's
 *      delay(1000 / baud) call but at microsecond resolution.
 *
 * The string MUST already be upper-cased before calling this function.
 *
 * @param text  Upper-cased message to transmit.
 * @param high  Mark (high) frequency in Hz.
 * @param low   Space (low) frequency in Hz.
 * @param baud  Baud rate (e.g. 45.45).
 */
static void rtty_transmit(const char *text, int high, int low, float baud)
{
    /* Pre-compute DDS phase increments for both tones.
     * Formula: increment = frequency × 2^32 / sample_rate                    */
    uint32_t inc_high = (uint32_t)((high * 4294967296.0) / SAMPLE_RATE);
    uint32_t inc_low  = (uint32_t)((low  * 4294967296.0) / SAMPLE_RATE);

    /* Ticks per bit: Timer1 runs at 80 kHz, so ticks = 80000 / baud.
     * For 45.45 baud: 80000 / 45.45 = 1760 ticks = 22.0 ms - same as Arduino's
     * delay(1000/45.45) = delay(22).                                          */
    uint16_t ticks = (uint16_t)(SAMPLE_RATE / baud + 0.5f);

    /* Determine starting shift mode from the first character. */
    bool figs_mode = (baudot_code(text[0], false) == NULL);

    /* Helper macro: send a framed 5-bit Baudot code (start + 5 bits + 2 stop) */
    #define SEND_CODE(code_str) do { \
        rtty_send_bit('0', inc_high, inc_low, ticks); /* start bit           */ \
        for (uint8_t _b = 0; _b < 5; _b++) \
            rtty_send_bit((code_str)[_b], inc_high, inc_low, ticks); \
        rtty_send_bit('1', inc_high, inc_low, ticks); /* stop bit 1          */ \
        rtty_send_bit('1', inc_high, inc_low, ticks); /* stop bit 2          */ \
    } while (0)

    /* Preamble: CR then LF for receiver synchronisation. */
    SEND_CODE(baudot_code('\r', false));
    SEND_CODE(baudot_code('\n', false));

    /* Initial shift character. */
    SEND_CODE(figs_mode ? BAUDOT_FIGS : BAUDOT_LTRS);

    /* Encode and transmit each character. */
    for (const char *p = text; *p; p++) {
        const char *code = baudot_code(*p, figs_mode);

        if (code == NULL) {
            /* Character not in current mode - try the other mode. */
            bool other_mode = !figs_mode;
            code = baudot_code(*p, other_mode);
            if (code == NULL) continue;          /* Truly unsupported - skip    */
            figs_mode = other_mode;
            SEND_CODE(figs_mode ? BAUDOT_FIGS : BAUDOT_LTRS);
        }
        SEND_CODE(code);
    }

    #undef SEND_CODE

    phaseIncrement = 0;                          /* Mute carrier after TX done   */
}


/* ============================================================================
 * BIT-BANG SPI FOR 25LC640A EEPROM  (PORTC)
 * ============================================================================
 *
 *  PC0 = /CS   (output) - active low chip select
 *  PC3 = MOSI  (output) - serial data in to EEPROM
 *  PC4 = SCK   (output) - clock (CPOL=0, CPHA=0)
 *  PC5 = MISO  (input)  - serial data out from EEPROM
 *
 * 25LC640A SPI commands used here:
 *   READ  = 0x03  - sequential read from given 16-bit address
 *   WRITE = 0x02  - byte-write (must be preceded by WREN within same CS cycle)
 *   WREN  = 0x06  - write-enable latch
 *   RDSR  = 0x05  - read status register (bit 0 = WIP write-in-progress)
 * ============================================================================ */

#define EE_CS_LOW()   (PORTC &= ~(1 << PC0))    /**< Assert /CS                */
#define EE_CS_HIGH()  (PORTC |=  (1 << PC0))    /**< Deassert /CS              */
#define EE_SCK_LOW()  (PORTC &= ~(1 << PC4))    /**< Clock low                 */
#define EE_SCK_HIGH() (PORTC |=  (1 << PC4))    /**< Clock high                */
#define EE_SI_HIGH()  (PORTC |=  (1 << PC3))    /**< MOSI high                 */
#define EE_SI_LOW()   (PORTC &= ~(1 << PC3))    /**< MOSI low                  */
#define EE_SO_READ()  ((PINC  &  (1 << PC5)) ? 1 : 0) /**< Read MISO          */

/** Total EEPROM capacity in bytes (25LC1024 = 1 Mbit = 131072 bytes). */
#define EEPROM_SIZE   131072UL

/**
 * @brief 25LC1024 SPI opcode definitions.
 *
 * The 25LC1024 uses a 24-bit address field: the READ and WRITE opcodes are
 * followed by three address bytes (MSB first), where only the lower 17 bits
 * are significant (addresses 0x000000 ? 0x01FFFF).
 */
#define EE_CMD_READ  0x03U   /**< Read data from memory array                  */
#define EE_CMD_WRITE 0x02U   /**< Write data to memory array                   */
#define EE_CMD_WREN  0x06U   /**< Set write-enable latch                       */
#define EE_CMD_RDSR  0x05U   /**< Read status register                         */

/**
 * @brief Shift one byte out/in via bit-bang SPI (MSB first, CPOL=0 CPHA=0).
 *
 * Simultaneously writes @p out to MOSI and samples MISO, returning the result.
 * When only writing (don't-care read), pass any value for @p out.
 * When only reading, pass 0x00 for @p out.
 *
 * @param out  Byte to send to the EEPROM.
 * @return     Byte received from the EEPROM.
 */
static uint8_t ee_spi_byte(uint8_t out)
{
    uint8_t in = 0;
    for (uint8_t i = 0; i < 8; i++) {
        /* Set MOSI before rising edge (setup time). */
        if (out & 0x80U) EE_SI_HIGH(); else EE_SI_LOW();
        out <<= 1;
        EE_SCK_HIGH();                           /* Rising edge - EEPROM samples MOSI  */
        in = (uint8_t)((in << 1) | EE_SO_READ()); /* Sample MISO on rising edge      */
        EE_SCK_LOW();                            /* Falling edge                        */
    }
    return in;
}

/**
 * @brief Poll the EEPROM status register until the Write-In-Progress bit clears.
 *
 * After a byte-write cycle the 25LC1024 holds WIP=1 for up to 5 ms.
 * This function busy-waits until the write completes before allowing the next
 * operation - essential for reliable sequential writes.
 */
static void ee_wait_ready(void)
{
    uint8_t sr;
    do {
        EE_CS_LOW();
        ee_spi_byte(EE_CMD_RDSR);                /* Issue Read-Status-Register  */
        sr = ee_spi_byte(0x00);                  /* Clock out the status byte   */
        EE_CS_HIGH();
    } while (sr & 0x01U);                        /* Spin while WIP bit is set   */
}

/**
 * @brief Send the 25LC1024 Read opcode followed by a 24-bit address and leave
 *        /CS asserted so the caller can continue clocking bytes.
 *
 * The 25LC1024 requires three address bytes (24 bits, only lower 17 are used).
 * /CS is held LOW after this call; call EE_CS_HIGH() to end the transaction.
 *
 * @param addr  Starting read address (0x000000 ? 0x01FFFF).
 */
static void ee_start_read(uint32_t addr)
{
    /* Send READ opcode + 24-bit address as one 32-bit word, MSB first -
     * identical to Arduino's start_eeprom_stream().                           */
    uint32_t cmd_addr = ((uint32_t)EE_CMD_READ << 24) | (addr & 0x00FFFFFFUL);
    EE_CS_LOW();
    for (uint8_t i = 0; i < 32; i++) {
        if (cmd_addr & 0x80000000UL) EE_SI_HIGH(); else EE_SI_LOW();
        EE_SCK_HIGH();
        EE_SCK_LOW();
        cmd_addr <<= 1;
    }
    /* /CS remains LOW; EEPROM streams data bytes on SO from here.             */
}

/**
 * @brief Read the next sequential byte from the EEPROM.
 *
 * Must only be called after ee_start_read() and before EE_CS_HIGH().
 * The 25LC1024 auto-increments its internal address pointer.
 *
 * @return The byte read from the current EEPROM address.
 */
static inline uint8_t ee_read_next(void)
{
    return ee_spi_byte(0x00);                    /* Clock in one byte           */
}

/**
 * @brief Write a single byte to the EEPROM using byte-write mode.
 *
 * Sequence: WREN (own /CS cycle) -> WRITE opcode + 24-bit address + data
 * (own /CS) -> poll WIP until done.
 *
 * The 25LC1024 write cycle takes up to 5 ms; ee_wait_ready() must be called
 * (and is called here) before the next operation.
 *
 * @param addr   Target address (0x000000 ? 0x01FFFF).
 * @param data   Byte value to store.
 */
static void ee_write_byte(uint32_t addr, uint8_t data)
{
    /* Step 1: Set write-enable latch (required before every WRITE). */
    EE_CS_LOW();
    ee_spi_byte(EE_CMD_WREN);                    /* WREN must precede each WRITE */
    EE_CS_HIGH();

    /* Step 2: Issue WRITE command with 24-bit address and data byte. */
    EE_CS_LOW();
    ee_spi_byte(EE_CMD_WRITE);
    ee_spi_byte((uint8_t)(addr >> 16));          /* Address byte 2 (MSB)        */
    ee_spi_byte((uint8_t)(addr >>  8));          /* Address byte 1              */
    ee_spi_byte((uint8_t)(addr));                /* Address byte 0 (LSB)        */
    ee_spi_byte(data);                           /* Data byte                   */
    EE_CS_HIGH();

    /* Step 3: Wait for internal write cycle to complete (up to 5 ms). */
    ee_wait_ready();
}


/* ============================================================================
 * EEPROM AUDIO PLAYBACK GLOBALS
 * ============================================================================ */

/**
 * @brief Number of consecutive 0xFF samples that signals end-of-audio.
 *
 * 250 samples at 8 kHz = 31.25 ms of silence.  Real audio is extremely
 * unlikely to sustain 0xFF for that long, making this a reliable sentinel.
 */
#define FF_THRESHOLD  250

volatile uint8_t  ee_sample          = 0;      /**< Current sample loaded for DAC   */
volatile bool     playback_active    = false;  /**< True while Timer2 is running     */
volatile uint8_t  saved_TIMSK1       = 0;      /**< Saved Timer1 interrupt mask      */
volatile uint16_t consecutive_ff_cnt = 0;      /**< Counter for end-of-audio detect  */


/* ============================================================================
 * INTERRUPT SERVICE ROUTINES
 * ============================================================================ */

/**
 * @brief Timer1 Compare-A ISR - 80 kHz DDS / sine-wave generator.
 *
 * Fires every 12.5 µs (OCR1A=199, no prescaler, 16 MHz).
 * Advances the 32-bit phase accumulator by phaseIncrement, then uses the
 * top 8 bits as an index into the sine table stored in Flash.
 * If phaseIncrement == 0 the DAC output is held at mid-scale (0x80) to
 * prevent DC offset from reaching the RF amplifier.
 */
ISR(TIMER1_COMPA_vect)
{
    /* Decrement RTTY bit-period counter (used by rtty_send_bit). */
    if (rtty_bit_ticks) rtty_bit_ticks--;

    /* Advance DDS phase accumulator and look up sine sample from SRAM table -
     * identical to the Arduino sketch (no PROGMEM, plain array index).        */
    phaseAccumulator += phaseIncrement;
    uint8_t sineValue = sineTable[phaseAccumulator >> 24];

    /* Write sample to MCP4901 DAC - same raw register sequence as Arduino.   */
    PORTB &= ~_BV(DAC_CS_PIN);
    SPDR = CONFIG_HIGH | (sineValue >> 4);
    while (!(SPSR & (1 << SPIF)));
    SPDR = (sineValue << 4);
    while (!(SPSR & (1 << SPIF)));
    PORTB |= _BV(DAC_CS_PIN);
}

/**
 * @brief Timer2 Compare-A ISR - 8 kHz audio playback from EEPROM.
 *
 * Fires every 125 µs (OCR2A=249, prescaler /8, 16 MHz).
 * Outputs the pre-fetched sample to the DAC, then clocks the next byte from
 * the EEPROM over bit-bang SPI.  Playback stops when FF_THRESHOLD consecutive
 * 0xFF samples have been detected, or when EEPROM end is reached.
 *
 * Note: The EEPROM /CS was asserted by ee_start_read() before this ISR was
 * enabled and is only released when playback ends.
 */
ISR(TIMER2_COMPA_vect)
{
    /* Output current sample to DAC - identical inline sequence to Arduino.    */
    uint8_t high = CONFIG_HIGH | (ee_sample >> 4);
    uint8_t low  = (ee_sample << 4);
    PORTB &= ~_BV(DAC_CS_PIN);
    SPDR = high; while (!(SPSR & (1 << SPIF)));
    SPDR = low;  while (!(SPSR & (1 << SPIF)));
    PORTB |= _BV(DAC_CS_PIN);

    /* Read next byte from EEPROM via inline bit-bang - identical to Arduino.  */
    uint8_t next = 0;
    for (uint8_t i = 0; i < 8; i++) {
        next <<= 1;
        EE_SCK_HIGH();
        if (EE_SO_READ()) next |= 1;
        EE_SCK_LOW();
    }
    ee_sample = next;

    /* End-of-audio detection - identical to Arduino.                          */
    if (ee_sample == 0xFF) consecutive_ff_cnt++;
    else                   consecutive_ff_cnt = 0;

    if (consecutive_ff_cnt >= FF_THRESHOLD) {
        EE_CS_HIGH();
        TIMSK2 &= ~(1 << OCIE2A);
        playback_active = false;
        TIMSK1 = saved_TIMSK1;
    }
}


/* ============================================================================
 * AT COMMAND PARSER
 * ============================================================================ */

/** Maximum number of bytes accepted in a single command line (incl. NUL). */
#define CMD_BUF_LEN  128U

/** Echo received characters back to the terminal (standard AT behaviour). */
static bool do_echo = true;

/**
 * @brief Runtime-configurable RTTY parameters with factory defaults.
 *
 * high: MARK tone frequency in Hz  (ITA-2 / CCIR default = 2125 Hz)
 * low : SPACE tone frequency in Hz (ITA-2 / CCIR default = 2295 Hz)
 * baud: Symbol rate in baud        (ITA-2 standard       = 45.45 Bd)
 */
typedef struct {
    int16_t high;   /**< Mark  (logic 1) frequency in Hz  */
    int16_t low;    /**< Space (logic 0) frequency in Hz  */
    float   baud;   /**< Symbol rate in baud              */
} rtty_params_t;

static rtty_params_t params = { 2125, 2295, 45.45f };  /**< Factory defaults  */

/* ---- Timeout for EEPROM upload idle detection ---- */

/**
 * @brief Milliseconds of UART silence that terminate an AT+upload session.
 *
 * Timer2 is reused in CTC mode at 1 kHz to generate the upload timeout tick.
 * After UPLOAD_IDLE_MS consecutive milliseconds with no new byte received the
 * upload routine concludes and returns "OK".
 */
#define UPLOAD_IDLE_MS  1500U

/* ---- Forward declarations for helper used inside exec() ---- */
static void cmd_upload(void);

/**
 * @brief Execute one complete AT command line.
 *
 * The function receives a mutable NUL-terminated string containing the full
 * command as typed by the user (without the terminating CR).  It uses
 * strtok_r / manual pointer arithmetic to split command verb from arguments -
 * avoiding the Arduino String class entirely in favour of standard C strings.
 *
 * Recognised commands
 * -------------------
 *  AT               - no-op, replies OK
 *  AT+send="..."    - Baudot-encode and transmit a message via RTTY
 *  AT+set=p,v       - Set parameter p (high|low|baud) to value v
 *  AT+get=p         - Query parameter p
 *  AT+write=p,v     - Drive GPIO pin p (0-5 -> PD2-PD7) to value v (0|1)
 *  AT+play          - Stream PCM audio from EEPROM through the DAC
 *  AT+upload        - Receive hex-encoded PCM data and write to EEPROM
 *  AT+reset         - Restore factory defaults
 *  AT+help          - Print command summary
 *
 * @param cmdline  Mutable command line buffer (will be modified by strtok_r).
 */
static void exec(char *cmdline)
{
    /* Split "VERB=ARGS" into verb and the rest. strtok_r returns the verb and
     * advances rest_ptr past the '=' separator (or to NULL if no '=' found).  */
    char *rest     = NULL;
    char *verb     = strtok_r(cmdline, "=", &rest);

    /* ------------------------------------------------------------------ AT -- */
    if (strcmp_P(verb, PSTR("AT")) == 0) {
        uart_puts_P(PSTR("OK\r\n"));

    /* --------------------------------------------------------------- AT+send */
    } else if (strcmp_P(verb, PSTR("AT+send")) == 0) {
        /* Expected syntax: AT+send="message text" */
        char *msg = rest;
        if (msg == NULL || msg[0] != '"') {
            uart_puts_P(PSTR("Error: Syntax error. Usage: AT+send=\"message\"\r\n"));
            return;
        }
        msg++;                                   /* Skip opening quote          */
        uint8_t len = (uint8_t)strlen(msg);
        if (len == 0 || msg[len - 1] != '"') {
            uart_puts_P(PSTR("Error: Missing closing quote.\r\n"));
            return;
        }
        msg[len - 1] = '\0';                     /* Strip closing quote         */

        /* Convert to upper case in-place (Baudot table is upper-case only).   */
        for (char *s = msg; *s; s++) *s = (char)toupper((unsigned char)*s);

        uart_puts_P(PSTR("Transmitting: "));
        uart_puts(msg);
        uart_crlf();

        LED_ON();                                /* LED on during TX            */
        rtty_transmit(msg, params.high, params.low, params.baud);
        LED_OFF();                               /* LED off when done           */

        uart_puts_P(PSTR("OK\r\n"));

    /* ---------------------------------------------------------------- AT+set */
    } else if (strcmp_P(verb, PSTR("AT+set")) == 0) {
        /* Expected syntax: AT+set=<param>,<value> */
        if (rest == NULL) {
            uart_puts_P(PSTR("Error: Missing argument.\r\n"));
            return;
        }
        char *saveptr = NULL;
        char *param = strtok_r(rest, ",", &saveptr);
        char *valstr = strtok_r(NULL, ",", &saveptr);

        if (param == NULL || valstr == NULL) {
            uart_puts_P(PSTR("Error: Syntax error. Usage: AT+set=<param>,<value>\r\n"));
            return;
        }
        float val = atof(valstr);

        if (strcmp_P(param, PSTR("high")) == 0) {
            params.high = (int16_t)val;
            uart_puts_P(PSTR("Set high frequency to: "));
            uart_print_int(params.high);
            uart_puts_P(PSTR(" Hz\r\nOK\r\n"));
        } else if (strcmp_P(param, PSTR("low")) == 0) {
            params.low = (int16_t)val;
            uart_puts_P(PSTR("Set low frequency to: "));
            uart_print_int(params.low);
            uart_puts_P(PSTR(" Hz\r\nOK\r\n"));
        } else if (strcmp_P(param, PSTR("baud")) == 0) {
            params.baud = val;
            uart_puts_P(PSTR("Set baud rate to: "));
            uart_print_float(params.baud);
            uart_puts_P(PSTR(" Bd\r\nOK\r\n"));
        } else {
            uart_puts_P(PSTR("Error: Unknown parameter. Valid: high, low, baud\r\n"));
        }

    /* ---------------------------------------------------------------- AT+get */
    } else if (strcmp_P(verb, PSTR("AT+get")) == 0) {
        if (rest == NULL) {
            uart_puts_P(PSTR("Error: Missing argument.\r\n"));
            return;
        }
        if (strcmp_P(rest, PSTR("high")) == 0) {
            uart_puts_P(PSTR("high = "));
            uart_print_int(params.high);
            uart_puts_P(PSTR(" Hz\r\nOK\r\n"));
        } else if (strcmp_P(rest, PSTR("low")) == 0) {
            uart_puts_P(PSTR("low = "));
            uart_print_int(params.low);
            uart_puts_P(PSTR(" Hz\r\nOK\r\n"));
        } else if (strcmp_P(rest, PSTR("baud")) == 0) {
            uart_puts_P(PSTR("baud = "));
            uart_print_float(params.baud);
            uart_puts_P(PSTR(" Bd\r\nOK\r\n"));
        } else {
            uart_puts_P(PSTR("Error: Unknown parameter. Valid: high, low, baud\r\n"));
        }

    /* -------------------------------------------------------------- AT+write */
    } else if (strcmp_P(verb, PSTR("AT+write")) == 0) {
        /* Expected syntax: AT+write=<pin>,<value>  (pin 0-5, value 0 or 1)   */
        if (rest == NULL) {
            uart_puts_P(PSTR("Error: Syntax error. Usage: AT+write=<pin>,<value>\r\n"));
            return;
        }
        char *saveptr = NULL;
        char *pinstr = strtok_r(rest,  ",", &saveptr);
        char *valstr = strtok_r(NULL, ",", &saveptr);

        if (pinstr == NULL || valstr == NULL) {
            uart_puts_P(PSTR("Error: Syntax error. Usage: AT+write=<pin>,<value>\r\n"));
            return;
        }
        int8_t  pin = (int8_t)atoi(pinstr);
        int8_t  val = (int8_t)atoi(valstr);

        if (pin < 0 || pin > 5 || (val != 0 && val != 1)) {
            uart_puts_P(PSTR("Error: Pin must be 0-5, value must be 0 or 1.\r\n"));
            return;
        }

        /* Map logical pin 0-5 to PD2-PD7 (bit positions 2-7 in PORTD). */
        uint8_t bit = (uint8_t)(pin + 2);
        DDRD  |=  (1 << bit);                    /* Ensure pin is configured out */
        if (val) PORTD |=  (1 << bit);
        else     PORTD &= ~(1 << bit);

        uart_puts_P(PSTR("OK\r\n"));

    /* --------------------------------------------------------------- AT+play */
    } else if (strcmp_P(verb, PSTR("AT+play")) == 0) {
        /* --- Prepare EEPROM read stream from address 0. --- */
        ee_start_read(0x000000UL);               /* Assert /CS, send READ+24-bit addr */
        ee_sample           = ee_read_next();    /* Pre-fetch first sample            */
        consecutive_ff_cnt  = 0;

        /* --- Save and suspend the RTTY timer while audio plays. --- */
        saved_TIMSK1    = TIMSK1;
        TIMSK1          = 0;                     /* Disable Timer1 ISR                */
        phaseIncrement  = 0;                     /* Ensure DDS outputs silence if
                                                    Timer1 fires spuriously            */

        /* --- Configure Timer2 for 8 kHz audio playback. --- */
        cli();
        TCCR2A = (1 << WGM21);                  /* CTC mode                          */
        TCCR2B = (1 << CS21);                   /* Prescaler /8                      */
        OCR2A  = 249U;                           /* (16 MHz / 8) / 8000 - 1 = 249    */
        TCNT2  = 0;
        TIMSK2 |= (1 << OCIE2A);                /* Enable Timer2 compare ISR         */
        sei();

        /* --- Block in main context until ISR clears playback_active. --- */
        LED_ON();                                /* Indicate playback to user         */
        playback_active = true;
        while (playback_active) {/* spin */}

        /* Clear any pending Timer1 output-compare flag that accumulated while
         * TIMSK1 was 0, so it does not fire spuriously the instant we re-enable
         * the interrupt mask (OCF1A is cleared by writing a logic 1 to it).    */
        TIFR1 = (1 << OCF1A);

        LED_OFF();
        uart_puts_P(PSTR("OK\r\n"));

    /* -------------------------------------------------------------- AT+upload */
    } else if (strcmp_P(verb, PSTR("AT+upload")) == 0) {
        cmd_upload();                            /* Delegate to dedicated handler */

    /* --------------------------------------------------------------- AT+reset */
    } else if (strcmp_P(verb, PSTR("AT+reset")) == 0) {
        params.high = 2125;
        params.low  = 2295;
        params.baud = 45.45f;
        uart_puts_P(PSTR("Parameters reset to factory defaults.\r\nOK\r\n"));

    /* ---------------------------------------------------------------- AT+help */
    } else if (strcmp_P(verb, PSTR("AT+help")) == 0) {
        uart_puts_P(PSTR(
            "\r\n"
            "Command Reference\r\n"
            "-----------------\r\n"
            "  AT                         Connection test\r\n"
            "  AT+send=\"<message>\"        Encode and transmit RTTY message\r\n"
            "  AT+set=high,<Hz>           Set MARK (high) frequency\r\n"
            "  AT+set=low,<Hz>            Set SPACE (low) frequency\r\n"
            "  AT+set=baud,<Bd>           Set symbol rate (e.g. 45.45)\r\n"
            "  AT+get=<high|low|baud>     Query a parameter value\r\n"
            "  AT+write=<pin>,<0|1>       Drive GPIO pin (0-5 = PD2-PD7)\r\n"
            "  AT+play                    Play PCM audio from EEPROM\r\n"
            "  AT+upload                  Write hex PCM data to EEPROM\r\n"
            "  AT+reset                   Restore factory defaults\r\n"
            "  AT+help                    Show this help\r\n"
            "\r\n"
            "OK\r\n"
        ));

    /* --------------------------------------------------------- Unknown command */
    } else {
        uart_puts_P(PSTR("Error: Unknown command: "));
        uart_puts(verb);
        uart_crlf();
    }
}


/* ============================================================================
 * AT+UPLOAD COMMAND IMPLEMENTATION
 * ============================================================================ */

/**
 * @brief Receive a hex-encoded PCM byte stream over UART and write it to the
 *        25LC640A EEPROM starting at address 0x0000.
 *
 * Protocol
 * --------
 * The host sends pairs of ASCII hexadecimal digits ('0'-'9', 'A'-'F', 'a'-'f').
 * Whitespace characters (space, tab, CR, LF) between pairs are silently ignored,
 * making it easy to upload data with a simple hex-dump tool or a Python script.
 *
 * Session termination
 * -------------------
 * The upload session ends when either:
 *   (a) No new character arrives within UPLOAD_IDLE_MS milliseconds, or
 *   (b) The EEPROM is full (131072 bytes written).
 *
 * A simple software millisecond counter is used for the timeout - each loop
 * iteration idles for 1 ms via _delay_ms(1) when no byte is available.
 *
 * End-of-data marker
 * ------------------
 * To allow the EEPROM to contain valid audio with a defined end, the firmware
 * pads any remaining unwritten EEPROM bytes with 0xFF after the upload completes.
 * This is the same sentinel value used by the AT+play command to detect silence.
 *
 * Example usage (Python host-side)
 * ---------------------------------
 *   import serial, binascii, time
 *   with serial.Serial('/dev/ttyUSB0', 9600) as s:
 *       s.write(b'AT+upload\r')
 *       time.sleep(0.1)
 *       data = open('sample.raw', 'rb').read()  # 8-bit unsigned PCM, 8 kHz
 *       s.write(binascii.hexlify(data) + b'\r\n')
 */
static void cmd_upload(void)
{
    uart_puts_P(PSTR(
        "Ready. Send hex data (e.g. A3 4F 00 ...). "
        "Upload ends after " ));
    uart_print_int((int16_t)UPLOAD_IDLE_MS);
    uart_puts_P(PSTR(" ms idle.\r\n"));

    LED_ON();                                    /* Indicate recording/write mode */

    uint32_t addr          = 0;                 /* Current write address (17-bit) */
    bool     session_done  = false;             /* Set when upload ends           */
    uint8_t  nibble_buf    = 0;                 /* High nibble pending             */
    bool     have_nibble   = false;             /* True when high nibble stored    */

    /* Timeout state: each iteration of the outer loop represents roughly 1 ms.
     * We use _delay_ms(1) + uart_try_getc() as a simple polling tick.           */
    uint16_t idle_ms = 0;                       /* Milliseconds since last char   */

    while (!session_done && addr < EEPROM_SIZE)
    {
        uint8_t rx;
        if (uart_try_getc(&rx)) {
            idle_ms = 0;                         /* Reset timeout on each byte  */

            /* Skip whitespace separators between hex pairs. */
            if (rx == ' ' || rx == '\t' || rx == '\r' || rx == '\n') continue;

            /* Validate: only hex digits are accepted. */
            if (!isxdigit((unsigned char)rx)) {
                uart_puts_P(PSTR("\r\nError: Non-hex character received.\r\n"));
                LED_OFF();
                return;
            }

            /* Convert ASCII hex digit to 4-bit value. */
            uint8_t nibble;
            if      (rx >= '0' && rx <= '9') nibble = (uint8_t)(rx - '0');
            else if (rx >= 'a' && rx <= 'f') nibble = (uint8_t)(rx - 'a' + 10);
            else                             nibble = (uint8_t)(rx - 'A' + 10);

            if (!have_nibble) {
                nibble_buf  = (uint8_t)(nibble << 4); /* Store high nibble     */
                have_nibble = true;
            } else {
                uint8_t data = nibble_buf | nibble;   /* Combine into full byte */
                have_nibble  = false;
                ee_write_byte(addr, data);             /* Write to EEPROM       */
                addr++;

                /* Progress indicator: print '.' every 256 bytes. */
                if ((addr & 0xFFU) == 0) uart_putc('.');
            }
        } else {
            /* No character available - advance idle counter. */
            _delay_ms(1);
            idle_ms++;
            if (idle_ms >= UPLOAD_IDLE_MS) session_done = true;
        }
    }

    /* If we have an orphaned high nibble, report it. */
    if (have_nibble) {
        uart_puts_P(PSTR("\r\nWarning: Odd nibble at end of stream (ignored).\r\n"));
    }

    /* Pad the rest of the EEPROM with 0xFF so AT+play detects end-of-audio. */
    uart_puts_P(PSTR("\r\nFinalising..."));
    while (addr < EEPROM_SIZE) {
        ee_write_byte(addr, 0xFFU);
        addr++;
        if ((addr & 0xFFU) == 0) uart_putc('.');  /* Show progress              */
    }

    LED_OFF();
    uart_puts_P(PSTR("\r\nUpload complete.\r\nOK\r\n"));
}


/* ============================================================================
 * STARTUP BANNER
 * ============================================================================ */

/**
 * @brief Print a formatted startup banner and command summary to UART.
 *
 * Called once from main() after all peripherals are initialised.  The banner
 * identifies the firmware, lists the command set, and ends with the prompt
 * so the user knows the device is ready.
 */
static void print_banner(void)
{
    uart_puts_P(PSTR("\r\n"));
    uart_puts_P(PSTR("  +-------------------------------------------------+\r\n"));
    uart_puts_P(PSTR("  |         " DEVICE_NAME " RTTY Transmitter  " FW_VERSION "          |\r\n"));
    uart_puts_P(PSTR("  |         ATmega328P @ 16 MHz  --  9600 8N1       |\r\n"));
    uart_puts_P(PSTR("  +-------------------------------------------------+\r\n"));
    uart_puts_P(PSTR("\r\n"));
    uart_puts_P(PSTR("  Commands:\r\n"));
    uart_puts_P(PSTR("    AT+send=\"msg\"       Transmit RTTY message\r\n"));
    uart_puts_P(PSTR("    AT+set=<p>,<v>     Set parameter (high/low/baud)\r\n"));
    uart_puts_P(PSTR("    AT+get=<p>         Query parameter\r\n"));
    uart_puts_P(PSTR("    AT+write=<p>,<v>   Set GPIO pin (0-5 -> PD2-PD7)\r\n"));
    uart_puts_P(PSTR("    AT+play            Play PCM audio from EEPROM\r\n"));
    uart_puts_P(PSTR("    AT+upload          Upload hex PCM data to EEPROM\r\n"));
    uart_puts_P(PSTR("    AT+reset           Restore factory defaults\r\n"));
    uart_puts_P(PSTR("    AT+help            Full command reference\r\n"));
    uart_puts_P(PSTR("\r\n"));
    uart_puts_P(PSTR("  Defaults: MARK=2125 Hz  SPACE=2295 Hz  BAUD=45.45\r\n"));
    uart_puts_P(PSTR("  Status LED: PD7 (lights during TX, upload, playback)\r\n"));
    uart_puts_P(PSTR("\r\n"));
    uart_puts_P(PSTR("  Ready.\r\n\r\n"));
}


/* ============================================================================
 * HARDWARE INITIALISATION
 * ============================================================================ */

/**
 * @brief Initialise all hardware peripherals.
 *
 * Order matters:
 *   1. UART  - set up first so debug/banner output works immediately.
 *   2. SPI   - hardware SPI for MCP4901 DAC.
 *   3. PORTC - bit-bang SPI pins for 25LC1024 EEPROM.
 *   4. PORTD - GPIO output pins (PD2-PD7) and status LED.
 *   5. Timer1 - 80 kHz DDS interrupt (started last to avoid ISR
 *               interfering with UART settling or banner output).
 */
static void hw_init(void)
{
    /* 1. UART ---------------------------------------------------------------- */
    uart_init();

    /* 2. Hardware SPI (MCP4901 DAC) ----------------------------------------- */
    /* PB2 = /CS (output), PB3 = MOSI (output), PB5 = SCK (output)            */
    DDRB |= (1 << PB2) | (1 << PB3) | (1 << PB5);
    DAC_CS_PORT |= (1 << DAC_CS_PIN);           /* Deassert /CS at startup     */

    /* Enable SPI as Master, SPI mode 0, MSB first.
     * Clock rate: SPR1=0, SPR0=0, SPI2X=1 -> F_CPU/2 = 8 MHz.
     * Note: SPR0 must NOT be set here - SPR0=1 + SPI2X=1 gives F_CPU/8 = 2 MHz
     * which would cause the 80 kHz Timer1 ISR to spend ~12 µs of every 12.5 µs
     * in SPI transfers, starving the CPU and causing garbled UART output.     */
    SPCR = (1 << SPE) | (1 << MSTR);            /* Enable SPI master, F_CPU/4 base */
    SPSR = (1 << SPI2X);                        /* SPI2X=1: double to F_CPU/2 = 8 MHz */

    /* 3. PORTC bit-bang EEPROM ----------------------------------------------- */
    /* PC0=/CS, PC3=MOSI, PC4=SCK outputs; PC5=MISO input.                     */
    DDRC |=  (1 << PC0) | (1 << PC3) | (1 << PC4);
    DDRC &= ~(1 << PC5);
    EE_CS_HIGH();                                /* Deassert EEPROM /CS         */
    EE_SI_LOW();                                 /* MOSI idle low               */
    EE_SCK_LOW();                                /* SCK idle low                */

    /* 4. PORTD GPIO + LED ---------------------------------------------------- */
    /* PD2-PD7 as outputs (PD0=RX, PD1=TX are managed by UART peripheral).     */
    DDRD |= (1 << PD2) | (1 << PD3) | (1 << PD4)
          | (1 << PD5) | (1 << PD6) | (1 << LED_PIN);
    LED_OFF();                                   /* LED off at startup          */

    /* 5. Timer1: 80 kHz CTC for DDS ----------------------------------------- */
    /* Started LAST so the ISR does not interfere with UART init or banner TX.  */
    cli();
    TCCR1A = 0;                                  /* Normal port operation       */
    TCCR1B = 0;
    TCNT1  = 0;
    OCR1A  = 199U;                               /* 16 MHz / (199+1) = 80 kHz  */
    TCCR1B |= (1 << WGM12);                      /* CTC mode (OCR1A as TOP)     */
    TCCR1B |= (1 << CS10);                       /* No prescaler                */
    TIMSK1 |= (1 << OCIE1A);                     /* Enable Output Compare A ISR */
    sei();
}


/* ============================================================================
 * MAIN
 * ============================================================================ */

/**
 * @brief Application entry point.
 *
 * Initialises hardware, prints the startup banner, then enters the AT command
 * receive loop.  The loop accumulates characters into a line buffer, echoes
 * them back to the terminal (backspace-aware), and dispatches complete lines
 * to exec() on reception of a carriage-return (CR).
 *
 * @return This function never returns (embedded infinite loop).
 */
int main(void)
{
    hw_init();                                   /* Bring up all peripherals    */
    print_banner();                              /* Welcome message over UART   */

    /* ---- AT command receive loop ---- */
    static char   buf[CMD_BUF_LEN];             /* Line accumulation buffer    */
    static uint8_t len = 0;                     /* Current buffer fill level   */

    for (;;) {
        if (!uart_available()) continue;         /* Nothing to do if UART empty */

        int16_t data = (int16_t)UDR0;           /* Read directly to minimise latency */

        if (data == '\b' || data == 127) {
            /* Backspace / DEL: erase last character if buffer non-empty. */
            if (len > 0) {
                len--;
                if (do_echo) uart_puts("\b \b"); /* Erase character on terminal */
            }
        } else if (data == '\r') {
            /* Carriage return: end of command line - dispatch to parser. */
            if (do_echo) uart_crlf();
            buf[len] = '\0';                     /* NUL-terminate               */
            if (len > 0) exec(buf);              /* Execute non-empty lines     */
            len = 0;                             /* Reset buffer                */
        } else if (len < CMD_BUF_LEN - 1U) {
            /* Printable character: accumulate into buffer. */
            buf[len++] = (char)data;
            if (do_echo) uart_putc((char)data);
        }
        /* If buffer is full the character is silently dropped - the 128-byte
         * limit is generous enough that this should not occur in practice.    */
    }

    return 0;  /* Never reached. */
}