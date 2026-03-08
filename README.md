# rtty-transmitter

An ATmega328P-based AM/RTTY transmitter card with USB serial control, Direct Digital Synthesis audio generation, PCM audio playback from SPI EEPROM, and a Wien bridge oscillator RF stage. The board is controlled entirely through an AT command interface over a virtual COM port, and includes a companion Python control center application for Windows, Linux, and macOS.

![RTTY Transmitter Board](docs/images/rtty-transmitter-1.jpg)

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware Description](#hardware-description)
  - [USB Interface and Power Supply](#usb-interface-and-power-supply)
  - [Microcontroller](#microcontroller)
  - [DAC and DDS Engine](#dac-and-dds-engine)
  - [EEPROM for PCM Audio Storage](#eeprom-for-pcm-audio-storage)
  - [Virtual Ground](#virtual-ground)
  - [Wien Bridge Oscillator and Carrier Wave](#wien-bridge-oscillator-and-carrier-wave)
  - [Audio Buffer](#audio-buffer)
  - [RF Output Stage](#rf-output-stage)
  - [Tank Circuit and Antenna Output](#tank-circuit-and-antenna-output)
  - [GPIO Expansion and Status LED](#gpio-expansion-and-status-led)
  - [Bill of Materials](#bill-of-materials)
  - [PCB](#pcb)
- [Firmware](#firmware)
  - [Signal Chain](#signal-chain)
  - [AT Command Interface](#at-command-interface)
  - [RTTY Encoding](#rtty-encoding)
  - [PCM Audio Playback](#pcm-audio-playback)
  - [EEPROM Upload Protocol](#eeprom-upload-protocol)
- [Getting Started](#getting-started)
  - [Requirements](#requirements)
  - [Drivers](#drivers)
  - [Connecting the Board](#connecting-the-board)
- [Building and Flashing the Firmware](#building-and-flashing-the-firmware)
  - [Command-Line Build with avr-gcc and avrdude](#command-line-build-with-avr-gcc-and-avrdude)
  - [Building with MPLAB X IDE](#building-with-mplab-x-ide)
- [Control Center Application](#control-center-application)
  - [Installation](#installation)
  - [Usage](#usage)
- [Antenna Selection and Range](#antenna-selection-and-range)
- [Testing and Reception](#testing-and-reception)
- [Comparison with Official RTTY](#comparison-with-official-rtty)
- [Regulatory Notice](#regulatory-notice)
- [Personal Note](#personal-note)

---

## Overview

This project is a self-contained AM/RTTY transmitter card designed and built around the ATmega328P microcontroller. The board generates FSK (Frequency Shift Keying) audio tones using a software Direct Digital Synthesis engine, feeds those tones through an 8-bit SPI DAC, buffers and amplifies them through a dual op-amp stage, and then modulates them onto a carrier wave produced by a Wien bridge oscillator running at approximately 802 kHz. The resulting AM signal is delivered to an SMA connector for antenna connection.

The board is powered and controlled through a single USB Type B cable. An MCP2221A USB-to-UART bridge makes the board appear as a standard COM port on the host computer, requiring no custom drivers beyond the standard MCP2221 package. All board functions are controlled through a straightforward AT command interface at 9600 baud. A Python-based control center application is included in the tools folder to provide a graphical interface for all features, including RTTY message transmission, text file transmission, PCM audio upload and playback, and GPIO control.

The board can also function as an audio transmitter: an 8-bit, 8 kHz PCM audio file can be uploaded to the onboard 25LC1024 SPI EEPROM (128 KB) and played back over the air using the AT+play command, AM-modulating the 802 kHz carrier with audio content.

---

## Features

- RTTY transmission using ITA-2 Baudot encoding over AM
- Configurable mark/space frequencies and baud rate via AT commands (defaults: MARK 2125 Hz, SPACE 2295 Hz, 45.45 baud)
- Direct Digital Synthesis sine wave generation at 80 kHz sample rate via Timer1 interrupt
- MCP4901 8-bit SPI DAC output
- 25LC1024 SPI EEPROM (128 KB) for PCM audio sample storage
- 8 kHz PCM audio playback from EEPROM via Timer2 interrupt, AM-modulating the carrier
- Wien bridge oscillator carrier wave at approximately 802 kHz
- Current-modulated cascode RF output stage (PN2222A BJTs)
- Tank circuit output filter with loading coil and SMA antenna connector
- MCP2221A USB-to-UART bridge, appearing as a standard COM port at 9600 8N1
- Powered entirely from USB (5V, USB Type B connector)
- Five user-accessible GPIO pins on a 2x5 pin header (J3), including onboard status LED control
- AVR ISP 10-pin shrouded box header for firmware flashing with USBasp or compatible programmer
- 120 x 60 mm, two-layer FR-4 PCB, designed in KiCad 7 and finalized in KiCad 9
- Companion Python control center application for Windows, Linux, and macOS

---

## Hardware Description

![Board Schematic](docs/images/rtty-transmitter-schematic.jpg)

### USB Interface and Power Supply

The board is connected to a host PC through a USB Type B connector (J1). Power is drawn from the USB 5V rail and distributed across the board through decoupling capacitors at each IC supply pin.

The USB-to-UART bridge is the MCP2221A (U1), a 14-pin DIP chip from Microchip. When connected, it presents itself to the operating system as a virtual COM port, allowing any standard serial terminal or the included control center application to communicate with the ATmega328P at 9600 8N1. No special kernel modules are required on most systems, though the MCP2221 driver package from Microchip's website may need to be installed on Windows for the COM port to appear. The driver can be downloaded from Microchip's product page for the MCP2221A.

### Microcontroller

The central processing element is the ATmega328P (U2), a 28-pin DIP AVR microcontroller running from a 16 MHz crystal (Y1) with no clock prescaler. It handles all firmware tasks: UART communication with the host through the MCP2221A, hardware SPI to the MCP4901 DAC, bit-bang SPI to the 25LC1024 EEPROM, GPIO control, and the two timer-driven interrupt service routines that form the core of the signal generation pipeline.

The reset line is connected through a capacitor to the DTR signal of the MCP2221A, allowing the control center application to reset the board on connect. A 10-pin shrouded box header (J2) exposes the AVR ISP signals (RESET, SCK, MISO, MOSI) for firmware flashing with a USBasp or compatible programmer.

### DAC and DDS Engine

The MCP4901 (U3) is an 8-bit, single-channel SPI DAC in an 8-pin DIP package. It receives samples from the ATmega328P over hardware SPI running at 8 MHz (F_CPU/2 with SPI2X). Timer1 is configured in CTC mode with OCR1A = 199 and no prescaler, producing an interrupt rate of exactly 80 kHz (16 MHz / 200). Each interrupt advances a 32-bit DDS phase accumulator by a pre-computed increment that determines the output frequency:

```
phase_increment = frequency x 2^32 / 80000
```

The upper 8 bits of the phase accumulator index into a 256-entry sine table stored in SRAM, and the resulting sample is written to the DAC in the same ISR. This produces a clean, digitally synthesized sine wave at the desired audio frequency with a sample rate of 80 kHz, giving well over 40 kHz of usable bandwidth and negligible harmonic content in the RTTY audio band. Setting the phase increment to zero mutes the carrier between transmissions.

For RTTY, the firmware uses two phase increments corresponding to the mark and space frequencies. Bit timing is achieved by loading a tick counter in the same ISR: for 45.45 baud, each bit lasts 80000 / 45.45 = 1760 Timer1 ticks, which equals exactly 22 ms.

### EEPROM for PCM Audio Storage

The 25LC1024 (U4) is a 1 Mbit (131072 byte) SPI EEPROM in an 8-pin DIP package. It is connected to the ATmega328P through a bit-bang SPI interface on PORTC (PC0 = /CS, PC3 = MOSI, PC4 = SCK, PC5 = MISO). A separate bit-bang interface is used for the EEPROM rather than the hardware SPI port so that the hardware SPI bus remains dedicated to the DAC and the time-critical 80 kHz Timer1 ISR is not disrupted by EEPROM accesses.

The EEPROM stores 8-bit unsigned PCM audio samples at 8 kHz. During playback, Timer1 is suspended and Timer2 takes over, configured in CTC mode to fire at exactly 8 kHz (OCR2A = 249, prescaler /8 on a 16 MHz clock). Each Timer2 ISR reads one byte from the EEPROM via bit-bang SPI and writes it directly to the DAC, producing an audio waveform that AM-modulates the carrier. End of audio is signaled by 250 consecutive 0xFF samples, which at 8 kHz corresponds to approximately 31 ms of silence, making it a reliable sentinel value.

### Virtual Ground

The audio signal produced by the DAC is centered around Vcc/2 (2.5 V) because the DAC output swings from 0 V to Vref and must be AC-coupled into any subsequent stage that references true ground. Rather than requiring a dual-rail supply, a virtual ground is generated on the board: a resistor divider (R20, R21) sets the midpoint at half the supply voltage, and a buffer stage decouples it from the load, providing a stable 2.5 V reference that the op-amp stage and the RF driver use as their signal ground. This is a standard technique in single-supply audio and RF circuits, and it eliminates the need for a negative supply rail while keeping signal headroom symmetric. A dedicated test point (TP1) is provided on the PCB for probing the virtual ground node.

### Wien Bridge Oscillator and Carrier Wave

The carrier wave is generated by one half of the MCP6022 dual op-amp (U5), wired as a Wien bridge oscillator. A Wien bridge oscillator uses an RC frequency-selective feedback network to produce a clean sine wave at the resonant frequency of the network. The resonant frequency is determined by:

```
f = 1 / (2 x pi x R x C)
```

The component values were chosen to set the oscillation frequency to approximately 802 kHz. This frequency falls in the AM medium-wave broadcast band (526-1606 kHz), which means a standard kitchen AM radio can receive the signal without any special equipment. At 802 kHz the board sits between standard broadcast channels, reducing interference with licensed stations while still being within easy tuning range of any AM receiver.

The MCP6022 was selected for this stage because it is a rail-to-rail op-amp with a gain-bandwidth product of 10 MHz, which is sufficient to sustain oscillation at 802 kHz. Two signal diodes (D5, D6, 1N4148) form an amplitude-limiting network in the feedback loop to stabilize the oscillation amplitude and prevent clipping.

The frequency counter measurement below confirms the oscillator running at 802 kHz as designed. The reading was taken directly at the carrier wave test point (TP3) on the assembled board.

<img src="docs/images/rtty-transmitter-carrier-wave-frequency.jpg" width="200" alt="Carrier wave frequency counter measurement showing 802 kHz">

### Audio Buffer

The second channel of the MCP6022 (U5B) is configured as a unity-gain voltage follower (buffer) for the DAC output. The DAC produces the synthesized audio signal referenced to the virtual ground, and the buffer provides a low-impedance version of this signal to drive the base of the RF output stage without loading the DAC output or pulling the virtual ground node. The buffered audio signal is available at test point TP4.

### RF Output Stage

The carrier wave from the Wien bridge oscillator and the buffered audio signal from the op-amp are combined in a cascode BJT amplifier formed by Q1 and Q2 (PN2222A NPN transistors). This stage operates as a current-modulated multiplier: the audio signal controls the collector current of one transistor while the other is driven by the carrier wave. The result is an amplitude-modulated signal where the carrier envelope follows the instantaneous amplitude of the audio signal, producing conventional double-sideband AM.

The cascode topology improves high-frequency performance and reduces the Miller effect, which is important at 802 kHz where parasitic capacitances can otherwise degrade modulation linearity. The output from Q1/Q2 carries the AM-modulated RF signal at a modest power level, intentionally limited so that the board operates legally within the unlicensed low-power limits applicable in most jurisdictions.

The carrier wave test point (TP3) is available on the PCB for monitoring the modulated output before the tank circuit.

### Tank Circuit and Antenna Output

After the RF output stage, the signal passes through a tank circuit consisting of L1 (1 mH), L2 (330 uH), and C10 (100 nF). The tank circuit acts as a bandpass filter centered on the carrier frequency, suppressing harmonics and spurious products from the amplifier stage before the signal reaches the antenna. L2 also acts as a loading coil, helping to match the electrical length of a short antenna to the transmit frequency and improving radiation efficiency.

The output of the tank circuit connects to J7, an SMA vertical female connector, to which any standard SMA antenna can be attached.

### GPIO Expansion and Status LED

Five GPIO pins (PD2 through PD6, mapped as AT command pins 0 through 4) are brought out to the 2x5 female pin socket header J3. These pins are configured as outputs and can be driven high or low through the AT+write command, making them available for user applications such as driving relays, LEDs, or interfacing with other hardware.

The onboard status LED (D4, white 5mm LED) is connected to PD7 through R6. It is mapped as GPIO pin 5 in the AT command interface and lights automatically during RTTY transmission, EEPROM upload, and audio playback. It can also be controlled directly with AT+write=5,1 and AT+write=5,0.

A tactile reset button (S1, 6x6mm) is connected to the ATmega328P reset line for manual resets without needing to disconnect USB.

### Bill of Materials

| Reference | Qty | Value / Part |
|---|---|---|
| U1 | 1 | MCP2221A-I/P (USB-UART bridge, DIP-14) |
| U2 | 1 | ATmega328P-P (MCU, DIP-28) |
| U3 | 1 | MCP4901 (8-bit SPI DAC, DIP-8) |
| U4 | 1 | 25LC1024 (1 Mbit SPI EEPROM, DIP-8) |
| U5 | 1 | MCP6022 (dual rail-to-rail op-amp, DIP-8) |
| Q1, Q2 | 2 | PN2222A (NPN BJT, TO-92) |
| Y1 | 1 | 16 MHz crystal (HC49-U) |
| L1 | 1 | 1 mH radial inductor |
| L2 | 1 | 330 uH radial inductor |
| D1 | 1 | LED 5mm Yellow |
| D2 | 1 | LED 5mm Green |
| D3 | 1 | LED 5mm Red |
| D4 | 1 | LED 5mm White |
| D5, D6 | 2 | 1N4148 signal diode |
| J1 | 1 | USB Type B connector |
| J2 | 1 | 2x5 shrouded box header (AVR ISP) |
| J3 | 1 | 2x5 female pin header (GPIO) |
| J7 | 1 | SMA vertical female connector |
| S1 | 1 | Tactile button 6x6mm |
| C1,C4,C8,C10,C14,C19 | 6 | 100nF |
| C2 | 1 | 470nF |
| C3,C7,C13,C16,C17 | 5 | 10uF electrolytic |
| C5, C9 | 2 | 22pF (crystal load) |
| C6,C15,C18 | 3 | 100pF |
| C11 | 1 | 10nF |
| C12 | 1 | 1nF |
| R1,R2,R8,R13,R15,R16,R18,R20,R21,R24 | 10 | 10k |
| R3,R4,R5,R9 | 4 | 470R |
| R6, R12 | 2 | 1k |
| R7 | 1 | 22k |
| R10 | 1 | 220R |
| R11 | 1 | 100R |
| R14, R23 | 2 | 1.5k |
| R17 | 1 | 2.2k |
| R19 | 1 | 4.7k |
| R22 | 1 | 15k |

### PCB

![Top View of PCB](docs/images/rtty-transmitter-2.jpg)

The PCB is a two-layer FR-4 board measuring 120 x 60 mm, 1.6 mm thick. It was designed in KiCad 7 and the project was finalized in KiCad 9. The board was manufactured by JLCPCB with green soldermask, white silkscreen, HASL (leaded) surface finish, and 1 oz outer copper weight. All components are through-hole for ease of hand assembly. Gerber files, drill files, and the complete KiCad project files are included in the schematic folder of this repository.

---

## Firmware

The firmware is a single C source file (rtty-transmitter/main.c), written for avr-gcc and targeting the ATmega328P at 16 MHz. It has no external library dependencies beyond the standard avr-libc. The current version is v1.0.0.

### Signal Chain

```
DDS engine (Timer1 ISR, 80 kHz)
  --> MCP4901 DAC (hardware SPI, 8 MHz)
    --> R12 --> op-amp buffer (U5B)
      --> RF output stage (Q2, Q1 cascode)
        --> tank circuit (L1, L2, C10)
          --> SMA antenna connector (J7)

Audio playback (Timer2 ISR, 8 kHz):
  25LC1024 EEPROM --> bit-bang SPI read --> MCP4901 DAC
```

### AT Command Interface

The firmware communicates at 9600 8N1. Commands are terminated with a carriage return (CR, 0x0D). All commands are case-insensitive. The board responds with OK on success or Error: ... on failure. On power-up or reset, the board prints a startup banner listing all available commands and the current default parameter values.

| Command | Description |
|---|---|
| AT | Echo test. Returns OK if the board is responsive. |
| AT+send="message" | Encode and transmit message as Baudot RTTY. Message is converted to upper case. Characters not representable in ITA-2 are silently skipped. |
| AT+set=param,value | Set a runtime parameter. Valid parameters: high (mark frequency in Hz), low (space frequency in Hz), baud (baud rate, e.g. 45.45). |
| AT+get=param | Query the current value of a parameter. Valid parameters: high, low, baud. |
| AT+write=pin,value | Set a GPIO pin. Pin 0-4 map to PD2-PD6 (header J3). Pin 5 controls the status LED on PD7. Value is 0 or 1. |
| AT+play | Play the 8-bit PCM audio stored in EEPROM. Suspends Timer1 and uses Timer2 at 8 kHz. Ends automatically when 250 consecutive 0xFF bytes are encountered. |
| AT+upload | Enter EEPROM upload mode. Accepts raw hex bytes (space or newline separated, e.g. A3 4F 00 ...) and writes them to the EEPROM starting at address 0. The session ends automatically after a configurable idle timeout with no incoming data. |
| AT+reset | Restore factory defaults: MARK = 2125 Hz, SPACE = 2295 Hz, BAUD = 45.45. |
| AT+help | Print a full command reference to the serial terminal. |

Default parameters on startup:

```
MARK  = 2125 Hz
SPACE = 2295 Hz
BAUD  = 45.45
```

These are the standard ITA-2 tones used in traditional 45.45 baud RTTY, chosen for maximum compatibility with RTTY decoders.

### RTTY Encoding

RTTY transmission uses the ITA-2 (Baudot) five-bit character encoding. Each character is framed with one start bit (mark tone), five data bits, and two stop bits (space tone). The firmware supports automatic shift mode switching between the Letters (LTRS) and Figures (FIGS) tables, and inserts the appropriate shift character whenever a mode change is needed. A CR and LF preamble is sent before each message to synchronize the receiver's shift register.

Bit timing is derived directly from the 80 kHz Timer1 tick rate. For 45.45 baud, each bit period is 1760 ticks (80000 / 45.45 = 1760), which corresponds exactly to 22 ms. The firmware spins on the tick countdown in the ISR, so bit timing is cycle-accurate regardless of main-loop activity.

### PCM Audio Playback

During AT+play, Timer1 is stopped, its interrupt mask is saved, and Timer2 is started in CTC mode at 8 kHz. Each Timer2 ISR reads one byte from the EEPROM sequential read stream (opened once at address 0 before playback begins) and writes it to the DAC. This produces the audio waveform that AM-modulates the carrier wave from the Wien bridge oscillator. The LED is lit for the duration of playback. After playback ends, Timer2 is stopped, Timer1 is restarted with its saved interrupt mask, and the firmware returns to the AT command loop.

The audio format for upload is 8-bit unsigned PCM at 8 kHz, mono. Any audio editing tool (Audacity, ffmpeg, SoX) can export in this format. The maximum recording length is 131072 bytes / 8000 samples per second = approximately 16.4 seconds.

### EEPROM Upload Protocol

The AT+upload command puts the firmware into upload mode. The host sends raw byte values as pairs of ASCII hexadecimal digits, optionally separated by spaces or newlines. The firmware accumulates complete byte pairs, converts them from ASCII hex to binary, and writes them to the EEPROM using 32-byte page writes for efficiency. The session ends automatically when no new data arrives within the idle timeout period.

The control center application handles all of this automatically, including computing the correct timing between bytes to avoid overrunning the EEPROM's internal write cycle.

---

## Getting Started

### Requirements

- ATmega328P-based board (this project)
- USB Type B cable
- A PC running Windows, Linux, or macOS
- Python 3.7 or later with the pyserial package (for the control center)
- avr-gcc and avrdude (for building and flashing firmware from the command line)
- A USBasp or compatible AVR ISP programmer (for firmware flashing)

### Drivers

On most Linux distributions no additional drivers are needed. On Windows, if the board does not appear as a COM port after connecting, install the MCP2221 driver from Microchip's website:

https://www.microchip.com/en-us/product/mcp2221a

### Connecting the Board

1. Connect the board to the PC with a USB Type B cable.
2. The power LED should illuminate.
3. On Windows, open Device Manager and confirm a new COM port has appeared under Ports (COM and LPT). On Linux the device will appear as /dev/ttyUSB0 or similar.
4. Open a serial terminal at 9600 8N1 (or launch the control center application) and press Enter. The firmware startup banner should appear, listing the available commands.

---

## Building and Flashing the Firmware

The firmware consists of a single source file: rtty-transmitter/main.c. No special IDE is required.

### Command-Line Build with avr-gcc and avrdude

Install avr-gcc and avrdude for your platform (on Debian/Ubuntu: `sudo apt install gcc-avr binutils-avr avrdude`; on Windows, install WinAVR or the Atmel toolchain).

Navigate to the rtty-transmitter directory and run:

```sh
avr-gcc -mmcu=atmega328p -DF_CPU=16000000UL -O1 \
    -funsigned-char -funsigned-bitfields \
    -ffunction-sections -fdata-sections \
    -fpack-struct -fshort-enums \
    -Wall -o main.elf main.c

avr-objcopy -O ihex main.elf main.hex
```

Then flash with your USBasp programmer:

```sh
avrdude -c usbasp -p m328p -U flash:w:main.hex
```

If avrdude reports a signature mismatch or permission error on Linux, ensure you are in the dialout group or run with sudo.

The final firmware image is approximately 10.4 KB, well within the ATmega328P's 32 KB Flash limit.

### Building with MPLAB X IDE

The repository includes an MPLAB X IDE project in the rtty-transmitter/nbproject directory. The project has two build configurations:

- default: Compiles only, produces the hex file.
- flash: Identical to default, but runs avrdude automatically after a successful build to flash the connected board via USBasp. The post-build step is:

```
avrdude -c usbasp -p m328p -U flash:w:${ImagePath}
```

where ImagePath is the MPLAB X macro pointing to the final production hex. With the flash configuration selected, clicking Build Main Project compiles the firmware and programs the board in a single step.

The project was developed with MPLAB X IDE v6.25 and the Microchip ATmega_DFP 3.3.279 device pack. The compiler used is avr-gcc from the Atmel Studio 7 toolchain.

A typical successful build and flash output looks like this:

```
avrdude: device signature = 0x1e950f (probably m328p)
avrdude: erasing chip
avrdude: writing 10444 bytes flash ...
Writing | ################################################## | 100% 4.62s
avrdude: 10444 bytes of flash verified
avrdude done.  Thank you.
BUILD SUCCESSFUL (total time: 10s)
```

---

## Control Center Application

![RTTY Transmitter Control Center](docs/images/rtty-transmitter-control-center.png)

The tools/rtty-transmitter-control-center.py Python script provides a graphical user interface for all board functions. It is built with Tkinter and requires no installation beyond Python 3 and the pyserial package.

### Installation

```sh
pip install pyserial
python tools/rtty-transmitter-control-center.py
```

### Usage

The control center window is divided into functional panels:

**Connection:** Select the COM port from the dropdown (the list refreshes automatically) and click Connect. The board will reset on connection and the startup banner will appear in the command log.

**RTTY Transmission:** Type a message in the text field and click Send. The message is automatically filtered to the ITA-2 character set, and characters that cannot be represented in Baudot are shown as unsupported before sending. A repetition counter allows the message to be sent multiple times consecutively. Unsupported characters are highlighted so you can edit the message before transmitting.

**File Transmission:** Click Send File to select a plain text file. The application reads the file, filters it to the ITA-2 character set, and splits it into chunks of up to 116 characters (the maximum that fits in the ATmega328P's 128-byte command buffer after the AT+send="..." wrapper). Each chunk is sent as a separate AT+send command, with the application waiting for the board to acknowledge each one before sending the next.

**PCM Audio Upload:** Click Upload Audio to select a raw 8-bit unsigned PCM file at 8 kHz. The application converts the file to a hex stream and uploads it to the EEPROM via AT+upload, displaying a progress bar. After upload completes, use the Play button or send AT+play from the manual command field to play back the audio.

**GPIO Control:** Six toggle buttons correspond to GPIO pins 0 through 5 (PD2 through PD7 on the ATmega328P). Pin 5 controls the onboard status LED. Each button sends an AT+write command immediately when clicked.

**Manual Command Entry:** Any AT command can be typed into the manual entry field and sent with the Enter key or the Send button, allowing direct interaction with the firmware.

**Command Log:** All transmitted commands and received responses are shown in a color-coded scrolling log. TX entries are shown in one color, RX in another, system messages in a third, and errors are highlighted separately for easy diagnosis.

---

## Antenna Selection and Range

![Board with Antennas](docs/images/rtty-transmitter-3.jpg)

The SMA connector (J7) accepts any standard SMA antenna. The board was tested with two antennas:

A generic ISM-band whip antenna (in the image above the PCB) primarily designed for 433/868/915 MHz use. While functional as a receive and general-purpose antenna, its electrical length is far shorter than the quarter-wave length at 802 kHz (approximately 93 meters), so it is not well matched to the transmit frequency and delivers limited range.

A telescopic SRH-789 multiband antenna (in the image below the PCB), which can be extended to approximately 1 meter. This antenna performs noticeably better at 802 kHz, and with it a range of a few meters was achieved in testing.

In both cases, a properly resonant loading coil or a ground-plane matched to the medium-wave band would significantly improve performance. With the default low-power output stage and a short antenna, the practical range of this board is limited to a few meters, which is entirely by design and appropriate for a bench demonstration or close-range testing setup.

Note that transmission in the AM medium-wave band is regulated in most countries. Always verify your local regulations before transmitting. See the Regulatory Notice section below.

---

## Testing and Reception

The board was tested using a standard portable AM/FM kitchen radio tuned to 802 kHz. With the radio placed within range, the RTTY tone bursts are clearly audible as alternating beeps corresponding to the mark and space tones.

For decoding, GRITTY 1.3 by Afreet Software was used. GRITTY is a free RTTY decoding program that processes the audio received by a soundcard and decodes the bit stream in real time. In the test setup, the radio received the AM signal and played back the audio tones, which were picked up by the laptop's built-in microphone and fed into GRITTY, which successfully decoded the transmitted text.

The AT+play audio playback function was also tested. A short 8-bit, 8 kHz PCM audio sample was uploaded to the EEPROM using the control center application and played back with AT+play. The audio was clearly audible through the kitchen radio receiver, with good intelligibility and no significant distortion. The 8 kHz sample rate is sufficient for voice and simple audio content, and the AM carrier does a faithful job of conveying the waveform at close range.

One observation during testing: the SPI interface between the ATmega328P and the EEPROM can produce noise that couples into the RF output, sometimes audible as a faint background interference. Standard bypass capacitors are placed at the relevant supply pins, but as with any RF design, further layout optimization and shielding can always improve results.

---

## Comparison with Official RTTY

This board transmits RTTY tones using conventional AM (amplitude modulation): the audio FSK signal from the DDS engine directly modulates the amplitude of the 802 kHz carrier produced by the Wien bridge oscillator. Understanding how this differs from professional RTTY is useful for anyone looking to extend or adapt the project.

**Modulation method.** Official RTTY, as used by amateur radio operators, coast stations, and maritime services, uses SSB (single-sideband) modulation. The operator feeds the FSK audio tones into the microphone or audio input of an SSB transceiver, and the transceiver shifts those tones up to the operating frequency, transmitting only one sideband. The mark and space tones then appear on the air as two discrete RF frequencies separated by the audio shift (typically 170 Hz for narrow-shift RTTY, or 850 Hz for wide-shift). In AM mode as used by this board, both sidebands are transmitted and the carrier is present at full strength, which wastes power and doubles the occupied bandwidth.

**Frequency bands.** Amateur RTTY operates on designated segments within the HF amateur bands. Typical allocations and commonly used frequencies are:

- 80 meters: around 3580-3600 kHz (Europe) and 3570-3600 kHz (North America)
- 40 meters: around 7035-7043 kHz (Europe) and 7080-7125 kHz (North America)
- 20 meters: around 14080-14099 kHz, with heavy activity at 14085 kHz
- 15 meters: around 21080-21099 kHz
- 10 meters: around 28080-28099 kHz

This board transmits at 802 kHz, which is in the AM medium-wave broadcast band (526-1606 kHz). This band is not allocated for amateur RTTY. It was chosen here because a standard AM kitchen radio can receive it without any specialized equipment, making the board immediately useful for demonstration and experimentation.

**Baud rate and shift.** The most common amateur RTTY standard is 45.45 baud with a 170 Hz shift. This means the mark tone sits at one audio frequency and the space tone sits 170 Hz away. This firmware defaults to 2125 Hz (mark) and 2295 Hz (space), which is exactly the 170 Hz narrow-shift standard used in modern amateur RTTY, making it directly compatible with any ITA-2 RTTY decoder such as GRITTY, Fldigi, or MMTTY when the audio is recovered from the AM carrier.

Some older or maritime RTTY systems used 850 Hz wide-shift at 50 baud, and some weather fax and RTTY broadcast services used 50 baud with 450 Hz shift. The firmware's AT+set command allows the mark frequency, space frequency, and baud rate to be changed freely, so any of these variants can be reproduced.

**What a typical professional RTTY transmitter looks like.** A conventional amateur RTTY station consists of an SSB transceiver (for example a Kenwood TS-590, Icom IC-7300, or Yaesu FT-991) connected to a PC running a soundcard-based RTTY modem such as Fldigi or MMTTY. The software generates the FSK audio tones at 2125 Hz and 2295 Hz, sends them into the transceiver's audio input, and the transceiver upconverts and transmits them on, say, 14085 kHz USB. The radiated signal occupies roughly 500 Hz of spectrum, carries no pilot carrier, and can be received thousands of kilometers away with a modest antenna and a sensitivity-optimized receiver.

**How to turn this board into a proper RTTY transmitter.** The firmware is already producing standard ITA-2 Baudot encoding at the correct baud rate and shift. The DDS audio engine output is present at test point TP4 (buffered audio). To transmit proper SSB RTTY on the amateur bands, this audio output could be fed directly into the microphone input or the auxiliary audio input of any SSB transceiver, bypassing the Wien bridge oscillator and the on-board RF stage entirely. The transceiver then handles upconversion, amplification, and antenna matching at the desired amateur frequency. No firmware changes would be needed, since the mark/space tones and baud rate are already standard.

---

## Regulatory Notice

Transmission on the AM medium-wave band (526-1606 kHz) is regulated in most countries. In many jurisdictions, unlicensed low-power transmitters are permitted below a defined field strength limit, but the rules vary significantly by country and frequency band. It is the user's responsibility to verify the applicable regulations in their country before operating this device as a transmitter. The board's output power is intentionally low, and with a short, unmatched antenna the radiated field is extremely limited, but this does not substitute for checking local law.

---

## Personal Note

I have always been fascinated by RTTY and medium-wave AM transmitters. The first idea for a project like this came when I was a kid, experimenting with home-built oscillators and imagining the day I would be able to design something that could transmit a signal to a small radio across the room. This board is a milestone in a long journey, and I am glad it worked out. The journey continues.
