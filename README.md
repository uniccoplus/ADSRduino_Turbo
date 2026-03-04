# ADSRduino Turbo
This is a simple ADSR envelope generator for the **Arduino**

original description at http://m0xpd.blogspot.co.uk/2017/02/signal-processing-on-arduino.html

The standard Arduino is **ATMEGA328P**, and running at 5V is convenient as hardware.

The original is "Arduino Nano", and as of 2026, it is "Arduino Nano R4".  It is expected to be 8x faster.

However, since it is an **8-bit MPU**, it is not good at floating-point arithmetic.

Therefore, in the original, the voltage was updated at a sample rate of *3.3 kHz*, so the 12-bit DAC was running like a 7-bit DAC.

Change to another platform? My conclusion is *No*.

## Features

Arduino Uno/Nano/Nano R4 Support.

The main result was that optimizing cos() and sqrt() was **750x faster**.

The SPI part, it has been increased by **62x faster**.

It works **10x faster** in total and produces a smooth, smooth waveform.

It's faster than expected, so you can consider midification.

## Hardware changes

There is a change in the SPI connection pin

|       Old     |        New         |
|---------------|--------------------|
|DAC_SCK → pin 5|pin 13 (PB5, HW SCK)|
|DAC_SDI → pin 4|pin 11 (PB3, HW MOSI)|


**ATmega328P-PU (DIP-28) compatibility table**

|Pin name on Nano|Function used|pin # of ATmega328P-PU (DIP-28)|Port/pin name|Arduino pin # (in sketch)| Need to change? |   Notes    |
|----------------|-------------|------------------|-------------|------------|-----------------|------------|
|A0|ADC Input 1|23|PC0 (ADC0/PCINT8)|A0|Not required|Exactly the same function|
|A1|ADC Input 2|24|PC1 (ADC1/PCINT9)|A1|Not required|Exactly the same|
|A2|ADC Input 3|25|PC2 (ADC2/PCINT10)|A2|Not required|Exactly the same|
|A3|ADC Input 4|26|PC3 (ADC3/PCINT11)|A3|Not required|Exactly the same|
|5.5V|Power Supply (5V Output/Logic Power Supply)|7 and 20 (VCC) + 18 (AVCC)|VCC / AVCC|—|Must be connected|"Connect VCC (7,20) and AVCC (18) to 5V (AVCC filtering recommended)"|
|GND|Power Supply (Ground)|8,22|GND|—|Required connection|Connect both to GND (multiple connections recommended for noise prevention)|
|RAW|+12V input (external power supply)|18 (requires external regulator)|—|—|Requires external|DIP does not have a regulator → Supply VCC/AVCC with an external 5V regulator (such as a 7805)|
|D2|Input: 5V pulse input|4|PD2 (INT0/PCINT18)|2|Not required|Supports external interrupt INT0|
|D3|Input: PushSW (press to GND)|5|PD3 (INT1/OC2B/PCINT19)|3|Not required|Internal/external pull-up resistor (INPUT_PULLUP available)|
|D6|DAC_CS (chip select)|12|PD6 (AIN0/OC0A/PCINT22)|6|Not required|PWM-compatible pin|
|D8|LDAC|14|PB0 (CLKO/ICP1/PCINT0) | 8 | Not required | Standard digital I/O |
|D11 | DAC_SDI (equivalent to MOSI) | 17 | PB3 (MOSI/OC2A/PCINT3) | 11 | Not required | SPI MOSI |
|D13 | DAC_SCK | 19 | PB5 (SCK/PCINT5) | 13 | Not required | SPI SCK |

**Additional required pins (when using DIP-28 alone)**
|Function|DIP-28 Physical Pin|Connection Method|Notes|
|--------|-----------|----------------------------------------|-------------------------------|
|RESET|1|Connect to VCC (5V) with a 10kΩ pull-up resistor + 0.1uF capacitor to GND (optional)|For reset. Required if bootloader is already programmed.|
|XTAL1 / XTAL2|9,10|16MHz crystal + 22pF capacitors to GND (when using an external clock)|Not required when using the internal oscillator, but recommended if accuracy is required.|
|AREF|21|Connect directly to VCC or external reference voltage (add a capacitor if ADC accuracy is important).|When using an ADC, connect close to AVCC to reduce noise.|
|AVCC|18|Connect to 5V (decoupled to GND with 0.1uF + 10uF capacitors).|ADC power supply. Recommended to be separated from VCC.|
|Loader|2,3|Connect to USB-Serial and PC||

The Arduino Nano uses the TQFP-32 version of the ATmega328P, so A6 and A7 (ADC6/ADC7) exist, but the DIP-28 does not physically have ADC6 and ADC7 (the ADC only has 6 channels, A0 to A5).

