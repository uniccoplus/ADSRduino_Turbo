# ADSRduino
This is a simple ADSR envelope generator for the Arduino

original description at http://m0xpd.blogspot.co.uk/2017/02/signal-processing-on-arduino.html

The standard Arduino is ATMEGA328P, and running at 5V is convenient as hardware.
However, since it is an 8-bit MPU, it is not good at floating-point arithmetic.
Therefore, in the original, the voltage was updated at a sample rate of 3.3 kHz, so the 12-bit DAC was running like a 7-bit DAC.
Change to another platform? My conclusion is No.

The main result was that optimizing cos() and sqrt() was 750x faster.
The SPI part, it has been increased by 62x faster.
It works 10x faster in total and produces a smooth, smooth waveform.
It's faster than expected, so you can consider midification.

There is a change in the SPI connection pin
Old: DAC_SCK → pin 5
New: pin 13 (PB5, HW SCK) 
Old: DAC_SDI → pin 4
New: pin 11 (PB3, HW MOSI)
