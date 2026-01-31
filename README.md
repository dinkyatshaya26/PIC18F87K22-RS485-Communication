# PIC18F87K22 RS-485 Communication (Master–Slave)

This project demonstrates RS-485 based serial communication between two PIC18F87K22 microcontrollers using UART and MAX487 transceivers.

## Hardware Used
- PIC18F87K22 (Master)
- PIC18F87K22 (Slave)
- MAX487 RS-485 Transceiver (2 units)
- 16x2 LCD (Slave side)
- Potentiometer (LCD contrast)
- Resistors for A/B line termination
- Common GND connection

## Communication Details
- Protocol: UART over RS-485
- Baud Rate: 9600
- Clock Frequency: 8 MHz (Internal Oscillator)
- Mode: Asynchronous
- Data: 8-bit, No parity, 1 stop bit

## Project Description
### Master Node
- Initializes UART and RS-485 driver
- Controls RE/DE pin for transmit mode
- Sends the message:
             HELLO FROM MASTER


### Slave Node
- Operates in receive mode
- Reads UART data via RS-485
- Displays received characters on a 16x2 LCD

## Pin Usage
- UART TX/RX: RC6 / RC7
- RS-485 RE/DE control
- LCD Data & Control: PORTD

## Tools Used
- MPLAB X IDE
- XC8 Compiler
- Proteus (for circuit simulation)

## Notes
- Master and Slave firmware are compiled separately.
- RE/DE pin is toggled to control RS-485 direction.
- This project focuses on low-level register configuration without libraries.

## Applications
- Industrial communication
- Multi-drop serial networks
- Embedded control systems