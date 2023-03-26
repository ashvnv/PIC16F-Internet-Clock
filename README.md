# PIC16F-Internet-Clock
Clock which can sync time from Internet and uses RTC for Timekeeping

![MIT License](https://img.shields.io/github/license/ashvnv/PIC16F-Internet-Clock)
![Language: Embedded C](https://img.shields.io/badge/language-Embedded%20C-red)

The Clock uses PIC16F887 chip (compatible with all PIC16F88x series) as the main microcontroller. Internet Time is obtained using ESP8266-01. Time can also be Set Manually using switches. Using RTC DS1307 timekeeping is achieved. Time is shown in 4 Digit 7-segment-displays driven by MAX7219. The Brightness of the display can be adjusted using a Potentiometer.

### No library functions are used while programming PIC. All the peripherals are controlled manually by manipulating the specific peripheral registers for achieving a functionality like I2C, UART etc.

<img src="https://github.com/ashvnv/PIC16F-Internet-Clock/blob/main/Images/Internet%20Clock.jpg" width="500" />

### Refer to the complete Documentation of the Project for more details on the Implementation
[![Download Documentation](https://custom-icon-badges.herokuapp.com/badge/-Documentation-blue?style=for-the-badge&logo=download&logoColor=white "Download Documentation")](https://raw.githubusercontent.com/ashvnv/PIC16F-Internet-Clock/main/Documentation.pdf)

---

## Software Used
| Software | Version | Used for |
| ------------- | ------------- | ------------- |
| MPLAB X IDE | 6.05 | Configuration Bits |
| mikroC Pro | 7.6.0 | PIC Programming |
| QL-PROG for QL-2006 | 2.37 | PIC Burner |
| Proteus Design Suite | 8.9 SP2 | Simulation |
| Arduino IDE | 1.8.19 | ESP8266-01 Programming |

## Project Directory Structure
- Block Diagram & Firmware Flowchart 
  - Contains the Block diagram of the project and algorithm flowchart
- Datasheets
  - Contains the datasheet of all the components used
- Final Simulation (OVERALL)
  - Proteus Simulation File of the project
- Firmware
  - mikroC PIC firmware and ESP8266-01 Arduino IDE firmware
- Images
  - Some images of the project
- Documentation.pdf
  - Detailed Implementation
- Test Report.pdf
  - Project functionality testing

## Block Diagram
<img src="https://github.com/ashvnv/PIC16F-Internet-Clock/blob/main/Block%20Diagrams%20%26%20Firmware%20Flowchart/Internet%20Clock%20BD.jpg" width="500" />

## Switch Functions
<img src="https://github.com/ashvnv/PIC16F-Internet-Clock/blob/main/Block%20Diagrams%20%26%20Firmware%20Flowchart/Switch%20Function.jpg" width="500" />

## PIC Firmware Flowchart
<img src="https://github.com/ashvnv/PIC16F-Internet-Clock/blob/main/Block%20Diagrams%20%26%20Firmware%20Flowchart/PIC%20Firmware%20Flowchart.jpg" width="900" />

## ESP8266-01 Firmware Flowchart
<img src="https://github.com/ashvnv/PIC16F-Internet-Clock/blob/main/Block%20Diagrams%20%26%20Firmware%20Flowchart/ESP-01%20Firmware%20Flowchart.jpg" width="600" />
