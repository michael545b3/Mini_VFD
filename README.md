# Mini VFD based on STM32

![Mini VFD](06_Pictures/Mini%20VFD.jpg)

This project is a compact Variable Frequency Drive (VFD) built around an  
STM32 microcontroller and an ST IPM (Intelligent Power Module).  
It is designed to control small AC motors efficiently and safely in a compact form factor.

---

## Hardware Overview

### Core Components
- **STM32F407** microcontroller
- **STGIPN3H60** IPM (Intelligent Power Module)
- **High-voltage measurement**
- **Analog inputs** for control signals
- **Opto-isolated digital inputs**
- **Relay output**
- **EEPROM** for parameter storage

### Power Supply
- **Flyback converter** for isolated supply
- **E-Fuse** for protection
- **Buck converter** with **LDOs** for logic power

### User Interface
- **LCD display**
- **Push buttons** and **rotary encoder** for input

---

## Software Status

The firmware is currently under development. As a hardware-focused developer,  
progress is steady but slow — writing robust embedded software takes time.

So far, the implemented functions run reliably, and more features are being added continuously.  
A demonstration video will be available soon.

---

## Roadmap / To-Do

- [ ] Motor control tuning
- [ ] UI menu system improvements
- [ ] Fault monitoring and protection
- [ ] IO implementation

---

## Contributions

Contributions, feedback, or suggestions are very welcome!

---
