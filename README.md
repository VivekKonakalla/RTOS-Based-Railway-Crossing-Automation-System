# RTOS-Based Railway Crossing Automation System

**Post Graduate Diploma in Embedded Systems Design (PG-DESD)**  
**C-DAC Bangalore**  
**Academic Period: August 2025 – February 2026**

## Team Members
- A. Rohith Naga Sai (PRN: 250850130004)  
- Dhanush Gowda K (PRN: 250850130012)  
- Kapil Rajgopal (PRN: 250850130017)  
- Konakalla Vivek Sri Krishna Chaitanya (PRN: 250850130021)  

**Supervisor:** Pranav Sangar, Embedded Software Engineer, C-DAC Bangalore

## Project Overview
This project implements a fully automated railway level crossing gate system using the **STM32H753ZI** microcontroller (Nucleo-H753ZI board). The system features:

- Dual independent servo-operated gates  
- IR-based train arrival/exit detection  
- Ultrasonic obstacle detection with independent per-gate safety pause  
- LED traffic lights (Red/Yellow/Green) and buzzers for feedback  
- Two versions: **bare-metal** (non-RTOS) and **FreeRTOS-based** (CMSIS-V2)  

The RTOS version uses tasks, event flags, mutexes, and queues for true concurrency and deterministic behavior.

## Key Features
- Gates close in ~1.8–2 seconds (10° steps @ 100 ms)  
- Independent 90° pause (1 s) if obstacle detected (2–15 cm)  
- 3-second train passing delay + 1-second open delay  
- Yellow warning blink (2 s) before closing  
- Detailed UART logging (115200 baud)  

## Repository Structure

```text
RTOS-Based-Railway-Crossing-Automation-System/
├── README.md                     # Project overview & documentation
├── LICENSE                       # MIT License
├── .gitignore                    # Ignores STM32CubeIDE build files
│
├── Report/
│   └── PG-DESD_Final_Report.pdf  # Complete final report
│
├── Code/
│   ├── Bare-Metal/               # STM32CubeIDE project – non-RTOS version
│   │   ├── RailwayCrossing.ioc
│   │   └── main.c
│   │
│   └── FreeRTOS/                 # STM32CubeIDE project – FreeRTOS version
│       ├── RailwayCrossingRTOS.ioc
│       └── main.c
│
├── Images/                       # All diagrams & screenshots
│   ├── Fig. 1.1   Block diagram of the automated railway level crossing system 
│   ├── Fig. 4.1   High-level system architecture (bare-metal vs RTOS) 
│   ├── Fig. 5.1   MCU pin configuration in STM32CubeIDE 
│   ├── Fig. 5.3   State transition diagram of the system
│   ├── Fig. 6.3.1   Photograph of the demonstrated project prototype
│   └── Fig. 6.3.2   Screenshot of UART log showing independent gate pause (RTOS)
│
└── Presentation/                 # Presentation slides
    └── PG-DESD_Presentation.pptx

## Hardware Requirements
- Nucleo-STM32H753ZI board  
- 2 × MG995 Servo Motors  
- 2 × HC-SR04 Ultrasonic Sensors  
- 2 × IR Obstacle Sensors  
- 2 × LED Traffic Light Modules  
- 2 × 5V Active Buzzers  
- Jumper wires, breadboard, 5V power supply for servos

## Software Requirements
- STM32CubeIDE (latest version)  
- STM32CubeMX (integrated)  
- FreeRTOS CMSIS-V2 middleware (for RTOS version)  
- USB-to-Serial terminal (PuTTY/Tera Term) @ 115200 baud

## How to Run
1. Open the desired project folder (`bare-metal` or `freertos`) in STM32CubeIDE  
2. Build → Debug (ST-LINK)  
3. Open serial terminal (115200, 8N1)  
4. Observe UART logs and gate/LED/buzzer behavior  
5. Trigger IR sensors (simulate train) and place object near ultrasonic to test pause

## Screenshots
![UART Log - Independent Pause](images/uart_log_screenshot.png)  
*Fig. 6.1: UART log showing Gate 2 pause while Gate 1 continues (RTOS)*

![Hardware Prototype](images/project_hardware_photo.jpg)  
*Fig. 6.3: Photograph of the demonstrated project prototype*

## Future Work
- Predictive maintenance using ML/DL on STM32CubeIDE (servo health monitoring)  
- IoT integration (GSM/Wi-Fi alerts)  
- Camera-based obstacle classification

## License
This project is licensed under the MIT License – see the [LICENSE](LICENSE) file for details.

---

Made with ❤️ for PG-DESD @ C-DAC Bangalore (2025–2026)
