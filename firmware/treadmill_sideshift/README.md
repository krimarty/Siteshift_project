# Treadmill sideshift module – firmware

## Project Overview
This is the Arduino IDE code for the Raspberry Pi Pico 2, which runs on the PCB `treadmill_sideshift_module v1.0.0`.

## Code structure
```bash
├── treadmill_sideshift.ino     # Main project file 
├── src/
├── include/                    # Classes used by main hardware handling, algorithms
│   ├── I2CCommunication.h
│   ├── ICommunication.h
│   ├── Motor.h
│   ├── MotorFeedback.h
│   ├── OtherDevice.h
│   ├── Pid.h
│   ├── PositionCalculator.h
│   ├── UARTCommunication.h
│   └── UserInterface.h
└── test/                       # Codes for solo hardware testing, other device simulation
    ├── ICommunication_test/   
    ├── Motor_test/            
    ├── MotorFeedback_test/     
    ├── SynMode_test/          
    └── UserInterface_test/    
```

## Hardware requirements
- Raspberry Pi Pico 2 on treadmill sideshift PCB v1.0.0
- Linear actuator Linak LA14 (12 VDC) with analog feedback 0–10 V
- Off-board buttons for manual control

## Authors
- Developed by Martin Kriz  

