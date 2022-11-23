# MasterFOC-Brief

This project contains:
1. The simulation of FOC algorithm using MATLAB, which includes,
  - The basic Clarke, Park transformation module
  - The SVPWM generation module
  - The FOC Current Loop Simulink Model
  - The FOC Speed Loop Simulink Model
  - The FOC Position Loop Simulink Model
  
2. Dual Motor driver library that suitable for STM32 microcontroller
  - [Documentation]()
  - Library structure
  - ![image](https://user-images.githubusercontent.com/82952761/203454754-2094ec53-be69-4241-af8b-210347a4c474.png)
  
  
3. STM32 Dual motor control board PCB file
  
![实物仿真图222](https://user-images.githubusercontent.com/82952761/203455180-d4bc92d9-d819-45d6-843c-e2b599aa0daf.png)

- The control board used a four-layer structure, the size was minimized to only 10 cm * 7.3 cm. It used an STM32F405RGT6 as the main controller, two DRV8302 as the gate driver and the detailed capability and features were: 
-	Power:
    - 24 V 4.6 A theoretical dual-channel Motor driving ability.
-	Control:
    -	Double channel BLDC FOC control (both sensored and sensorless).
    -	The potential of servo motor control
-	Measurement:
    -	ADC phase current and phase voltage sensing.
    -	Support double-channel PWM and I2C encoder.
-	Communication:
    -	UART to USB serial communication port.
    -	USB OTG port.
    -	CAN port.
    -	Serial Wire debug port.
-	Safety:
    -	Temperature sensing, with the ability of over-temperature protection.
    -	Supply voltage sensing to provide under-voltage protection.

4. Test result for this library
  - Speed test:
  

https://user-images.githubusercontent.com/82952761/203460799-ee4c0337-1777-4134-aac8-05b7b4c1f8ba.mp4


  - Angle test:
  

https://user-images.githubusercontent.com/82952761/203460561-26430715-400c-463d-a066-c38650719768.mp4


  - Drag mode, Force feedback mode, and multiple-stage switch mode test:
  
  

https://user-images.githubusercontent.com/82952761/203461097-02575e66-421a-477a-82b6-556301c99445.mp4


  
