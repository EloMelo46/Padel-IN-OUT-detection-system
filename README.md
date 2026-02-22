# Padel-IN-OUT-Detection-System

## Overview

This project implements a hardware-based solution to objectively determine ball impact order in Padel. Instead of relying on cameras or image processing, the system uses direct physical sensing combined with precise timing evaluation.

The focus lies on reliability, fast response, and clear decision logic under realistic playing conditions.

## Detection Mechanism

Three infrared laser barriers are positioned at floor level. When the ball interrupts a beam, the corresponding signal changes immediately and is processed by the controller.

A force-sensitive resistor mounted on the glass wall detects mechanical impact. The measured voltage drop indicates a wall hit.

The microcontroller compares the timestamps of both events. If the floor event occurs first, the system outputs an IN signal. If the wall event occurs first, the result is classified as OUT.

## System Design

An ARM-based microcontroller (Arduino Due) handles sensor acquisition and state evaluation. The firmware is structured as a finite-state machine to ensure deterministic behavior.

Visual feedback is provided through red and green LEDs. Sensor monitoring LEDs support alignment and diagnostics during installation.

Separate voltage domains are used for logic and signaling to maintain electrical stability.

Floor sensors can be temporarily disabled if continuously blocked, reducing the risk of incorrect decisions during player movement.


## Hardware Components

The system is built using the following hardware components:

- Arduino Due (ARM Cortex-M3, 84 MHz)  
- Three infrared laser modules (850 nm)  
- Three photodiodes for beam detection  
- Force Sensitive Resistor (FSR) array for wall impact detection  
- External red and green LED indicators  
- Three floor status LEDs  
- Three FSR status LEDs  
- 5V logic power supply  
- 24V supply for LED signaling  
- Relay module for external LED control  
- Aluminum mounting brackets for laser alignment  
- Plexiglass wall segment  
- Artificial grass surface for realistic testing  
- Wooden base structure for prototype integration

All electronic components are integrated inside a custom-built electrical control enclosure.

## Results

### Control Box 
![Control Box 1](media/Control_box_1.jpg)

![Control Box 2](media/Control_box_2.jpg)

![Control Box 3](media/Control_box_3.jpg)


## Scaled Padel Court Test Platform

### Test Platform 1
![Padel Court Test Platform 1](media/Padel_Court_Test_Platform_1.jpg)

![Padel Court Test Platform 2](media/Padel_Court_Test_Platform_2.jpg)


## Demonstration

### System Test Video
[Watch test demonstration](media/test.mp4)
