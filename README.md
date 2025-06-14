# Posix Test

Two projects are used to compose the grade for the Posix course (Stellantis program in partnership with UFPE), leveraging core POSIX concepts such as processes, threads, mutexes, semaphores, shared memory, signal handling, and message queues.

---

## Project 1: Car Controller Virtual Simulation

In this project, POSIX concepts are applied to simulate the activation of actuators (including ADAS activation when necessary). The system is composed of three processes:

1. **Sensor Process**  
   – Spawns threads that generate random values for vehicle speed, engine temperature, and engine RPM.  
2. **Panel Process**  
   – Reads the sensor values and prompts the user to enter commands to activate actuators. These commands are sent via a message queue.  
3. **Controller Process**  
   – Retrieves commands from the message queue and activates the corresponding actuators (and ADAS if required).  
4. **Sequencing with Semaphores**  
   – Semaphores ensure that the processes execute in the defined order (Sensor → Panel → Controller).

### How to Run

1. cd path/to/project1
2. open a terminal execute "make"
3. Execute "./program" command at the same terminal


### Videos
   
    - Explanation Video:
    - https://drive.google.com/file/d/1Qw0hX_vkCu9jvhqGbV-jVTibWe3c0SEA/view?usp=sharing

    - Running Video:
    - https://drive.google.com/file/d/1Tl1XqonlVNH2gg_Mnt_5RyZBKAdfcBcV/view?usp=sharing

## Project 2: Raspberry Pi Implementation

This project applies the same concepts on a Raspberry Pi board. It monitors button presses to activate actuators (and ADAS when requested). Vehicle speed is calculated via an ISR “beep” that triggers every time the wheel completes one rotation.

### How to Run(Assuming you have cross‑compiled or compiled on the Pi itself)

1. cd path/to/project2 
2. open make 
3. ./program 

## **Videos**

    Explanation Video:
    https://drive.google.com/file/d/1N3m4Gwt_u89IhHNt-pSYXOBnFeKVtcaa/view?usp=drive_link

    Running Video:
    https://drive.google.com/file/d/1c4MYvrdS6a6ZHhLw6Xvivmq0wvkxOmz0/view?usp=drive_link
