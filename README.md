This project implements a cycle-aware 6502 CPU emulator running on an STM32F401RE, hosted by a custom preemptive sempaphore-based RTOS written from scratch. 

The emulator executes one 6502 instruction per timer event in debugger-friendly step mode, allowing full inspection of registers, flags, memory, and cycle counts at each instruction boundary.

<img width="241" height="154" alt="Screenshot 2026-01-28 at 3 02 39 AM" src="https://github.com/user-attachments/assets/8cca92b1-48b6-4c1f-9323-75c19fd4244f" />

Demo with an older version of the code:



https://github.com/user-attachments/assets/345b53f4-a585-42f3-9e49-c3a5b6faf61e


