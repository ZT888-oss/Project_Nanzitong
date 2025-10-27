# Laser Dodger Game 🎮

A real-time VGA game implemented in C for an FPGA-based system. Avoid moving boxes and lasers while testing your reflexes!

---

## Overview
**Laser Dodger** is a hardware-level game where the player controls a square (“player”) to dodge moving boxes connected by lines.  
It demonstrates graphics programming, input handling, collision detection, and animation logic directly on an FPGA.

---

## Features
- Smooth VGA graphics using double buffering  
- Eight moving boxes with velocity-based physics  
- Lines dynamically connecting the boxes  
- Real-time keyboard input (PS/2)  
- Pixel-perfect collision detection  
- Modular C code for maintainable hardware interfacing  

---

## How It Works
1. **Graphics:** 320x240 resolution, double-buffered to prevent flickering.  
2. **Input:** Arrow keys move the player in real-time.  
3. **Collision Detection:** Detects overlap between player and box “laser” zones.  
4. **Game Loop:** Updates player, moves boxes, redraws screen, and swaps buffers for smooth animation.  

---

## Controls
| Key         | Action     |
|-------------|-----------|
| Up Arrow    | Move Up    |
| Down Arrow  | Move Down  |
| Left Arrow  | Move Left  |
| Right Arrow | Move Right |

---

## Code Highlights
- **Double Buffering:** Smooth animation without flicker  
- **Bresenham Line Algorithm:** Efficient line drawing between boxes  
- **Modular Functions:** Separate logic for boxes, player, collision, and rendering  
- **Hardware Interfacing:** Direct VGA pixel buffer manipulation and PS/2 keyboard input  

---

## Getting Started
1. Set up the FPGA development environment (e.g., DE-series board with Nios II).  
2. Compile the C code with the toolchain.  
3. Load the executable onto the FPGA.  
4. Play the game using arrow keys.  

*Note:* Designed for FPGA hardware with PS/2 keyboard and VGA output.

---

## Skills Demonstrated
- Embedded C programming  
- Real-time graphics and input handling  
- Low-level hardware interfacing (VGA, PS/2)  
- Collision detection algorithms  
- Modular, maintainable coding practices  

---

