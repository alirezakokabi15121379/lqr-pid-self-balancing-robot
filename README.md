# LQR-PID Hybrid Controller for Two-Wheeled Self-Balancing Robot

**MATLAB simulation of a hybrid LQR-PID controller for a two-wheeled balancing robot**

This project designs and compares three controllers (LQR, PID, and hybrid LQR-PID) to stabilize a two-wheeled self-balancing robot under large tilt angles (up to ±20°).

### Key Features
- Full nonlinear dynamic modeling using Lagrange method
- Linearized state-space model for controller design
- Hybrid LQR-PID with smooth switching based on tilt angle
- Simulation of disturbance rejection (10° impulse at t=15s)

### Performance
- **Hybrid LQR-PID** showed the best performance
- Fast settling time and minimal overshoot
- Superior to standalone LQR (limited to ±5°) and PID (oscillatory)

### Technologies
- MATLAB 
- LQR (Linear Quadratic Regulator)
- PID (tuned with `pidtune`)
- State-space control

### Repository Contents
- `main_simulation.m` – Complete simulation script
- `lqr_pid_hybrid.m` – Hybrid controller implementation
- All simulation plots (roll, pitch, yaw responses)

Made by Alireza Kokabi Dezfuli  
