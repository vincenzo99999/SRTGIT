# Redundant Controller with Diagnostics

This project is for a real-time systems course. It simulates a control system with fault tolerance and diagnostics.

## What It Does

- A main controller runs a control loop.
- A monitor checks if the controller fails.
- A backup controller is activated if the main one stops.
- A polling server (PS) manages the activation and diagnostics.
- A diagnostic requester (`d_req`) sends periodic diagnostic requests.

## Run

Use the provided `Makefile` and `run.sh` to build and run the system.

## Files

- `controller.c`, `backup_controller.c`: Control logic.
- `monitor.c`: Monitor and polling server.
- `diag.c`, `d_req.c`: Diagnostics.
- `reference.c`, `plant.c`: System simulation.
- `Makefile`, `run.sh`: Build and run tools.

## To Do

- [ ] Allow reactivation of the main controller.
- [ ] Improve diagnostics (e.g., save to file).
- [ ] Simulate different plant behaviors.
