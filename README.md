# Crazyflie 2.1 Flight Control

This flight control firmware is built on the [Hopter](https://github.com/hopter-project/hopter) embedded operating system, developed for the [Crazyflie 2.1](https://www.bitcraze.io/products/old-products/crazyflie-2-1/) miniature drone.
Its structure closely follows that of the [official](https://github.com/bitcraze/crazyflie-firmware) firmware, which was developed using FreeRTOS.

To operate correctly, the firmware requires the drone to be equipped with the [Flow Deck V2](https://www.bitcraze.io/products/flow-deck-v2/).
This deck features theVL53L1x ToF sensor that measures the distance to the ground and the PMW3901 optical flow sensor that measures movements in relation to the ground.
In addition, the firmware relies on the BMI088 inertial measurement unit to provide data on acceleration and angular velocity.

## Build & Flash

First, follow the instructions provided by [Hopter](https://github.com/hopter-project/hopter) to install the custom Rust compiler toolchain.

Next, run `make` to build the program. The firmware expects the drone to have the [bootloader](https://github.com/bitcraze/crazyflie2-stm-bootloader) installed.
To flash over radio, run `make cload`.
To flash over wire, run `make load`.

## Fly

For safety reasons, the propellers will not turn unless the `FLY` constant in `flight_tasks/crtp_rx.rs` is set to true.
Currently, the flight control commands are hard-coded in the firmware.
The program commands the drone to hover for 10 seconds at an altitude of 0.5 meters.
