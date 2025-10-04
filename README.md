# MPU6050 Driver

Source code for the MPU6050 I2C Inertial Measurement Unit for ESP-IDF.

## Introduction

This driver implements communications with the MPU6050 using the new i2c esp-idf driver (`driver/i2c_master.h`).

It is used to communicate with the MPU6050 gyroscope / accelerometer / thermometer using the ESP-IDF framework I2C master driver.

NOTE : interrupts are not yet supported;

## Installation

To install the driver, you can clone the repository and place it in the `components` folder of your project.

```bash
git clone https://github.com/TNY-Robotics/mpu6050-esp-idf.git components/mpu6050
```

## Usage

The driver exposes a simple header file named `mpu6050.h`, that you can include using :

```c
#include "mpu6050.h"
```

## Examples

An example of how to create, configure, read and delete an mpu6050 object using the MPU6050 driver can be found in the `examples` folder.

## Licence

This driver is under the MIT Licence.

## Author

This driver was created by the [TNY Robotics](https://tny-robotics.com) team, for any questions or suggestions, please contact us at [contact@furwaz.com](mailto:contact@furwaz.com).