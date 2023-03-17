# 4B25 Activity Classifier

## Summary

Activity classifier for the FRDM-KL03 development board, using the MMA8451Q
accelerometer (internal) and the SSD1331 OLED display (external, over SPI).

The current model supports four classes: stationary, walking, running, and
jumping.

Accelerometer is configured to use its 32-entry FIFO (circular buffer mode),
with the high-pass filter enabled, sampling at 50 Hz.

The MCU then polls* the accelerometer for data, and reads all available readings
until it fills its own 32-entry buffer. It then processes all 32-entry entries
at once:
- Estimates the variance in the readings, per axis
- Normalizes the variances to the [0, 1] range (roughly)
- Sorts the variances by magnitude, for rotation invariance
- Computes the feature vector used for inference
- Evaluates the probability of the feature vector against each class, computes
        the assigned class and class probability with the soft-max function
- Updates the OLED display with the results (red squares represent the
class---one square for walking and four for jumping---and the green bar
represents the class probability)

* Updating the OLED takes 300-700 ms, depending on the content, therefore the
MCU is not polling very often at all.

## Compiling

- Set up your environment, e.g.

```shell
export JLINKPATH=/usr/bin/JLinkExe
export ARMGCC_DIR=/usr
```

- Make the `frdmkl03` target

```shell
make frdmkl03
```

- Flash your FRDM-KL03 board

```shell
make load-warp
```

### Binary size

The FRDM-KL03 only has 32 KB of flash. This project makes full use of it,
therefore your binary might not fit if you use a different compiler or implement
additional functionality.

With `arm-none-eabi-gcc (Fedora 12.2.0-1.fc37) 12.2.0`, `objdump` reports:
- `.text`: 31456 bytes (including ~6000 bytes of strings)
- `.data`: 224 bytes
- `.bss`: 760 bytes
- a few more small sections with ~200 bytes total

This brings the total very close to 32768 bytes!

If you are having issues, you could try compiling with `-Os` instead of `-O3`.

## Interesting files

### Implementation

- `src/boot/ksdk1.1.0/boot.c`: support for the INA219 & SSD1331, new menu option
        to start the activity classifier, bypasses the menu when
        `WARP_BUILD_BOOT_TO_ACTIVITY_CLASSIFIER` is defined
- `src/boot/ksdk1.1.0/config.h`: new build options for sensors and activity
        classifier
- `src/boot/ksdk1.1.0/devMMA8451Q.{c,h}`: MMA8451Q driver and activity
        classifier implementation (including inference)
- `src/boot/ksdk1.1.0/devSSD1331.{c,h}`: SSD1331 driver and activity classifier
        display code

### Inference

- `inference/learn.py`: offline logistic regression training and evaluation
- `inference/*.csv`: data used for offline training and testing (one per class)

### Less interesting changes

For the sake of completeness:
- `Makefile`
- a bunch of unnecessary build scripts were removed
- `setup.conf`
- `CMakeLists-FRDMKL03.txt`
- `src/boot/ksdk1.1.0/devINA219.{c,h}`: INA219 driver
- `src/boot/ksdk1.1.0/gpio_pins.h`: updated pins for the INA219 and SSD1331
- `src/boot/ksdk1.1.0/powermodes.c`
- `src/boot/ksdk1.1.0/warp.h`
- `tools/scripts/warp.jlink.commands`
- `tools/sdk/ksdk1.1.0/platform/startup/startup.c`
