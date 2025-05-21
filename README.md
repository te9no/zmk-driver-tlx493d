# TLX493D 3D Magnetic Sensor Driver for ZMK

This ZMK module implements a driver for the Infineon TLX493D 3D magnetic sensor.

## Features

- 3-axis magnetic field measurement
- Temperature sensing
- I2C interface support
- Configurable sensitivity
- Optional interrupt support

## Installation

Add this to your `west.yml` manifest:

```yaml
manifest:
  remotes:
    - name: te9no
      url-base: https://github.com/te9no
  projects:
    - name: zmk-driver-tlx493d
      remote: te9no
      revision: main
```

## Usage

Add the following to your device tree:

```dts
&i2c0 {
    tlx493d@c {
        compatible = "infineon,tlx493d";
        reg = <0x0C>;
        sensitivity = "1x";
        int-gpios = <&gpio0 3 GPIO_ACTIVE_LOW>;
    };
};
```

### Configuration Options

- `sensitivity`: Magnetic sensitivity ("1x", "2x", "4x")
- `int-gpios`: Optional interrupt pin
- `rotate-90`: Rotate sensor readings by 90 degrees

## Technical Documentation

For more details about the sensor, refer to the [TLX493D Datasheet](https://www.infineon.com/dgdl/Infineon-TLX493D-DataSheet-v01_00-EN.pdf).
