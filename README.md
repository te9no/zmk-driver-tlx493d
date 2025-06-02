# TLX493D 3D Magnetic Sensor Driver for ZMK

This ZMK module implements a driver for the Infineon TLX493D 3D magnetic sensor.

## Features

- 3-axis magnetic field measurement
- Temperature sensing
- I2C interface support
- Configurable sensitivity
- Optional interrupt support
- Auto-calibration on startup
- Analog stick functionality with hysteresis control

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
        hysteresis = <100>;           /* 0.1 = 10% hysteresis */
        center-threshold = <400>;     /* 0.4 = 40% center deadzone */
        calibration-samples = <300>;  /* Number of samples for calibration */
    };
};
```

### Configuration Options

- `sensitivity`: Magnetic sensitivity ("1x", "2x", "4x")
- `rotate-90`: Rotate sensor readings by 90 degrees
- `hysteresis`: Hysteresis threshold in thousandths (default: 100 = 0.1)
- `center-threshold`: Center deadzone threshold in thousandths (default: 400 = 0.4)
- `calibration-samples`: Number of samples to take during calibration (default: 300)

### Configuration File Settings

You can also configure the sensor behavior in your `prj.conf`:

```conf
# Enable TLX493D driver
CONFIG_INPUT_TLX493D=y

# Calibration settings
CONFIG_TLX493D_CALIBRATION_SAMPLES=300    # Number of samples for calibration
CONFIG_TLX493D_HYSTERESIS_THRESHOLD=100   # 0.1 = 10% hysteresis
CONFIG_TLX493D_CENTER_THRESHOLD=400       # 0.4 = 40% center deadzone
```

### Using as an Analog Stick

The sensor can be used as a magnetic analog stick. The driver:
- Automatically calibrates on startup to set the center position
- Applies hysteresis to prevent jitter
- Implements a center deadzone for stability
- Provides normalized outputs suitable for analog stick input

Example behavior binding:
```dts
&behavior_sensor_rotate {
    compatible = "zmk,behavior-sensor-rotate";
    label = "MAGNETIC_ANALOG";
    #sensor-binding-cells = <2>;
    bindings = <&move_analog>, <&orbit_analog>;
};
```

## Technical Documentation

For more details about the sensor, refer to the [TLX493D Datasheet](https://www.infineon.com/dgdl/Infineon-TLX493D-DataSheet-v01_00-EN.pdf).
