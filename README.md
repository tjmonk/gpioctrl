# gpioctrl
GPIO Controller with VarServer interface

## Overview

The gpioctrl service provides a simple configuration based mechanism to
define GPIO pin behaviors and map them to VarServer variables.  Under the
hood it uses the libgpiod library for interfacing with the hardware.
It also implements a sofware based pwm mode for GPIO pins, however,
this comes with some CPU overhead.

## Configuration File

The configuration file defines an array of GPIO chips.  Each GPIO chip
has a name so it can be accessed via the gpiodlibrary.  Within each GPIO
chip, an array of lines (or GPIO pins) are defined.

Each line has the following attributes:

| Attribute | Description |
| --- | --- |
| line | gpio pin number |
| var | VarServer variable name |
| active_state | defines the pin active state: high or low |
| direction | defines the pin as in input, output, or pwm |
| drive | specifies the pin drive type: push-pull, open-source, or open_drain |
| bias | specifies the pin drive bias: pull-up, pull-down, or disabled |
| event | specifies if the pin generates an interrupt on RISING_EDGE, FALLING_EDGE, or BOTH_EDGES. If no event is specified, the pin will not generate an interrupt |

The line, var, and direction attributes are mandatory.  If other attributes
are omitted, the line is configured to the default configuration as defined
by the gpiod library.

A sample configuration file for the Raspberry Pi 4 is shown below:

```
{ "gpiodef" : [
        { "chip" : "gpiochip0",
          "lines" : [
            {
              "line" : "4",
              "var" : "/HW/GPIO/P4",
              "active_state" : "high",
              "direction" : "output"},
            {
              "line" : "5",
              "var" : "/HW/GPIO/P5",
              "active_state" : "high",
              "direction" : "pwm"},
            {
              "line" : "6",
              "var" : "/HW/GPIO/P6",
              "active_state" : "high",
              "direction" : "pwm"},
            {
              "line" : "12",
              "var" : "/HW/GPIO/P12",
              "active_state" : "high",
              "direction" : "output"},
            {
              "line" : "13",
              "var" : "/HW/GPIO/P13",
              "active_state" : "high",
              "direction" : "pwm"},
            {
              "line" : "16",
              "var" : "/HW/GPIO/P16",
              "active_state" : "high",
              "direction" : "output"},
            {
              "line" : "17",
              "var" : "/HW/GPIO/P17",
              "active_state" : "high",
              "direction" : "output"},
            {
              "line" : "18",
              "var" : "/HW/GPIO/P18",
              "active_state" : "high",
              "direction" : "output"},
            {
              "line" : "19",
              "var" : "/HW/GPIO/P19",
              "active_state" : "high",
              "direction" : "output"},
            {
              "line" : "20",
              "var" : "/HW/GPIO/P20",
              "active_state" : "high",
              "direction" : "pwm"},
            {
              "line" : "21",
              "var" : "/HW/GPIO/P21",
              "active_state" : "high",
              "direction" : "output"},
            {
              "line" : "22",
              "var" : "/HW/GPIO/P22",
              "active_state" : "high",
              "direction" : "output"},
            {
              "line" : "23",
              "var" : "/HW/GPIO/P23",
              "active_state" : "high",
              "direction" : "output"},
            {
              "line" : "24",
              "var" : "/HW/GPIO/P24",
              "active_state" : "high",
              "direction" : "output"},
            {
              "line" : "25",
              "var" : "/HW/GPIO/P25",
              "active_state" : "high",
              "direction" : "output"},
            {
              "line" : "26",
              "var" : "/HW/GPIO/P26",
              "event" : "BOTH_EDGES",
              "direction" : "input",
              "drive" : "open-source",
              "bias" : "pull-up" },
            {
              "line" : "27",
              "var" : "/HW/GPIO/P27",
              "active_state" : "high",
              "direction" : "output"}
            ]
        }
    ]
}

```
## Polling

Any input which does not have an event definition will be set up to be
polled whenever a client requests the value of its associated VarServer
variable. If this pin is high, the value of the VarServer variable will be
set to 1.  If the pin is low, the value of the VarServer variable will
be set to 0.

## Interrupts

Any input which has an event definition will automatically change the
value of its associated VarServer variable when the state of the input
pin changes.  This can generate a SIG_VAR_MODIFIED signal to any
client which has requested a MODIFIED notification on this variable.
If this pin is high, the value of the VarServer variable will be
set to 1.  If the pin is low, the value of the VarServer variable will
be set to 0.

## Outputs

Any GPIO output which is not a PWM will update the pin state whenever
the value of the VarServer variable is changed.  If the VarServer
variable is 0, the pin will be set to 0.  If the VarServer variable
is any non-zero value, the pin will be set to 1.

## PWM

Any pin which is configured as a software pwm will be controlled via
its own thread.  The PWM frequency is approximately 10 mS.  The duty cycle is
controlled via the value written to the associated VarServer variable in the
range [0..255].  0 is fully off, 255 is fully on, and 128 is a 50% duty cycle.
Bear in mind this is a very course PWM, and the number of PWMs utilized will
have an impact on the CPU utilization.

## Prerequisites

The gpioctrl service requires the following components:

- varserver : variable server ( https://github.com/tjmonk/varserver )
- tjson : JSON parser library ( https://github.com/tjmonk/libtjson )
- libgpiod (v1.6.3): gpio library ( https://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git )

Of course you must have a device with GPIO pins to run the gpioctrl service with
any meaningful result.  The examples shown below are designed to work on a
Raspberry Pi 4.

## Build

The build script will automatically download and build the gpio library v1.6.3

It will then build and install the gpioctrl service.

```
./build.sh
```

## Set up the VarServer variables

```
varserver &
varcreate test/vars.json
```

## Start the gpioctrl service

```
gpioctrl test/gpiocfg.json &
```

## Get the gpioctrl info

```
getvar /sys/gpioctrl/info
```

```
[
  {
    "chip": "gpiochip0",
    "lines": [
      {
        "line": 4,
        "name": "GPIO_GCLK",
        "var": "/HW/GPIO/P4"
      },
      {
        "line": 5,
        "name": "GPIO5",
        "var": "/HW/GPIO/P5"
      },
      {
        "line": 6,
        "name": "GPIO6",
        "var": "/HW/GPIO/P6"
      },
      {
        "line": 12,
        "name": "GPIO12",
        "var": "/HW/GPIO/P12"
      },
      {
        "line": 13,
        "name": "GPIO13",
        "var": "/HW/GPIO/P13"
      },
      {
        "line": 16,
        "name": "GPIO16",
        "var": "/HW/GPIO/P16"
      },
      {
        "line": 17,
        "name": "GPIO17",
        "var": "/HW/GPIO/P17"
      },
      {
        "line": 18,
        "name": "GPIO18",
        "var": "/HW/GPIO/P18"
      },
      {
        "line": 19,
        "name": "GPIO19",
        "var": "/HW/GPIO/P19"
      },
      {
        "line": 20,
        "name": "GPIO20",
        "var": "/HW/GPIO/P20"
      },
      {
        "line": 21,
        "name": "GPIO21",
        "var": "/HW/GPIO/P21"
      },
      {
        "line": 22,
        "name": "GPIO22",
        "var": "/HW/GPIO/P22"
      },
      {
        "line": 23,
        "name": "GPIO23",
        "var": "/HW/GPIO/P23"
      },
      {
        "line": 24,
        "name": "GPIO24",
        "var": "/HW/GPIO/P24"
      },
      {
        "line": 25,
        "name": "GPIO25",
        "var": "/HW/GPIO/P25"
      },
      {
        "line": 26,
        "name": "GPIO26",
        "var": "/HW/GPIO/P26"
      },
      {
        "line": 27,
        "name": "GPIO27",
        "var": "/HW/GPIO/P27"
      }
    ]
  }
]
```

## Get the GPIO pin status

```
vars -vn /HW/GPIO/
```

```
/HW/GPIO/P4=0
/HW/GPIO/P5=255
/HW/GPIO/P6=255
/HW/GPIO/P12=0
/HW/GPIO/P13=0
/HW/GPIO/P16=0
/HW/GPIO/P17=0
/HW/GPIO/P18=0
/HW/GPIO/P19=0
/HW/GPIO/P20=0
/HW/GPIO/P21=0
/HW/GPIO/P22=0
/HW/GPIO/P23=0
/HW/GPIO/P24=0
/HW/GPIO/P25=0
/HW/GPIO/P26=0
/HW/GPIO/P27=0
```

## Set a GPIO output state

```
setvar /HW/GPIO/P4 1
```

```
OK
```

## Get a GPIO output state

```
getvar /HW/GPIO/P4
```

```
1
```
