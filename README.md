# GPIO Controller (gpioctrl)

The `gpioctrl` service maps General Purpose I/O pins to variables
managed by the Variable Server (`varserver`).  It uses a JSON
definition file to describe the mapping between GPIO chip lines and
named variables, and the `libgpiod` v2 character-device API to
interact with the hardware.

## Building

```bash
mkdir build && cd build
cmake ..
make
sudo make install
```

### Building the unit tests

```bash
mkdir build_test && cd build_test
cmake ../test
make
ctest --output-on-failure
```

Or from the top-level CMakeLists:

```bash
cmake -DBUILD_TESTS=ON ..
make
ctest
```

## Usage

```
gpioctrl [-v] [-h] -f <filename>
```

| Option | Description |
|--------|-------------|
| `-f <filename>` | Path to the JSON GPIO definition file (required) |
| `-v` | Enable verbose output |
| `-h` | Display usage help |

### Operating modes

The service name determines the operating mode:

| Invocation | Mode | Behaviour |
|------------|------|-----------|
| `gpioctrl -f config.json` | Normal | Listens for variable-server signals; writes outputs on `MODIFIED`, reads inputs on `CALC` |
| `gpiowatch -f config.json` | Watch | Polls for GPIO edge events and updates variables when edges are detected |

Create a symlink to switch mode:

```bash
ln -s gpioctrl gpiowatch
```

## JSON Definition File

The definition file describes one or more GPIO chips and the lines
within each chip that should be mapped to variables.

### Structure

```json
{
  "gpiodef": [
    {
      "chip": "<chip-name>",
      "lines": [
        {
          "line": "<offset>",
          "var": "<variable-path>",
          "direction": "<direction>",
          "active_state": "<active-state>",
          "event": "<edge-type>",
          "bias": "<bias>",
          "drive": "<drive>"
        }
      ]
    }
  ]
}
```

### Top-level

| Key | Type | Description |
|-----|------|-------------|
| `gpiodef` | array | Array of chip definition objects |

### Chip object

| Key | Type | Required | Description |
|-----|------|----------|-------------|
| `chip` | string | yes | Chip device name (e.g. `"gpiochip0"`). Opened as `/dev/<chip>`. |
| `lines` | array | yes | Array of line definition objects for this chip |

### Line object

| Key | Type | Required | Default | Description |
|-----|------|----------|---------|-------------|
| `line` | string | yes | — | Line offset on the chip (e.g. `"4"`) |
| `var` | string | yes | — | Variable path in the variable server (e.g. `"/HW/GPIO/P4"`) |
| `direction` | string | no | `"input"` | `"input"`, `"output"`, or `"pwm"` |
| `active_state` | string | no | `"high"` | `"high"` or `"low"` |
| `event` | string | no | none | `"RISING_EDGE"`, `"FALLING_EDGE"`, or `"BOTH_EDGES"` |
| `bias` | string | no | as-is | `"disabled"`, `"pull-up"`, or `"pull-down"` |
| `drive` | string | no | `"push-pull"` | `"push-pull"`, `"open-drain"`, or `"open-source"` |

### Example

```json
{
  "gpiodef": [
    {
      "chip": "gpiochip0",
      "lines": [
        {
          "line": "4",
          "var": "/HW/GPIO/P4",
          "active_state": "high",
          "direction": "output"
        },
        {
          "line": "5",
          "var": "/HW/GPIO/P5",
          "direction": "pwm"
        },
        {
          "line": "26",
          "var": "/HW/GPIO/P26",
          "direction": "input",
          "event": "BOTH_EDGES",
          "bias": "pull-up"
        }
      ]
    }
  ]
}
```

## Internal Architecture

```
+-------------------------------------------------------------+
|                        gpioctrl                              |
|                                                             |
|  +------------------+     +-----------------------------+   |
|  | JSON Parser      |     | Variable Server Client      |   |
|  | (libtjson)       |     | (libvarserver)              |   |
|  +--------+---------+     +----+---+---+----------------+   |
|           |                    |   |   |                    |
|           v                    |   |   |                    |
|  +------------------+          |   |   |                    |
|  | GPIOCtrlState    |<---------+   |   |                    |
|  |  - chip list     |             |   |                    |
|  |  - edge buffer   |             |   |                    |
|  +--------+---------+             |   |                    |
|           |                        |   |                    |
|           v                        |   |                    |
|  +------------------+              |   |                    |
|  | GPIOChip (list)  |              |   |                    |
|  |  - gpiod_chip    |              |   |                    |
|  |  - pLineRequest  |              |   |                    |
|  |  - GPIO lines    |              |   |                    |
|  +--------+---------+              |   |                    |
|           |                        |   |                    |
|           v                        |   |                    |
|  +------------------+              |   |                    |
|  | GPIO (list)      |              |   |                    |
|  |  - line_num      |              |   |                    |
|  |  - hVar          |<-------------+   |                    |
|  |  - direction     |                  |                    |
|  |  - edge/bias/drv |                  |                    |
|  +------------------+                  |                    |
|                                        |                    |
|  +------------------+                  |                    |
|  | libgpiod v2      |<-----------------+                    |
|  | (kernel chardev) |                                       |
|  +------------------+                                       |
+-------------------------------------------------------------+
```

### Data structures

| Structure | Purpose |
|-----------|---------|
| `GPIOCtrlState` | Top-level state: chip list, varserver handle, edge event buffer, operating mode |
| `GPIOChip` | One per physical GPIO chip. Holds the `gpiod_chip` pointer and a single `gpiod_line_request` for all managed lines on that chip. Contains a linked list of `GPIO` objects. |
| `GPIO` | One per managed line. Maps a line offset to a variable handle and stores per-line attributes (direction, bias, drive, edge, active-low). |

### Startup sequence

1. Parse command line (`ProcessOptions`)
2. Load and parse JSON definition (`JSON_Process`)
3. Open variable server connection (`VARSERVER_Open`)
4. For each chip in `gpiodef`:
   - Open the chip device (`gpiod_chip_open`)
   - Parse each line definition and populate `GPIO` structs
   - Build per-line `gpiod_line_settings` (direction, bias,
     drive, edge, output value)
   - Issue a single `gpiod_chip_request_lines` for all
     eligible lines on the chip
   - Start PWM threads for any `"pwm"` lines (normal mode)
5. Enter the run loop

### Run loop

**Normal mode (`gpioctrl`)**:
- Waits for variable-server signals via `VARSERVER_WaitSignal`
- `SIG_VAR_MODIFIED` -> writes the new variable value to the
  GPIO output
- `SIG_VAR_CALC` -> reads the GPIO input and updates the
  variable value
- `SIG_VAR_PRINT` -> dumps GPIO status as JSON

**Watch mode (`gpiowatch`)**:
- Uses `poll()` on the file descriptors of each chip's
  `gpiod_line_request` that has edge-configured lines
- When an edge event fires, reads the event buffer and
  updates the associated variable (1 for rising, 0 for
  falling)

### Shutdown

On `SIGTERM` or `SIGINT`:
1. Sets `running = false` to exit the run loop
2. Frees the edge event buffer
3. Releases each chip's `gpiod_line_request`
4. Closes each `gpiod_chip`
5. Closes the variable server connection

## Dependencies

| Library | Version | Purpose |
|---------|---------|---------|
| libgpiod | >= 2.0 | GPIO character device access |
| libvarserver | — | Variable server client API |
| libtjson | — | JSON parsing |
| pthreads | — | PWM output threads |

## License

Copyright (C) Trevor Monk - All Rights Reserved.
Proprietary and confidential.
