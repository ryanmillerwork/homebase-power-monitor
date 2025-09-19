## HOMEBase Power Monitor (RP2040 + INA226)

Monitors HOMEBase power using a Waveshare RP2040-Zero and an INA226 current/voltage sensor. It exposes a simple USB CDC serial JSON protocol for reading measurements and configuring voltage thresholds used for percentage estimation.

### Hardware
- **MCU**: Waveshare RP2040-Zero — `https://www.amazon.com/dp/B0C77Y4ZLS`
- **Sensor**: INA226 breakout — `https://www.amazon.com/dp/B0BZ8D7Y42?th=1`

### Wiring
- **I2C instance**: i2c0
- **SDA**: GPIO 0
- **SCL**: GPIO 1
- **I2C speed**: 100 kHz (can be increased to 400 kHz)
- **INA226 address**: 0x40 (default)

### Build & Flash
This firmware uses the Raspberry Pi Pico SDK. Build as a standard RP2040 project and flash the resulting UF2 in BOOTSEL mode. Any RP2040 USB CDC serial terminal can be used to interact with the device.

### Connecting
The device enumerates as a USB CDC serial port (e.g., `/dev/ttyACM0` on Linux). You can interact with it using a serial terminal. Examples below use `screen` and `jq` for readability; USB CDC ignores baud settings.

Open a session:
```bash
screen /dev/ttyACM0 115200
```

Send a JSON object (no trailing newline required; one complete `{ ... }` per request) and read the single-line JSON response.

### Device identification
- USB descriptor strings:
  - Manufacturer: `Homebase`
  - Product: `power_monitor`

On Linux you can find the stable device path by matching the product string:
```bash
ls -l /dev/serial/by-id | grep power_monitor
```
This typically yields a symlink like `/dev/serial/by-id/usb-Homebase_power_monitor_*-if00` that points to the active `/dev/ttyACM*`. Use that symlink in your scripts for a stable identifier.

### JSON Protocol
- Each request is a single JSON object containing either a `get` or a `set` key, not both.
- Responses are single-line JSON objects.
- Errors are reported as `{"error":"<code>"}`.

#### GET
Request selected measurement fields by name:
```json
{"get": ["v", "a", "w", "pct", "charging"]}
```

Supported fields:
- **v**: Bus voltage in volts (float, 3 decimals)
- **a**: Current in amps (float, 4 decimals)
- **w**: Power in watts (float, 4 decimals)
- **pct**: Estimated state-of-charge percentage (0–100, 2 decimals) computed from `min_v`/`max_v`
- **charging**: Boolean; true when charging is detected

Example response (fields only for those requested):
```json
{"v": 28.523, "a": 0.1234, "w": 3.5123, "pct": 67.12, "charging": true}
```

Notes:
- Percentage calculation: `pct = 100 * clamp((v - min_v) / (max_v - min_v), 0, 1)`
- Charging heuristic: `charging = (current > 0.05A)`

#### SET
Configure the voltage range used for percentage estimation. Values are persisted to on-chip flash.
```json
{"set": {"min_v": 21.0, "max_v": 32.2}}
```

Keys:
- **min_v**: Minimum voltage (float)
- **max_v**: Maximum voltage (float)

Behavior:
- If both `min_v` and `max_v` are provided and out of order, sane ordering is enforced internally.
- Persisted across resets.

Example response:
```json
{"ok": true, "min_v": 21.000, "max_v": 32.200}
```

#### Constraints & Defaults
- Defaults if unset: `min_v = 21.0`, `max_v = 32.2`
- `max_v` must be greater than `min_v` for valid percentage computation (ordering is enforced if needed).

#### Errors
Returned as a JSON object with an `error` code:
- **both_get_and_set**: Request contained both `get` and `set`
- **bad_request**: Unrecognized or malformed request
- **i2c_read**: Sensor read failure

### Quick Examples
- Read voltage and current:
```json
{"get": ["v", "a"]}
```

- Read everything:
```json
{"get": ["v", "a", "w", "pct", "charging"]}
```

- Set thresholds then verify:
```json
{"set": {"min_v": 20.5, "max_v": 31.8}}
```
```json
{"get": ["pct"]}
```

### Implementation Notes
- Shunt value assumed: 0.1Ω; full-scale current: 2.0A (adjust in firmware if your hardware differs)
- Averages and conversion times are configured for moderate smoothing and responsiveness



