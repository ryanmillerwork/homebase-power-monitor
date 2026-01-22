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

#### INA226 connections (what goes where)
- **Power / I2C (RP2040 ↔ INA226)**
  - **GND** → GND
  - **3.3V** → **VS/VCC** (sensor supply)
  - **GPIO0 (SDA)** → SDA
  - **GPIO1 (SCL)** → SCL

- **Sense wiring (shunt + bus voltage)**
  - Place the shunt resistor in series with the **positive** rail (high-side sensing):

    `SUPPLY+ → VIN+ → [ Rshunt ] → VIN- → LOAD+`

  - Connect **VBUS** (sometimes labeled **VBS** on breakouts) to the **bus voltage you want reported**. Most commonly you want *load voltage*, so tie it to the **load-side** node (same as **VIN- / LOAD+**):

    `VBUS/VBS → VIN- (LOAD+)`

  - Connect **LOAD-** to **SUPPLY-**, and ensure the INA226 **GND** and RP2040 **GND** share the same ground reference.

- **Notes / gotchas**
  - **I2C pull-ups**: many INA226 breakouts include SDA/SCL pull-ups to VCC; if yours doesn’t, add ~4.7k–10k pull-ups to **3.3V**.
  - **Address**: if you change `A0/A1` straps on the breakout, update the firmware address from the default `0x40`.
  - **Voltage range**: INA226 measures bus voltage up to ~36V relative to GND (check your board/datasheet for limits).

### Build & Flash
This firmware uses the Raspberry Pi Pico SDK. You can either:
- Flash the **prebuilt** `power_monitor.uf2` included in this repo, or
- Build a **fresh** UF2 from source (recommended once you start making changes)

#### Option A: Flash the prebuilt UF2 (fastest)
1. Put the RP2040-Zero into BOOTSEL mode:
   - Unplug the board
   - Hold **BOOT**
   - Plug it into USB
   - Release **BOOT**
2. A drive named `RPI-RP2` should appear (usually mounted under `/media/$USER/RPI-RP2`).
3. Copy the UF2:
```bash
cp power_monitor.uf2 /media/$USER/RPI-RP2/
sync
```
The board will reboot automatically and the `RPI-RP2` drive will disappear (expected).

#### Option B: Build from source on a Raspberry Pi (no SDK preinstalled)
The commands below mirror a clean Raspberry Pi/Debian setup.

1. Install build tools and helpers (includes `screen` for serial testing):
```bash
sudo apt update
sudo apt install -y \
  git cmake build-essential \
  gcc-arm-none-eabi libnewlib-arm-none-eabi \
  python3 screen
```

2. Clone the Pico SDK (tag `2.2.0`) and its submodules:
```bash
mkdir -p ~/pico
cd ~/pico
git clone --branch 2.2.0 https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init --recursive
```

3. Configure and build this project:
```bash
cd ~/homebase-power-monitor
mkdir -p build
cd build
cmake .. -DPICO_SDK_PATH=$HOME/pico/pico-sdk -DPICO_BOARD=waveshare_rp2040_zero
cmake --build . -j"$(nproc)"
ls -lh power_monitor.uf2
```

4. Flash your newly built UF2 in BOOTSEL mode:
```bash
cp ~/homebase-power-monitor/build/power_monitor.uf2 /media/$USER/RPI-RP2/
sync
```

#### Verify the device enumerates (USB CDC serial)
After flashing, the device should appear as `/dev/ttyACM*`. For a stable path:
```bash
ls -l /dev/serial/by-id | grep power_monitor
```

#### Troubleshooting notes
- If `RPI-RP2` doesn’t show up, try a different USB cable (many are power-only) and watch `dmesg -w` while plugging in (holding BOOT).
- **If the INA226 is not connected / not responding on I2C**, the firmware will fail early during `ina226_init` and then stop in a tight loop. In that state it can look like “serial isn’t responding” because the main request loop never starts. Connect the INA226 (address `0x40`) to I2C0 (`SDA=GPIO0`, `SCL=GPIO1`) and power it from **3.3V** + GND.

### Connecting
The device enumerates as a USB CDC serial port (e.g., `/dev/ttyACM0` on Linux). You can interact with it using a serial terminal. Examples below use `screen` and `jq` for readability; USB CDC ignores baud settings.

Open a session and exit:
```bash
screen /dev/ttyACM1 115200
ctrl+a k
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

#### Tcl helper to locate the device
```tcl
proc find_power_monitor {} {
  set links [glob -nocomplain -types l /dev/serial/by-id/*]
  foreach l $links {
    if {[string match *power_monitor* [file tail $l]]} { return $l }
    if {![catch {file readlink $l} target] && [string match *power_monitor* $target]} { return $l }
  }
  return ""
}

set port [find_power_monitor]
if {$port eq ""} {
  puts stderr "power_monitor not found under /dev/serial/by-id (looking for *power_monitor*)."
  puts stderr "Tip: ls -l /dev/serial/by-id | grep power_monitor"
  exit 1
}
puts $port
```

### JSON Protocol
- Each request is a single JSON object containing either a `get` or a `set` key, not both.
- Responses are single-line JSON objects.
- Errors are reported as `{"error":"<code>"}`.

#### GET
Request selected measurement fields by name:
```json
{"get": ["v", "a", "w", "pct", "charging", "fw", "chg_threshold_a"]}
```

Supported fields:
- **v**: Bus voltage in volts (float, 3 decimals)
- **a**: Current in amps (float, 4 decimals)
- **w**: Power in watts (float, 4 decimals)
- **pct**: Estimated state-of-charge percentage (0–100, 2 decimals) computed from `min_v`/`max_v`
- **charging**: Boolean; true when charging is detected
- **hrs_capacity**: Persisted capacity proxy (hours at 100%); returned when requested
- **hrs_remaining**: Estimated hours remaining (`hrs_capacity * pct/100`, 0.1 hr resolution)
- **chg_threshold_a**: Signed charging threshold in amps; sign encodes direction (see notes)
- **fw**: Firmware version string (e.g. `v1.2.3` or `a1438df-dirty` depending on build configuration)

Example response (fields only for those requested):
```json
{"fw": "v1.2.3", "v": 28.523, "a": 0.1234, "w": 3.5123, "pct": 67.12, "charging": true, "hrs_remaining": 6.7, "chg_threshold_a": 0.050}
```

Notes:
- Percentage calculation: `pct = 100 * clamp((v - min_v) / (max_v - min_v), 0, 1)`
- Charging heuristic (signed threshold): `charging = (chg_threshold_a > 0 ? i >= chg_threshold_a : i <= chg_threshold_a)`
- Hours remaining: `hrs_remaining = hrs_capacity * (pct / 100)`

#### SET
Configure the voltage range and charging threshold. Values are persisted to on-chip flash.
```json
{"set": {"min_v": 21.0, "max_v": 32.2, "hrs_capacity": 10.0, "chg_threshold_a": 0.05}}
```

Keys:
- **min_v**: Minimum voltage (float)
- **max_v**: Maximum voltage (float)
- **hrs_capacity**: Capacity proxy in hours at 100% (float; used only to scale hrs_remaining)
- **chg_threshold_a**: Signed charging threshold in amps; positive means charging when current is greater-or-equal; negative means charging when current is less-or-equal; zero is invalid.

Behavior:
- If both `min_v` and `max_v` are provided and out of order, sane ordering is enforced internally.
- Persisted across resets.
- `chg_threshold_a` must be non-zero and within (-100, 100); requests outside this range are rejected with `invalid_chg_threshold`.

Example response:
```json
{"ok": true, "min_v": 21.000, "max_v": 32.200, "hrs_capacity": 10.0, "chg_threshold_a": 0.050}
```

#### Constraints & Defaults
- Defaults if unset: `min_v = 21.0`, `max_v = 32.2`, `hrs_capacity = 10.0`, `chg_threshold_a = 0.05`
- `max_v` must be greater than `min_v` for valid percentage computation (ordering is enforced if needed).

#### Errors
Returned as a JSON object with an `error` code:
- **both_get_and_set**: Request contained both `get` and `set`
- **bad_request**: Unrecognized or malformed request
- **i2c_read**: Sensor read failure
- **ina226_not_found**: INA226 missing or not responding at boot (response will also include `message: "INA226 not found"`)
- **invalid_chg_threshold**: `chg_threshold_a` was zero or out of the allowed range

### Quick Examples
- Read voltage and current:
```json
{"get": ["v", "a"]}
```

- Read everything (all supported GET fields):
```json
{"get": ["v", "a", "w", "pct", "charging", "min_v", "max_v", "hrs_capacity", "hrs_remaining", "fw", "chg_threshold_a"]}
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



