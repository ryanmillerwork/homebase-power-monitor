#!/usr/bin/env python3
"""Flash power_monitor.uf2 via picotool and run serial JSON unit tests."""

from __future__ import annotations

import argparse
import glob
import json
import os
import re
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

try:
    import serial
except ImportError:
    print("error: pyserial not installed (try: sudo apt install python3-serial)", file=sys.stderr)
    sys.exit(1)

SCRIPT_DIR = Path(__file__).resolve().parent
SERIAL_GLOB = "/dev/serial/by-id/*power_monitor*"
TRACK_SERIAL_RE = re.compile(r"Tracking device serial number (\S+) for reboot")
VERSION_RE = re.compile(r"^\s*version:\s+(\S+)", re.MULTILINE)


@dataclass
class Results:
    flash: str = "SKIP"
    serial_comm: str = "FAIL"
    firmware: str = "FAIL"
    protocol_pass: int = 0
    protocol_total: int = 4
    sensor: str = "FAIL"
    sensor_detail: str = ""
    comm_failed: bool = False
    flash_failed: bool = False
    messages: list[str] = field(default_factory=list)

    def exit_code(self) -> int:
        if self.flash_failed:
            return 1
        if self.comm_failed:
            return 2
        if self.sensor != "PASS":
            return 3
        return 0


def log(msg: str, *, verbose: bool) -> None:
    if verbose:
        print(msg)


def resolve_uf2(explicit: str | None) -> Path:
    if explicit:
        path = Path(explicit)
        if not path.is_file():
            raise FileNotFoundError(f"UF2 not found: {path}")
        return path.resolve()

    for candidate in (SCRIPT_DIR / "power_monitor.uf2", SCRIPT_DIR / "build" / "power_monitor.uf2"):
        if candidate.is_file():
            return candidate.resolve()

    raise FileNotFoundError("No power_monitor.uf2 found in repo root or build/")


def run_cmd(cmd: list[str], *, verbose: bool) -> subprocess.CompletedProcess[str]:
    log(f"$ {' '.join(cmd)}", verbose=verbose)
    return subprocess.run(cmd, capture_output=True, text=True)


def picotool_info_version(uf2: Path, *, verbose: bool) -> str:
    if not shutil.which("picotool"):
        raise RuntimeError("picotool not found in PATH (try: sudo apt install picotool)")

    result = run_cmd(["picotool", "info", "-a", str(uf2)], verbose=verbose)
    if result.returncode != 0:
        raise RuntimeError(f"picotool info failed:\n{result.stderr or result.stdout}")

    match = VERSION_RE.search(result.stdout)
    if not match:
        raise RuntimeError(f"Could not parse firmware version from picotool info:\n{result.stdout}")
    return match.group(1)


def picotool_flash(uf2: Path, serial: str | None, *, verbose: bool) -> str | None:
    cmd = ["picotool", "load", "-f", "-v", "-x"]
    if serial:
        cmd.extend(["--ser", serial])
    cmd.append(str(uf2))

    result = run_cmd(cmd, verbose=verbose)
    output = (result.stdout or "") + (result.stderr or "")

    if result.returncode != 0:
        hint = ""
        if "permission" in output.lower() or "unable to connect" in output.lower():
            hint = "\nHint: try running with sudo, or add your user to the dialout group."
        raise RuntimeError(f"picotool load failed (exit {result.returncode}):\n{output}{hint}")

    match = TRACK_SERIAL_RE.search(output)
    tracked = match.group(1) if match else serial
    if tracked:
        log(f"Tracked serial: {tracked}", verbose=verbose)
    return tracked


def maybe_build(*, verbose: bool) -> None:
    build_dir = SCRIPT_DIR / "build"
    if not build_dir.is_dir():
        raise RuntimeError("build/ directory not found; run cmake first or omit --build")

    jobs = os.cpu_count() or 1
    result = run_cmd(["cmake", "--build", str(build_dir), "-j", str(jobs)], verbose=verbose)
    if result.returncode != 0:
        raise RuntimeError(f"Build failed:\n{result.stderr or result.stdout}")


def find_port(serial: str | None, timeout: float) -> str:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ports = sorted(glob.glob(SERIAL_GLOB))
        if serial:
            matched = [p for p in ports if serial in p]
            if matched:
                return matched[0]
        elif len(ports) == 1:
            return ports[0]
        elif len(ports) > 1 and not serial:
            raise RuntimeError(
                "Multiple power_monitor devices found; use --serial to select one:\n"
                + "\n".join(f"  {p}" for p in ports)
            )
        time.sleep(0.1)

    raise TimeoutError(f"Serial port not found within {timeout:.0f}s (glob: {SERIAL_GLOB})")


def drain_input(ser: serial.Serial, settle_s: float) -> None:
    """Wait for firmware boot, then discard any pending CDC lines."""
    time.sleep(settle_s)
    old_timeout = ser.timeout
    ser.timeout = 0.2
    try:
        while True:
            line = ser.readline()
            if not line:
                break
    finally:
        ser.timeout = old_timeout


def is_boot_only_message(resp: dict) -> bool:
    return (
        resp.get("error") == "ina226_not_found"
        and "code" in resp
        and "fw" not in resp
        and "ok" not in resp
        and "result" not in resp
    )


class Device:
    def __init__(self, port: str, timeout: float, *, settle_s: float = 2.5) -> None:
        self.port = port
        self.timeout = timeout
        self.ser = serial.Serial(port, 115200, timeout=timeout)
        drain_input(self.ser, settle_s)

    def close(self) -> None:
        self.ser.close()

    def query(self, req: dict, *, verbose: bool, retries: int = 3) -> dict | None:
        msg = json.dumps(req, separators=(",", ":"))
        log(f"  SEND {msg}", verbose=verbose)

        for attempt in range(retries):
            if attempt:
                time.sleep(0.3)
            self.ser.write(msg.encode())
            self.ser.flush()

            deadline = time.monotonic() + self.timeout
            while time.monotonic() < deadline:
                line = self.ser.readline().decode("utf-8", errors="replace").strip()
                if not line:
                    break
                log(f"  RECV {line}", verbose=verbose)
                try:
                    resp = json.loads(line)
                except json.JSONDecodeError:
                    return {"_raw": line, "_parse_error": True}
                if is_boot_only_message(resp):
                    continue
                return resp

        log("  RECV (timeout)", verbose=verbose)
        return None


def response_ok(resp: dict) -> bool:
    if resp.get("ok"):
        return True
    result = resp.get("result")
    return isinstance(result, dict) and bool(result.get("ok"))


def run_comm_tests(
    dev: Device, expected_fw: str, *, verbose: bool
) -> tuple[bool, int, bool, str | None]:
    protocol_pass = 0
    device_fw: str | None = None

    resp = dev.query({"get": ["fw"]}, verbose=verbose)
    if resp is None:
        return False, protocol_pass, True, device_fw
    device_fw = resp.get("fw")
    fw_ok = device_fw == expected_fw
    if not fw_ok:
        return fw_ok, protocol_pass, True, device_fw

    resp = dev.query({"get": "all"}, verbose=verbose)
    if resp is not None and "_parse_error" not in resp and resp.get("error") != "bad_request":
        protocol_pass += 1
    else:
        return fw_ok, protocol_pass, True, device_fw

    resp = dev.query({"get": ["bogus"]}, verbose=verbose)
    if resp and resp.get("error") == "invalid_get_field":
        protocol_pass += 1
    else:
        return fw_ok, protocol_pass, True, device_fw

    resp = dev.query({"get": ["v"], "set": {"min_v": 21.0}}, verbose=verbose)
    if resp and resp.get("error") == "both_get_and_set":
        protocol_pass += 1
    else:
        return fw_ok, protocol_pass, True, device_fw

    set_vals = {"min_v": 20.5, "max_v": 31.8, "hrs_capacity": 9.5, "chg_threshold_a": 0.05}
    resp = dev.query({"set": set_vals}, verbose=verbose)
    if not resp or not response_ok(resp):
        return fw_ok, protocol_pass, True, device_fw

    resp = dev.query({"get": ["min_v", "max_v", "hrs_capacity", "chg_threshold_a"]}, verbose=verbose)
    if resp is None:
        return fw_ok, protocol_pass, True, device_fw

    ok = (
        abs(resp.get("min_v", 0) - set_vals["min_v"]) < 0.01
        and abs(resp.get("max_v", 0) - set_vals["max_v"]) < 0.01
        and abs(resp.get("hrs_capacity", 0) - set_vals["hrs_capacity"]) < 0.1
        and abs(resp.get("chg_threshold_a", 0) - set_vals["chg_threshold_a"]) < 0.001
    )
    if ok:
        protocol_pass += 1

    return fw_ok, protocol_pass, protocol_pass < 4, device_fw


def run_sensor_tests(dev: Device, *, verbose: bool) -> tuple[str, str]:
    resp = dev.query({"get": ["v", "a", "w"]}, verbose=verbose)
    if resp is None:
        return "FAIL", "no response"
    if resp.get("error") == "ina226_not_found":
        return "FAIL", "ina226_not_found"
    if resp.get("error") == "i2c_read":
        return "FAIL", "i2c_read"
    for key in ("v", "a", "w"):
        if key not in resp:
            return "FAIL", f"missing {key}"

    v, a, w = resp["v"], resp["a"], resp["w"]
    if not (0.0 <= v <= 40.0):
        return "FAIL", f"v out of range: {v}"
    if abs(a) > 5.0:
        return "FAIL", f"a out of range: {a}"
    if abs(w) > 200.0:
        return "FAIL", f"w out of range: {w}"

    for i in range(5):
        r = dev.query({"get": ["v", "a", "w"]}, verbose=verbose)
        if r is None:
            return "FAIL", f"stability read {i + 1}: no response"
        if r.get("error") == "i2c_read":
            return "FAIL", f"stability read {i + 1}: i2c_read"
        if r.get("error") == "ina226_not_found":
            return "FAIL", "ina226_not_found"
        for key in ("v", "a", "w"):
            if key not in r:
                return "FAIL", f"stability read {i + 1}: missing {key}"

    return "PASS", ""


def print_summary(results: Results, expected_fw: str, device_fw: str | None) -> None:
    print()
    print(f"FLASH/VERIFY     {results.flash}")
    print(f"SERIAL/COMM      {results.serial_comm}")
    fw_status = results.firmware
    if device_fw:
        fw_status = f"{results.firmware} ({device_fw})"
    elif expected_fw:
        fw_status = f"{results.firmware} (expected {expected_fw})"
    print(f"FIRMWARE         {fw_status}")
    proto_label = "PASS" if results.protocol_pass == results.protocol_total else "FAIL"
    print(f"PROTOCOL         {proto_label} ({results.protocol_pass}/{results.protocol_total})")
    sensor_line = results.sensor
    if results.sensor_detail:
        sensor_line = f"{results.sensor} ({results.sensor_detail})"
    print(f"SENSOR           {sensor_line}")
    for msg in results.messages:
        print(f"  {msg}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Flash power_monitor.uf2 and run unit tests")
    parser.add_argument("--uf2", help="Path to UF2 (default: power_monitor.uf2 or build/power_monitor.uf2)")
    parser.add_argument("--skip-flash", action="store_true", help="Skip picotool flash; run tests only")
    parser.add_argument("--build", action="store_true", help="Build firmware before flashing")
    parser.add_argument("--serial", help="Target device serial number (picotool --ser)")
    parser.add_argument("--timeout", type=float, default=10.0, help="Serial wait/read timeout in seconds")
    parser.add_argument("--verbose", action="store_true", help="Print every serial request/response")
    args = parser.parse_args()

    results = Results()
    expected_fw = ""
    device_fw: str | None = None
    tracked_serial = args.serial

    try:
        if args.build:
            maybe_build(verbose=args.verbose)

        uf2 = resolve_uf2(args.uf2)
        print(f"Using UF2: {uf2}")

        expected_fw = picotool_info_version(uf2, verbose=args.verbose)
        print(f"Expected firmware version: {expected_fw}")

        if args.skip_flash:
            results.flash = "SKIP"
        else:
            tracked_serial = picotool_flash(uf2, args.serial, verbose=args.verbose) or tracked_serial
            results.flash = "PASS"

        port = find_port(tracked_serial, args.timeout)
        print(f"Serial port: {port}")
        results.serial_comm = "PASS"

        settle_s = 0.5 if args.skip_flash else 2.5
        dev = Device(port, args.timeout, settle_s=settle_s)
        try:
            fw_ok, protocol_pass, comm_failed, device_fw = run_comm_tests(
                dev, expected_fw, verbose=args.verbose
            )
            results.protocol_pass = protocol_pass
            results.protocol_total = 4
            results.firmware = "PASS" if fw_ok else "FAIL"
            results.comm_failed = comm_failed or not fw_ok

            if results.comm_failed:
                results.sensor = "SKIP"
                results.sensor_detail = "comm tests failed"
            else:
                results.sensor, results.sensor_detail = run_sensor_tests(dev, verbose=args.verbose)
        finally:
            dev.close()

    except FileNotFoundError as exc:
        print(f"error: {exc}", file=sys.stderr)
        results.flash_failed = not args.skip_flash
        if args.skip_flash:
            results.comm_failed = True
        print_summary(results, expected_fw, device_fw)
        return 1 if results.flash_failed else 2
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        if not args.skip_flash and results.flash != "PASS":
            results.flash = "FAIL"
            results.flash_failed = True
        else:
            results.comm_failed = True
        print_summary(results, expected_fw, device_fw)
        code = results.exit_code()
        return code if code != 0 else 2
    except TimeoutError as exc:
        print(f"error: {exc}", file=sys.stderr)
        results.comm_failed = True
        print_summary(results, expected_fw, device_fw)
        return 2

    print_summary(results, expected_fw, device_fw)
    return results.exit_code()


if __name__ == "__main__":
    sys.exit(main())
