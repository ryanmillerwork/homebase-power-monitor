#!/usr/bin/env tclsh9.0

# usage:
# tclsh9.0 power_mon.tcl
# returns {"v":29.680,"a":0.3683,"w":10.9314,"pct":77.50,"charging":true}



# Find the power monitor device
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
  puts stderr "power_monitor not found under /dev/serial/by-id"
  exit 1
}

# Open and configure the serial port
set fd [open $port r+]
fconfigure $fd -buffering none -translation binary -blocking 1

# Send the request
puts -nonewline $fd {{"get": ["v", "a", "w", "pct", "charging"]}}
flush $fd

# Read the response (single line)
gets $fd response

# Close the connection
close $fd

# Output the result
puts $response