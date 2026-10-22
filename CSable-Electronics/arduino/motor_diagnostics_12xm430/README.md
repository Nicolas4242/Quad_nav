# Read-only XM430 commissioning diagnostics

Standalone Arduino Mega 2560 sketch using the ROBOTIS DYNAMIXEL Shield and
Dynamixel2Arduino **0.8.2**. It does not include or change the production firmware,
ROS, calibration gates, or stored motor settings. No upload is performed by the
verification commands below.

## Current hardware versus target

The robot currently has **8 XM430-W350-R and 4 AX-12A**, not twelve XM430 motors.
The four XM430 replacements have not arrived. The configuration in `diagnostics.h`
copies only the established walking-joint mapping:

| Role | ID |
| --- | --- |
| FR thigh / crank | 1 / 2 |
| FL thigh / crank | 3 / 4 |
| RR thigh / crank | 7 / 8 |
| RL thigh / crank | 5 / 6 |
| FL Z / FR Z / RL Z / RR Z | UNSET / UNSET / UNSET / UNSET |

`UNSET = 255` prevents a future-mode scan. Legacy AX IDs **11, 12, 13, 14** are
unconditionally rejected, as are reserved/broadcast IDs 253 and 254 and values
above 255. Duplicate configured IDs invalidate the entire scan before transmission.
Valid primary IDs are 0–252 excluding 11–14. There is no runtime ID entry,
all-ID discovery, broadcast, baud search, or Protocol 1.0 fallback.

Only after physically identifying a replacement separately should its actual,
unique, nonlegacy ID replace its `UNSET` entry and its `replacementVerified`
flag be set to `true`. The flag records human verification; software cannot
establish physical identity. This utility does not assign IDs
or provision baud/protocol. A new motor at a factory default baud or a conflicting
ID must be provisioned separately in an isolated supported fixture. Never infer
replacement IDs from old AX IDs. If future commissioning deliberately reuses
11–14, that requires a separate reviewed policy change after legacy removal;
this version will refuse those IDs.

## Operating modes

Both modes are available in the same sketch through exact uppercase Serial2
commands, each terminated by CR or LF. Reset always selects `CURRENT_8XM`;
mode selection is held only in MCU RAM and never writes any motor setting.

| Mode command | Active entries | Scan requirements |
| --- | --- | --- |
| `CURRENT_8XM` (default) | Eight walking motors, IDs 1–8 in the existing role order | Exact existing mapping; unique valid IDs; replacement entries ignored |
| `FUTURE_12XM` | Eight existing motors and all four Z replacements | Exact existing mapping plus four unique valid replacement IDs, each explicitly marked `replacementVerified = true` |

Selecting a mode only prints the mode and active configuration. `HELP` and `LIST`
include `configuration_valid=0` or `1`; this is a software configuration check,
not proof that motors exist or are safe. Only a subsequent `SCAN` sends READs.
An invalid configuration blocks **all** reads, with no partial future scan.
The supplied future configuration is intentionally invalid: all four IDs are
still `UNSET` and all four verification flags remain `false`.

For the current mixed robot, send these lines separately:

```text
CURRENT_8XM
LIST
SCAN
```

This reads only IDs 1–8 using Protocol 2.0. The AX-12A IDs 11–14 receive no
addressed requests; there is no AX protocol selection or discovery. Current
mode remains usable even when the inactive replacement entries are incomplete.

For the future robot, first identify and provision each replacement individually
in a supported fixture. Edit only its actual ID and verification flag in
`diagnostics.h`, rebuild, and later install the reviewed sketch manually. Do not
assign guessed IDs 9–12. Confirm the legacy motors are removed and every primary
ID is unique. Then select and inspect the future configuration before scanning:

```text
FUTURE_12XM
LIST
SCAN
```

All twelve devices must independently return model 1020 and the expected primary
ID before their remaining registers are decoded. A missing/wrong-model device
is reported and skipped; it does not stop reporting the other configured motors.

## Read-only boundary

The only bus instruction invoked is unicast **READ** with a 20 ms receive timeout.
There are no torque commands (including torque-off), goals, writes, reboot,
reset, action, or EEPROM changes. `begin()` initializes the local UART/direction
pin; `setPortProtocolVersion(2.0f)` configures the local packet parser only.
`read_only_bus.h` uses the library's packet builder and CRC-validating parser
with fixed instruction `0x02` (READ). It explicitly checks the response sender
ID, which the library's generic `Master::read()` does not check in version 0.8.2.
Boot, mode selection, HELP and LIST send no motor requests. Exact uppercase `SCAN` followed by
CR or LF triggers one finite pass; CRLF triggers only one pass. Nothing repeats
on a timer. Invalid, binary and overlong lines are discarded through their line
ending, so an overlong command's suffix cannot become a scan.

Protocol 2.0 is fixed, and known AX addresses are excluded. Protocol 1.0 AX-12A
devices cannot interpret the Protocol 2.0 frame as a valid addressed instruction.
This is not a substitute for identifying the physical wiring and devices.

**Read-only does not mean a powered robot cannot move.** Existing torque/goals,
startup settings, faults and other controllers still matter. READ packets can
refresh a configured motor bus watchdog; stopping reads can allow it to expire.
The tool neither maintains holding nor stops motion and has no emergency-stop
command. Never use it as a safety controller or on an unsupported loaded robot.

## Controlled bench procedure (either mode)

1. Support/restrain every joint independently and provide an independent power
   cutoff. Plan the consequences of reset, power changes and watchdog expiry.
   Disconnect motor power and the motor bus before manually uploading this
   diagnostic sketch in a future commissioning session. Do not upload the gait
   sketch by mistake. No upload or physical connection is part of this change.
2. Verify the Shield's actual **RS-485 path** for the `-R` motors, polarity,
   power, grounds and switch/jumper positions. Keep UART0 (`Serial`) exclusive
   to the Shield: 1,000,000 baud, direction pin 2. Do not use the Mega USB serial
   monitor as the diagnostic console; it shares UART0 and can disturb the bus.
3. Use a suitable UART adapter or the Pi on **Serial2**, 115200 baud, 8N1:
   Mega TX2 pin 16 to adapter RX, RX2 pin 17 from adapter TX, common ground,
   with appropriate level translation (Mega TX is 5 V; Pi RX is not 5 V tolerant).
   Stop ROS/other command senders and ensure no other motor-bus master is active.
4. Confirm physical IDs and that legacy AX devices are excluded. Restore the
   verified bench connections and power only when the mechanism is supported.
   Boot prints the current/target distinction and configured list without reads.
   Select `CURRENT_8XM` for the mixed robot, or the fully verified `FUTURE_12XM`
   after migration. Send `LIST`, require `configuration_valid=1`, then send
   `SCAN` with a newline when ready. Mode selection alone does not scan.
5. Save the serial output through `# END_SCAN`. Send another `SCAN` only when
   another observation is wanted. Do not send commands during a scan: UART input
   can buffer a later scan or overflow. Unplugged/missing motors produce failures;
   never treat missing feedback as zero position or proof that torque is off.

## Output and interpretation

CSV is long-form (one register per row) with ten fixed columns:

```csv
scan,role,requested_id,field,address,width,value,status,library_error,packet_error
1,FR_THIGH,1,model_number,0,2,1020,ok,0,0
```

These are **format examples, not measured results**. Strip lines beginning with
`#` and repeated CSV headers when combining scans. Pivot by `scan`, `role` and
`field` for a calibration table. Each scan first prints `# SCAN_MODE,CURRENT_8XM`
or `# SCAN_MODE,FUTURE_12XM`; keep this metadata with the capture. The ten-column
CSV schema is unchanged. Current mode produces no Z rows; an invalid future
configuration produces an error and no measurement rows. `requested_id` is the configured bus address;
`motor_id` is the separately read primary ID register. Values are raw decimal
register values, not joint angles or calibration sign-off. Position and homing
use encoder counts, velocity uses the motor's raw velocity units. Negative
32-bit position/velocity/homing values and unsigned byte values 128–255 are
preserved. Present position already includes the internal homing reference;
do not add that offset again. Limits are stored motor settings, not verified
mechanical safe limits.

Fields: model number, primary motor ID, firmware version, present position,
present velocity, torque enable, operating mode, drive mode, homing offset,
minimum/maximum position limits, hardware error status and secondary ID.
The model must be 1020 (XM430-W350) and primary ID must match before later
register reads proceed. A different model is reported and skipped, never decoded
using the XM430 table. Model number alone cannot distinguish physical TTL/RS-485
variants or identify the joint.

`ok` means a complete error-free register read, not that its value is safe.
`alert` preserves data with the motor's hardware-alert bit set; inspect hardware
error status. `transport_error`, `length_error`, `wrong_response_id` and `status_error` have blank
values, with library and packet errors retained. `unexpected_model` and
`id_mismatch` preserve the received value and skip the rest of that motor.
A valid instruction-error reply is labelled `status_error` even if it contains
no register data; a length mismatch without an instruction error is `length_error`.
`identity_failed_remaining_skipped`
explicitly marks omitted fields. A failed later field does not suppress the
remaining reads, so unsupported/erroring registers are individually visible.

At most 13 reads per configured motor (104 with defaults; 156 with all twelve).
No retries or wait-for-motor loops exist. Receive-timeout budgets sum to 2.08 s
for eight or 3.12 s for twelve, plus UART transmission, formatting and driver
overhead; these are **not end-to-end real-time bounds**. Failed identity reads
skip subsequent reads. A timeout can also mean wrong baud/protocol, wiring,
collision, or a status-return setting that suppresses READ replies, not necessarily
an absent motor. Duplicate physical IDs, aliases and stale responses cannot be
reliably ruled out by this utility. Samples are sequential, not simultaneous.
Protocol 2.0 replies do not echo the register address: a late same-ID, same-width
response cannot be conclusively distinguished from the expected response.
The stock library also has direction-pin readback loops and UART flush waits;
the receive timeout does not protect against a stuck GPIO, disabled interrupts,
or a failed MCU/UART. No software timeout is a physical safety guarantee.

## Software verification

Mega build verified on **2026-10-22**, using AVR core 1.8.8 and
Dynamixel2Arduino 0.8.2: **11,378 bytes flash, 1,335 bytes static RAM**.
The remaining 6,857 bytes must also accommodate stack and library heap use
(including the default 1,024-byte packet buffer). `--warnings all` reported
warnings in the upstream library/core, none in the diagnostic sources.
Manual source and library-path review confirmed that the only emitted opcode
is READ; no motor write/motion API calls are present. Production/ROS files are
unchanged. No upload, physical robot access, commit or push was performed.
The host regression suite also **passed** on 2026-10-22 using Zig 0.15.2's C++17
compiler with `-Wall -Wextra`, including both runtime modes, all 65,536 ID-policy
inputs and the parser, CSV, decoding, boot, identity and injected-fault checks below.
A separate temporary Mega fixture with synthetic, verified replacement IDs
21–24 also compiled successfully with the same flash/RAM figures, exercising
the configured twelve-motor path. Those IDs are test data only: the repository
configuration still has four `UNSET` IDs and false verification flags. Never
upload the synthetic fixture. All eleven injected communication/identity fault
scenarios were exercised in both eight- and twelve-motor host scans.

### Final source review (2026-10-22)

Reviewed setup, parser dispatch, both mode gates, every scan/error branch, CSV
decoding, and the pinned library's packet transmit/receive and serial-port paths.
Only unicast Protocol 2.0 READ (`0x02`) is emitted. Model 1020, primary ID,
response sender, response length, transport errors and packet errors are checked.
No torque, goal, configuration-write, reboot, broadcast or Protocol 1.0 path
exists in the diagnostic call graph, including failure handling.

One reporting defect was found and fixed: an instruction-error status with zero
data bytes was previously classified as `length_error`. A regression first
reproduced the failure; after correcting error priority, the full host suite and
both clean Mega builds passed again. Tests now include this case in both modes.
No remaining software blocker was identified for a controlled supported bench
test. This is a source review and simulated verification, not independent
hardware certification. The receive-only timeout scope, same-ID late-reply
ambiguity, watchdog effects and physical-ID/alias limits above remain applicable.

Compile only, with Arduino CLI and the pinned core/library installed:

```sh
arduino-cli core install arduino:avr@1.8.8
arduino-cli lib install Dynamixel2Arduino@0.8.2
arduino-cli compile --fqbn arduino:avr:mega CSable-Electronics/arduino/motor_diagnostics_12xm430
```

Host tests compile the **actual sketch and shared header**, replacing only the
Arduino/DYNAMIXEL interface with a mock that exposes no motor-write API:

```sh
c++ -std=c++17 -Wall -Wextra -I CSable-Electronics/arduino/motor_diagnostics_12xm430/tests CSable-Electronics/arduino/motor_diagnostics_12xm430/tests/verify.cpp -o /tmp/verify_diagnostics
/tmp/verify_diagnostics
```

Tests exercise all 65,536 configuration-ID inputs, duplicates and UNSET entries,
current-mode exact ID mapping and exclusion of inactive replacements, mode
selection without traffic, rejection of incomplete/unverified future configuration,
a host-only verified twelve-motor fixture (synthetic IDs never enter production),
exact commands, partial lines, CRLF, binary/overflow rejection, silent boot,
CSV columns and blank values, signed 32-bit decoding, high unsigned byte values,
model/ID gating, wrong-sender replies, transport/short-packet/status faults, alert-only responses,
and local protocol initialization failure. Mocks do not test the real wire
parser, electrical timing, or motor behavior.

Hardware checks still required: physical ID-to-joint mapping, replacements and
unique IDs, disabled/unambiguous secondary aliases, actual firmware/register
support, RS-485/Shield wiring, baud, voltage levels, status-return behavior,
timeouts/late packets/collisions, watchdog behavior, encoder reference, torque
status, mechanical ranges and all commissioning calibration. Software tests
cannot establish these or certify that the quadruped is safe to load.

**Current-mode readiness:** the software is ready for a controlled, supported
eight-XM bench test subject to the procedure above. It has not been tested on
motors and is not clearance for unsupported standing or walking. First confirm
that boot/mode commands produce no motor traffic, `SCAN` emits only Protocol 2.0
READs to IDs 1–8, actual identity/register results are credible, and missing
motors return bounded failures. Future-mode hardware testing remains blocked
until all four replacements are physically identified and verified.

References: [ROBOTIS XM430-W350 control table](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/),
[Protocol 2.0](https://emanual.robotis.com/docs/en/dxl/protocol2/),
[Dynamixel2Arduino 0.8.2](https://github.com/ROBOTIS-GIT/Dynamixel2Arduino/tree/0.8.2).
