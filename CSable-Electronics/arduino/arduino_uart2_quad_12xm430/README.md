# 12 × XM430 migration — Stage 2.1

This firmware continues the CentraleSupélec QUAD_NAV implementation: Stage 1 copied the working 8 × XM430-W350-R + 4 × AX-12A sketch. Stage 2 targets 12 × XM430-W350-R, all using Protocol 2.0. `../arduino_uart2_quad_standard_gait/` remains the untouched reference baseline; the committed Stage 1 copy is available at `3e7472c`.

**The supplied configuration intentionally prevents motion.** The four replacement Z motors have not been physically calibrated. Their configuration contains invalid IDs and unset values, and every `verified` flag is `false`. Stage 2.1 also requires verified mechanical startup limits for the eight walking joints in `legStartup[]` and keeps `MOTION_COMMISSIONING_VERIFIED = false`. None of these values is supplied by this change. Boot fails closed to `MODE_STOPPED`; no motor is torque-enabled and no position goal is sent by this firmware. Cleanup attempts torque-off for known IDs, but cannot address unknown motors.

## Architecture and mapping

The eight thigh/crank motors retain the existing planar gait and inverse kinematics. The four Z motors are fixed-position holding joints. There is no dynamic 3-DOF gait control.

| Role | Identifier | ID | Direction | Software offset |
| --- | --- | --- | --- | --- |
| FR thigh | `FR_THIGH` | 1 | +1 | 0° |
| FR crank | `FR_CRANK` | 2 | +1 | 0° |
| FL thigh | `FL_THIGH` | 3 | -1 | 0° |
| FL crank | `FL_CRANK` | 4 | -1 | -17° |
| RR thigh | `RR_THIGH` | 7 | +1 | 0° |
| RR crank | `RR_CRANK` | 8 | +1 | 0° |
| RL thigh | `RL_THIGH` | 5 | -1 | 0° |
| RL crank | `RL_CRANK` | 6 | -1 | 0° |
| FL Z | `FL_Z` | Unset | Unset | Unset |
| FR Z | `FR_Z` | Unset | Unset | Unset |
| RL Z | `RL_Z` | Unset | Unset | Unset |
| RR Z | `RR_Z` | Unset | Unset | Unset |

The value `UNCONFIGURED_MOTOR_ID = 255` is a sentinel, not a proposed motor ID. The software rejects reserved/broadcast IDs and duplicate configured IDs. The previous AX IDs and holding angles have been removed; none are replacement calibration.

The Arduino Mega interfaces remain `Serial` at 1,000,000 baud through the DYNAMIXEL Shield (`DXL_DIR_PIN = 2`), `Serial2` at 115200 baud for commands/status, and I2C for the BNO055. Verify the physical RS-485 connection for all twelve `-R` motors. All physical IDs must be unique, and secondary-ID aliases must be disabled across the bus; the software now checks all twelve motors for disabled secondary IDs, pending registered instructions, automatic startup settings and position-mode assumptions.

## Calibration gate

Populate the four Z entries in `motors[]` and the corresponding entries of `zCalibration[]`, in `Z_INDICES` order: `FL_Z`, `FR_Z`, `RL_Z`, `RR_Z`. There is no serial command to unlock calibration.

| Configuration | Required physical verification |
| --- | --- |
| `MotorConfig.id` | Actual unique ID, matching the identified physical joint; Protocol 2.0 and the existing bus baud |
| `MotorConfig.direction`, `offsetRad` | Model-to-motor sign and software offset, measured after horn indexing and internal settings are finalized |
| `MotorConfig.tolerance` | Approved neutral holding/readback tolerance in raw motor counts; used for Z only |
| `neutralJointRad` | Desired fixed **model joint angle** in radians |
| `minRaw`, `maxRaw` | Safe mechanical interval, including margins, in absolute motor counts; must match provisioned internal position limits |
| `startupToleranceRaw` | Approved maximum initial deviation from neutral; also bounds the permitted startup correction |
| `profileVelocityRaw`, `profileAccelerationRaw` | Explicit positive profile settings validated under the intended load |
| `expectedDriveMode` | Verified internal normal/reverse setting, 0 or 1; velocity-based profiles and automatic torque-on-goal-update disabled |
| `expectedHomingOffsetRaw` | Verified internal `HOMING_OFFSET`, distinct from the software `offsetRad` |
| `expectedFirmwareVersion` | Actual motor firmware version |
| `expectedPwmLimitRaw`, `expectedCurrentLimitRaw`, `expectedVelocityLimitRaw` | Verified internal limits; profile velocity must not exceed the configured velocity limit |
| `verified` | Deliberate sign-off for that joint only after all calibration and internal-settings checks are complete |

`verified = true` alone does not unlock motion. `validateZCalibration()` rejects unset/nonfinite values, invalid bounds, duplicate IDs, invalid directions, zero profiles and inconsistent tolerances. All four entries must pass before startup proceeds. `zCalibrationReady` records this software check; `zAxisReady` is set only after hardware preparation and settling succeed. `setMode()`, the control loop, startup helpers and the leg command helper enforce readiness.

The sign-off also requires reviewing physical stops, collisions, assembly orientation, safe holding force, gains, shutdown protections, supply limits and the other internal settings. The firmware checks a selected set of critical registers; it cannot establish physical safety from these numbers or verify that a human actually measured them.

### Model angles and absolute motor positions

For Z holding, `zJointRadToRawChecked()` calculates:

```text
motorAbsRad = offsetRad + direction * neutralJointRad
raw = round(motorAbsRad * 4096 / (2*pi))
```

`motorAbsRad` is the absolute motor coordinate used by the goal register, including the effect of the calibrated internal reference frame. The internal homing offset is not added again by this formula. The conversion rejects values outside the single-turn encoding or calibrated mechanical interval; it never wraps or clamps a Z goal. A joint whose safe range crosses the encoder boundary is not supported by this Stage 2 holding path and needs a reviewed calibration/assembly solution.

Stage 1's `holdZAxis()` used absolute AX angles and bypassed `direction`/`offsetRad`. The new neutral target uses both fields. The measured position used to preload a goal is already an absolute motor count and is sent directly, without applying those fields again. The eight walking motors retain their original conversion and wrapping behavior.

## Stage 2.1 safety corrections

The six review findings are addressed as follows. Tests are software evidence, not proof of physical safety.

| Finding / previous behavior | Correction and why it addresses the defect | Verification | Still requires physical validation |
| --- | --- | --- | --- |
| Walking torque was enabled with an old/unknown goal, then standup assumed `START_X`/`START_Y` | `validateLegStartup()`, `prepareLegMotor()`, `readLegStartupPosition()` and `setupOneMotor()` require verified mechanical ranges, a stationary pose near the existing IK start, a measured absolute goal with readback before torque, and limited displacement after enable. `verifyLegStartHolds()` checks again before the first ramp command. | Unknown pose, unverified/invalid limits, failed preload, torque-on shift and post-setup drift tests; goal-before-enable ordering for all twelve motors | Actual assembly and encoder reference, support against collapse, approved tolerances, safe path from every accepted initial pose into standup, drive/homing settings, gains and torque-on transients |
| D2A 0.8.2 sign-extended `SECONDARY_ID` and `FIRMWARE_VERSION` bytes | `readMotorItem()` reads raw bytes, requires the complete register width and checks transport/status errors. It zero-extends 1/2-byte registers and preserves signed 4-byte bit patterns. Disabled IDs 253, 254 and 255 remain positive. | Compiles the actual library decoder function body and actual `actuator.cpp`; reproduces negative old results and checks corrected results, including firmware 200 and signed homing offset | Readbacks on the actual motor firmware and physical bus |
| Cleanup delayed Z disable behind eight walking motors and per-motor verification/delays | `safeCleanup()` dispatches the affected motor first, then remaining Z joints, then remaining walking joints. All write attempts precede all readbacks. The 40 ms delays are removed. Failure paths dispatch shutdown before diagnostic printing. | Ordered transaction trace, partial initialization, failed write/read, lost ACK with successful readback, runtime Z fault priority | Actual disable latency, late responses, unreachable motors, mechanical consequences of losing support |
| Walking setup ignored critical failures | Checks model, torque-off, mode, drive/homing, limits, status return level, secondary ID, startup configuration, registered instruction, watchdog and hardware errors. Profile writes/readbacks, preload and torque-enable/readback must succeed. Runtime goal writes are checked too. | Failed critical writes/readbacks, bad settings on the last motor, partial enable and position-shift tests | Provisioned settings, physical response and power/bus integrity under all twelve loads |
| `zAxisReady` could remain true after loss of holding | Fresh complete feedback is required immediately before standup and during operation. `readZFeedback()` checks position, movement, torque and hardware errors; a failed/late/stale observation revokes readiness and latches stop. A commissioning gate blocks unsupported operation by default. | Pre-standup loss, position/torque/error/movement faults, communication/short-packet faults, stale cache, insufficient timing slack and over-budget reads | Feedback latency and coverage, sustainable schedule, acceptable fault-detection delay, independently safe physical fault response |
| Dwell was timed before the first complete good scan | `zSettleObservation()` starts its 200 ms window after the first complete good observation, resets after a bad observation, and accepts only a later good observation spanning the window. Incomplete reads abort. Overall timeout is checked before acceptance and after final revalidation. | Delayed first scan, interrupted stability, incomplete scan, 199/200 ms boundary, scan completing after the 5 s timeout | Real settling behavior, sampling interval and suitable acceptance thresholds |

### Walking startup calibration and commissioning gate

`legStartup[]` has one entry for each of the existing eight `motors[]` entries, in the same order. It does **not** change their IDs, directions or software offsets. Each entry requires:

- `verified`: deliberate mechanical sign-off.
- `minRaw`, `maxRaw`: measured safe absolute motor interval, matching provisioned internal position limits.
- `startToleranceRaw`: approved deviation from the **existing** IK result at `START_X`/`START_Y` that is safe for entry into the existing ramp.
- `holdToleranceRaw`: approved maximum change from the measured preload position, no larger than `startToleranceRaw`.
- `expectedDriveMode`, `expectedHomingOffsetRaw`: verified internal reference settings, distinct from software direction/offset.

No physical values are invented here. `legStartRaw[]` is derived from the unchanged IK and existing motor mapping; it is an expected encoder position, not evidence that the assembled mechanism is safe there. Validate the whole accepted pose interval, standup path and gait envelope, including collisions and load-dependent deflection. Runtime leg goals outside the calibrated interval cause a fault rather than being clamped.

`MOTION_COMMISSIONING_VERIFIED` remains `false` until there is reviewed evidence for mechanical support/fault response, startup pose and motion corridor, bus feedback, and timing with the existing command/IMU workload. It is an additional commissioning sign-off, not a way to override missing calibration or a serial unlock. Do not set it just to obtain movement for testing. Use a dedicated commissioning utility and supported fixture to obtain the missing evidence first. If acceptable timing/fault response cannot be demonstrated with this controller, keep it false; further reviewed work is required before loaded operation.

## Startup

1. Initialize the existing serial/IMU interfaces and verify the unchanged planar references. Validate all four Z calibrations, all eight walking startup calibrations, and the commissioning sign-off. A failure blocks goals and torque-enable and leads to latched `MODE_STOPPED` and cleanup.
2. Dispatch torque-off to all known IDs, then read back all twelve. Any unknown ID or unconfirmed torque-off blocks startup.
3. Preflight all eight walking motors with torque off. Require pre-provisioned Position Control Mode (`OPERATING_MODE = 3`), verified drive/homing and position limits, disabled secondary IDs, no pending registered instruction/error/watchdog, and `STATUS_RETURN_LEVEL = 2`. Require `STARTUP_CONFIGURATION = 0` for firmware 45 or later. Write and verify the original profiles: velocity 0, acceleration 20. Check stationary measured positions near the existing IK start.
4. Preflight all four Z motors, including their firmware and power/velocity limits. Require position mode and the same control/alias protections. Write and verify the calibrated **positive** Z profiles. Do not rewrite operating mode or other EEPROM settings here: changing mode can reset gains and position.
5. For each Z joint, require a stationary, unambiguous single-turn position within its calibrated interval and startup tolerance. Preload and read back that measured absolute goal with torque off. Enable torque with readback, reject an excessive counter/position change, then command only the bounded near-neutral correction.
6. Observe complete Z settling scans spanning at least 200 ms; abort on failed reads or after 5 s. Require neutral position within tolerance, no movement, torque on and no hardware error. Revalidate complete feedback before reporting Z holding.
7. Revalidate Z feedback before each walking motor enable. Re-read the walking position, preload and verify its measured absolute goal, confirm torque-off, enable with readback, and check actual position, velocity and hardware errors. Any failure cleans up the partially enabled set.
8. Immediately before the first standup ramp goal, verify all eight measured holds and revalidate all four Z joints. Only then run the unchanged smooth standup calculations. Continue feedback supervision during the ramp. A failure or processed stop command requires reset before restart.

All twelve joints must be externally supported and stationary during commissioning. A valid encoder reading does not establish a safe physical pose. Reading before enable and checking after enable cannot exclude movement between samples or transients under load.

The [ROBOTIS XM430 control table](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/) specifies that zero profile velocity is unlimited and torque enable can reset the position counter in position mode. The eight walking motors retain velocity 0; this is not a slow startup profile. Calibration must account for that response and ensure a physically safe, sufficiently close start pose.

`Dynamixel2Arduino 0.8.2` omits the XM430 `STARTUP_CONFIGURATION` mapping and lists `BUS_WATCHDOG` with a two-byte width. `readMotorItem()` overrides these with one-byte reads at addresses 60 and 98. Other addresses/widths come from the library's XM430 control table. Raw requests avoid the library's signed-byte conversion and cached-model lookup. Every production read/write uses `DXL_IO_TIMEOUT_MS = 2` for the receive timeout; this is not an end-to-end transaction bound.

## Feedback supervision and timing limits

`readZFeedback()` performs one 72-byte Protocol 2.0 read covering addresses 64 through 135. It uses `TORQUE_ENABLE` (64), `HARDWARE_ERROR_STATUS` (70), `MOVING` (122) and signed `PRESENT_POSITION` (132). It requires the full response, no transport/status error, torque on, no movement/error, and a position within both mechanical limits and neutral tolerance. Its timestamp is taken before the request, conservatively accounting for transaction time.

During standup and the main loop, `serviceZMonitor()` attempts one joint per 10 ms slot, rotating across the four joints, only when at least 4 ms remain before the next nominal control deadline. A snapshot taking more than 3,000 microseconds causes a fault. A complete round is approximately 40?50 ms when slack is consistently available. Every walking goal checks that all four cached observations are at most 200 ms old. If monitoring is starved, the next freshness check stops motion; it does not reuse stale readiness indefinitely. The 200 ms threshold is a software policy requiring mechanical validation, not a demonstrated safe detection time.

At 1 Mbit/s, a 14-byte read request plus an 83-byte response takes approximately 0.97 ms on the wire before byte stuffing, return delay, turnaround and software overhead. Eight ordinary four-byte goal writes with status replies account for roughly 2.16 ms wire time before those costs. The monitor therefore needs real spare time; average bandwidth alone does not establish a 10 ms deadline.

**The existing system is not hard real-time.** Status formatting/Serial2 transmission, an unbounded stream of incoming commands, and BNO055/I2C calls remain blocking. Detailed `STATUS`/`SCAN` still reads twelve positions and can overrun a control period; those reads now use the short checked helper, and a failed read triggers cleanup. A stalled call can postpone both freshness detection and serial STOP processing. Catch-up scheduling, trajectory calculations, command vocabulary, status format and IMU integration are preserved. The new monitor alone does not fix these inherited scheduling limitations. Keep loaded operation blocked until worst-case measurements and a reviewed physical fault plan establish suitability; otherwise retain the lock and address those limitations separately.

## Commands, status and shutdown

The command vocabulary remains `WALK/W/START`, `STAND/S`, `TOGGLE/T/SPACE`, `QUIT/Q/STOP/DISABLE`, `STATUS/STATE?/PING/SCAN`, and `HELP`. `STAND` keeps torque enabled during normal operation. A processed stop command or safety fault latches `MODE_STOPPED`; reset is required to restart. ROS files and the existing command parser are unchanged. There is no calibration override command.

`safeCleanup()` immediately revokes software readiness. It sends bounded individual torque-off requests to the affected joint first, then remaining Z joints, then remaining walking joints, without duplicate requests. Unknown IDs are skipped; there is no broadcast. Each request consumes its status reply before the next request to avoid deliberate overlap on the half-duplex bus. Receive timeout is 2 ms per request, plus transmission/driver overhead; late responses and failure behavior must be measured on the actual bus. There are no fixed inter-motor delays and no readbacks until every disable has been attempted.

The subsequent verification phase reads `TORQUE_ENABLE` for every configured motor, continuing after failure:

- `STATE,torque_off_dispatch_ack=1`: all twelve write requests received successful acknowledgments. This alone does not confirm torque-off.
- `torque_off_unconfirmed=0` / `OK,cleanup_done`: all twelve readbacks returned zero at verification time. This can occur even if a write acknowledgment was lost.
- `torque_off_unconfirmed=1` / `WARN,cleanup_incomplete_unknown_or_unresponsive_motors`: at least one motor is unknown, failed a read, or was not off. Never interpret a sent command as proof of de-energization.

Existing `STATUS` fields remain, including `raw_leg`/`raw_z` when ready, and the Stage 2 fields `z_calibrated`, `z_ready`, `torque_off_unconfirmed`. `torque` is software state, not continuous confirmation for the walking motors. Failed status reads abort the status frame and enter fault cleanup. Fault reports may therefore interrupt a partially printed status line. Successful Z setup still reports `OK,z_axis_holding_raw=`.

**Torque-off can cause collapse or release a loaded joint.** No software response implemented here makes an unsupported robot safe. An unreachable motor can remain powered, and a readback is only a sampled register state. Provide restraint/support and an independent accessible motor-power cutoff; validate what happens when one joint releases while others remain powered. Serial STOP is not a hardware emergency stop.

## Differences from Stage 1 and preserved behavior

- Removed AX motor type, Protocol 1.0 configuration, AX position encoding and AX control-table setup from this sketch.
- Renamed `AX_FL_Z`/`AX_FR_Z`/`AX_RL_Z`/`AX_RR_Z`, `AX_MOTOR_COUNT` and `AX_INDICES` to mechanical-role names. Replaced `AX_HOLD_DEG` with calibrated model angles in `zCalibration`.
- Added default-locked calibration, checked Z setup/holding, measured-position goal preload, startup readiness, diagnostics and torque-off confirmation.
- Preserved all eight thigh/crank IDs, directions, offsets, gait constants, phase logic, inverse/forward kinematics, limiter, nominal scheduling and ordered leg position commands. Added readiness/stop guards without changing their trajectory calculations.
- Preserved the serial interfaces, command vocabulary and BNO055 integration. As in Stage 1, the stopped loop returns before periodic IMU/status transmission; explicit status requests remain available.
- Changed the boot robot label to `quadruped_12xm430`. The original implementation, MATLAB, ROS and experimental files are untouched.

## Cautious bench sequence

1. Review and compile the default-locked sketch. With motor power disconnected, check that boot and all walking/standing commands remain locked. Do not use the synthetic test fixtures on hardware.
2. Support/restrain the mechanism and provide an independent power cutoff. Identify motors individually with a dedicated setup utility; this sketch provides no single-joint calibration mode or bypass.
3. Measure all four Z calibrations and the eight walking startup intervals/tolerances. Verify physical IDs, RS-485 wiring, power budget, firmware, horn indexing, reference frames, collision margins, gains, protection settings and actual internal limits. Provision mode/limits/aliases/automatic-start settings separately, then remeasure against the final reference frame.
4. In supported fixtures, establish measured-hold torque-on behavior, bounded Z corrections, safe entry into the existing standup path, and consequences of torque removal or one unreachable joint. Determine approved load/current/thermal limits and acceptable response times. Setting `verified` is a recorded sign-off, not a test shortcut.
5. Measure worst-case bus return delay, 72-byte feedback behavior, packet loss/late replies, all twelve goal/write latencies, Mega execution time, stack margin, serial command/status traffic and IMU/I2C latency. Verify monitoring freshness and fault response under stress. If the inherited blocking paths cannot satisfy the required response, keep `MOTION_COMMISSIONING_VERIFIED = false`; do not authorize loaded operation.
6. Only after reviewed commissioning evidence, configure the sign-offs and test the complete startup in a fully supported fixture with the expected physical start pose. This sketch automatically enters standup after successful initialization. Observe preload-before-enable, counter behavior, partial initialization failure and out-of-range rejection.
7. Exercise STOP, loss of Z torque/position/readiness, disconnected motors, stale feedback and failed torque-disable in controlled conditions. Measure physical stopping/holding behavior, not only log messages. Unsupported standing/walking remains prohibited until the corresponding loaded fault response and timing have been validated.

## Verification

No physical robot validation or upload has been performed. Software checks on 2026-10-12:

| Check | Result |
| --- | --- |
| Mega 2560 default-locked build | Passed: 26,176 bytes flash (10%); 2,873 bytes static RAM (35%), leaving 5,319 bytes for stack/local variables and runtime allocation |
| Isolated synthetic full-path Mega build | Passed: 32,156 bytes flash (12%); 3,172 bytes static RAM (38%), leaving 5,020 bytes before stack/runtime allocation. Never upload this fixture. |
| Static Stage 1 comparison (`3e7472c`) | Eight exact walking motor entries; unchanged geometry/gait constants, 14 preserved functions and main-loop trajectory block |
| Trajectory and command regression | 300 standup samples plus 1,000 walking/standing samples; all 10,400 ordered raw motor commands match Stage 1 exactly |
| Decoder regression | Actual Dynamixel2Arduino 0.8.2 decoder body and actual control-table implementation compiled; reproduces sign extension and verifies corrected byte interpretation |
| Safety regression | 24,000 assertions passed, including trajectory checks: default/incomplete gates; walking preloads/poses/settings; partial initialization; failed communication/write/readback; prioritized shutdown; acknowledgment versus readback; pre-standup and runtime Z faults; stale/late/incomplete feedback; settling intervals and timeouts |
| Scope | Only this implementation folder changed; original standard firmware and user-owned `stage2.diff` left untouched; no branch switch, commit or push |

The production build can optimize away paths behind constant locked gates, so the temporary synthetic Mega build separately checks the compiled startup/motion paths. The compiler's RAM figures exclude heap allocation and stack: Dynamixel2Arduino allocates a 1,024-byte packet buffer plus its serial-port object, and other libraries also allocate runtime objects. The new feedback read has a 72-byte local buffer; cleanup uses a twelve-element order array (24 bytes on AVR). Stack/heap margin and worst-case call nesting have not been measured on hardware.

The reproducible host tests are `tests/verify_stage21.py` and `tests/verify_stage21.cpp`. The runner extracts Stage 1 from Git and the library decoder from the installed 0.8.2 source, then compiles both sketch versions with simulated serial, time, IMU and transport. Synthetic configuration is applied only to generated host includes outside the sketch; there is no production test bypass. Tests do not model actual motor physics, UART interrupts, packet framing, collisions, I2C stalls or AVR timing.

```text
python tests/verify_stage21.py --library <Dynamixel2Arduino-0.8.2-directory> --compiler <g++-or-clang++-or-zig> --build-dir <temporary-directory-outside-sketch>
arduino-cli compile --fqbn arduino:avr:mega --warnings all CSable-Electronics/arduino/arduino_uart2_quad_12xm430
```

Run the first command from this sketch directory and the second from the repository root. The runner requires Python 3.9+ and Git with commit `3e7472c` available. Build tools used: Arduino CLI 1.5.2-rc.1, `arduino:avr` 1.8.8, Dynamixel2Arduino 0.8.2, Adafruit BNO055 1.6.4, Unified Sensor 1.1.15, BusIO 1.17.4, Zig 0.15.2. Isolated tool/build directories are under `%TEMP%/quad_nav_stage2_tools/`. Library signedness warnings and Arduino core unused-parameter warnings also occur in the baseline; no sketch warning was reported.

### Remaining Critical/High hardware dependencies

- **Critical if motion is unlocked without physical evidence:** wrong assembly/reference/pose, unsafe accepted startup interval or unsafe torque-off can cause sudden motion/collapse. Encoder checks cannot certify these; all calibration and commissioning gates remain locked.
- **High:** real-time fault detection is not guaranteed through inherited blocking serial/I2C paths. Bus loss can prevent torque-off or its confirmation. A sampled Z check can miss motion between observations, and walking motor feedback is not continuously supervised by this change.
- **High:** firmware cannot control motor automatic behavior before boot, another bus master, physical duplicate IDs, wrong wiring, power brownouts or a failed motor controller. Require unique physical IDs and exclusive bus ownership. Mode/power/limit readbacks do not identify the physical joint or certify joint force.
- **High:** the short receive timeout, snapshot range, monitor budget, freshness threshold, profiles, dwell and startup tolerances must be validated with the actual hardware. Adjusting thresholds to hide faults is not a valid commissioning procedure.

Default-locked software is available for review and powered-off checks. Motor-powered commissioning and especially loaded operation still require the physical evidence and safeguards above. Compilation and simulated tests do not establish physical safety.
