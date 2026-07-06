# Motion-types demo tooling

Generates and runs the **PTP / LIN / CIRC / SPLINE demonstration program** for HexaArm Mini:
the same zigzag pattern executed four times with the four motion types, stacked with small height
steps, tool pointing straight down, ending with a return to the home (all-zero) position. Every
pose in the generated program is **verified reachable against the URDF kinematics before it is
written**, so the program plans and runs on the controller without IK faults.

## Files

| File | Purpose |
|---|---|
| `generate_motion_demo.py` | Generates `programs/motion_types_demo.json` (pendant HMI envelope, formatVersion 1). Pure Python 3, no third-party packages. |
| `run_motion_demo.py` | Headless runner: uploads the program to a live HexaCore over the RDT wire (port 30002) and drives RUN to completion, printing the executing line. |
| `dump_controlstate.cpp` | Tiny tool that prints a default-constructed `ControlState` as wire JSON — the runner's packet template. Rebuild it whenever the protocol changes. |
| `controlstate_default.json` | The dumped template (protocol v2 at the time of writing). The runner refuses to talk to a controller with a different `protocolVersion`. |

## Generating the program

```bash
python Enterprise-RDT-dev/tools/motion_demo/generate_motion_demo.py
```

Runs from any working directory (paths resolve from the script location). Output goes to BOTH
program directories: `Enterprise-RDT-dev/programs/` (controller/pendant, `HEXA*_PROGRAMS_DIR`)
and `Enterprise-RDT-dev/build/programs/` (pendant default when launched from `build/bin`).

What the generator does, in order:

1. **FK/IK straight from the URDF** (`releases/HexaArm_Mini_.../urdf/...urdf`), with the config
   `root_transform` (modelRot*, the URDF up-axis fix) prepended — so it works in the SAME Z-up
   world frame as the controller's KDL chain. Do not remove this: poses generated in the raw URDF
   frame are rotated 90 degrees from what the controller executes.
2. Picks a **reference arm configuration** whose TCP satisfies the placement preferences
   (see Tuning below), then searches the **orientation candidates** (tool straight down) and
   accepts the first that passes a **dense reachability verification**: IK solved every ~8 mm
   along every line, 25 samples along the CIRC arc, seeds chained exactly like the runtime
   interpolator, joint limits from `configs/hexacore_config.json` enforced on every sample.
3. Emits the 25-step program. Every Cartesian step (LIN/CIRC/SPLINE) carries a **realistic
   `Joints` array** (chained IK solutions): the planner seeds inter-segment IK from the authored
   waypoint's joint_target, and a zero seed cannot fold the wrist to a tool-down pose.
4. Ends with `PTP home` — joints all zero (the initial position).

## Running the demo

### From the HexaStudioNG GUI (normal path)

1. Start the stack (HexaCore + HexaStudioNG).
2. `FILE -> LOCAL -> motion_types_demo -> LOAD` — the program appears in the editor and uploads.
3. `RUN`. Watch the 3D viewport: the executed part of the trajectory fades to a ghosted trace
   (v0.6.4 fade-executed feature); a re-RUN restores full brightness.

Do NOT press RUN with an EMPTY editor: the GUI uploads its editor content on RUN, replacing the
loaded program with an empty one (the sequencer rejects it fail-closed as `EmptyProgram`).

### Headless (`run_motion_demo.py`)

```bash
python Enterprise-RDT-dev/tools/motion_demo/run_motion_demo.py
```

Requires a live HexaCore on `127.0.0.1:30002`. The runner: checks the wire `protocolVersion`
against the template; uploads the program (`newProgram` + `programUpdateReqId`); clears a latched
controller error (a previously faulted run makes the controller ignore program commands); sends
RUN and monitors `prog.currentLine` until `PROGRAM FINISHED` / `PROGRAM FAULT` (exit 0/…).

Wire lessons baked into the runner (do not simplify them away):

- **Commands ride every snapshot** (audit F2): one-shot fields live in a single overwritable slot
  on the controller; the runner keeps the active req-id command in every uplink packet until the
  controller confirms it. A single fire-and-forget packet gets overwritten before the NRT tick.
- **Ack the versions**: every uplink echoes `configVersion/trajVersion/programVersion/fileOpId`;
  without acks the server re-sends the heavy payloads in every broadcast until the socket chokes.
- **Keep-alive**: the server disconnects clients with no uplink traffic.
- **Single writer**: run it with the GUI closed (or the GUI idle). Two clients with different
  req-id counters fight over the single command slot and the loaded program.

### Regenerating the wire template after a protocol change

```bash
cd <git root>
g++ -std=c++20 Enterprise-RDT-dev/tools/motion_demo/dump_controlstate.cpp \
    Enterprise-RDT-dev/src/shared/data_types/src/DataTypes.cpp \
    -IEnterprise-RDT-dev/src/shared/rdt_protocol/src -IEnterprise-RDT-dev/src/shared/rdt_protocol \
    -IEnterprise-RDT-dev/src/shared/data_types/src \
    -o dump_controlstate.exe
./dump_controlstate.exe > Enterprise-RDT-dev/tools/motion_demo/controlstate_default.json
```

The runner exits with a clear message when the template version no longer matches the controller.

## Tuning the demo

All knobs are constants near the top of the geometry section of `generate_motion_demo.py`:

- `CORNERS_FULL` — the zigzag corner offsets [mm] in the horizontal plane (current: 220 x 170 mm).
  The search shrinks the pattern (`scale 1.0 / 0.85 / 0.7`) only if the full size fails
  verification.
- `PASS_UP` — per-pass height offsets [mm] relative to the centre (current: +18 / +6 / -6 / -18).
- Placement preferences — in the reference-configuration scoring: horizontal distance filter and
  preferred value (`abs(horiz-255)`), height filter and preferred value (`abs(up-275)`).
- Orientation family — `orientation_candidates()`. On THIS flange the tool axis runs along the
  tip frame's **-Z**, so tool-down is the `(rx=0, ry=0, rz=*)` euler family; `(rx=180, ...)`
  points the tool UP. Candidates keep `|ry| <= 70 deg` (away from the ZYX gimbal lock) and must
  reproduce their rotation matrix exactly through the euler round trip.
- Speeds / waits — in the program-emission section (`motion_params(40|45|60, ...)`, `wait(0.7)`).

Any change re-runs the full dense verification automatically; the generator refuses to write a
program whose path it could not solve.

## Troubleshooting

- `Program error: Motion planning failed` on the controller — read the interpolator lines in the
  controller log (`build/logs/hexacore_debug.log`); typical causes are a stale `HexaCore.exe`
  built mid-refactor (rebuild), or hand-edited poses that skipped verification.
- `no upload confirmation` from the runner — another client (the GUI) is overwriting the loaded
  program; close it or leave its editor untouched.
- Runner exits with `protocol version mismatch` — regenerate `controlstate_default.json` (above).
