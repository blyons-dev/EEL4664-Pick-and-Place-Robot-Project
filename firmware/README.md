# Python GUI and Arduino reference — GUI v2.2.1 / firmware v2.1.0, Ngo Lab

For the required workflow and submissions, use the [student guide](../STUDENT_GUIDE.md). This page explains the interface and resolves common setup problems.

## Beginner setup, terminal and COM port

### Fork and clone a writable student copy

1. Sign in to GitHub, open [the instructor repository](https://github.com/hoanbklucky/eel4664-robotic-arm-project-miniArm), and click **Fork** into your account.
2. On your fork, choose **Code → HTTPS** and copy its URL. The owner must be your own username.
3. In VS Code press **Ctrl+Shift+P**, select **Git: Clone**, paste that URL, choose where to save it, and open the cloned folder. Install Git if prompted. The local repository is linked to your own fork.
4. Edit locally. In **Source Control**, review/stage the intended files, enter a message, Commit, then Push/Sync Changes. Complete GitHub sign-in if requested. Do not upload virtual environments or large recordings.
5. Before a fresh lab start, GitHub's **Sync fork** can bring instructor updates into your fork. Commit your own work before pulling updates into the local clone; preserve your measured calibration.

If you prefer Git commands, open a terminal in a parent folder such as Documents/Robotics and use your fork's actual HTTPS URL:

```powershell
git clone https://github.com/YOUR_USERNAME/eel4664-robotic-arm-project-miniArm.git
cd .\eel4664-robotic-arm-project-miniArm
```

`YOUR_USERNAME` is a placeholder: replace it. The repository already ignores `calibration.json`, caches and virtual environments. Submit your real calibration file separately through the course system, even if it does not appear in Git's changed-file list. Put your fork URL in the DH derivation; verify that the instructor can access submitted repositories/video links.

### Navigate to the Python GUI folder

**Method A — VS Code (recommended).** Open the cloned repository as the VS Code folder. Select **Terminal → New Terminal**, choose PowerShell, and run:

```powershell
cd .\lab03_inverse_kinematics\miniarm_controlC
Get-Location
dir miniarm_controlC.py
```

The printed path should end in `lab03_inverse_kinematics\miniarm_controlC`; the last command must list `miniarm_controlC.py`. Commands below assume this working directory. If you already changed into this folder, do not repeat the relative cd command.

**Method B — File Explorer.** Navigate into that same folder, click the address bar, type `powershell`, and press Enter. This opens PowerShell in the folder. Run `dir miniarm_controlC.py` to verify it.

**Method C — any existing terminal.** Use cd with the full path copied from File Explorer, in quotation marks, for example:

```powershell
cd "C:\Users\YourName\Documents\Robotics\eel4664-robotic-arm-project-miniArm\lab03_inverse_kinematics\miniarm_controlC"
dir miniarm_controlC.py
```

Use your actual path; `YourName` is an example. Quotes matter when a path contains spaces. The nested `miniarm_controlC\miniarm_controlC.ino` belongs to Arduino IDE; you should not cd into that nested sketch folder to launch Python. Do not type the prompt (`PS C:\...`) as part of a command.

From the verified GUI folder, install dependencies and test offline:

```powershell
python --version
python -m pip install -r requirements.txt
python miniarm_controlC.py --simulate
```

If your Windows Python command is `py`, use `py --version`, `py -m pip install -r requirements.txt` and `py miniarm_controlC.py --simulate` consistently. Using the same interpreter prevents the common "pyserial installed but module missing" issue.

### Select the real COM port before launching hardware mode

Find the Arduino in **Device Manager → Ports (COM & LPT)**, or check Arduino IDE **Tools → Port**. Close Serial Monitor and other GUI instances afterward. For example, if your board is COM6, use either:

**Option 1 — parameter, no file edit:**

```powershell
python miniarm_controlC.py --port COM6
```

**Option 2 — saved Python default:** edit this line near the top of `miniarm_controlC.py` and save:

```python
SERIAL_PORT = "COM6"
```

Then run:

```powershell
python miniarm_controlC.py
```

Replace COM6 with your actual port. An explicit `--port` overrides the file default. Omit `--simulate` for real movement; simulation opens no serial connection. The running GUI also provides an editable port field and Retry connection for recovery.

**After connecting, click Enable servos one by one and wait for initialization to finish before using other robot controls.** Connection/READY alone does not initialize the physical arm. Before a first calibration, the instructor-supervised button reads Enable raw neutral (uncalibrated); after saving/reloading measured calibration, enable again. Simulation initializes automatically and is the exception to the startup-click requirement.

## Install and launch

Keep this folder beside `../python`; install and run with the same Python interpreter:

```console
python -m pip install -r requirements.txt
python miniarm_controlC.py --simulate
```

Simulation opens automatically in IK CONTROL at q=[90,90,0,0] degrees. It opens no serial port and uses synthetic calibration without overwriting your file. To test your measured mapping offline:

```console
python miniarm_controlC.py --simulate --use-calibration
```

For hardware, upload `miniarm_controlC/miniarm_controlC.ino` using Servo and FastLED 3.10.3, close Serial Monitor, then run:

```console
python miniarm_controlC.py --port COM3
```

Use your actual port. Confirm GUI v2.2.1 and Firmware 2.1.0. The simulation emulator may identify its timed-command protocol as 2.0.0; it does not need hardware sequential enable.

The hardware connection opens in a background thread, pulses DTR to reset the board, preserves fragmented serial lines, and waits for FW/READY. Startup retries up to three times, with a ten-second boot timeout per attempt. The editable port field and **Retry connection** let you retry without restarting the GUI. A live disconnection cancels queued commands and requires a retry and fresh initialization; it never replays motion automatically.

## First enable and later upright returns

Connection alone leaves servos off. With a measured calibration, **Enable servos one by one** requests the upright joint reference q=[90,90,0,0] through your own mapping. With no calibration, **Enable raw neutral (uncalibrated)** requests raw 90 on every channel; the physical joint angles are unknown and this is an instructor-supervised calibration step.

Firmware 2.1.0's SINIT command enables **shoulder → elbow → wrist → base → claw**, with one-second pauses. The claw uses its calibrated startup opening. Support the unloaded arm without entering pinch points: each first enable can still move quickly, and unpowered joints may sag. Controls unlock after the final pause/acknowledgement. STOP cancels remaining enables; already-enabled joints hold and others remain off.

After initialization, **Slowly straighten upright** uses the checked coordinated path, preserves APP/IK mode and the claw command, and uses at least five seconds. Larger changes extend the duration to limit the cubic peak command speed to 15 raw units/s. In KNOB mode, choose APP or IK before requesting upright.

The first-enable popup and the old all-at-once INIT workflow are not used for real hardware by this GUI. A firmware without SEQINIT is rejected rather than silently falling back to simultaneous enable. See [motion/protocol details](../SMOOTH_MOTION.md).

## Modes and preview

| Mode | Inputs and behavior |
|---|---|
| APP — green | Base, Shoulder, Elbow, Wrist, Claw sliders; typed raw values; +/- buttons; per-servo and all-servo Send |
| KNOB — orange | Physical knobs control the arm; displays knob/servo values |
| IK — blue | XYZ in mm; Calculate / Preview; Move Arm; error curve and claw control |

Raw sliders are **servo commands, not DH joint angles**. Calibration performs the conversion. APP and KNOB bypass IK collision checks.

- **Solid purple:** current estimated pose, computed from reported servo commands.
- **Dashed blue:** one target pose. There is no second unsent-slider ghost.
- **Red × in IK:** requested XYZ. The dashed executable endpoint includes calibrated command rounding, so its TCP can differ slightly from the red mark.
- In IK, Calculate / Preview sets the dashed snapshot; serial reports move only the solid arm. Editing XYZ/time invalidates Move Arm without moving the retained drawing. Calculate again before executing.
- In APP, editing sliders, +/- or applying typed values updates the dashed preview without sending. Send freezes the endpoint while the solid estimate follows the command sequence. Per-servo Send shows only that channel's intended change; Send all five uses the whole slider pose.
- Switching back to APP synchronizes sliders with the current commands, discarding old unsent edits. Disabled/hidden sliders also receive programmatic updates; deferred Tk callbacks do not create new targets.
- Changing camera or resizing redraws the same world scene. World arrows are frame {0}: X right, Y forward, Z up; they are not all moving DH frames.

These are commanded-position estimates, not encoder measurements. The display cannot detect sag, stall or a physical collision. Claw opening is not modeled as an extra positioning link.

## Calculate, Move and STOP

Calculate uses the shared 3D solver, measured limits and modeled collision clearance. Move sends **one T packet** containing final calibrated targets and duration. Arduino supplies common cubic progress; solver iteration history is not streamed as a trajectory.

The GUI rechecks the start, target, rounded endpoint and direct coordinated path. A rejected path requires a clear intermediate target. Geometry is approximate and does not fully represent the camera, fingers, loads or surroundings.

STOP cancels queued commands, sends the software stop, then waits for STOPPED and a P report. In KNOB it first exits to APP. Servos already enabled retain holding power; STOP is not power removal or a guaranteed instantaneous physical stop. Recalculate an IK plan after stopping.

## Adjustable claw Open / Closed presets

Click **Claw settings...** in the toolbar. Enter the S1 raw values for Open and Closed; either servo direction is allowed, but values must be different integers within your existing calibrated safe range.

- **Apply (session only)** updates presets without moving or writing a file.
- **Test Open / Test Closed** explicitly applies the edited presets and moves only the claw using timed interpolation. Initialize first and select APP or IK. Tests and editing lock during motion.
- **Save to calibration** preserves the joint measurements, angle convention, metadata and safe limits. It writes only the two claw presets, atomically replaces calibration.json and creates calibration.json.before_claw_edit as a backup. Saving sends no motion and does not require reinitialization.
- Synthetic simulation permits session edits/tests but disables persistence, protecting the hardware file. Use saved-calibration mode to edit a real mapping.
- Old IK/Pick & Place plans are invalidated when presets change; preview again. Future claw actions, first-enable opening and Pick & Place use the updated values. Later upright moves still preserve the current claw command.

These are position commands, not gripping-force settings. Adjust in small steps and observe the physical claw. To change safe min/max, use the calibration tool after verifying the usable range; this panel never expands limits. If the calibration editor is also open, reload its file before saving there again.

Focused checks: `python -m unittest test_claw_settings test_pick_place -v`.

## Block-to-cup task

**Move Arm stays a single XYZ move.** Preview Pick & Place plans the complete task without sending commands; Run Pick & Place executes only a successful fresh plan. Defaults match the instructor's cup: center [127,0] mm, base Z=0, rim Z=127 mm, opening diameter 101.6 mm. Cup settings also include measured held-object radius, lift/release clearances and grip pauses.

All stages are prechecked, including a conservative cup envelope. During execution, ACKs and cancellable timers advance the stages. STOP/disconnection cancels the rest without automatically releasing a held block. This remains position-only IK and does not sensor-verify grasp/drop. Read [the complete task guide](../PICK_PLACE.md) before a hardware trial.

Update both `miniarm_controlC.py` and the new `miniarm_pick_place.py` together. Existing firmware 2.1.0 suffices. Focused tests: `python -m unittest test_pick_place -v`.

## Calibration and important files

Read [CALIBRATION_GUIDE.md](../CALIBRATION_GUIDE.md). New files explicitly store `"angle_convention": "joint"` with measured DH angles. Do not copy synthetic data to the real arm or relabel legacy angles without conversion.

| File | Role |
|---|---|
| `miniarm_controlC.py` | Connection, modes, previews and final-target execution |
| `miniarm_pick_place.py` | Full-task planner, cup/object envelope and preflight checks |
| `miniarm_ik.py` | Model/units adapters, calibration conversion and executed-path checks |
| `../python/kinematics.py` | Shared modified-DH transforms, FK, Jacobian and numerical IK |
| `miniarm_calibrate.py` | Measurement-entry form; does not move the arm or open a serial port |
| `miniarm_calibration.py` | Calibration validation and forward/inverse interpolation |
| `calibration.json` | Your measured mapping, raw limits and claw settings |
| `miniarm_controlC/miniarm_controlC.ino` | Servo pulses, sequential enable, local interpolation and serial replies |

JSON names `arm` and `forearm` remain compatible aliases for Shoulder and Elbow. Channels remain S1 claw, S2 wrist, S3 elbow, S4 shoulder, S5 base; GUI order is reversed for readability.

## Troubleshooting

| Symptom | Check |
|---|---|
| Missing serial/numpy | `python -m pip install -r requirements.txt` using the same interpreter |
| Missing kinematics | Restore the sibling `../python` folder |
| Port busy/access denied | Close Serial Monitor and all other controller instances; check Device Manager; Retry connection |
| Port opens but no READY | Check baud 115200, correct miniArm sketch/board, USB cable and power; retry after boot timeout |
| Unknown/older firmware or no SEQINIT | Upload the current nested `.ino`, close Serial Monitor, retry; expect Firmware 2.1.0 |
| Repeated READY/reset messages | Check USB/power stability and board resets; do not start a trial until the connection is stable |
| Move Arm gray | Require initialization, measured calibration, IK mode, timed firmware and a fresh successful checked preview |
| Upright bends backward | Verify actual joint zeros/directions against the file; read the logged Upright calibration conversion |
| Works in synthetic simulation only | Saved limits/mapping can differ; inspect calibration, geometry and start pose |
| Tiny residual but poor physical accuracy | Check actual XYZ origin, measured link lengths, calibration, backlash and load |

For optional software checks, run `python -m unittest discover -s ../python -v` from this folder. GUI tests require desktop Tk and must match the current interface; historical test records are in [maintenance](../docs/maintenance/GUI_VALIDATION.md), not additional student work. [Firmware host checks](qa/README.md) do not replace an Arduino build or physical test.

`patch_preview_stability.py` is a legacy one-time updater for older customized GUIs. **Do not run it on this current controller**; the preview changes are already integrated.
