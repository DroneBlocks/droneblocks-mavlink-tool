# Flashing DEXI-3 Flight Controllers

Take a **DroneBlocks H743-AIO** (the DEXI-3 FC, board_id 1240) from bare/foreign
to a fully-provisioned DEXI-3 — bootloader + PX4 firmware + indoor-flow params —
over USB from your Mac. No QGroundControl, no web configurator.

> **Internal tool.** Built and used by [DroneBlocks](https://droneblocks.io) for
> production flashing runs. Runs on **macOS, Windows, and Linux** — the FC's USB
> serial port is auto-detected on each (see [Running on Windows](#running-on-windows-pc)
> for the extra one-time Windows setup).

## What it does

Three stages, run for you by one command:

| # | Stage | Tool |
|---|---|---|
| 1 | **Bootloader** (PX4 ROM loader) | `dfu-util` |
| 2 | **App firmware** (DroneBlocks PX4 `v1.16.2`) | `px_uploader.py` |
| 3 | **Params** (DEXI-3 indoor optical flow, 37 params) | `provision_dexi3_flow.py` |

Two entry points:
- **`flash_new_fc.py`** — one board, walks you through it.
- **`flash_batch.py`** — back-to-back production runs (e.g. 10 drones), zero
  keypresses per board. Auto-detects each board and does the right thing.

The pinned firmware in `firmware/droneblocks-h743-aio/` is committed to the repo,
so a fresh clone is ready to flash — no separate download.

---

## 1. Clone

```bash
git clone https://github.com/DroneBlocks/droneblocks-mavlink-tool.git
cd droneblocks-mavlink-tool
```

## 2. Install

You need **two** things: the Python deps **and** `dfu-util` (a native tool —
*not* a pip package, easy to forget).

```bash
# Python deps — pick one:
uv venv && uv pip install -r requirements.txt      # uv
#   …or…
python3 -m venv venv && ./venv/bin/pip install -r requirements.txt   # pip

# dfu-util (one-time, Homebrew) — needed for the bootloader stage:
brew install dfu-util
```

> All commands below assume the repo venv: `./venv/bin/python` (or just `python`
> with the venv/uv env activated). **On Windows** the venv Python is
> `venv\Scripts\python` — see [Running on Windows](#running-on-windows-pc) first.

---

## Running on Windows (PC)

The flasher runs on Windows, but needs three one-time setup steps macOS doesn't.
The FC's COM port is auto-detected (by USB vendor ID, so it ignores Bluetooth and
other COM ports) — you never pass a port number.

**1. Python venv (note the `Scripts` path):**

```powershell
py -m venv venv
venv\Scripts\pip install -r requirements.txt
```

Then run every command below as `venv\Scripts\python flash_batch.py …` (instead
of `./venv/bin/python …`).

**2. `dfu-util` (needed for the bootloader stage on bare boards):**

```powershell
winget install dfu-util      # …or: choco install dfu-util
```

Confirm it's on `PATH`: `dfu-util -l` should run. If "not recognized", add its
install dir to PATH (or reopen the terminal).

**3. Zadig — WinUSB driver for the DFU device (the #1 Windows gotcha).**
On Windows a bare board in DFU (`hold BOOT + plug`) enumerates as `STM32
BOOTLOADER` (USB `0483:df11`), but `dfu-util` can't talk to it until the
**WinUSB** (or libusbK) driver is bound to it:

1. Download **[Zadig](https://zadig.akeo.ie/)**.
2. Put a board in DFU (hold BOOT while plugging in USB).
3. In Zadig: **Options → List All Devices**, select **STM32 BOOTLOADER**
   (`0483 DF11`), choose **WinUSB**, click **Replace/Install Driver**. One-time
   per PC. `dfu-util -l` should now list `0483:df11`.

> Already-flashed DroneBlocks boards (plugged in **normally**, no BOOT) don't use
> DFU — they re-flash through their PX4 bootloader over the COM port, so Zadig
> isn't needed for a `--params-only` or re-flash run. Zadig is only for bringing
> up a **bare/foreign** board.

**Flash — same commands, Windows Python:**

```powershell
venv\Scripts\python flash_new_fc.py                 # one board
venv\Scripts\python flash_batch.py --count 10       # a batch
venv\Scripts\python flash_batch.py --params-only    # already-flashed → params only
```

> **Tip:** during a batch run, unplug other USB-serial gadgets (Arduinos, adapters).
> The FC is matched by vendor ID first, but a dedicated flashing session is cleanest.
> Sanity-check what Windows sees: `venv\Scripts\python -c "import serial_ports; print(serial_ports.fc_ports())"`.

## 3. Plug in the board

- **Bare / foreign board** → enter **DFU**: hold the **BOOT** button while plugging
  in USB, hold ~2 s, release. (Confirm with `dfu-util -l` → `0483:df11`.)
- **Board that already runs DroneBlocks PX4** → just plug it in normally. No BOOT
  button — `px_uploader` reboots it into its own bootloader to re-flash.

No battery needed: USB 5 V powers the FC (and the UP-T201 flow sensor) for flashing.

## 4. Flash

**One board:**

```bash
./venv/bin/python flash_new_fc.py
```

**A batch (e.g. 10 drones)** — plug → wait for the ✅ → unplug → next:

```bash
./venv/bin/python flash_batch.py --count 10
```

`flash_batch.py` auto-detects each board:

| You plug in… | Detected | Action | with `--params-only` |
|---|---|---|---|
| Bare board **in DFU** (held BOOT) | `dfu` | bootloader + app + params | full flash (no app to skip to yet) |
| Already-flashed, **plugged normally** | `app` | re-flash same app + params | **params only** |
| Bootloader-only (interrupted board) | `usb` | app + params | app + params |

Default re-flashes the firmware on every board so the whole fleet is byte-identical.
Add `--params-only` to skip the firmware on already-flashed boards (faster, but
trusts the existing version). Drop `--count` to run until you press Ctrl-C.

Each board prints a **PROVISION PASS** when its params verify.

---

## 5. After flashing — calibrate (per board, before flight)

The flash + params are identical across boards, but **sensor calibration is
per-board** and can't be cloned. Before a board flies, in
[px4-web-configurator](https://px4-configurator.web.app) `/calibrate`:

- **Gyro**, **Level horizon**, **6-position accel**
- **Skip magnetometer** — DEXI-3 runs `EKF2_MAG_TYPE=5` (mag off, no onboard compass)

Audit a unit any time without re-flashing:

```bash
./venv/bin/python provision_dexi3_flow.py --verify-only   # all params OK?
./venv/bin/python watch_flow.py /dev/cu.usbmodem01        # live OPTICAL_FLOW_RAD (quality ~245/255 over texture)
```

---

## What version am I flashing?

`firmware/droneblocks-h743-aio/manifest.json` is the source of truth — the batch
tool prints it on startup and on every PASS line. Currently:

- **Board:** `droneblocks_h743-aio`, board_id **1240**
- **Firmware:** **`v1.16.2-3-g39e04ffe23`** (the manifest is authoritative; the tool
  prints the live version at runtime, so trust that over this line)

> **Why 1.16.2 and not 1.17:** DEXI-3 ships on the PX4 1.16 line so the FC's
> uXRCE-DDS message versions match the companion `px4_msgs 1.16` the rest of
> DEXI-OS uses. A 1.17 FC against a 1.16 companion registers **zero** ROS 2 topics.

Pull a newer cloud build into `firmware/` with `./firmware/fetch-latest.sh`.

---

## Troubleshooting

For the full hard-won list (DFU glitches, the benign `get_status` error on
`:leave`, nsh-vs-MAVLink silence, why to avoid the web flasher), see
**[`firmware/README.md`](firmware/README.md)**. The quick ones:

- **`dfu-util -l` shows nothing** → board isn't in DFU; hold **BOOT** *while*
  plugging in.
- **`dfu-util: Error during download get_status`** → **benign** if
  `File downloaded successfully` printed just above; the write succeeded.
- **px_uploader stuck at "Waiting for bootloader"** → unplug/replug USB to catch
  the bootloader window.
- **No `/dev/cu.usbmodem*` after flashing** → one USB power-cycle (unplug ~5 s,
  replug); the MAVLink-first tools here will grab the port correctly.
