# Flashing a DroneBlocks H743-AIO (DEXI-3 FC) from scratch — over USB

Take a **bare / foreign-firmware** H743-AIO board to a fully-working DEXI-3
flight controller, entirely over USB from a Mac. No web configurator.

Three stages:

| # | Stage | Tool |
|---|---|---|
| 1 | **Bootloader** (PX4 ROM loader) | `dfu-util` (STM32 DFU) |
| 2 | **App firmware** (DroneBlocks PX4 branch) | `px_uploader.py` |
| 3 | **Parameters** (DEXI-3 indoor optical flow) | `../provision_dexi3_flow.py` |

> If the board **already runs PX4** (boots, shows up as a MAVLink serial device),
> skip Stage 1 — you only need Stages 2–3.

---

## Path of least resistance — one command

After a one-time `brew install dfu-util`, just run the orchestrator and follow its
single prompt (hold BOOT, plug in). It does all three stages back-to-back:

```bash
cd ~/_dev/droneblocks-mavlink-tool
./venv/bin/python flash_new_fc.py
```

The manual stages below are the same thing broken out, for when you need to debug
a step. **Verified live 2026-06-11:** after the DFU flash the board lands in the
fresh bootloader and px_uploader syncs **immediately (no replug)**, and the app
**boots clean (no power-cycle)** — the only thing you physically touch is the
BOOT button.

---

## Pinned assets (this folder)

Built by DroneBlocks/PX4-Autopilot's cloud GHA. Pinned here so flashes are
reproducible; pull newer builds with `./fetch-latest.sh`.

| File | What |
|---|---|
| `droneblocks-h743-aio/droneblocks_h743-aio_bootloader.bin` | PX4 bootloader (flash @ `0x08000000`) |
| `droneblocks-h743-aio/droneblocks_h743-aio_default.px4` | App firmware — **v1.17.0-13-g2e6f68d5a8**, board_id 1240 |
| `droneblocks-h743-aio/manifest.json` | version / git_sha / source URLs |
| `px_uploader.py` | PX4 app flasher (from PX4-Autopilot/Tools) |

Source of truth (latest): `https://pub-a9128812de294697bc4f590727d409c8.r2.dev/droneblocks_h743-aio/latest/manifest.json`

---

## One-time setup (Mac)

```bash
brew install dfu-util
```

The Python tools use this repo's venv (pymavlink + pyserial already installed):
`~/_dev/droneblocks-mavlink-tool/venv/bin/python`.

---

## Stage 1 — Flash the bootloader (DFU)

A fresh / MicoAir-stock board needs the PX4 bootloader installed via the STM32
ROM DFU loader.

1. **Enter DFU:** hold the board's **BOOT** button while plugging in USB (keeps
   BOOT0 high so the chip stays in the ROM loader). Release once it enumerates.
2. Confirm it's in DFU — should list `0483:df11 ... STM32 BOOTLOADER`:
   ```bash
   dfu-util -l
   ```
3. Flash the bootloader to internal flash and leave DFU:
   ```bash
   cd ~/_dev/droneblocks-mavlink-tool/firmware
   dfu-util -a 0 -d 0483:df11 -s 0x08000000:leave \
     -D droneblocks-h743-aio/droneblocks_h743-aio_bootloader.bin
   ```
4. **Power-cycle** (unplug/replug USB). The board now runs the PX4 bootloader.

> ⚠️ Confirm the BOOT-button location for your H743-AIO. If `dfu-util -l` shows
> nothing, the board isn't in DFU — redo step 1.

---

## Stage 2 — Flash the app firmware (DroneBlocks PX4 branch)

```bash
cd ~/_dev/droneblocks-mavlink-tool/firmware
~/_dev/droneblocks-mavlink-tool/venv/bin/python px_uploader.py \
  --port "/dev/cu.usbmodem01" \
  droneblocks-h743-aio/droneblocks_h743-aio_default.px4
```

- **Coming straight from Stage 1 it syncs immediately** — `Found board id: 1240,0`
  → Erase → Program → Verify → `Rebooting`. No replug needed.
- *Only if* it sits at **"Waiting for bootloader…"** (e.g. the board had been idle
  a while) → unplug/replug USB to catch the bootloader window. It will not flash a
  running app.
- The app normally boots clean. *Only if* it comes up silent (no MAVLink) → one
  power-cycle (unplug ~5 s, replug).

Port is usually `/dev/cu.usbmodem01` — check with `ls /dev/cu.usbmodem*`.

---

## Stage 3 — Provision DEXI-3 indoor optical-flow params

```bash
cd ~/_dev/droneblocks-mavlink-tool
./venv/bin/python provision_dexi3_flow.py
```

Connects MAVLink-first, sets the DEXI-3 airframe (`SYS_AUTOSTART=4701`) + the full
indoor-flow param set, force-saves, and verifies every value. Prints **PASS/FAIL**.
`--watch` for assembly-line mass updates, `--verify-only` to audit a unit.

---

## Verify

```bash
cd ~/_dev/droneblocks-mavlink-tool
./venv/bin/python provision_dexi3_flow.py --verify-only   # all 16 params OK?
./venv/bin/python watch_flow.py /dev/cu.usbmodem01        # OPTICAL_FLOW_RAD rate/quality/distance
```

Good flow over a **textured surface at ~0.3–0.8 m** reads quality ~245/255.

---

## Troubleshooting (hard-won)

- **`dfu-util -l` empty** → board isn't in DFU; hold **BOOT** *while* plugging in.
- **dfu-util ends with `Error during download get_status`** → **benign** STM32-DFU
  quirk on the `:leave` step (device detaches before status read). The write
  succeeded — `File downloaded successfully` prints just above, and the board
  re-enumerates as the PX4 bootloader (`0x3162:0x004b`, serial 0).
- **px_uploader stuck at "Waiting for bootloader"** → unplug/replug USB so it
  catches the bootloader window. It cannot flash a running app.
- **"connected but no data" / SYS 0 / total silence after a reboot** → PX4's USB
  console latched into **nsh mode** (it picks nsh vs MAVLink by whoever opens the
  port first). The board is USB-powered, so **unplug/replug = a real power
  cycle**; then let the MAVLink-first tools here open it first. The web
  configurator hits this constantly; these CLI tools don't.
- **Avoid the web configurator's flasher** — its "port is already open" bug can
  leave the board sitting in the bootloader with no app (how a board got bricked
  mid-flash once; recovered with the Stage-2 steps above).
- **Use these pinned assets**, not a local `PX4-Autopilot/build/...` — that tree
  may be on an older/different branch (we caught a v1.16.2-vs-v1.17.0 mismatch).
- **No battery needed** for flashing or for confirming flow data — the FC + UP-T201
  run off USB 5 V. (Battery is only needed to actually fly.)
