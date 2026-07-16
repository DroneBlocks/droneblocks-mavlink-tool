# One-time Windows setup: Zadig (WinUSB driver for DFU)

Flashing a **bare/blank** DEXI-3 flight controller on Windows needs a one-time
driver step. When a board is held in **DFU** (bootloader) mode it enumerates as
`STM32 BOOTLOADER` (USB `0483:df11`), and Windows won't let `dfu-util` talk to it
until the **WinUSB** driver is bound to that device.

- **One time per PC.** All DEXI-3 boards share the same USB ID, so one binding
  covers every board. It survives reboots and unplugs.
- **Only bare boards need it.** Already-flashed boards re-flash over their COM
  port (no DFU), so `--params-only` / app re-flash don't need Zadig.
- **Can't be scripted remotely** — it needs a board physically in DFU plus an
  admin (UAC) prompt.

A ready-to-hand plain-text version of these steps ships in
[`windows/READ-ME-FIRST-Zadig-Setup.txt`](../windows/READ-ME-FIRST-Zadig-Setup.txt)
(copy it, `zadig.exe`, and the launchers to the flashing PC's Desktop).

## Steps

1. **Put the board in DFU.** Unplug it, hold the **BOOT** button, plug USB in
   while holding, keep ~2 s, release.
2. **Run Zadig as administrator.** Get it from the libwdi releases
   (<https://github.com/pbatard/libwdi/releases> → `zadig-*.exe`; the
   `zadig.akeo.ie/downloads/...` hotlink 404s).
3. **Options → List All Devices** (so it's checked).
4. In the dropdown, select **`STM32 BOOTLOADER`** (`0483 DF11`). If it's not
   listed, the board isn't in DFU — redo step 1.
5. Set the target driver to **WinUSB**, then click **Install Driver** /
   **Replace Driver**. Wait for "installed successfully."

## Verify

With the board still in DFU, run the flasher (`Flash-DEXI-debug.cmd`, or
`flash_batch.py --count 1 --verbose`) — it should flash bootloader + PX4 +
params and finish with a green `DONE`. Or just confirm the driver took:

```
dfu-util -l      # should list 0483:df11
```

## Desktop launchers

`windows/Flash-DEXI.cmd` (batch loop) and `windows/Flash-DEXI-debug.cmd` (one
board, verbose, keeps the window open) are the double-click entry points copied
to the Desktop. They `cd` into the repo, `git pull`, prepend `%USERPROFILE%\bin`
to `PATH` (so `dfu-util` is found even when Explorer's PATH is stale after a
fresh install), then run the flasher.
