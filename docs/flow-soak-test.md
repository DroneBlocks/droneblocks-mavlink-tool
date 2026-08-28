# Optical flow soak test

Answers one question: **does the optical flow module drop out on its own?**

The aircraft sits still on a box for ten minutes. Because nothing moves, the floor,
the airframe's motion, the position controller and the state estimator are all out of
the loop. The only thing left that can vary is the sensor.

- **Any sustained dropout means the module is at fault.** Replace it.
- **A clean run means the module is sound**, and in-flight failures are coming from
  the surface it is looking at, not the hardware.

That distinction is hard to make from flight logs, because in the air everything moves
at once. This test is the cheapest way to separate the two, and it runs unattended.

## Before you start

- **Take the props off.** The motors never spin during this test, but the aircraft is
  powered and armed states can change. Props off is not optional.
- **Battery connected and USB connected.** The flow module is powered from the flight
  controller's 5V rail, so USB alone may not bring it up.
- **Park it about 1.2 m above the floor**, looking down, on a box, a stool or a shelf.
  Wedge it so it cannot shift. Anything that moves during the run invalidates the
  result, and the tool reports how far the height wandered so you can check.
- **Pick a patch of floor the sensor reads well.** This test is about the sensor, so
  do not deliberately park it over a spot you already know is bad.

## Running it

### Windows

```
cd C:\Users\denni\droneblocks-mavlink-tool
git pull
venv\Scripts\python flow_soak.py --minutes 10 --label cf
```

If the repo is not on this PC yet, run this once in PowerShell, then use the commands
above:

```powershell
irm https://raw.githubusercontent.com/DroneBlocks/droneblocks-mavlink-tool/main/windows/setup-windows.ps1 | iex
```

No Zadig step is needed. That is only for bare boards in DFU mode. This test talks to
an already-flashed flight controller over its normal COM port.

### macOS and Linux

```bash
cd ~/_dev/droneblocks-mavlink-tool
git pull
./venv/bin/python flow_soak.py --minutes 10 --label cf
```

### Options

| flag | meaning |
|---|---|
| `--minutes N` | run length, default 10 |
| `--label NAME` | tag for this run, used in the saved file name |
| `--out PATH` | where to write the JSON, default `soak_<label>.json` |

Use `--label` to keep aircraft apart, for example `--label cf` and `--label devkit`,
so two runs do not overwrite each other.

## Reading the output

It prints a line a minute so you can watch it, then a summary.

```
#  min      n   q245%   q122%    q0%     h m   degC
     1   2401   100.0%    0.0%   0.0%    1.21   34.2
     2   2398   100.0%    0.0%   0.0%    1.21   35.1
```

### What the quality numbers mean

The UP-T201 does not report a quality *level*. It reports a **binary valid flag**:
**245 means it has a fix, 0 means it does not.** There is nothing in between and there
is no early warning.

PX4 publishes at half the module's rate and averages two consecutive readings, so what
you see is one of three values:

| value | meaning |
|---|---|
| **245** | both underlying readings were good |
| **122** | one of the two was blind |
| **0** | both were blind |

The summary combines these into a **true sensor-level blind rate**, which is
`%(q=0) + 0.5 x %(q=122)`. That figure, not the raw `q0%`, is the number to judge the
module by.

### The verdict line

```
  TRUE sensor-level blind: 0.00%
  VERDICT: CLEAN, the sensor is not the fault
```

Anything above 0.05% flips the verdict. The summary also lists every dropout with a
timestamp and duration, so a single 40 ms glitch at minute 7 is visible rather than
being averaged away.

### The two sanity checks

The summary ends with two lines worth reading before you trust the result:

- **Height held x to y m.** If the spread is more than a couple of centimetres, the
  aircraft moved and the run is not a valid soak. Wedge it better and repeat.
- **Temperature start to end.** The module draws up to 1.5 W in a 25 mm package, so it
  warms up. If dropouts begin partway through and the temperature is still climbing,
  that is a thermal fault rather than a random one, which is a different repair.

## Comparing two aircraft

Run the same test on a known-good aircraft and a suspect one, over the same patch of
floor, on the same day.

```
venv\Scripts\python flow_soak.py --minutes 10 --label good
venv\Scripts\python flow_soak.py --minutes 10 --label suspect
```

Two clean runs mean both modules are fine and the problem is elsewhere. One clean and
one not is a confirmed bad module, and the saved JSON files are the evidence.

## Troubleshooting

**`no flight controller found`**. The board is not enumerating. Check USB, check the
battery is in, and try another cable. Port detection filters by USB vendor ID, so a
Bluetooth serial port will not be picked up by mistake.

**`no OPTICAL_FLOW_RAD received: is the module powered?`**. The flight controller is
talking but the flow module is not. It is powered from the FC's 5V rail and speaks
MAVLink over TELEM3 at 115200. Check the cable to the module and confirm `MAV_2_CONFIG`
is 103 and `SER_TEL3_BAUD` is 115200.

**Height reads a flat 0.05 m.** That is the rangefinder's minimum. Either the aircraft
is sitting on the bench rather than propped up, or the laser is not reading. Lift it and
confirm the height number follows.

**Rate is far below 40 Hz.** Something else is competing for the serial link. Close any
other tool that is connected to the same board.
