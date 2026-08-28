# DroneBlocks MAVLink Tool

Tools for managing PX4 flight controllers — get/set parameters over the network, access the MAVLink console, and inspect live message traffic.

> **Flashing a fresh DEXI-3 FC** (bootloader + PX4 + params over USB, single board or a 10-drone batch)? See **[FLASHING.md](FLASHING.md)**.

> **Testing an optical flow module?** The soak test parks a stationary aircraft over
> the floor and records flow quality, which separates a faulty module from a bad
> surface. See **[docs/flow-soak-test.md](docs/flow-soak-test.md)**.

## Requirements

- Python 3.10+
- PX4 flight controller connected via USB or accessible via mavlink-router over the network

## Setup

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## Remote Parameter Management (params.py)

Get and set PX4 parameters over WiFi — no USB connection needed. Connects to the flight controller through mavlink-router running on the Raspberry Pi.

```bash
source venv/bin/activate

# List all parameters
python params.py 192.168.68.60

# Filter by prefix
python params.py 192.168.68.60 -f EKF2

# Get a single parameter
python params.py 192.168.68.60 EKF2_HGT_REF

# Set a parameter
python params.py 192.168.68.60 EKF2_HGT_REF 2
```

### Common parameter workflows

**Enable AprilTag visual odometry:**
```bash
python params.py 192.168.68.60 EKF2_EV_CTRL 1
```

**Check all EKF2 sensor fusion settings:**
```bash
python params.py 192.168.68.60 -f EKF2
```

**Indoor optical flow setup (all at once):**
```bash
python params.py 192.168.68.60 EKF2_HGT_REF 2
python params.py 192.168.68.60 EKF2_RNG_CTRL 2
python params.py 192.168.68.60 EKF2_OF_CTRL 1
python params.py 192.168.68.60 EKF2_OF_QMIN 50
python params.py 192.168.68.60 EKF2_GPS_CTRL 0
python params.py 192.168.68.60 EKF2_EV_CTRL 1
python params.py 192.168.68.60 COM_ARM_WO_GPS 1
```

### How it works

The tool connects via `udpout:<IP>:14550` to the Pi's mavlink-router, sends a GCS heartbeat to register, then uses the MAVLink parameter protocol to read/write params on the flight controller.

### mavlink-router configuration

The Pi needs mavlink-router with a UDP Server endpoint. Example `/etc/mavlink-router/main.conf`:

```ini
[General]

[UartEndpoint FCUSB]
Device = /dev/serial/by-id/usb-ARK_ARK_Pi6X.x_0-if00
Baud = 2000000

[UdpEndpoint GCS]
Mode = Server
Address = 0.0.0.0
Port = 14550
```

With Server mode, the client must send a packet (e.g. a heartbeat) before mavlink-router will route traffic back to it.

## Web UI (main.py)

Web-based interface for when the FC is connected via USB directly to your machine.

```bash
source venv/bin/activate
python main.py
```

Open http://localhost:8000 in your browser.

Features:
- **Parameters**: View and edit flight controller parameters
- **Console**: MAVLink shell access (run commands like `mavlink status`, `listener sensor_optical_flow 1`)
- **Inspector**: Live view of all MAVLink messages with Hz rates
- **Firmware Upload**: Flash `.px4` firmware files

## Legacy: fetch_params.py

Read-only parameter fetcher. Superseded by `params.py` which supports both get and set.

```bash
python fetch_params.py 192.168.68.60
```

## Troubleshooting

**"No heartbeat"**: Make sure mavlink-router is running on the Pi (`systemctl status mavlink-router`) and the GCS endpoint is in Server mode on port 14550.

**"Parameter not found"**: Check the exact parameter name — PX4 param names are case-sensitive and up to 16 characters.

**"No serial port found" (Web UI)**: Make sure your flight controller is connected via USB and powered on.

**Console not responding (Web UI)**: Switch to another tab and back, or refresh. Only one feature can use the serial port at a time.

**SSH timeout to Pi**: The Pi's IP may have changed via DHCP. Check your router's lease table or scan with `nmap -sn 192.168.68.0/24`.
