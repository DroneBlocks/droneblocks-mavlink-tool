# DroneBlocks MAVLink Tool

A lightweight web-based tool for PX4 flight controllers. View and edit parameters, access the MAVLink console, and inspect live message traffic.

## Requirements

- Python 3.10+
- PX4 flight controller connected via USB

## Setup

```bash
# Create virtual environment
python3 -m venv venv

# Activate it
source venv/bin/activate  # macOS/Linux
# or: venv\Scripts\activate  # Windows

# Install dependencies
pip install -r requirements.txt
```

## Usage

```bash
# Activate virtual environment (if not already)
source venv/bin/activate

# Run the server
python main.py
```

Open http://localhost:8000 in your browser.

## Features

- **Parameters**: View and edit flight controller parameters
- **Console**: MAVLink shell access (run commands like `mavlink status`, `listener sensor_optical_flow 1`)
- **Inspector**: Live view of all MAVLink messages with Hz rates

## Troubleshooting

**"No serial port found"**: Make sure your flight controller is connected via USB and powered on.

**Console not responding**: Switch to another tab and back, or refresh the page. Only one feature can use the serial port at a time.

**Parameters not loading completely**: Stay on the Parameters tab while loading. The Inspector runs in the background and can interfere.
