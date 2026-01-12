#!/usr/bin/env python3
"""
DroneBlocks MAVLink Tool
A lightweight web-based tool for PX4 parameter management and console access.
"""

import asyncio
import glob
import math
import struct
import sys
import threading
import time
from contextlib import asynccontextmanager
from typing import Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from pymavlink import mavutil

# MAV_PARAM_TYPE enum values
MAV_PARAM_TYPE_UINT8 = 1
MAV_PARAM_TYPE_INT8 = 2
MAV_PARAM_TYPE_UINT16 = 3
MAV_PARAM_TYPE_INT16 = 4
MAV_PARAM_TYPE_UINT32 = 5
MAV_PARAM_TYPE_INT32 = 6
MAV_PARAM_TYPE_REAL32 = 9


def decode_param_value(float_value: float, param_type: int):
    """Decode MAVLink parameter value based on its type.

    MAVLink transmits all params as floats, but integer types need
    the float bytes reinterpreted as integers.
    """
    if param_type == MAV_PARAM_TYPE_REAL32:
        return float_value

    # Pack float to bytes, then unpack as appropriate integer type
    float_bytes = struct.pack('f', float_value)

    if param_type == MAV_PARAM_TYPE_UINT8:
        return struct.unpack('I', float_bytes)[0] & 0xFF
    elif param_type == MAV_PARAM_TYPE_INT8:
        val = struct.unpack('I', float_bytes)[0] & 0xFF
        return val if val < 128 else val - 256
    elif param_type == MAV_PARAM_TYPE_UINT16:
        return struct.unpack('I', float_bytes)[0] & 0xFFFF
    elif param_type == MAV_PARAM_TYPE_INT16:
        val = struct.unpack('I', float_bytes)[0] & 0xFFFF
        return val if val < 32768 else val - 65536
    elif param_type == MAV_PARAM_TYPE_UINT32:
        return struct.unpack('I', float_bytes)[0]
    elif param_type == MAV_PARAM_TYPE_INT32:
        return struct.unpack('i', float_bytes)[0]
    else:
        return float_value


# MAVLink connection state
mavlink_connection: Optional[mavutil.mavlink_connection] = None
parameters: dict = {}
param_count: int = 0
params_loaded: bool = False
connection_lock = threading.Lock()

# Message inspector state
message_stats: dict = {}  # {msg_type: {count, last_time, hz, last_msg}}
message_stats_lock = threading.Lock()
inspector_running = False
inspector_thread: Optional[threading.Thread] = None


def find_serial_port() -> Optional[str]:
    """Auto-detect PX4 USB serial port."""
    patterns = [
        "/dev/ttyACM*",       # Linux
        "/dev/ttyUSB*",       # Linux USB-serial
        "/dev/cu.usbmodem*",  # macOS
        "/dev/cu.usbserial*", # macOS USB-serial
    ]
    for pattern in patterns:
        ports = glob.glob(pattern)
        if ports:
            return sorted(ports)[0]
    return None


def connect_mavlink(port: Optional[str] = None) -> bool:
    """Establish MAVLink connection."""
    global mavlink_connection, parameters, param_count, params_loaded

    with connection_lock:
        if mavlink_connection is not None:
            return True

        if port is None:
            port = find_serial_port()

        if port is None:
            print("No serial port found")
            return False

        try:
            print(f"Connecting to {port}...")
            mavlink_connection = mavutil.mavlink_connection(port, baud=115200)
            mavlink_connection.wait_heartbeat(timeout=10)
            print(f"Connected! System {mavlink_connection.target_system}, Component {mavlink_connection.target_component}")

            # Reset parameter state
            parameters = {}
            param_count = 0
            params_loaded = False

            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            mavlink_connection = None
            return False


def disconnect_mavlink():
    """Close MAVLink connection."""
    global mavlink_connection, parameters, params_loaded

    with connection_lock:
        if mavlink_connection is not None:
            mavlink_connection.close()
            mavlink_connection = None
            parameters = {}
            params_loaded = False
            print("Disconnected")


def request_parameters():
    """Request all parameters from the flight controller."""
    global param_count, params_loaded

    if mavlink_connection is None:
        return False

    params_loaded = False
    parameters.clear()

    mavlink_connection.mav.param_request_list_send(
        mavlink_connection.target_system,
        mavlink_connection.target_component
    )
    return True


def receive_parameters(timeout: float = 60.0) -> bool:
    """Receive parameters until complete or timeout."""
    global param_count, params_loaded

    if mavlink_connection is None:
        return False

    start_time = time.time()
    last_msg_time = start_time

    while time.time() - start_time < timeout:
        msg = mavlink_connection.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.0)

        if msg:
            last_msg_time = time.time()
            param_id = msg.param_id
            if isinstance(param_id, bytes):
                param_id = param_id.decode('utf-8').rstrip('\x00')

            parameters[param_id] = {
                'value': decode_param_value(msg.param_value, msg.param_type),
                'type': msg.param_type,
                'index': msg.param_index
            }
            param_count = msg.param_count

            if len(parameters) >= param_count:
                params_loaded = True
                print(f"Received all {param_count} parameters")
                return True

        # If no message for 5 seconds and we have some params, consider done
        if time.time() - last_msg_time > 5.0 and len(parameters) > 0:
            params_loaded = True
            print(f"Received {len(parameters)}/{param_count} parameters (timeout)")
            return True

    return False


def encode_param_value(value: float, param_type: int) -> float:
    """Encode a value for MAVLink parameter setting.

    For integer types, we need to pack the integer bytes and interpret as float.
    """
    if param_type == MAV_PARAM_TYPE_REAL32:
        return float(value)

    # Convert to integer, pack as uint32, unpack as float
    int_val = int(value)

    if param_type in (MAV_PARAM_TYPE_UINT8, MAV_PARAM_TYPE_UINT16, MAV_PARAM_TYPE_UINT32):
        int_bytes = struct.pack('I', int_val & 0xFFFFFFFF)
    elif param_type in (MAV_PARAM_TYPE_INT8, MAV_PARAM_TYPE_INT16, MAV_PARAM_TYPE_INT32):
        int_bytes = struct.pack('i', int_val)
    else:
        return float(value)

    return struct.unpack('f', int_bytes)[0]


def set_parameter(name: str, value: float) -> bool:
    """Set a parameter value."""
    if mavlink_connection is None:
        return False

    # Get current param info for type
    param_info = parameters.get(name)
    param_type = param_info['type'] if param_info else MAV_PARAM_TYPE_REAL32

    # Encode the value for MAVLink transmission
    encoded_value = encode_param_value(value, param_type)

    # Encode parameter name
    name_bytes = name.encode('utf-8')
    name_padded = name_bytes.ljust(16, b'\x00')[:16]

    mavlink_connection.mav.param_set_send(
        mavlink_connection.target_system,
        mavlink_connection.target_component,
        name_padded,
        encoded_value,
        param_type
    )

    # Wait for confirmation
    start_time = time.time()
    while time.time() - start_time < 5.0:
        msg = mavlink_connection.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.0)
        if msg:
            msg_id = msg.param_id
            if isinstance(msg_id, bytes):
                msg_id = msg_id.decode('utf-8').rstrip('\x00')

            if msg_id == name:
                parameters[name] = {
                    'value': decode_param_value(msg.param_value, msg.param_type),
                    'type': msg.param_type,
                    'index': msg.param_index
                }
                return True

    return False


def send_shell_command(command: str) -> bool:
    """Send a command to the MAVLink shell."""
    if mavlink_connection is None:
        return False

    # Add newline if not present
    if not command.endswith('\n'):
        command += '\n'

    data = command.encode('utf-8')

    # SERIAL_CONTROL_DEV_SHELL = 10
    # SERIAL_CONTROL_FLAG_REPLY = 1 (set in responses)
    # SERIAL_CONTROL_FLAG_RESPOND = 2 (request a response)
    # SERIAL_CONTROL_FLAG_EXCLUSIVE = 4 (exclusive access)
    # SERIAL_CONTROL_FLAG_BLOCKING = 8
    # SERIAL_CONTROL_FLAG_MULTI = 16 (multiple responses until drained)
    flags = 2 | 16  # RESPOND | MULTI - request response with multiple parts

    # Pad data to 70 bytes (max for SERIAL_CONTROL)
    data_padded = list(data[:70]) + [0] * (70 - min(len(data), 70))

    # Ensure we're targeting the autopilot
    mavlink_connection.target_component = 1

    mavlink_connection.mav.serial_control_send(
        10,     # device: SHELL
        flags,  # flags
        0,      # timeout
        0,      # baudrate
        len(data),
        data_padded
    )
    return True


def receive_shell_output(timeout: float = 0.5) -> str:
    """Receive shell output."""
    if mavlink_connection is None:
        return ""

    output = ""
    start_time = time.time()

    while time.time() - start_time < timeout:
        msg = mavlink_connection.recv_match(type='SERIAL_CONTROL', blocking=True, timeout=0.1)
        if msg and msg.count > 0:
            data = bytes(msg.data[:msg.count])
            try:
                output += data.decode('utf-8', errors='replace')
            except:
                pass
            start_time = time.time()  # Reset timeout on data received

    return output


def start_inspector():
    """Start the background message inspector thread."""
    global inspector_running, inspector_thread, message_stats

    if inspector_running:
        return

    with message_stats_lock:
        message_stats.clear()

    inspector_running = True
    inspector_thread = threading.Thread(target=inspector_loop, daemon=True)
    inspector_thread.start()
    print("Inspector: started")


def stop_inspector():
    """Stop the background message inspector thread."""
    global inspector_running, inspector_thread

    inspector_running = False
    if inspector_thread is not None:
        inspector_thread.join(timeout=3.0)
        inspector_thread = None
    # Small delay to ensure serial port is fully released
    time.sleep(0.2)
    print("Inspector: stopped")


def inspector_loop():
    """Background loop that receives all MAVLink messages and tracks stats."""
    global message_stats

    while inspector_running and mavlink_connection is not None:
        try:
            # Non-blocking receive of any message type
            msg = mavlink_connection.recv_match(blocking=True, timeout=0.1)
            if msg is None:
                continue

            msg_type = msg.get_type()
            if msg_type == 'BAD_DATA':
                continue

            current_time = time.time()

            with message_stats_lock:
                if msg_type not in message_stats:
                    message_stats[msg_type] = {
                        'count': 0,
                        'first_time': current_time,
                        'last_time': current_time,
                        'hz': 0.0,
                        'last_msg': None
                    }

                stats = message_stats[msg_type]
                stats['count'] += 1
                stats['last_time'] = current_time

                # Calculate Hz based on count and elapsed time
                elapsed = current_time - stats['first_time']
                if elapsed > 0.5:
                    stats['hz'] = stats['count'] / elapsed
                    # Reset periodically to get recent rate
                    if elapsed > 5.0:
                        stats['count'] = 1
                        stats['first_time'] = current_time

                # Store last message data (convert to dict)
                try:
                    msg_dict = {}
                    for field in msg.get_fieldnames():
                        value = getattr(msg, field)
                        # Handle bytes
                        if isinstance(value, bytes):
                            value = value.decode('utf-8', errors='replace').rstrip('\x00')
                        # Handle lists/arrays
                        elif isinstance(value, (list, tuple)):
                            value = list(value)
                        # Sanitize floats
                        elif isinstance(value, float):
                            if math.isnan(value) or math.isinf(value):
                                value = None
                        msg_dict[field] = value
                    stats['last_msg'] = msg_dict
                except Exception:
                    pass

        except Exception as e:
            if inspector_running:
                time.sleep(0.1)


# FastAPI app
@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: try to connect and start inspector
    connect_mavlink()
    start_inspector()
    yield
    # Shutdown: stop inspector and disconnect
    stop_inspector()
    disconnect_mavlink()


app = FastAPI(title="DroneBlocks MAVLink Tool", lifespan=lifespan)


@app.get("/api/status")
async def get_status():
    """Get connection status."""
    connected = mavlink_connection is not None
    return {
        "connected": connected,
        "target_system": mavlink_connection.target_system if connected else None,
        "target_component": mavlink_connection.target_component if connected else None,
        "params_loaded": params_loaded,
        "param_count": len(parameters),
        "total_params": param_count
    }


@app.post("/api/connect")
async def api_connect(port: Optional[str] = None):
    """Connect to flight controller."""
    if connect_mavlink(port):
        return {"status": "connected"}
    return JSONResponse(status_code=500, content={"error": "Connection failed"})


@app.post("/api/disconnect")
async def api_disconnect():
    """Disconnect from flight controller."""
    disconnect_mavlink()
    return {"status": "disconnected"}


@app.get("/api/ports")
async def get_ports():
    """List available serial ports."""
    patterns = [
        "/dev/ttyACM*",
        "/dev/ttyUSB*",
        "/dev/cu.usbmodem*",
        "/dev/cu.usbserial*",
    ]
    ports = []
    for pattern in patterns:
        ports.extend(glob.glob(pattern))
    return {"ports": sorted(ports)}


@app.post("/api/params/refresh")
async def refresh_params():
    """Request and receive all parameters."""
    if mavlink_connection is None:
        return JSONResponse(status_code=400, content={"error": "Not connected"})

    # Pause inspector while fetching parameters to avoid serial port conflicts
    was_running = inspector_running
    if was_running:
        stop_inspector()

    try:
        # Run in thread pool to avoid blocking
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, request_parameters)
        success = await loop.run_in_executor(None, receive_parameters)

        if success:
            return {"status": "ok", "count": len(parameters)}
        return JSONResponse(status_code=500, content={"error": "Failed to receive parameters"})
    finally:
        # Restart inspector if it was running
        if was_running:
            start_inspector()


def sanitize_float(value):
    """Convert nan/inf to JSON-safe values."""
    if isinstance(value, float):
        if math.isnan(value):
            return None
        if math.isinf(value):
            return None
    return value


@app.get("/api/params")
async def get_params():
    """Get all cached parameters."""
    # Sanitize params - some PX4 values can be nan/inf which aren't JSON serializable
    sanitized = {}
    for name, info in parameters.items():
        sanitized[name] = {
            'value': sanitize_float(info['value']),
            'type': info['type'],
            'index': info['index']
        }
    return {
        "loaded": params_loaded,
        "count": len(parameters),
        "total": param_count,
        "params": sanitized
    }


@app.post("/api/params/{name}")
async def api_set_param(name: str, value: float):
    """Set a parameter value."""
    if mavlink_connection is None:
        return JSONResponse(status_code=400, content={"error": "Not connected"})

    loop = asyncio.get_event_loop()
    success = await loop.run_in_executor(None, set_parameter, name, value)

    if success:
        return {"status": "ok", "name": name, "value": sanitize_float(parameters.get(name, {}).get('value'))}
    return JSONResponse(status_code=500, content={"error": "Failed to set parameter"})


@app.get("/api/debug/messages")
async def debug_messages():
    """Debug: show recent MAVLink messages."""
    if mavlink_connection is None:
        return JSONResponse(status_code=400, content={"error": "Not connected"})

    messages = []
    start_time = time.time()
    while time.time() - start_time < 2.0:
        msg = mavlink_connection.recv_match(blocking=True, timeout=0.1)
        if msg:
            msg_type = msg.get_type()
            messages.append(msg_type)
            if len(messages) >= 20:
                break

    return {"messages": messages}


@app.get("/api/inspector")
async def get_inspector():
    """Get all message types with their rates."""
    with message_stats_lock:
        result = []
        for msg_type, stats in message_stats.items():
            result.append({
                'type': msg_type,
                'hz': round(stats['hz'], 1),
                'count': stats['count'],
                'last_time': stats['last_time']
            })
        # Sort alphabetically by message type
        result.sort(key=lambda x: x['type'])
        return {"messages": result, "running": inspector_running}


@app.get("/api/inspector/{msg_type}")
async def get_inspector_message(msg_type: str):
    """Get details of a specific message type."""
    with message_stats_lock:
        if msg_type not in message_stats:
            return JSONResponse(status_code=404, content={"error": "Message type not found"})

        stats = message_stats[msg_type]
        return {
            'type': msg_type,
            'hz': round(stats['hz'], 1),
            'count': stats['count'],
            'fields': stats['last_msg']
        }


@app.post("/api/inspector/start")
async def start_inspector_endpoint():
    """Start the message inspector."""
    start_inspector()
    return {"status": "started"}


@app.post("/api/inspector/stop")
async def stop_inspector_endpoint():
    """Stop the message inspector."""
    stop_inspector()
    return {"status": "stopped"}


@app.post("/api/shell")
async def shell_command(command: str = ""):
    """Send a shell command and get response (for testing)."""
    if mavlink_connection is None:
        return JSONResponse(status_code=400, content={"error": "Not connected"})

    loop = asyncio.get_event_loop()
    await loop.run_in_executor(None, send_shell_command, command)
    await asyncio.sleep(0.5)  # Wait for response
    output = await loop.run_in_executor(None, receive_shell_output, 1.0)

    return {"command": command, "output": output}


@app.websocket("/ws/console")
async def console_websocket(websocket: WebSocket):
    """WebSocket for MAVLink shell console."""
    await websocket.accept()
    print("WebSocket console: client connected")

    if mavlink_connection is None:
        await websocket.send_json({"error": "Not connected to flight controller"})
        await websocket.close()
        return

    # Stop inspector to avoid serial port conflicts
    was_inspector_running = inspector_running
    if was_inspector_running:
        stop_inspector()

    stop_event = asyncio.Event()

    # Start a background task to receive shell output
    async def receive_output():
        loop = asyncio.get_event_loop()
        while not stop_event.is_set():
            try:
                output = await loop.run_in_executor(None, receive_shell_output, 0.5)
                if output:
                    print(f"WebSocket console: sending {len(output)} bytes")
                    await websocket.send_json({"output": output})
                await asyncio.sleep(0.05)
            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"WebSocket console receive error: {e}")
                await asyncio.sleep(0.1)

    receive_task = asyncio.create_task(receive_output())

    try:
        # Send initial newline to get prompt
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, send_shell_command, "")
        await websocket.send_json({"output": "Connecting to MAVLink shell...\n"})

        while True:
            data = await websocket.receive_json()
            command = data.get("command", "")
            print(f"WebSocket console: received command '{command}'")
            await loop.run_in_executor(None, send_shell_command, command)
    except WebSocketDisconnect:
        print("WebSocket console: client disconnected")
    except Exception as e:
        print(f"WebSocket console error: {e}")
    finally:
        stop_event.set()
        receive_task.cancel()
        try:
            await receive_task
        except asyncio.CancelledError:
            pass
        # Restart inspector if it was running
        if was_inspector_running:
            start_inspector()


# Mount static files
app.mount("/", StaticFiles(directory="static", html=True), name="static")


if __name__ == "__main__":
    import uvicorn

    # Check for port argument
    port = None
    if len(sys.argv) > 1:
        port = sys.argv[1]
        print(f"Using specified port: {port}")

    # Pre-connect if port specified
    if port:
        connect_mavlink(port)

    uvicorn.run(app, host="0.0.0.0", port=8000)
