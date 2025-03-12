#!/usr/bin/env python3

import serial
import serial.tools.list_ports
import time
from flask import Flask, jsonify, send_from_directory, Response, request
from flask_cors import CORS
import threading
import logging
import requests
import os
import signal
from queue import Queue, Empty

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__)
CORS(app)

class WX200:
    def __init__(self):
        self.serial_port = None
        self.connected = False
        self.available_ports = []
        self.DEFAULT_BAUD = 4800
        self.HIGH_BAUD = 38400
        self.running = False
        self.data_thread = None
        
        # Message buffer for serial terminal
        self.message_buffer = []
        self.max_buffer_size = 1000
        self.new_messages = Queue()
        
        # Start the background process
        self.start_background_process()

    def start_background_process(self):
        """Start the continuous background process"""
        if not self.running:
            self.running = True
            self.data_thread = threading.Thread(target=self.background_process)
            self.data_thread.daemon = True
            self.data_thread.start()
            logger.info("Background process started")

    def background_process(self):
        """Main background process that handles serial data reading"""
        while self.running:
            try:
                if self.serial_port and self.serial_port.in_waiting:
                    try:
                        # Use errors='replace' to handle non-UTF-8 characters
                        line = self.serial_port.readline().decode('utf-8', errors='replace').strip()
                        if line:  # Only process non-empty lines
                            self.add_to_buffer(f"RX: {line}")
                    except Exception as e:
                        logger.error(f"Error reading serial data: {str(e)}")
                else:
                    time.sleep(0.01)  # Small sleep to prevent CPU spinning
            except Exception as e:
                logger.error(f"Background process error: {str(e)}")
                time.sleep(1)  # Delay before retry

    def add_to_buffer(self, message):
        """Add a message to the terminal buffer"""
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        formatted_message = f"[{timestamp}] {message}"
        
        self.message_buffer.append(formatted_message)
        # Keep buffer at max size
        if len(self.message_buffer) > self.max_buffer_size:
            self.message_buffer.pop(0)
            
        # Add to new messages queue for SSE
        self.new_messages.put(formatted_message)
        
        logger.info(formatted_message)

    def get_available_ports(self):
        """List all available serial ports"""
        self.available_ports = [
            {"port": port.device, "description": port.description}
            for port in serial.tools.list_ports.comports()
        ]
        return self.available_ports

    def calculate_checksum(self, command):
        """Calculate NMEA checksum"""
        checksum = 0
        for char in command[1:]:  # Skip the $
            checksum ^= ord(char)
        return f"{command}*{checksum:02X}\r\n"

    def send_command(self, command):
        """Send command to the serial port and log it"""
        if not self.connected or not self.serial_port:
            self.add_to_buffer("ERROR: Not connected to device")
            return False
            
        if not command.endswith('\r\n'):
            command = self.calculate_checksum(command)
        
        self.add_to_buffer(f"TX: {command.strip()}")
        self.serial_port.write(command.encode())
        self.serial_port.flush()  # Ensure command is sent immediately
        return True

    def connect(self, port, baud=None):
        """Connect to the WX200 device"""
        try:
            if self.serial_port:
                self.disconnect()
            
            if baud is None:
                baud = self.DEFAULT_BAUD
                
            self.add_to_buffer(f"Connecting to {port} at {baud} baud...")
            
            # Use 8-N-1 configuration explicitly
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            
            # Flush buffers at startup
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            
            self.connected = True
            self.add_to_buffer(f"Connected to {port} at {baud} baud")
            return True
                
        except Exception as e:
            error_msg = f"Connection error: {str(e)}"
            logger.error(error_msg)
            self.add_to_buffer(f"ERROR: {error_msg}")
            self.connected = False
            return False

    def change_baud_rate(self, new_baud):
        """Change baud rate manually"""
        if not self.connected or not self.serial_port:
            self.add_to_buffer("ERROR: Not connected to device")
            return False

        try:
            port = self.serial_port.port
            # Step 1: Suspend ALL transmissions
            self.add_to_buffer("Stopping all transmissions")
            self.send_command("$PAMTX,0")
            time.sleep(1)
            
            # Step 2: Send baud rate change command
            baud_cmd = f"$PAMTC,BAUD,{new_baud}"
            self.send_command(baud_cmd)
            self.add_to_buffer(f"Sent baud rate change command to {new_baud}")
            
            # Step 3: Close port
            time.sleep(2)
            if self.serial_port:
                self.serial_port.close()
                self.serial_port = None
                self.connected = False
            
            # Step 4: Wait for device to change baud
            time.sleep(3)
            
            # Step 5: Reconnect at new baud rate
            return self.connect(port, new_baud)
            
        except Exception as e:
            error_msg = f"Baud rate change error: {str(e)}"
            logger.error(error_msg)
            self.add_to_buffer(f"ERROR: {error_msg}")
            return False

    def stop_transmissions(self):
        """Stop transmissions from the device"""
        return self.send_command("$PAMTX,0")

    def start_transmissions(self):
        """Start transmissions from the device"""
        return self.send_command("$PAMTX,1")

    def set_message_rate(self, message_type, rate):
        """Set the rate for a specific message type
        rate is in tenths of seconds (1 = 0.1s or 10Hz)"""
        return self.send_command(f"$PAMTC,EN,{message_type},1,{rate}")

    def disconnect(self):
        """Disconnect from the device"""
        if self.serial_port:
            # Try to set back to default baud rate before disconnecting
            try:
                self.send_command("$PAMTC,BAUD,4800")
            except:
                pass
                
            self.serial_port.close()
            self.serial_port = None
            
        self.connected = False
        self.add_to_buffer("Disconnected from device")

# Create global instance
wx200 = WX200()

def signal_handler(signum, frame):
    """Handle shutdown signals"""
    logger.info("Shutdown signal received")
    wx200.running = False
    if wx200.data_thread:
        wx200.data_thread.join(timeout=2)
    wx200.disconnect()
    os._exit(0)

# Register signal handlers
signal.signal(signal.SIGTERM, signal_handler)
signal.signal(signal.SIGINT, signal_handler)

@app.route('/api/ports')
def get_ports():
    """Get available serial ports"""
    return jsonify(wx200.get_available_ports())

@app.route('/api/connect/<port>')
def connect(port):
    """Connect to specified port"""
    success = wx200.connect(port)
    return jsonify({"success": success})

@app.route('/api/disconnect')
def disconnect():
    """Disconnect from device"""
    wx200.disconnect()
    return jsonify({"success": True})

@app.route('/api/status')
def get_status():
    """Get current connection status"""
    return jsonify({
        "connected": wx200.connected,
        "port": wx200.serial_port.port if wx200.serial_port else None,
        "baud": wx200.serial_port.baudrate if wx200.serial_port else None
    })

@app.route('/api/terminal')
def get_terminal():
    """Get terminal buffer history"""
    return jsonify(wx200.message_buffer)

@app.route('/api/terminal/events')
def terminal_events():
    """Server-sent events for terminal updates"""
    def generate():
        while True:
            try:
                message = wx200.new_messages.get(timeout=30)
                yield f"data: {message}\n\n"
            except Empty:
                yield "data: ping\n\n"  # Keep connection alive
            
    return Response(generate(), mimetype='text/event-stream')

@app.route('/api/send', methods=['POST'])
def send_command():
    """Send a command to the device"""
    command = request.json.get('command')
    if not command:
        return jsonify({"success": False, "error": "No command provided"})
    
    success = wx200.send_command(command)
    return jsonify({"success": success})

@app.route('/api/baud/change/<int:baud>')
def change_baud(baud):
    """Change the baud rate"""
    success = wx200.change_baud_rate(baud)
    return jsonify({"success": success})

@app.route('/api/transmissions/stop')
def stop_transmissions():
    """Stop device transmissions"""
    success = wx200.stop_transmissions()
    return jsonify({"success": success})

@app.route('/api/transmissions/start')
def start_transmissions():
    """Start device transmissions"""
    success = wx200.start_transmissions()
    return jsonify({"success": success})

@app.route('/api/message/rate', methods=['POST'])
def set_message_rate():
    """Set message rate for specific type"""
    data = request.json
    if not data or 'type' not in data or 'rate' not in data:
        return jsonify({"success": False, "error": "Missing type or rate"})
    
    success = wx200.set_message_rate(data['type'], data['rate'])
    return jsonify({"success": success})

@app.route('/register')
def register_service():
    """Register the extension as a service in BlueOS."""
    return jsonify({
        "name": "WX200 Weather Station",
        "description": "Interface for WX200 Weather Station",
        "icon": "mdi-weather-windy",
        "company": "Blue Robotics",
        "version": "0.1.0",
        "webpage": "",
        "api": "/v1.0/docs",
        "route": "/wx200"
    })

@app.route('/favicon.ico')
def favicon():
    """Serve favicon to prevent 404 errors."""
    return send_from_directory('static', 'favicon.ico') if os.path.exists('static/favicon.ico') else ('', 204)

@app.route('/')
def index():
    """Serve the main frontend page."""
    return send_from_directory('frontend', 'index.html')

@app.route('/<path:path>')
def static_files(path):
    """Serve static files."""
    return send_from_directory('frontend', path)

if __name__ == '__main__':
    # Start the Flask app
    app.run(host='0.0.0.0', port=6567) 