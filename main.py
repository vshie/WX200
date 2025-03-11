#!/usr/bin/env python3

import serial
import serial.tools.list_ports
import time
from flask import Flask, jsonify, send_from_directory, Response, request
from flask_cors import CORS
import threading
import logging
import requests
import json
from datetime import datetime
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2
import os
import signal

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
        self.current_data = {}
        self.data_thread = None
        self.running = False
        self.DEFAULT_BAUD = 4800
        self.HIGH_BAUD = 38400
        self.auto_reconnect = True
        self.last_reconnect_attempt = 0
        self.reconnect_interval = 5  # seconds
        self.default_port = "/dev/ttyUSB0"
        
        # Initialize MAVLink connection
        self.mav = mavutil.mavlink_connection(
            'udpout:host.docker.internal:14550',
            source_system=1,
            source_component=mavutil.mavlink.MAV_COMP_ID_PERIPHERAL
        )
        
        # Message timing control
        self.last_wind_send = 0
        self.last_weather_send = 0
        self.last_gps_send = 0
        self.last_attitude_send = 0
        self.wind_send_interval = 0.2  # 5Hz
        self.weather_send_interval = 1.0  # 1Hz
        self.gps_send_interval = 0.2  # 5Hz
        self.attitude_send_interval = 0.2  # 5Hz

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
        """Main background process that handles connection and data reading"""
        while self.running:
            try:
                if not self.connected:
                    current_time = time.time()
                    if (current_time - self.last_reconnect_attempt) >= self.reconnect_interval:
                        self.last_reconnect_attempt = current_time
                        logger.info("Attempting auto-reconnect...")
                        self.connect(self.default_port)
                elif self.serial_port and self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode().strip()
                    self.parse_nmea(line)
                else:
                    time.sleep(0.01)  # Small sleep to prevent CPU spinning
            except Exception as e:
                logger.error(f"Background process error: {str(e)}")
                self.handle_connection_error()
                time.sleep(1)  # Delay before retry

    def handle_connection_error(self):
        """Handle connection errors and cleanup"""
        if self.serial_port:
            try:
                self.serial_port.close()
            except:
                pass
            self.serial_port = None
        self.connected = False
        self.current_data = {}

    def send_mavlink_wind(self):
        """Send wind data via MAVLink"""
        if 'wind' not in self.current_data:
            return

        wind = self.current_data['wind']
        if not all(k in wind for k in ['speed', 'angle', 'status']):
            return
        
        # Only send if data is valid
        if wind['status'] != 'A':
            return

        current_time = time.time()
        if current_time - self.last_wind_send < self.wind_send_interval:
            return

        # Convert wind speed to m/s if needed
        speed = wind['speed']
        if wind['speed_units'] == 'N':  # Knots
            speed *= 0.514444
        elif wind['speed_units'] == 'K':  # km/h
            speed *= 0.277778
        elif wind['speed_units'] == 'M':  # m/s
            pass  # Already in m/s

        self.mav.mav.wind_send(
            direction=float(wind['angle']),  # degrees
            speed=float(speed),  # m/s
            speed_z=0.0  # m/s
        )
        self.last_wind_send = current_time

    def send_mavlink_weather(self):
        """Send weather data via MAVLink"""
        if 'meteorological' not in self.current_data:
            return

        met = self.current_data['meteorological']
        current_time = time.time()
        if current_time - self.last_weather_send < self.weather_send_interval:
            return

        # Convert pressure to hPa/mbar if in different units
        pressure = None
        if met['barometric_pressure_bars'] is not None:
            pressure = met['barometric_pressure_bars'] * 1000  # Convert bars to mbar
        elif met['barometric_pressure_inches'] is not None:
            pressure = met['barometric_pressure_inches'] * 33.8639  # Convert inHg to mbar

        wind_speed = 0.0
        wind_direction = 0.0
        if 'wind' in self.current_data:
            wind = self.current_data['wind']
            if wind.get('status') == 'A':
                wind_speed = float(wind['speed'])
                wind_direction = float(wind['angle'])

        self.mav.mav.weather_send(
            temperature=float(met['air_temp_c']) if met['air_temp_c'] is not None else 0,
            pressure=float(pressure) if pressure is not None else 0,
            humidity=float(met['relative_humidity']) if met['relative_humidity'] is not None else 0,
            wind_dir=wind_direction,
            wind_speed=wind_speed,
            wind_speed_z=0
        )
        self.last_weather_send = current_time

    def send_mavlink_gps(self):
        """Send GPS data via MAVLink"""
        if 'gps' not in self.current_data:
            return

        gps = self.current_data['gps']
        current_time = time.time()
        if current_time - self.last_gps_send < self.gps_send_interval:
            return

        # Convert to appropriate units and types
        lat = int(float(gps['latitude']) * 1e7)  # Convert to int32 degE7
        lon = int(float(gps['longitude']) * 1e7)  # Convert to int32 degE7
        alt = int(float(gps['altitude']) * 1000)  # Convert to mm
        hdop = int(float(gps['hdop']) * 100) if 'hdop' in gps else 0
        vdop = int(float(gps['vdop']) * 100) if 'vdop' in gps else 0
        
        # Convert speed to m/s if needed
        speed = float(gps.get('speed', 0))
        if gps.get('speed_units') == 'N':  # Knots
            speed *= 0.514444
        
        # Get number of satellites
        sats = int(gps.get('satellites', 0))
        
        # Determine fix type
        fix_type = mavlink2.GPS_FIX_TYPE_NO_GPS
        if gps.get('fix') == '1':
            fix_type = mavlink2.GPS_FIX_TYPE_2D_FIX
        elif gps.get('fix') == '2':
            fix_type = mavlink2.GPS_FIX_TYPE_3D_FIX
        
        self.mav.mav.gps_raw_int_send(
            time_usec=int(time.time() * 1e6),
            fix_type=fix_type,
            lat=lat,
            lon=lon,
            alt=alt,
            eph=hdop,
            epv=vdop,
            vel=int(speed * 100),  # cm/s
            cog=int(float(gps.get('track', 0)) * 100),  # cdeg
            satellites_visible=sats,
            alt_ellipsoid=alt,
            h_acc=hdop,
            v_acc=vdop,
            vel_acc=0,
            hdg_acc=0
        )
        self.last_gps_send = current_time

    def send_mavlink_attitude(self):
        """Send compass heading as attitude data via MAVLink"""
        if 'compass' not in self.current_data:
            return

        compass = self.current_data['compass']
        current_time = time.time()
        if current_time - self.last_attitude_send < self.attitude_send_interval:
            return

        # Convert heading to radians
        heading_rad = float(compass['heading']) * 0.0174533

        self.mav.mav.attitude_send(
            time_boot_ms=int(time.time() * 1000),
            roll=0,  # Only have heading data
            pitch=0,  # Only have heading data
            yaw=heading_rad,
            rollspeed=0,
            pitchspeed=0,
            yawspeed=0
        )
        self.last_attitude_send = current_time

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

    def send_command(self, command, read_response=True):
        """Send command and optionally read response"""
        if not command.endswith('\r\n'):
            command = self.calculate_checksum(command)
        
        logger.debug(f"Sending command: {command.strip()}")
        self.serial_port.write(command.encode())
        
        if read_response:
            time.sleep(0.1)  # Wait for response
            response = self.serial_port.readline().decode().strip()
            logger.debug(f"Response: {response}")
            return response
        return None

    def parse_nmea(self, line):
        """Parse NMEA sentences from the device"""
        try:
            if line.startswith('$'):
                parts = line.split(',')
                sentence_type = parts[0]
                
                # Store the raw data and timestamp
                self.current_data['timestamp'] = time.time()
                self.current_data['last_sentence'] = sentence_type
                self.current_data['raw_data'] = line
                
                # Parse specific sentence types
                if sentence_type in ['$WIMWV', '$WIVWR']:  # Wind data
                    self.parse_wind_data(parts)
                    self.send_mavlink_wind()
                elif sentence_type == '$WIMDA':  # Meteorological composite
                    self.parse_meteorological_data(parts)
                    self.send_mavlink_weather()
                elif sentence_type == '$GPRMC':  # GPS data
                    self.parse_gps_rmc(parts)
                    self.send_mavlink_gps()
                elif sentence_type == '$GPGGA':  # GPS fix data
                    self.parse_gps_gga(parts)
                    self.send_mavlink_gps()
                elif sentence_type == '$HCHDT':  # Heading data
                    self.parse_compass_data(parts)
                    self.send_mavlink_attitude()
                elif sentence_type == '$GPZDA':  # Time & Date
                    self.parse_time_data(parts)
                
        except Exception as e:
            logger.error(f"Parse error: {str(e)}")

    def parse_wind_data(self, parts):
        """Parse wind-related NMEA sentences"""
        try:
            if parts[0] == '$WIMWV':  # Wind Speed and Angle
                self.current_data['wind'] = {
                    'angle': float(parts[1]) if parts[1] else None,
                    'reference': parts[2],  # R = Relative, T = True
                    'speed': float(parts[3]) if parts[3] else None,
                    'speed_units': parts[4],
                    'status': parts[5].split('*')[0]  # A = Valid
                }
        except Exception as e:
            logger.error(f"Wind data parse error: {str(e)}")

    def parse_meteorological_data(self, parts):
        """Parse meteorological composite data"""
        try:
            self.current_data['meteorological'] = {
                'barometric_pressure_inches': float(parts[1]) if parts[1] else None,
                'barometric_pressure_bars': float(parts[3]) if parts[3] else None,
                'air_temp_c': float(parts[5]) if parts[5] else None,
                'water_temp_c': float(parts[7]) if parts[7] else None,
                'relative_humidity': float(parts[9]) if parts[9] else None,
                'dew_point_c': float(parts[11]) if parts[11] else None
            }
        except Exception as e:
            logger.error(f"Meteorological data parse error: {str(e)}")

    def parse_time_data(self, parts):
        """Parse time and date data"""
        try:
            if len(parts) >= 4:
                self.current_data['datetime'] = {
                    'utc_time': parts[1],
                    'day': parts[2],
                    'month': parts[3],
                    'year': parts[4]
                }
        except Exception as e:
            logger.error(f"Time data parse error: {str(e)}")

    def parse_gps_rmc(self, parts):
        """Parse GPS RMC data"""
        try:
            if len(parts) >= 12:
                if 'gps' not in self.current_data:
                    self.current_data['gps'] = {}
                
                # Convert DDMM.MMMM to decimal degrees
                if parts[3] and parts[4] and parts[5] and parts[6]:
                    lat = float(parts[3][:2]) + float(parts[3][2:]) / 60.0
                    if parts[4] == 'S':
                        lat = -lat
                    lon = float(parts[5][:3]) + float(parts[5][3:]) / 60.0
                    if parts[6] == 'W':
                        lon = -lon
                    
                    self.current_data['gps'].update({
                        'latitude': lat,
                        'longitude': lon,
                        'speed': float(parts[7]) if parts[7] else 0,
                        'speed_units': 'N',  # Speed is in knots
                        'track': float(parts[8]) if parts[8] else 0,
                        'fix': '1' if parts[2] == 'A' else '0'
                    })
        except Exception as e:
            logger.error(f"GPS RMC parse error: {str(e)}")

    def parse_gps_gga(self, parts):
        """Parse GPS GGA data"""
        try:
            if len(parts) >= 15:
                if 'gps' not in self.current_data:
                    self.current_data['gps'] = {}
                
                self.current_data['gps'].update({
                    'altitude': float(parts[9]) if parts[9] else 0,
                    'altitude_units': parts[10],
                    'hdop': float(parts[8]) if parts[8] else 0,
                    'satellites': int(parts[7]) if parts[7] else 0
                })
        except Exception as e:
            logger.error(f"GPS GGA parse error: {str(e)}")

    def parse_compass_data(self, parts):
        """Parse compass heading data"""
        try:
            if len(parts) >= 3:
                self.current_data['compass'] = {
                    'heading': float(parts[1]) if parts[1] else 0,
                    'reference': parts[2]  # Should be 'T' for True
                }
        except Exception as e:
            logger.error(f"Compass data parse error: {str(e)}")

    def change_baud_rate(self, port, new_baud):
        """Change baud rate following the manual's sequence"""
        try:
            # 1. Disable periodic transmissions at current baud rate
            self.send_command("$PAMTX,0")
            logger.info("Disabled periodic transmissions")
            
            # 2. Send baud rate change command at current baud rate
            self.send_command(f"$PAMTC,BAUD,{new_baud}")
            logger.info(f"Sent baud rate change command to {new_baud}")
            
            # 3. Wait for any queued messages and ensure command is processed
            time.sleep(2)
            
            # 4. Close port properly
            if self.serial_port:
                self.serial_port.flush()  # Ensure all data is written
                self.serial_port.close()
                self.serial_port = None
            
            # 5. Wait for device to complete baud rate change
            time.sleep(2)
            
            # 6. Open new connection at new baud rate
            try:
                self.serial_port = serial.Serial(
                    port=port,
                    baudrate=new_baud,
                    timeout=2,  # Increased timeout for stability
                    write_timeout=2
                )
            except Exception as e:
                logger.error(f"Failed to open port at new baud rate: {str(e)}")
                return False
            
            # 7. Verify port is open
            if not self.serial_port.is_open:
                self.serial_port.open()
            
            # 8. Clear any buffered data
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            
            # 9. Wait for device to stabilize at new rate
            time.sleep(1)
            
            # 10. Re-enable periodic transmissions and verify communication
            for _ in range(3):  # Try up to 3 times
                response = self.send_command("$PAMTX,1")
                if response and ('$' in response):  # Verify we got a valid NMEA response
                    logger.info("Re-enabled periodic transmissions at new baud rate")
                    return True
                time.sleep(0.5)
            
            logger.error("Failed to re-enable transmissions at new baud rate")
            return False
            
        except Exception as e:
            logger.error(f"Baud rate change error: {str(e)}")
            return False

    def configure_device(self):
        """Configure WX200 for optimal performance"""
        if not self.connected:
            return False
        
        try:
            # First disable all outputs while we configure
            self.send_command("$PAMTX,0")
            time.sleep(0.1)

            # Configuration commands for WX200
            commands = [
                "$PAMTC,EN,ALL,1",  # Enable all sentences
                "$PAMTC,OPTION,HEATERCTRL,1",  # Enable heater control if available
                "$PAMTX,1,1",  # Enable true wind calculation
                "$PAMTX,2,1",  # Enable apparent wind calculation
                "$PAMTX,3,1",   # Enable theoretical wind calculation
                "$PAMTX,0,1"  # Set fastest update rate - do this last
            ]
            
            for cmd in commands:
                response = self.send_command(cmd)
                logger.info(f"Command: {cmd} -> Response: {response}")
                time.sleep(0.1)  # Give device time to process each command
                
            logger.info("Device configured for optimal performance")
            return True
            
        except Exception as e:
            logger.error(f"Configuration error: {str(e)}")
            return False

    def connect(self, port="/dev/ttyUSB0"):
        """Connect to the WX200 device"""
        try:
            if self.serial_port:
                self.disconnect()
            
            self.default_port = port
            
            # Initially connect at default baud rate
            logger.info(f"Connecting at initial baud rate: {self.DEFAULT_BAUD}")
            self.serial_port = serial.Serial(
                port=port,
                baudrate=self.DEFAULT_BAUD,
                timeout=1
            )
            time.sleep(0.5)  # Give device time to stabilize
            
            # Test communication at default baud
            test_response = self.send_command("$PAMTC,EN,ALL,0")
            if not test_response:
                logger.error("No response from device at default baud rate")
                self.disconnect()
                return False
            
            # Try to increase baud rate
            logger.info("Testing communication successful, attempting to increase baud rate")
            if self.change_baud_rate(port, self.HIGH_BAUD):
                self.connected = True
                logger.info(f"Connected to {port} at {self.HIGH_BAUD} baud")
                
                # Configure device for optimal performance
                if self.configure_device():
                    return True
                else:
                    logger.error("Failed to configure device")
                    self.disconnect()
                    return False
            else:
                logger.error("Failed to set high baud rate")
                self.disconnect()
                return False
                
        except Exception as e:
            logger.error(f"Connection error: {str(e)}")
            self.connected = False
            return False

    def disconnect(self):
        """Disconnect from the device"""
        if self.serial_port:
            # Try to set back to default baud rate before disconnecting
            try:
                self.send_command("$PAMTC,BAUD,4800", read_response=False)
            except:
                pass
            self.serial_port.close()
            self.serial_port = None
        self.connected = False
        logger.info("Disconnected from device")

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

@app.route('/api/data')
def get_data():
    """Get current data"""
    return jsonify({
        "connected": wx200.connected,
        "data": wx200.current_data
    })

@app.route('/register_service')
def register_service():
    """Register the extension as a service in BlueOS."""
    return jsonify({
        "name": "WX200 Weather Station",
        "description": "Interface for WX200 Weather Station",
        "icon": "mdi-windy-dust",
        "company": "Blue Robotics",
        "version": "0.1.0",
        "webpage": "",
        "api": "/v1.0/docs",
        "route": "/wx200"
    })

@app.route('/v1.0/docs')
def swagger_docs():
    """Provide API documentation."""
    return jsonify({
        "openapi": "3.0.0",
        "info": {
            "title": "WX200 Weather Station API",
            "version": "1.0.0"
        },
        "paths": {}
    })

@app.route('/favicon.ico')
def favicon():
    """Serve favicon to prevent 404 errors."""
    return send_from_directory('static', 'favicon.ico') if os.path.exists('static/favicon.ico') else ('', 204)

@app.route('/version')
def version():
    """Return version information."""
    return jsonify({
        "version": "0.1.0",
        "name": "wx200"
    })

if __name__ == '__main__':
    # Start the Flask app
    app.run(host='0.0.0.0', port=6567) 