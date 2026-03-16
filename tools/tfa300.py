#!/usr/bin/env python3
"""
TFA300 LIDAR Reader and Plotter

Reads distance measurements from a Benewake TFA300 LIDAR sensor via UART
and displays real-time graphs.

Usage:
    # First time setup (configure for 10kHz at 921600 baud):
    python3 tfa300.py --setup [serial_port]
    # Then power cycle the sensor

    # Normal operation (after setup):
    python3 tfa300.py --baud 921600 [serial_port]

    # Use with unconfigured sensor (default 115200 baud, 50Hz):
    python3 tfa300.py --baud 115200 [serial_port]

Arguments:
    serial_port: Serial port device (default: /dev/ttyUSB0)
"""

import sys
import time
import serial
import struct
import argparse
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import queue


class TFA300:
    """TFA300 LIDAR sensor interface"""

    # Frame headers
    HEADER_STANDARD = bytes([0x59, 0x59])
    HEADER_HIGH_RATE = bytes([0x20, 0x20])

    # Command header
    CMD_HEADER = 0x5A

    def __init__(self, port, baudrate=115200):
        """Initialize TFA300 connection"""
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.high_rate_mode = False

        # Background reading thread
        self.frame_queue = queue.Queue(maxsize=10000)  # Buffer up to 10k frames
        self.reader_thread = None
        self.reading = False

    def connect(self):
        """Open serial connection"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0
            )
            time.sleep(0.1)  # Wait for connection to stabilize
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            return False

    def send_command(self, cmd_bytes):
        """Send command to TFA300 and wait for response"""
        if not self.ser:
            return None

        self.ser.write(cmd_bytes)
        time.sleep(0.05)  # Wait for response

        # Try to read response
        if self.ser.in_waiting > 0:
            response = self.ser.read(self.ser.in_waiting)
            return response
        return None

    def calculate_checksum(self, data):
        """Calculate checksum (lower 8 bits of sum)"""
        return sum(data) & 0xFF

    def set_frame_rate(self, rate_hz):
        """Set operating frequency (1-10000 Hz)"""
        rate_hz = max(1, min(10000, rate_hz))

        # Command: 5A 06 03 LL HH SU
        cmd = bytes([
            self.CMD_HEADER,
            0x06,  # Length
            0x03,  # ID for frame rate
            rate_hz & 0xFF,  # Low byte
            (rate_hz >> 8) & 0xFF,  # High byte
        ])
        cmd += bytes([self.calculate_checksum(cmd)])

        print(f"Setting frame rate to {rate_hz} Hz...")
        response = self.send_command(cmd)
        if response:
            print(f"Response: {response.hex()}")
        return response

    def set_high_rate_format(self):
        """Switch to 6-byte high frame rate format"""
        # Command: 5A 05 05 20 84
        cmd = bytes([0x5A, 0x05, 0x05, 0x20, 0x84])

        print("Setting 6-byte high frame rate format...")
        response = self.send_command(cmd)
        if response:
            print(f"Response: {response.hex()}")
            self.high_rate_mode = True
        return response

    def set_standard_format(self):
        """Switch to 9-byte standard format"""
        # Command: 5A 05 05 01 65
        cmd = bytes([0x5A, 0x05, 0x05, 0x01, 0x65])

        print("Setting 9-byte standard format...")
        response = self.send_command(cmd)
        if response:
            print(f"Response: {response.hex()}")
            self.high_rate_mode = False
        return response

    def set_baud_rate(self, baudrate):
        """Set UART baud rate"""
        # Command: 5A 08 06 H1 H2 H3 H4 SU
        cmd = bytes([
            self.CMD_HEADER,
            0x08,  # Length
            0x06,  # ID for baud rate
            baudrate & 0xFF,
            (baudrate >> 8) & 0xFF,
            (baudrate >> 16) & 0xFF,
            (baudrate >> 24) & 0xFF,
        ])
        cmd += bytes([self.calculate_checksum(cmd)])

        print(f"Setting baud rate to {baudrate}...")
        response = self.send_command(cmd)
        if response:
            print(f"Response: {response.hex()}")
        return response

    def disable_data_output(self):
        """Disable data output (required for configuration)"""
        # Command: 5A 05 07 00 66
        cmd = bytes([0x5A, 0x05, 0x07, 0x00, 0x66])

        print("Disabling data output...")
        response = self.send_command(cmd)
        if response:
            print(f"Response: {response.hex()}")
        return response

    def enable_data_output(self):
        """Enable data output"""
        # Command: 5A 05 07 01 67
        cmd = bytes([0x5A, 0x05, 0x07, 0x01, 0x67])

        print("Enabling data output...")
        response = self.send_command(cmd)
        if response:
            print(f"Response: {response.hex()}")
        return response

    def save_configuration(self):
        """Save current configuration to non-volatile memory"""
        # Command: 5A 04 11 6F
        cmd = bytes([0x5A, 0x04, 0x11, 0x6F])

        print("Saving configuration...")
        response = self.send_command(cmd)
        if response:
            print(f"Response: {response.hex()}")
        return response

    def configure_10khz(self):
        """Configure sensor for 10kHz operation at 921600 baud"""
        print("\n=== Configuring TFA300 for 10kHz ===")

        # Step 1: Disable data output so commands are processed
        print("Step 1: Disabling data output...")
        self.disable_data_output()
        time.sleep(0.3)

        # Clear any buffered data
        if self.ser.in_waiting > 0:
            cleared = self.ser.read(self.ser.in_waiting)
            print(f"  Cleared {len(cleared)} bytes from buffer")

        # Step 2: Set 10kHz frame rate
        print("Step 2: Setting 10kHz frame rate...")
        self.set_frame_rate(10000)
        time.sleep(0.2)

        # Step 3: Set 6-byte high frame rate format
        print("Step 3: Setting 6-byte format...")
        self.set_high_rate_format()
        time.sleep(0.2)

        # Step 4: Set 921600 baud rate
        print("Step 4: Setting 921600 baud...")
        self.set_baud_rate(921600)
        time.sleep(0.2)

        # Step 5: Save configuration
        print("Step 5: Saving configuration...")
        self.save_configuration()
        time.sleep(0.2)

        # Step 6: Re-enable data output
        print("Step 6: Enabling data output...")
        self.enable_data_output()
        time.sleep(0.5)

        print("\nStep 7: Switching to 921600 baud...")
        # Close and reopen at 921600
        self.ser.close()
        self.baudrate = 921600
        if not self.connect():
            print("ERROR: Failed to reconnect at 921600 baud")
            return False

        # Give sensor time to stabilize
        time.sleep(0.5)

        # Test high rate mode
        print("Step 8: Testing high frame rate mode at 921600 baud...")
        success = False
        for i in range(10):
            test_frame = self.read_frame(debug=False)
            if test_frame:
                print(f"  SUCCESS: Got frame #{i+1} - dist={test_frame['distance_m']:.2f}m, strength={test_frame['strength']}")
                success = True
                if i >= 2:  # Got at least 3 frames
                    break
            else:
                print(f"  Attempt {i+1}: No data yet...")
            time.sleep(0.05)

        if not success:
            print("\nHigh frame rate mode failed at 921600. Trying standard mode...")
            self.high_rate_mode = False
            for i in range(10):
                test_frame = self.read_frame(debug=False)
                if test_frame:
                    print(f"  SUCCESS with standard mode: dist={test_frame['distance_m']:.2f}m, strength={test_frame['strength']}")
                    success = True
                    if i >= 2:
                        break
                time.sleep(0.05)

        if not success:
            print("\nNo data at 921600 in either mode. Testing if sensor is still at 115200...")
            self.ser.close()
            self.baudrate = 115200
            if not self.connect():
                return False
            time.sleep(0.3)

            # Try high rate mode
            self.high_rate_mode = True
            test_frame = self.read_frame(debug=False)
            if test_frame:
                print(f"  Sensor is at 115200 in high rate mode: dist={test_frame['distance_m']:.2f}m")
            else:
                # Try standard mode
                self.high_rate_mode = False
                test_frame = self.read_frame(debug=False)
                if test_frame:
                    print(f"  Sensor is at 115200 in standard mode: dist={test_frame['distance_m']:.2f}m")
                    print("  WARNING: Frame rate/format changes didn't apply. Sensor may need power cycle.")
                else:
                    print("  ERROR: Lost communication with sensor")
                    return False

        print("\n=== Configuration complete ===")
        print(f"  Baud rate: {self.baudrate}")
        print(f"  Frame format: 6-byte high rate (0x20 0x20 header)")
        print(f"  Frame rate: 10000 Hz")
        print(f"  Expected data rate: ~58 KB/s")
        print()
        return True

    def read_frame(self, debug=False):
        """Read one data frame from the sensor"""
        if not self.ser or not self.ser.is_open:
            if debug:
                print("DEBUG: Serial port not open")
            return None

        # Look for frame header
        if self.high_rate_mode:
            header = self.HEADER_HIGH_RATE
            frame_size = 6
        else:
            header = self.HEADER_STANDARD
            frame_size = 9

        if debug:
            print(f"DEBUG: Looking for header {header.hex()}, frame size {frame_size}, high_rate_mode={self.high_rate_mode}")

        # Check if data is available (non-blocking for bulk reads)
        if self.ser.in_waiting < frame_size:
            if debug:
                print(f"DEBUG: Not enough data waiting ({self.ser.in_waiting} bytes)")
            return None

        # Read until we find header
        buffer = bytearray()
        max_attempts = 100
        attempts = 0

        while attempts < max_attempts:
            if self.ser.in_waiting > 0:
                byte = self.ser.read(1)
                buffer.append(byte[0])

                if debug and len(buffer) <= 10:
                    print(f"DEBUG: Read byte 0x{byte[0]:02x}, buffer len={len(buffer)}")

                # Check for header match
                if len(buffer) >= 2:
                    if bytes(buffer[-2:]) == header:
                        # Found header, read rest of frame
                        remaining = frame_size - 2
                        data = self.ser.read(remaining)

                        if debug:
                            print(f"DEBUG: Found header! Reading {remaining} more bytes")
                            print(f"DEBUG: Got {len(data)} bytes: {data.hex()}")

                        if len(data) == remaining:
                            frame = bytes(buffer[-2:]) + data
                            if debug:
                                print(f"DEBUG: Complete frame: {frame.hex()}")
                            return self.parse_frame(frame)
            else:
                if debug and attempts % 20 == 0:
                    print(f"DEBUG: No data waiting (attempt {attempts})")
                # Return None quickly if no data (for bulk reads)
                return None

            attempts += 1

        if debug:
            print(f"DEBUG: Timeout after {max_attempts} attempts, buffer: {buffer[:20].hex() if len(buffer) > 0 else 'empty'}")
        return None

    def parse_frame(self, frame):
        """Parse a data frame and extract distance and signal strength"""
        if not frame:
            return None

        # Check frame type by header
        if frame[:2] == self.HEADER_HIGH_RATE:
            # 6-byte high frame rate format
            if len(frame) != 6:
                return None
            distance = struct.unpack('<H', frame[2:4])[0]  # cm
            strength = struct.unpack('<H', frame[4:6])[0]

        elif frame[:2] == self.HEADER_STANDARD:
            # 9-byte standard format
            if len(frame) != 9:
                return None
            distance = struct.unpack('<H', frame[2:4])[0]  # cm
            strength = struct.unpack('<H', frame[4:6])[0]
            # Verify checksum
            checksum_calc = sum(frame[:8]) & 0xFF
            checksum_recv = frame[8]
            if checksum_calc != checksum_recv:
                return None
        else:
            return None

        return {
            'distance_cm': distance,
            'distance_m': distance / 100.0,
            'strength': strength,
            'timestamp': time.time()
        }

    def _reader_thread_func(self):
        """Background thread that continuously reads frames from serial port"""
        frame_size = 6 if self.high_rate_mode else 9
        header = self.HEADER_HIGH_RATE if self.high_rate_mode else self.HEADER_STANDARD

        buffer = bytearray()

        while self.reading:
            try:
                # Read chunk of data
                if self.ser.in_waiting > 0:
                    chunk = self.ser.read(min(self.ser.in_waiting, 1024))
                    buffer.extend(chunk)

                    # Parse all complete frames from buffer
                    while len(buffer) >= frame_size:
                        # Look for header
                        header_idx = buffer.find(header)
                        if header_idx == -1:
                            # No header found, keep last byte in case it's start of header
                            buffer = buffer[-1:]
                            break

                        # Remove data before header
                        if header_idx > 0:
                            buffer = buffer[header_idx:]

                        # Check if we have a complete frame
                        if len(buffer) >= frame_size:
                            frame = bytes(buffer[:frame_size])
                            buffer = buffer[frame_size:]

                            # Parse and queue frame
                            parsed = self.parse_frame(frame)
                            if parsed:
                                try:
                                    self.frame_queue.put_nowait(parsed)
                                except queue.Full:
                                    # Queue full, drop oldest frame
                                    try:
                                        self.frame_queue.get_nowait()
                                        self.frame_queue.put_nowait(parsed)
                                    except:
                                        pass
                        else:
                            break
                else:
                    # No data available, small sleep
                    time.sleep(0.0001)

            except Exception as e:
                if self.reading:
                    print(f"Reader thread error: {e}")
                break

    def start_reading(self):
        """Start background reading thread"""
        if not self.reader_thread:
            self.reading = True
            self.reader_thread = threading.Thread(target=self._reader_thread_func, daemon=True)
            self.reader_thread.start()
            print(f"Started background reader thread")

    def stop_reading(self):
        """Stop background reading thread"""
        self.reading = False
        if self.reader_thread:
            self.reader_thread.join(timeout=1.0)
            self.reader_thread = None
            print("Stopped background reader thread")

    def read_frame_from_queue(self):
        """Read a frame from the queue (non-blocking)"""
        try:
            return self.frame_queue.get_nowait()
        except queue.Empty:
            return None

    def get_queue_size(self):
        """Get current number of frames in queue"""
        return self.frame_queue.qsize()

    def close(self):
        """Close serial connection"""
        self.stop_reading()
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Connection closed")


class TFA300Plotter:
    """Real-time plotter for TFA300 data"""

    def __init__(self, sensor, max_points=10000, debug=False, frame_rate=10000, resample_factor=10):
        self.sensor = sensor
        self.max_points = max_points
        self.debug = debug
        self.frame_rate = frame_rate
        self.frame_interval = 1.0 / frame_rate  # Time between frames in seconds
        self.resample_factor = resample_factor  # Group N samples into one bucket

        # Raw data buffers (more points since we resample)
        self.raw_times = deque(maxlen=max_points)
        self.raw_distances = deque(maxlen=max_points)
        self.raw_strengths = deque(maxlen=max_points)

        # Resampled data for plotting
        self.plot_times = deque(maxlen=max_points // resample_factor)
        self.dist_min = deque(maxlen=max_points // resample_factor)
        self.dist_max = deque(maxlen=max_points // resample_factor)
        self.dist_avg = deque(maxlen=max_points // resample_factor)
        self.strength_min = deque(maxlen=max_points // resample_factor)
        self.strength_max = deque(maxlen=max_points // resample_factor)
        self.strength_avg = deque(maxlen=max_points // resample_factor)

        self.start_time = time.time()
        self.frame_count = 0  # Total frames received
        self.sample_number = 0  # Sample counter for timestamp calculation
        self.last_fps_time = time.time()
        self.fps = 0

        # Setup plot
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(14, 10))
        self.fig.suptitle('TFA300 LIDAR Real-time Data', fontsize=14, fontweight='bold')

        # Distance plot with min/max/avg
        self.line1_min, = self.ax1.plot([], [], 'b-', linewidth=0.5, alpha=0.6, label='Min')
        self.line1_max, = self.ax1.plot([], [], 'r-', linewidth=0.5, alpha=0.6, label='Max')
        self.line1_avg, = self.ax1.plot([], [], 'g-', linewidth=1.5, label='Avg')
        self.ax1.fill_between([], [], [], alpha=0.2, color='gray', label='Range')
        self.ax1.set_ylabel('Distance (m)', fontsize=10)
        self.ax1.set_xlabel('Time (s)', fontsize=10)
        self.ax1.grid(True, alpha=0.3)
        self.ax1.set_title(f'Distance Measurement (resampled {self.resample_factor}x)')
        self.ax1.legend(loc='upper right')

        # Signal strength plot with min/max/avg
        self.line2_min, = self.ax2.plot([], [], 'b-', linewidth=0.5, alpha=0.6, label='Min')
        self.line2_max, = self.ax2.plot([], [], 'r-', linewidth=0.5, alpha=0.6, label='Max')
        self.line2_avg, = self.ax2.plot([], [], 'g-', linewidth=1.5, label='Avg')
        self.ax2.set_ylabel('Signal Strength', fontsize=10)
        self.ax2.set_xlabel('Time (s)', fontsize=10)
        self.ax2.grid(True, alpha=0.3)
        self.ax2.set_title(f'Signal Strength (resampled {self.resample_factor}x)')
        self.ax2.axhline(y=40, color='orange', linestyle='--', alpha=0.5, label='Low reliability threshold')
        self.ax2.legend(loc='upper right')

        plt.tight_layout()

    def resample_data(self):
        """Resample raw data into buckets with min/max/avg"""
        if len(self.raw_times) < self.resample_factor:
            return  # Not enough data yet

        # Clear resampled data
        self.plot_times.clear()
        self.dist_min.clear()
        self.dist_max.clear()
        self.dist_avg.clear()
        self.strength_min.clear()
        self.strength_max.clear()
        self.strength_avg.clear()

        # Convert to lists for easier indexing
        times = list(self.raw_times)
        distances = list(self.raw_distances)
        strengths = list(self.raw_strengths)

        # Process in buckets
        num_buckets = len(times) // self.resample_factor
        for i in range(num_buckets):
            start_idx = i * self.resample_factor
            end_idx = start_idx + self.resample_factor

            # Get bucket data
            bucket_times = times[start_idx:end_idx]
            bucket_dists = distances[start_idx:end_idx]
            bucket_strengths = strengths[start_idx:end_idx]

            # Calculate statistics
            self.plot_times.append(bucket_times[0])  # Use first timestamp in bucket
            self.dist_min.append(min(bucket_dists))
            self.dist_max.append(max(bucket_dists))
            self.dist_avg.append(sum(bucket_dists) / len(bucket_dists))
            self.strength_min.append(min(bucket_strengths))
            self.strength_max.append(max(bucket_strengths))
            self.strength_avg.append(sum(bucket_strengths) / len(bucket_strengths))

    def update_plot(self, frame_num):
        """Update plot with new data - processes multiple frames per update"""
        frames_read = 0

        debug_this_frame = self.debug and frame_num < 5  # Debug first 5 updates

        # Read all available frames from queue
        # The background thread is continuously filling this queue
        while True:
            data = self.sensor.read_frame_from_queue()
            if data:
                # Timestamp based on sample number and known frame rate
                # Each sample is exactly frame_interval apart
                timestamp = self.sample_number * self.frame_interval

                self.raw_times.append(timestamp)
                self.raw_distances.append(data['distance_m'])
                self.raw_strengths.append(data['strength'])

                self.sample_number += 1
                self.frame_count += 1
                frames_read += 1
            else:
                # No more frames available
                break

        if debug_this_frame:
            queue_size = self.sensor.get_queue_size()
            print(f"DEBUG: Update {frame_num}: Read {frames_read} frames, sample# {self.sample_number}, queue: {queue_size}")

        # Resample data for plotting
        self.resample_data()

        # Update FPS calculation (based on actual frames processed)
        now = time.time()
        if now - self.last_fps_time >= 1.0:
            self.fps = self.frame_count / (now - self.last_fps_time)
            self.frame_count = 0
            self.last_fps_time = now

        # Update plots if we have resampled data
        if len(self.plot_times) > 0:
            times_list = list(self.plot_times)

            # Update distance plot
            self.line1_min.set_data(times_list, list(self.dist_min))
            self.line1_max.set_data(times_list, list(self.dist_max))
            self.line1_avg.set_data(times_list, list(self.dist_avg))
            self.ax1.relim()
            self.ax1.autoscale_view()

            # Update signal strength plot
            self.line2_min.set_data(times_list, list(self.strength_min))
            self.line2_max.set_data(times_list, list(self.strength_max))
            self.line2_avg.set_data(times_list, list(self.strength_avg))
            self.ax2.relim()
            self.ax2.autoscale_view()

            # Update title with stats
            if len(self.raw_distances) > 0:
                avg_dist = sum(self.raw_distances) / len(self.raw_distances)
                avg_str = sum(self.raw_strengths) / len(self.raw_strengths)
                time_span = times_list[-1] - times_list[0] if len(times_list) > 1 else 0
                self.fig.suptitle(
                    f'TFA300 LIDAR Real-time Data | '
                    f'FPS: {self.fps:.0f} Hz | '
                    f'Time span: {time_span:.2f}s | '
                    f'Avg Dist: {avg_dist:.3f}m | '
                    f'Avg Strength: {avg_str:.0f}',
                    fontsize=12, fontweight='bold'
                )

        return self.line1_min, self.line1_max, self.line1_avg, self.line2_min, self.line2_max, self.line2_avg

    def run(self):
        """Start the plotting animation"""
        ani = animation.FuncAnimation(
            self.fig, self.update_plot,
            interval=10,  # Update every 10ms
            blit=False,
            cache_frame_data=False
        )
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='TFA300 LIDAR Reader and Plotter',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        'port',
        nargs='?',
        default='/dev/ttyUSB0',
        help='Serial port device (default: /dev/ttyUSB0)'
    )
    parser.add_argument(
        '--no-config',
        action='store_true',
        help='Skip 10kHz configuration (use current sensor settings)'
    )
    parser.add_argument(
        '--setup',
        action='store_true',
        help='Configure sensor for 10kHz and exit (requires power cycle after)'
    )
    parser.add_argument(
        '--baud',
        type=int,
        default=921600,
        choices=[115200, 921600],
        help='Baud rate to use (default: 921600 for 10kHz)'
    )
    parser.add_argument(
        '--max-points',
        type=int,
        default=10000,
        help='Maximum number of raw samples to buffer (default: 10000 = 1 second at 10kHz)'
    )
    parser.add_argument(
        '--resample',
        type=int,
        default=10,
        help='Resample factor - group N samples into buckets for min/max/avg (default: 10)'
    )
    parser.add_argument(
        '--debug',
        action='store_true',
        help='Enable debug output'
    )

    args = parser.parse_args()

    print("TFA300 LIDAR Reader and Plotter")
    print("================================\n")

    # Setup mode: configure sensor and exit
    if args.setup:
        print("SETUP MODE: Configuring sensor for 10kHz operation")
        print("This will configure the sensor to 921600 baud with 10kHz frame rate")
        print()

        # Always connect at 115200 for setup
        sensor = TFA300(args.port, baudrate=115200)
        if not sensor.connect():
            print("ERROR: Failed to connect at 115200 baud")
            return 1

        # Send configuration
        print("Step 1: Disabling data output...")
        sensor.disable_data_output()
        time.sleep(0.3)
        if sensor.ser.in_waiting > 0:
            sensor.ser.read(sensor.ser.in_waiting)
            print("  Cleared buffer")

        print("Step 2: Setting 10kHz frame rate...")
        sensor.set_frame_rate(10000)
        time.sleep(0.2)

        print("Step 3: Setting 6-byte high frame rate format...")
        sensor.set_high_rate_format()
        time.sleep(0.2)

        print("Step 4: Setting 921600 baud rate...")
        sensor.set_baud_rate(921600)
        time.sleep(0.2)

        print("Step 5: Saving configuration...")
        sensor.save_configuration()
        time.sleep(0.2)

        print("Step 6: Re-enabling data output...")
        sensor.enable_data_output()
        time.sleep(0.2)

        sensor.close()

        print("\n" + "="*60)
        print("CONFIGURATION COMPLETE")
        print("="*60)
        print("\nIMPORTANT: Power cycle the sensor now for changes to take effect.")
        print("\nAfter power cycle, run:")
        print(f"  python3 tools/tfa300.py {args.port} --baud 921600")
        print("\nThe sensor will operate at:")
        print("  - Baud rate: 921600 (persists)")
        print("  - Frame rate: 10000 Hz (persists)")
        print("  - Frame format: 9-byte standard (reverts to default)")
        print("\nNote: Frame format doesn't persist, but 921600 baud")
        print("      can handle 10kHz with 9-byte frames (max ~12kHz)")
        print()
        return 0

    # Normal mode: connect and plot
    # Create sensor interface with specified baud rate
    sensor = TFA300(args.port, baudrate=args.baud)

    # Connect to sensor
    if not sensor.connect():
        print("Failed to connect to sensor")
        return 1

    # Auto-detect frame format
    print("Detecting frame format...")
    sensor.high_rate_mode = False  # Try standard first
    test_frame = sensor.read_frame(debug=False)
    if test_frame:
        print(f"  Detected: Standard 9-byte format (0x59 0x59 header)")
        print(f"  Test read: dist={test_frame['distance_m']:.2f}m, strength={test_frame['strength']}")
    else:
        # Try high rate mode
        sensor.high_rate_mode = True
        test_frame = sensor.read_frame(debug=False)
        if test_frame:
            print(f"  Detected: High rate 6-byte format (0x20 0x20 header)")
            print(f"  Test read: dist={test_frame['distance_m']:.2f}m, strength={test_frame['strength']}")
        else:
            print(f"  ERROR: No data in either format at {args.baud} baud")
            print(f"  Check:")
            print(f"    - Sensor is powered on")
            print(f"    - Baud rate matches sensor configuration")
            print(f"    - Sensor was power cycled after --setup")
            sensor.close()
            return 1

    # Start background reader thread to prevent buffer overflow
    print("Starting background reader thread...")
    sensor.start_reading()
    time.sleep(0.1)  # Let it start reading

    try:
        # Skip configuration in normal mode
        if not args.no_config and not args.setup:
            print("\nWARNING: Sensor configuration requires --setup mode and power cycle")
            print("Run with --setup first if sensor is not configured")
            print("Continuing with current sensor settings...\n")
            time.sleep(1.0)

        # Test read a few frames first if debug
        if args.debug:
            print("\n=== Debug: Testing frame reading ===")
            for i in range(5):
                print(f"\nAttempt {i+1}:")
                data = sensor.read_frame(debug=True)
                if data:
                    print(f"SUCCESS: {data}")
                else:
                    print("FAILED: No data")
                time.sleep(0.1)
            print("=== End debug test ===\n")
        else:
            # Quick connection test
            print("Testing sensor connection...")
            test_frame = sensor.read_frame(debug=False)
            if test_frame:
                print(f"  Connected: dist={test_frame['distance_m']:.2f}m, strength={test_frame['strength']}")
            else:
                print("  WARNING: No data received. Check baud rate and sensor configuration.")
                print("  If sensor was configured with --setup, ensure it was power cycled.")
            print()

        # Create plotter and run
        print("Starting real-time plot...")
        print(f"  Buffer: {args.max_points} samples ({args.max_points/10000:.1f}s at 10kHz)")
        print(f"  Resample: {args.resample}x (plots {args.max_points//args.resample} points)")
        print("Close the plot window to exit.\n")

        # Use known frame rate (10kHz if configured, or lower for default)
        plotter = TFA300Plotter(
            sensor,
            max_points=args.max_points,
            debug=args.debug,
            frame_rate=10000,
            resample_factor=args.resample
        )
        plotter.run()

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        sensor.close()

    return 0


if __name__ == '__main__':
    sys.exit(main())
