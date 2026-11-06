import socket
import struct
import time
from pylsl import StreamInfo, StreamOutlet, local_clock

# --- Configuration ---
DEVICE_PORT = 1112      
DISCOVERY_PORT = 4445   

# --- Packet Constants (MUST MATCH FIRMWARE) ---
SAMPLING_RATE_HZ = 250.0
SAMPLE_PERIOD = 1.0 / SAMPLING_RATE_HZ

# Hardware Physics
HARDWARE_VREF = 4.50
HARDWARE_GAIN = 24
GUI_CORRECTION_FACTOR = 0.94 * 24.0 / 10 / 2
# Old Gui Correction GUI_CORRECTION_FACTOR = 0.6133 * 24.0 

# Protocol Markers
DATA_PACKET_START_MARKER = 0xABCD
DATA_PACKET_END_MARKER = 0xDCBA
DATA_PACKET_TOTAL_SIZE = 37 # 2 Start + 1 Len + 4 Time + 27 Data + 1 Sum + 2 End
PACKET_IDX_LENGTH = 2
PACKET_IDX_CHECKSUM = 34

ADS1299_NUM_CHANNELS = 8
ADS1299_NUM_STATUS_BYTES = 3
ADS1299_BYTES_PER_CHANNEL = 3

class TcpSerial:
    def __init__(self, target_ip, port):
        self.sock = None
        self.buffer = bytearray()
        self.target_ip = target_ip
        self.port = port
        self.connect()

    def connect(self):
        print(f"Connecting to {self.target_ip}:{self.port}...")
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0) 
            self.sock.connect((self.target_ip, self.port))
            # Critical: Disable Nagle's algorithm for realtime streaming
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(">>> CONNECTED <<<")
        except Exception as e:
            print(f"Connection Failed: {e}")
            raise e

    def read(self, size):
        # Fetch from socket if buffer is low
        try:
            # We try to read more than needed to keep buffer full
            if len(self.buffer) < size:
                chunk = self.sock.recv(4096)
                if chunk:
                    self.buffer.extend(chunk)
        except socket.timeout:
            pass # Return what we have
        except Exception:
            return b''

        if len(self.buffer) > 0:
            # Return requested amount or whatever is there
            pull_size = min(len(self.buffer), size)
            ret = self.buffer[:pull_size]
            self.buffer = self.buffer[pull_size:]
            return bytes(ret)
        return b''

    @property
    def in_waiting(self):
        # Peek at socket to see if data is ready
        try:
            self.sock.setblocking(False)
            try:
                chunk = self.sock.recv(4096)
                if chunk: self.buffer.extend(chunk)
            except BlockingIOError:
                pass # No data right now
            except Exception:
                pass
            self.sock.setblocking(True) # Restore blocking
        except Exception:
            pass
        return len(self.buffer)

    def close(self):
        if self.sock: self.sock.close()

def find_device_ip():
    print(f"Scanning UDP port {DISCOVERY_PORT}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(1.0)
    try: sock.bind(('', 0))
    except: pass

    for i in range(3): 
        try:
            sock.sendto(b"CERELOG_FIND_ME", ('<broadcast>', DISCOVERY_PORT))
            start = time.time()
            while time.time() - start < 1.0:
                try:
                    data, addr = sock.recvfrom(1024)
                    if b"CERELOG_HERE" in data:
                        print(f"Found Device: {addr[0]}")
                        sock.close()
                        return addr[0]
                except socket.timeout: break
        except Exception: pass
    sock.close()
    return None

def convert_to_microvolts(raw_val):
    scale_factor = (2 * HARDWARE_VREF / HARDWARE_GAIN) / (2**24)
    return raw_val * scale_factor * 1000000 * GUI_CORRECTION_FACTOR

def main():
    print("Creating LSL Stream Outlet...")
    info = StreamInfo('Cerelog_EEG', 'EEG', ADS1299_NUM_CHANNELS, SAMPLING_RATE_HZ, 'float32', 'cerelog_uid_1234')
    outlet = StreamOutlet(info)
    
    target_ip = find_device_ip()
    if not target_ip:
        target_ip = input("Could not find device. Enter IP: ").strip()

    ser = None
    try:
        ser = TcpSerial(target_ip, DEVICE_PORT)
    except:
        return

    # --- FILTER STATE ---
    R = 0.995
    prev_x = [0.0] * ADS1299_NUM_CHANNELS
    prev_y = [0.0] * ADS1299_NUM_CHANNELS
    first_sample = True

    buffer = bytearray()
    start_marker = DATA_PACKET_START_MARKER.to_bytes(2, 'big')
    end_marker = DATA_PACKET_END_MARKER.to_bytes(2, 'big')
    
    print("\n>>> STREAMING STARTED >>>")
    
    try:
        while True:
            # Pull data
            if ser.in_waiting > 0:
                buffer.extend(ser.read(ser.in_waiting))
            else:
                # Small sleep to prevent CPU hogging
                time.sleep(0.001)
                continue
            
            # Parse
            while True:
                start_idx = buffer.find(start_marker)
                if start_idx == -1: 
                    # Discard junk except last byte just in case
                    if len(buffer) > 0: buffer = buffer[-1:]
                    break

                # Do we have a full packet?
                if len(buffer) < start_idx + DATA_PACKET_TOTAL_SIZE:
                    break # Wait for more data

                potential_packet = buffer[start_idx : start_idx + DATA_PACKET_TOTAL_SIZE]
                
                if potential_packet.endswith(end_marker):
                    # Checksum Validation
                    payload = potential_packet[2:34] # Len + Time + Data
                    calc_sum = sum(payload) & 0xFF
                    recv_sum = potential_packet[PACKET_IDX_CHECKSUM]

                    if calc_sum == recv_sum:
                        ads_data = potential_packet[7:34] 
                        lsl_sample = []
                        
                        for ch in range(ADS1299_NUM_CHANNELS):
                            idx = ADS1299_NUM_STATUS_BYTES + ch * ADS1299_BYTES_PER_CHANNEL
                            raw_val = int.from_bytes(ads_data[idx : idx + ADS1299_BYTES_PER_CHANNEL], byteorder='big', signed=True)
                            
                            current_x = convert_to_microvolts(raw_val)
                            
                            # DC Blocker
                            if first_sample: current_y = 0.0
                            else: current_y = current_x - prev_x[ch] + (R * prev_y[ch])
                            
                            prev_x[ch] = current_x
                            prev_y[ch] = current_y
                            lsl_sample.append(current_y)
                        
                        first_sample = False
                        outlet.push_sample(lsl_sample)
                        
                        # Remove processed packet
                        buffer = buffer[start_idx + DATA_PACKET_TOTAL_SIZE:]
                        continue
                
                # If marker found but valid packet not formed, move forward 1 byte
                buffer = buffer[start_idx + 1:]

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        if ser: ser.close()

if __name__ == "__main__":
    main()
