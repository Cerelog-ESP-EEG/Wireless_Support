import asyncio
import struct
import time
from bleak import BleakClient, BleakScanner
from pylsl import StreamInfo, StreamOutlet, local_clock

# --- Configuration ---
# MUST MATCH V2_Devices_BLE_hostfw.ino
BLE_DEVICE_NAME = "CERELOG_EEG"
SERVICE_UUID   = "7a1e8b00-9c3d-4b7e-8f21-6d0a5c2e91f3"
DATA_CHAR_UUID = "7a1e8b01-9c3d-4b7e-8f21-6d0a5c2e91f3"   # notify: EEG packets
CTRL_CHAR_UUID = "7a1e8b02-9c3d-4b7e-8f21-6d0a5c2e91f3"   # write:  start/stop

CMD_START = 0x01
CMD_STOP  = 0x00

SCAN_TIMEOUT = 8.0
STATS_PERIOD = 5.0      # seconds between link-quality printouts

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

# BLE carries far less than TCP, so the firmware coalesces several packets into
# one notification. Deriving LSL timestamps from the device's own sample clock
# (the 4-byte ms field in every packet) instead of the arrival time keeps the
# stream evenly spaced instead of arriving in ~50 ms lumps. Set False to fall
# back to arrival-time stamping, exactly like the WiFi script does.
USE_DEVICE_TIMESTAMPS = True


class BleLink:
    """Same job as TcpSerial in the WiFi script: hand the parser a byte stream.

    BLE notifications land in the buffer from bleak's callback; the main loop
    drains it. Notifications are delivered in order, so a packet split across
    two notifications reassembles correctly.
    """

    def __init__(self):
        self.buffer = bytearray()
        self.client = None
        self.notifications = 0
        self.bytes_in = 0
        self.disconnected = False

    def on_notify(self, _sender, data):
        self.buffer.extend(data)
        self.notifications += 1
        self.bytes_in += len(data)

    def on_disconnect(self, _client):
        self.disconnected = True
        print("\n!!! Device disconnected !!!")

    def read_all(self):
        chunk = self.buffer
        self.buffer = bytearray()
        return chunk

    @property
    def in_waiting(self):
        return len(self.buffer)


async def find_device():
    """BLE stand-in for the WiFi script's UDP broadcast discovery."""
    print(f"Scanning for BLE device '{BLE_DEVICE_NAME}'...")
    target_uuid = SERVICE_UUID.lower()

    def matches(device, adv):
        name = adv.local_name or device.name or ""
        if name.upper() == BLE_DEVICE_NAME.upper():
            return True
        return target_uuid in [u.lower() for u in (adv.service_uuids or [])]

    for attempt in range(3):
        device = await BleakScanner.find_device_by_filter(matches, timeout=SCAN_TIMEOUT)
        if device:
            print(f"Found Device: {device.address}  ({device.name})")
            return device
        print(f"  ...not found (attempt {attempt + 1}/3)")
    return None


def convert_to_microvolts(raw_val):
    scale_factor = (2 * HARDWARE_VREF / HARDWARE_GAIN) / (2**24)
    return raw_val * scale_factor * 1000000 * GUI_CORRECTION_FACTOR


async def main():
    print("Creating LSL Stream Outlet...")
    info = StreamInfo('Cerelog_EEG', 'EEG', ADS1299_NUM_CHANNELS, SAMPLING_RATE_HZ, 'float32', 'cerelog_uid_1234')
    outlet = StreamOutlet(info)

    device = await find_device()
    if not device:
        device = input("Could not find device. Enter BLE address: ").strip()
        if not device:
            return

    link = BleLink()

    async with BleakClient(device, timeout=20.0, disconnected_callback=link.on_disconnect) as client:
        print(">>> CONNECTED <<<")

        try:
            mtu = int(client.mtu_size)
        except Exception:
            mtu = 23
        if mtu < 23:
            mtu = 23
        print(f"Negotiated ATT MTU: {mtu} ({max(1, (mtu - 3) // DATA_PACKET_TOTAL_SIZE)} packet(s) per notification)")

        await client.start_notify(DATA_CHAR_UUID, link.on_notify)

        # Tell the firmware to start the ADC, and hand it the MTU we negotiated
        # so it never stages a notification the link would truncate. This write
        # is the BLE equivalent of the TCP connect in the WiFi script.
        await client.write_gatt_char(
            CTRL_CHAR_UUID,
            bytes([CMD_START, (mtu >> 8) & 0xFF, mtu & 0xFF]),
            response=True,
        )

        # --- FILTER STATE ---
        R = 0.995
        prev_x = [0.0] * ADS1299_NUM_CHANNELS
        prev_y = [0.0] * ADS1299_NUM_CHANNELS
        first_sample = True

        # --- LINK QUALITY / DROP ACCOUNTING ---
        pushed = 0
        bad_checksum = 0
        resync_bytes = 0
        lost_samples = 0        # gaps in the device's own sample clock
        last_device_ms = None
        clock_offset = None     # local_clock() - device_ms/1000, set on first packet
        last_stats = time.time()

        buffer = bytearray()
        start_marker = DATA_PACKET_START_MARKER.to_bytes(2, 'big')
        end_marker = DATA_PACKET_END_MARKER.to_bytes(2, 'big')

        print("\n>>> STREAMING STARTED >>>")

        try:
            while True:
                if link.disconnected:
                    break

                # Pull data
                if link.in_waiting > 0:
                    buffer.extend(link.read_all())
                else:
                    # Yield to the event loop so notifications can be delivered
                    await asyncio.sleep(0.001)
                    continue

                # Parse
                while True:
                    start_idx = buffer.find(start_marker)
                    if start_idx == -1:
                        # Discard junk except last byte just in case
                        if len(buffer) > 0:
                            resync_bytes += len(buffer) - 1
                            buffer = buffer[-1:]
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
                            device_ms = int.from_bytes(potential_packet[3:7], byteorder='big')

                            # Any jump bigger than one sample period is data the
                            # BLE link could not carry - count it, never fake it.
                            if last_device_ms is not None:
                                delta_ms = device_ms - last_device_ms
                                missing = int(round(delta_ms / (1000.0 * SAMPLE_PERIOD))) - 1
                                if missing > 0:
                                    lost_samples += missing
                            last_device_ms = device_ms

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

                            if USE_DEVICE_TIMESTAMPS:
                                if clock_offset is None:
                                    clock_offset = local_clock() - device_ms / 1000.0
                                outlet.push_sample(lsl_sample, clock_offset + device_ms / 1000.0)
                            else:
                                outlet.push_sample(lsl_sample)
                            pushed += 1

                            # Remove processed packet
                            buffer = buffer[start_idx + DATA_PACKET_TOTAL_SIZE:]
                            continue
                        else:
                            bad_checksum += 1

                    # If marker found but valid packet not formed, move forward 1 byte
                    resync_bytes += start_idx + 1
                    buffer = buffer[start_idx + 1:]

                # --- Link quality report ---
                now = time.time()
                if now - last_stats >= STATS_PERIOD:
                    elapsed = now - last_stats
                    last_stats = now
                    print(f"[STATS] {pushed} samples pushed | {pushed / max(elapsed, 1e-9):.0f} Hz recent "
                          f"| lost={lost_samples} bad_checksum={bad_checksum} resync_bytes={resync_bytes} "
                          f"| notifications={link.notifications}")
                    pushed = 0

                await asyncio.sleep(0)

        except KeyboardInterrupt:
            print("Stopping...")
        finally:
            print(f"Session totals: lost={lost_samples} bad_checksum={bad_checksum} resync_bytes={resync_bytes}")
            try:
                if client.is_connected:
                    await client.write_gatt_char(CTRL_CHAR_UUID, bytes([CMD_STOP]), response=False)
                    await client.stop_notify(DATA_CHAR_UUID)
            except Exception:
                pass


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Stopping...")
