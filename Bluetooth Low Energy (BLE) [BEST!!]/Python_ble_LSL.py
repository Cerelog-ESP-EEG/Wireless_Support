import asyncio
import struct
import time
from bleak import BleakClient, BleakScanner
from pylsl import StreamInfo, StreamOutlet, local_clock

# --- Configuration ---
# MUST MATCH the firmware:
#   V1_Devices_BLE_hostfw.ino    ->  CERELOG_EEG      (8 channels)
#   V2_Devices_BLE_hostfw.ino    ->  CERELOG_EEG      (8 channels)
#   16CH_Devices_BLE_hostfw.ino  ->  CERELOG_EEG_16  (16 channels)
#
# All three boards use the SAME service and characteristic UUIDs, so one scan
# filter finds any of them. Only the advertised name differs, and that is what
# says how many channels the packets carry - the LSL outlet has to be sized
# before the first sample arrives, so it cannot be worked out from the data
# after the fact. (If a board is somehow matched by service UUID with no name
# available, the channel count is sniffed from the first packet instead; see
# sniff_channel_count.)
BOARD_CHANNELS_BY_NAME = {
    "CERELOG_EEG":    8,    # V1 and V2
    "CERELOG_EEG_16": 16,   # 16-channel
}

SERVICE_UUID   = "7a1e8b00-9c3d-4b7e-8f21-6d0a5c2e91f3"
DATA_CHAR_UUID = "7a1e8b01-9c3d-4b7e-8f21-6d0a5c2e91f3"   # notify: EEG packets
CTRL_CHAR_UUID = "7a1e8b02-9c3d-4b7e-8f21-6d0a5c2e91f3"   # write:  start/stop

CMD_START = 0x01
CMD_STOP  = 0x00

SCAN_TIMEOUT = 8.0
STATS_PERIOD = 5.0      # seconds between link-quality printouts
SNIFF_TIMEOUT = 5.0     # how long to wait for a packet when sniffing the width

# --- Packet Constants (MUST MATCH FIRMWARE) ---
# Every board samples at 250 Hz. The 16-channel board's 125 Hz downsampling
# existed only in its USB firmware, where BrainFlow's 16-channel serial board
# forces that rate; over BLE it streams the native 250 SPS like the others.
SAMPLING_RATE_HZ = 250.0
SAMPLE_PERIOD = 1.0 / SAMPLING_RATE_HZ

# Hardware Physics
# Both ADS1299s on the 16-channel board run the same gain of 24 as the single
# ADC on V1/V2, so one scale factor covers every channel on every board.
HARDWARE_VREF = 4.50
HARDWARE_GAIN = 24
GUI_CORRECTION_FACTOR = 0.94 * 24.0 / 10 / 2
# Old Gui Correction GUI_CORRECTION_FACTOR = 0.6133 * 24.0

# Protocol Markers
DATA_PACKET_START_MARKER = 0xABCD
DATA_PACKET_END_MARKER = 0xDCBA

ADS1299_NUM_STATUS_BYTES = 3
ADS1299_BYTES_PER_CHANNEL = 3

# Fixed offsets, identical on all three boards:
#   [0:2] start marker | [2] message length | [3:7] uint32 BE device ms
PACKET_IDX_LENGTH = 2
PACKET_IDX_TIMESTAMP = 3
PACKET_IDX_DATA = 7

# BLE carries far less than TCP, so the firmware coalesces several packets into
# one notification. Deriving LSL timestamps from the device's own sample clock
# (the 4-byte ms field in every packet) instead of the arrival time keeps the
# stream evenly spaced instead of arriving in ~50 ms lumps. Set False to fall
# back to arrival-time stamping, exactly like the WiFi script does.
USE_DEVICE_TIMESTAMPS = True


class PacketFormat:
    """Packet geometry for a given channel count.

    The three firmwares share one frame layout and differ only in how many
    24-bit channel words sit between the status word and the checksum, so all
    the sizes follow from the channel count:

        8 channels  -> 27 data bytes, length field 31, 37-byte packet  (V1/V2)
        16 channels -> 51 data bytes, length field 55, 61-byte packet  (16CH)
    """

    def __init__(self, num_channels):
        self.num_channels = num_channels
        self.data_bytes = ADS1299_NUM_STATUS_BYTES + num_channels * ADS1299_BYTES_PER_CHANNEL
        self.msg_length = 4 + self.data_bytes          # 4-byte timestamp + data
        self.total_size = 2 + 1 + self.msg_length + 1 + 2
        self.idx_checksum = PACKET_IDX_DATA + self.data_bytes
        self.idx_end_marker = self.idx_checksum + 1

    def __str__(self):
        return (f"{self.num_channels} channels, {self.total_size}-byte packets "
                f"(length field {self.msg_length})")


def channels_from_msg_length(msg_length):
    """Inverse of PacketFormat: channel count implied by a packet's length byte.

    Returns None if the value is not a whole number of 24-bit channels, which
    means the byte was not really a length field.
    """
    data_bytes = msg_length - 4 - ADS1299_NUM_STATUS_BYTES
    if data_bytes <= 0 or data_bytes % ADS1299_BYTES_PER_CHANNEL != 0:
        return None
    return data_bytes // ADS1299_BYTES_PER_CHANNEL


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
    """BLE stand-in for the WiFi script's UDP broadcast discovery.

    Returns (device, advertised_name). The name is what identifies the board
    variant, so it is passed back rather than thrown away; it is None when the
    device was matched by service UUID alone.
    """
    names = "' / '".join(BOARD_CHANNELS_BY_NAME)
    print(f"Scanning for BLE device '{names}'...")
    target_uuid = SERVICE_UUID.lower()
    known_names = {n.upper() for n in BOARD_CHANNELS_BY_NAME}

    # Holds the advertised name of whichever device matched, so the variant can
    # be identified even on backends where device.name comes back empty.
    matched = {}

    def matches(device, adv):
        name = adv.local_name or device.name or ""
        if name.upper() in known_names:
            matched[device.address] = name
            return True
        if target_uuid in [u.lower() for u in (adv.service_uuids or [])]:
            matched[device.address] = name or None
            return True
        return False

    for attempt in range(3):
        device = await BleakScanner.find_device_by_filter(matches, timeout=SCAN_TIMEOUT)
        if device:
            name = matched.get(device.address) or device.name
            print(f"Found Device: {device.address}  ({name})")
            return device, name
        print(f"  ...not found (attempt {attempt + 1}/3)")
    return None, None


def channels_from_name(name):
    """Channel count for an advertised device name, or None if unrecognised."""
    if not name:
        return None
    return BOARD_CHANNELS_BY_NAME.get(name.strip().upper())


async def sniff_channel_count(link, buffer, start_marker):
    """Fallback for a board whose advertised name told us nothing.

    Waits for the first framed packet and reads its length field. Whatever is
    consumed from the link is appended to `buffer`, so the caller can hand that
    same buffer straight to the parse loop with no bytes lost.

    Returns the channel count, or None on timeout.
    """
    print("Device name did not identify the board - sniffing the packet width...")
    deadline = time.time() + SNIFF_TIMEOUT
    while time.time() < deadline:
        if link.disconnected:
            return None
        if link.in_waiting > 0:
            buffer.extend(link.read_all())
        start_idx = buffer.find(start_marker)
        # Need the start marker plus the length byte that follows it.
        if start_idx != -1 and len(buffer) >= start_idx + PACKET_IDX_LENGTH + 1:
            channels = channels_from_msg_length(buffer[start_idx + PACKET_IDX_LENGTH])
            if channels is not None:
                return channels
            # Not a real frame - step past this marker and keep looking.
            del buffer[:start_idx + 1]
            continue
        await asyncio.sleep(0.01)
    return None


def make_outlet(fmt):
    """Build the LSL outlet for a board of this width.

    Name, type, rate, value format and source id are unchanged from the
    8-channel-only version of this script, so existing consumers resolve the
    stream exactly as before; only channel_count differs on a 16-channel board.
    """
    info = StreamInfo('Cerelog_EEG', 'EEG', fmt.num_channels, SAMPLING_RATE_HZ,
                      'float32', 'cerelog_uid_1234')
    return StreamOutlet(info)


def convert_to_microvolts(raw_val):
    scale_factor = (2 * HARDWARE_VREF / HARDWARE_GAIN) / (2**24)
    return raw_val * scale_factor * 1000000 * GUI_CORRECTION_FACTOR


async def main():
    device, adv_name = await find_device()
    if not device:
        device = input("Could not find device. Enter BLE address: ").strip()
        if not device:
            return
        adv_name = None

    # Channel count from the advertised name. Every current firmware puts its
    # name in the scan response, so this is the path all three boards take, and
    # the outlet is created here - before connecting - exactly as it was when
    # this script only knew about 8-channel boards.
    num_channels = channels_from_name(adv_name)

    fmt = None
    outlet = None
    if num_channels is not None:
        fmt = PacketFormat(num_channels)
        print(f"Board format: {fmt}")
        print("Creating LSL Stream Outlet...")
        outlet = make_outlet(fmt)

    link = BleLink()

    async with BleakClient(device, timeout=20.0, disconnected_callback=link.on_disconnect) as client:
        print(">>> CONNECTED <<<")

        # On BlueZ (Linux) mtu_size is only populated by _acquire_mtu(); without
        # this it warns and reports 23, which makes the firmware split every
        # packet across two notifications and drop samples. macOS and Windows
        # report the real MTU directly and have no such method, so the hasattr
        # guard makes this a no-op there.
        if hasattr(client, "_acquire_mtu"):
            try:
                await client._acquire_mtu()
            except Exception:
                pass

        try:
            mtu = int(client.mtu_size)
        except Exception:
            mtu = 23
        if mtu < 23:
            mtu = 23
        print(f"Negotiated ATT MTU: {mtu}")

        await client.start_notify(DATA_CHAR_UUID, link.on_notify)

        # Tell the firmware to start the ADC, and hand it the MTU we negotiated
        # so it never stages a notification the link would truncate. This write
        # is the BLE equivalent of the TCP connect in the WiFi script.
        await client.write_gatt_char(
            CTRL_CHAR_UUID,
            bytes([CMD_START, (mtu >> 8) & 0xFF, mtu & 0xFF]),
            response=True,
        )

        buffer = bytearray()
        start_marker = DATA_PACKET_START_MARKER.to_bytes(2, 'big')
        end_marker = DATA_PACKET_END_MARKER.to_bytes(2, 'big')

        # Only reached when the scan matched by service UUID with no usable
        # name: work the width out from the stream, then build the outlet. Any
        # bytes consumed here stay in `buffer` and are parsed below.
        if fmt is None:
            num_channels = await sniff_channel_count(link, buffer, start_marker)
            if num_channels is None:
                print("No recognisable packet arrived - giving up. Is the firmware up to date?")
                try:
                    await client.write_gatt_char(CTRL_CHAR_UUID, bytes([CMD_STOP]), response=False)
                    await client.stop_notify(DATA_CHAR_UUID)
                except Exception:
                    pass
                return
            fmt = PacketFormat(num_channels)
            print(f"Board format: {fmt}")
            print("Creating LSL Stream Outlet...")
            outlet = make_outlet(fmt)

        print(f"{max(1, (mtu - 3) // fmt.total_size)} packet(s) per notification")

        # True only while `buffer` may already hold packets that arrived during
        # the sniff above. Without it the parse pass below would be skipped
        # until the next notification landed.
        pending_parse = len(buffer) > 0

        # --- FILTER STATE ---
        R = 0.995
        prev_x = [0.0] * fmt.num_channels
        prev_y = [0.0] * fmt.num_channels
        first_sample = True

        # --- LINK QUALITY / DROP ACCOUNTING ---
        pushed = 0
        bad_checksum = 0
        resync_bytes = 0
        lost_samples = 0        # gaps in the device's own sample clock
        last_device_ms = None
        clock_offset = None     # local_clock() - device_ms/1000, set on first packet
        last_stats = time.time()

        print("\n>>> STREAMING STARTED >>>")

        try:
            while True:
                if link.disconnected:
                    break

                # Pull data
                if link.in_waiting > 0:
                    buffer.extend(link.read_all())
                elif not pending_parse:
                    # Yield to the event loop so notifications can be delivered
                    await asyncio.sleep(0.001)
                    continue
                pending_parse = False

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
                    if len(buffer) < start_idx + fmt.total_size:
                        break # Wait for more data

                    potential_packet = buffer[start_idx : start_idx + fmt.total_size]

                    # The length field is the board's own statement of how wide
                    # the packet is. Checking it rejects a frame from a board of
                    # a different width instead of silently misreading it.
                    if (potential_packet[PACKET_IDX_LENGTH] == fmt.msg_length
                            and potential_packet.endswith(end_marker)):
                        # Checksum Validation
                        payload = potential_packet[PACKET_IDX_LENGTH : fmt.idx_checksum] # Len + Time + Data
                        calc_sum = sum(payload) & 0xFF
                        recv_sum = potential_packet[fmt.idx_checksum]

                        if calc_sum == recv_sum:
                            device_ms = int.from_bytes(
                                potential_packet[PACKET_IDX_TIMESTAMP : PACKET_IDX_TIMESTAMP + 4],
                                byteorder='big')

                            # Any jump bigger than one sample period is data the
                            # BLE link could not carry - count it, never fake it.
                            if last_device_ms is not None:
                                delta_ms = device_ms - last_device_ms
                                missing = int(round(delta_ms / (1000.0 * SAMPLE_PERIOD))) - 1
                                if missing > 0:
                                    lost_samples += missing
                            last_device_ms = device_ms

                            ads_data = potential_packet[PACKET_IDX_DATA : fmt.idx_checksum]
                            lsl_sample = []

                            for ch in range(fmt.num_channels):
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
                            buffer = buffer[start_idx + fmt.total_size:]
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
