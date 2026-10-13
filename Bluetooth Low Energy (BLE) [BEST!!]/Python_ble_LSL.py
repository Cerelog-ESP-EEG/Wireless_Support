import asyncio
import json
import os
import socket
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

# --- Native OpenBCI GUI support (16-channel boards only) --------------------
# On top of the LSL outlet, a 16-channel board is also published as an OpenBCI
# Cyton+Daisy WiFi shield, which the stock OpenBCI GUI connects to natively.
#
# WHY THE WIFI SHIELD AND NOT A SERIAL PORT. The 16-channel USB firmware got
# itself listed in the GUI by presenting a real USB CDC device whose product
# string began with "VCP" - the GUI calls jSerialComm's getCommPorts() and keeps
# only ports whose name starts with "FT231X USB UART" or "VCP". A script cannot
# do that: the only serial endpoint a user-space process can create is a pty,
# jSerialComm enumerates macOS serial ports out of the IOKit registry (where a
# pty never appears), the GUI additionally drops every "tty*" name on macOS, and
# "VCP" comes from a USB descriptor a pty does not have. The WiFi shield is the
# GUI's other native 16-channel path and is pure network protocol, so it is the
# one a script can actually serve - and it carries the very same 33-byte
# packets, so the wire format below is the USB firmware's, unchanged.
#
# TWO DIFFERENCES FROM THE USB FIRMWARE'S EMUL OUTPUT, both required by
# BrainFlow's cyton_daisy_wifi reader rather than chosen:
#   * Pairing. The serial EMUL reader splits a 16-channel sample by the parity
#     of the counter byte (even = ch9-16 first). The WiFi reader instead pairs
#     two packets that carry the SAME counter, channels 1-8 first, and commits
#     the sample on the second one.
#   * Rate. The serial path had to average two conversions down to 125 Hz
#     because BrainFlow fixes that rate for the 16-channel serial board. The
#     WiFi board's rate is whatever the GUI is told, so 250 Hz goes out
#     untouched - set the GUI's WiFi sample rate to 250.
#
# Values on the wire are raw ADS1299 counts, NOT microvolts: BrainFlow applies
# 4.5 V / (2^23-1) / gain, and its 16-channel gain tracker defaults to 24 on
# every channel, which is exactly this hardware. So the GUI shows true
# microvolts without this script scaling anything. Note that means the GUI
# amplitudes are the physically correct ones and will NOT match the LSL
# stream's, which carries the fork-specific GUI_CORRECTION_FACTOR above.
OPENBCI_EMULATION = True

# BrainFlow builds its URLs as "http://" + <the IP you typed> + "/board", with
# no port, and validates that string with inet_pton - so it must be a bare IPv4
# address and the HTTP port is unavoidably 80. Binding 80 needs root; if that
# fails the script says so and carries on with LSL alone. Override the port here
# only for testing, since the GUI cannot be told about it.
OPENBCI_HTTP_HOST = os.environ.get("CERELOG_OBCI_HOST", "0.0.0.0")
OPENBCI_HTTP_PORT = int(os.environ.get("CERELOG_OBCI_PORT", "80"))

OBCI_PACKET_SIZE = 33
OBCI_START_BYTE = 0xA0
OBCI_STOP_BYTE_STANDARD = 0xC0
OBCI_NUM_CHANNELS_PER_PACKET = 8

# Bound on the staged backlog, in 16-channel samples. On a stalled socket the
# oldest pair is dropped rather than queued - the same drop-rather-than-stall
# policy the firmware uses on the BLE link, for the same reason.
OBCI_MAX_STAGED_SAMPLES = 250


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


class ObciWifiShieldEmulator:
    """Publishes the 16-channel stream as an OpenBCI Cyton+Daisy WiFi shield.

    Serves the four endpoints BrainFlow drives, in the order it drives them:

        GET  /board         board info; BrainFlow only needs a reply at all
        POST /tcp           {"ip","port",...} - we connect BACK to that address
        POST /command       {"command": "~4" | "d" | "~6" | "x..X" | ...}
        GET  /stream/start  begin pushing packets down the TCP connection
        GET  /stream/stop   stop

    The TCP connection is outbound by design: BrainFlow binds a server socket of
    its own and waits for the shield to dial in, which is why /tcp carries an
    address rather than returning one.
    """

    # ~N sample-rate commands, as the Cyton SDK defines them. The board always
    # streams 250 Hz, so only "~6" is consistent with what the GUI will be told.
    SAMPLE_RATE_COMMANDS = {"~0": 16000, "~1": 8000, "~2": 4000,
                            "~3": 2000, "~4": 1000, "~5": 500, "~6": 250}

    def __init__(self):
        self.http_server = None
        self.data_writer = None
        self.streaming = False
        self.seen_defaults_command = False
        self.sample_id = 0
        self.frames_sent = 0
        self.samples_dropped = 0
        self.client_addr = None
        self.last_command = None
        self.command_count = 0
        self._out = bytearray()
        self._wake = asyncio.Event()
        self._writer_task = None
        self._cap = OBCI_MAX_STAGED_SAMPLES * 2 * OBCI_PACKET_SIZE

    # --- lifecycle ---------------------------------------------------------
    async def start(self):
        """Bind the HTTP port. Returns False (having explained why) on failure.

        A failure here is never fatal: the LSL stream is the primary output and
        must keep running whatever the GUI side is doing.
        """
        try:
            self.http_server = await asyncio.start_server(
                self._serve_http, host=OPENBCI_HTTP_HOST, port=OPENBCI_HTTP_PORT)
        except PermissionError:
            print(f"OpenBCI GUI emulation DISABLED: no permission to bind port {OPENBCI_HTTP_PORT}.")
            print("  BrainFlow always talks to a WiFi shield on port 80, and ports below 1024")
            print("  need root. Re-run with 'sudo -E python3 Python_ble_LSL.py' to enable it.")
            print("  The LSL stream below is unaffected.")
            return False
        except OSError as e:
            print(f"OpenBCI GUI emulation DISABLED: cannot bind port {OPENBCI_HTTP_PORT} ({e}).")
            print("  Something else is already listening there. The LSL stream is unaffected.")
            return False

        print(f"OpenBCI GUI emulation listening on {OPENBCI_HTTP_HOST}:{OPENBCI_HTTP_PORT}")
        print("  In the GUI: CYTON (LIVE) -> WIFI -> STATIC IP, enter the address below,")
        print("  set CHANNEL COUNT to 16 and SAMPLE RATE to 250, then START SESSION.")
        for addr in self._advertised_addresses():
            print(f"    {addr}")
        return True

    @staticmethod
    def _advertised_addresses():
        """Addresses the GUI can be pointed at, loopback first."""
        addrs = ["127.0.0.1  (GUI on this same computer)"]
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))     # no traffic sent; just picks the route
            lan = s.getsockname()[0]
            s.close()
            if lan and lan != "127.0.0.1":
                addrs.append(f"{lan}  (GUI on another computer on this network)")
        except Exception:
            pass
        return addrs

    async def close(self):
        self.streaming = False
        if self._writer_task is not None:
            self._writer_task.cancel()
            try:
                await self._writer_task
            except (asyncio.CancelledError, Exception):
                pass
            self._writer_task = None
        await self._close_data_connection()
        if self.http_server is not None:
            self.http_server.close()
            try:
                await self.http_server.wait_closed()
            except Exception:
                pass
            self.http_server = None

    # --- HTTP --------------------------------------------------------------
    async def _serve_http(self, reader, writer):
        try:
            request_line = await reader.readline()
            if not request_line:
                return
            fields = request_line.decode("latin-1").split()
            if len(fields) < 2:
                return
            method, path = fields[0].upper(), fields[1].split("?", 1)[0]

            length = 0
            while True:
                line = await reader.readline()
                if line in (b"\r\n", b"\n", b""):
                    break
                name, _, value = line.decode("latin-1").partition(":")
                if name.strip().lower() == "content-length":
                    try:
                        length = int(value.strip())
                    except ValueError:
                        length = 0
            body = await reader.readexactly(length) if length > 0 else b""

            payload = await self._dispatch(method, path, body)
            response = (
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: application/json\r\n"
                f"Content-Length: {len(payload)}\r\n"
                "Connection: close\r\n"
                "\r\n"
            ).encode("latin-1") + payload
            writer.write(response)
            await writer.drain()
        except (ConnectionError, asyncio.IncompleteReadError):
            pass
        except Exception as e:
            print(f"[GUI] HTTP handler error: {e}")
        finally:
            try:
                writer.close()
            except Exception:
                pass

    async def _dispatch(self, method, path, body):
        if path == "/tcp" and method == "POST":
            return await self._handle_tcp(body)

        if path == "/command" and method == "POST":
            try:
                command = str(json.loads(body.decode("utf-8")).get("command", ""))
            except Exception:
                command = ""
            self.last_command = command
            self.command_count += 1
            # Nothing to apply. Gain, montage, bias and sample rate are all
            # fixed by the BLE firmware's register tables, exactly as the USB
            # firmware locked them with OBCI_ALLOW_GUI_HARDWARE_SETTINGS = 0:
            # the command is acknowledged so the GUI stays happy, and not one
            # ADS1299 register is touched. BrainFlow ignores this reply body.
            # Sample-rate sanity check. BrainFlow unconditionally sends "~4"
            # then "d" while preparing the session, and only afterwards does the
            # GUI send the rate the user picked - so a "~N" arriving AFTER "d"
            # is the user's choice, and anything but "~6" means the GUI will
            # plot our 250 Hz stream against the wrong time base.
            if command == "d":
                self.seen_defaults_command = True
            elif self.seen_defaults_command and command in self.SAMPLE_RATE_COMMANDS:
                asked = self.SAMPLE_RATE_COMMANDS[command]
                if asked != int(SAMPLING_RATE_HZ):
                    print(f"[GUI] *** WARNING: the GUI is set to {asked} Hz but this board "
                          f"streams {int(SAMPLING_RATE_HZ)} Hz. ***")
                    print(f"[GUI] *** Time axis, filters and FFT will all be wrong by "
                          f"{asked / SAMPLING_RATE_HZ:.3g}x. Set SAMPLE RATE to "
                          f"{int(SAMPLING_RATE_HZ)} in the GUI and restart the session. ***")
                else:
                    print(f"[GUI] sample rate confirmed at {asked} Hz")
                return json.dumps({"command": command}).encode()

            print(f"[GUI] command {command!r} acknowledged (board config is fixed in firmware)")
            return json.dumps({"command": command}).encode()

        if path == "/stream/start":
            self.sample_id = 0
            self.frames_sent = 0
            self.samples_dropped = 0
            self._out.clear()
            self.streaming = True
            print("[GUI] stream started")
            return b'{"stream":"started"}'

        if path == "/stream/stop":
            self.streaming = False
            self._out.clear()
            print(f"[GUI] stream stopped after {self.frames_sent} packets")
            return b'{"stream":"stopped"}'

        if path == "/board":
            return json.dumps({
                "board_connected": True,
                "board_type": "daisy",          # what a 16-channel shield reports
                "num_channels": 16,
                "gains": [24] * 16,
            }).encode()

        # /version, /all, /latency and anything else: a well-formed reply is all
        # BrainFlow checks for, and it never reads these bodies.
        return json.dumps({"ok": True, "path": path}).encode()

    async def _handle_tcp(self, body):
        """Open the reverse data connection before replying.

        BrainFlow polls for at most ~3 s after this POST returns, so connecting
        first removes the race entirely (and is what BrainFlow's own shield
        emulator does).
        """
        try:
            cfg = json.loads(body.decode("utf-8"))
            ip = str(cfg.get("ip"))
            port = int(cfg.get("port"))
        except Exception as e:
            print(f"[GUI] malformed /tcp request: {e}")
            return b'{"connected":false}'

        await self._close_data_connection()
        try:
            _, writer = await asyncio.wait_for(
                asyncio.open_connection(ip, port), timeout=5.0)
        except Exception as e:
            print(f"[GUI] could not connect back to {ip}:{port} - {e}")
            return b'{"connected":false}'

        self.data_writer = writer
        self.client_addr = f"{ip}:{port}"
        self._wake.clear()
        self._writer_task = asyncio.create_task(self._writer_loop(writer))
        print(f"[GUI] data connection open to {self.client_addr}")
        return json.dumps({"connected": True, "ip": ip, "port": port}).encode()

    async def _close_data_connection(self):
        writer, self.data_writer = self.data_writer, None
        if writer is not None:
            try:
                writer.close()
            except Exception:
                pass

    # --- data path ---------------------------------------------------------
    @staticmethod
    def _build_packet(sample_id, counts8):
        """One 33-byte Cyton frame.

        [0] 0xA0 | [1] counter | [2:26] 8 x 24-bit BE | [26:32] aux | [32] 0xC0

        The six aux bytes stay zero: no accelerometer is fitted, and with the
        standard 0xC0 stop byte BrainFlow reads them as accel but ignores a
        zero first axis, so the accel channels stay at 0 instead of showing
        noise.
        """
        pkt = bytearray(OBCI_PACKET_SIZE)
        pkt[0] = OBCI_START_BYTE
        pkt[1] = sample_id & 0xFF
        for i, value in enumerate(counts8):
            v = value & 0xFFFFFF           # back to two's-complement 24-bit
            base = 2 + 3 * i
            pkt[base] = (v >> 16) & 0xFF
            pkt[base + 1] = (v >> 8) & 0xFF
            pkt[base + 2] = v & 0xFF
        pkt[OBCI_PACKET_SIZE - 1] = OBCI_STOP_BYTE_STANDARD
        return pkt

    def push_sample(self, counts):
        """Stage one 16-channel sample as the two packets BrainFlow expects.

        `counts` are raw signed ADS1299 counts, ch1-16. Both packets carry the
        same counter; channels 1-8 go first, which is what makes BrainFlow treat
        the second packet as the completion of the pair.
        """
        if not self.streaming or self.data_writer is None:
            return

        sample_id = self.sample_id
        self.sample_id = (sample_id + 1) & 0xFF

        if len(self._out) + 2 * OBCI_PACKET_SIZE > self._cap:
            del self._out[:2 * OBCI_PACKET_SIZE]   # drop the oldest pair, stay live
            self.samples_dropped += 1

        self._out += self._build_packet(sample_id, counts[0:8])
        self._out += self._build_packet(sample_id, counts[8:16])
        self._wake.set()

    async def _writer_loop(self, writer):
        """Drain staged packets to the GUI, with drain() providing backpressure."""
        try:
            while True:
                if not self._out:
                    self._wake.clear()
                    await self._wake.wait()
                    continue
                chunk = bytes(self._out)
                self._out.clear()
                writer.write(chunk)
                await writer.drain()
                self.frames_sent += len(chunk) // OBCI_PACKET_SIZE
        except asyncio.CancelledError:
            raise
        except Exception as e:
            print(f"[GUI] data connection lost: {e}")
            self.streaming = False
            if self.data_writer is writer:
                self.data_writer = None

    def status(self):
        if self.data_writer is None:
            return "waiting for GUI"
        if not self.streaming:
            return f"connected to {self.client_addr}, idle"
        return f"{self.frames_sent} packets, dropped={self.samples_dropped}"


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

        # Native OpenBCI GUI output, 16-channel boards only. Nothing about the
        # V1/V2 path changes: for an 8-channel board obci stays None and every
        # branch below is skipped.
        obci = None
        if OPENBCI_EMULATION and fmt.num_channels == 16:
            emulator = ObciWifiShieldEmulator()
            if await emulator.start():
                obci = emulator

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
                            # Raw counts for the GUI, which does its own
                            # scaling. Stays None on an 8-channel board.
                            obci_counts = [] if obci is not None else None

                            for ch in range(fmt.num_channels):
                                idx = ADS1299_NUM_STATUS_BYTES + ch * ADS1299_BYTES_PER_CHANNEL
                                raw_val = int.from_bytes(ads_data[idx : idx + ADS1299_BYTES_PER_CHANNEL], byteorder='big', signed=True)
                                if obci_counts is not None:
                                    obci_counts.append(raw_val)

                                current_x = convert_to_microvolts(raw_val)

                                # DC Blocker
                                if first_sample: current_y = 0.0
                                else: current_y = current_x - prev_x[ch] + (R * prev_y[ch])

                                prev_x[ch] = current_x
                                prev_y[ch] = current_y
                                lsl_sample.append(current_y)

                            first_sample = False

                            if obci is not None:
                                obci.push_sample(obci_counts)

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
                    gui = f" | gui: {obci.status()}" if obci is not None else ""
                    print(f"[STATS] {pushed} samples pushed | {pushed / max(elapsed, 1e-9):.0f} Hz recent "
                          f"| lost={lost_samples} bad_checksum={bad_checksum} resync_bytes={resync_bytes} "
                          f"| notifications={link.notifications}{gui}")
                    pushed = 0

                await asyncio.sleep(0)

        except KeyboardInterrupt:
            print("Stopping...")
        finally:
            print(f"Session totals: lost={lost_samples} bad_checksum={bad_checksum} resync_bytes={resync_bytes}")
            if obci is not None:
                await obci.close()
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
