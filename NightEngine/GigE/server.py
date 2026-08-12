# server.py
# owns every socket for a set of emulated GigE Vision devices and
# bridges them to the render thread.
#
# threading model (OpenGL is single-threaded, so this matters):
#   * one GVCP thread runs a selector over all control sockets and the
#     stream sockets. it never touches OpenGL.
#   * process() runs on the render thread once per frame and is the ONLY
#     place a frame is rendered.
#   * one sender thread per device packetises queued frames and pushes
#     them onto the wire, so packet pacing never stalls rendering.
#
# the GVCP port is fixed at 3956 by the standard, so consumers address
# devices by IP: N cameras need N distinct local IP addresses. Hosting
# them all in one process avoids sharing port 3956 across processes,
# which is undefined behaviour on Windows.

import threading
import selectors
import socket
import struct
import time

from NightEngine.GigE import protocol as P
from NightEngine.GigE import registers as R
from NightEngine.GigE.device import NightGigEDevice


def _subnet_broadcast(ip, mask):
    ip_u32 = R.ip_to_u32(ip)
    mask_u32 = R.ip_to_u32(mask)
    return R.u32_to_ip((ip_u32 & mask_u32) | (~mask_u32 & 0xFFFFFFFF))


class NightGigEServer:
    """hosts one or more NightGigEDevice instances."""

    def __init__(self, engine, devices=(), bind_broadcast=True, verbose=True,
                 bind_any=False):
        """bind_any binds 0.0.0.0:3956 instead of the device's own address.
        Measured on Windows: binding a broadcast address is rejected
        outright (WinError 10049), while a socket bound to a unicast
        address *does* still receive subnet-directed broadcasts -- so the
        default works. 0.0.0.0 also receives them and is the fallback if
        a consumer's discovery never arrives. Only valid for one device,
        since a wildcard socket cannot tell which device a unicast
        command was addressed to."""
        self.engine = engine
        self.devices = []
        self.verbose = verbose
        self.bind_any = bind_any
        self._selector = selectors.DefaultSelector()
        self._sockets = []
        self._running = True

        for spec in devices:
            self.add_device(**spec) if isinstance(spec, dict) else self.add_device(device=spec)

        if bind_broadcast:
            self._bind_broadcast_sockets()

        threading.Thread(target=self._gvcp_loop, daemon=True,
                         name="gige-gvcp").start()

    # ------------------------------------------------------------
    # setup
    # ------------------------------------------------------------

    def add_device(self, device=None, camera=None, ip=None, **kwargs):
        """registers a device and binds its control and stream sockets."""
        if device is None:
            device = NightGigEDevice(camera=camera, ip=ip, **kwargs)

        if self.bind_any and self.devices:
            raise ValueError("bind_any supports a single device only: a wildcard "
                             "socket cannot attribute a unicast command to one of "
                             "several devices. Give each device its own IP instead.")

        # control socket, bound to this device's own address so several
        # devices can coexist on port 3956
        bind_address = "0.0.0.0" if self.bind_any else device.ip
        control = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        control.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            control.bind((bind_address, P.GVCP_PORT))
        except OSError as e:
            raise OSError(
                f"NightGigEServer: cannot bind {bind_address}:{P.GVCP_PORT} ({e}). "
                f"Is {device.ip} assigned to a local interface? On Windows add it "
                f'with: netsh interface ipv4 add address "Ethernet" {device.ip} '
                f"255.255.0.0") from e

        # stream socket: ephemeral source port, reported through SCSP
        stream = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        stream.bind((bind_address, 0))
        device.source_port = stream.getsockname()[1]
        device.registers.write_u32(R.SCSP, device.source_port)

        device._control_socket = control
        device._stream_socket = stream
        device.stream_sender = lambda data, d=device: self._send_stream(d, data)

        self._selector.register(control, selectors.EVENT_READ,
                                ("control", device))
        self._selector.register(stream, selectors.EVENT_READ,
                                ("stream", device))
        self._sockets += [control, stream]
        self.devices.append(device)

        threading.Thread(target=self._sender_loop, args=(device,), daemon=True,
                         name=f"gige-gvsp-{device.camera.name}").start()

        if self.verbose:
            print(f"NightGigEServer: {device.model} '{device.camera.name}' at "
                  f"{device.ip}:{P.GVCP_PORT} (stream port {device.source_port}, "
                  f"{device.width}x{device.height}, "
                  f"XML {device.registers.xml_size} B)")
        return device

    def _bind_broadcast_sockets(self):
        """best-effort extra sockets on the broadcast addresses.

        Measured on Windows 11: binding either 255.255.255.255 or a subnet
        broadcast address fails with WinError 10049, so this is a no-op
        there -- and it does not matter, because a socket bound to a
        unicast address receives subnet-directed broadcasts anyway, and
        HALCON sends discovery to both the subnet and global broadcast.
        Kept for platforms (notably Linux) where these binds succeed."""
        if self.bind_any:
            return          # the wildcard socket already receives broadcasts
        targets = {"255.255.255.255"}
        for device in self.devices:
            mask = R.u32_to_ip(device.registers.read_u32(R.SUBNET_MASK))
            try:
                targets.add(_subnet_broadcast(device.ip, mask))
            except ValueError:
                pass
        for target in sorted(targets):
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            try:
                sock.bind((target, P.GVCP_PORT))
            except OSError:
                # windows will not always allow binding a broadcast
                # address; unicast discovery still works
                sock.close()
                continue
            self._selector.register(sock, selectors.EVENT_READ,
                                    ("broadcast", None))
            self._sockets.append(sock)
            if self.verbose:
                print(f"NightGigEServer: listening for discovery on "
                      f"{target}:{P.GVCP_PORT}")

    # ------------------------------------------------------------
    # GVCP thread
    # ------------------------------------------------------------

    def _gvcp_loop(self):
        while self._running:
            try:
                events = self._selector.select(timeout=0.25)
            except OSError:
                break
            for key, _mask in events:
                kind, device = key.data
                try:
                    data, sender = key.fileobj.recvfrom(65535)
                except OSError:
                    continue
                if kind == "stream":
                    # HALCON sends 'stream_keep_alive\0' from its receive
                    # port to ours to prime the path. it is not in any
                    # spec; silently ignore anything unrecognised here.
                    continue
                if kind == "broadcast":
                    self._handle_broadcast(key.fileobj, data, sender)
                    continue
                reply = device.handle_gvcp(data, sender)
                if reply:
                    try:
                        key.fileobj.sendto(reply, sender)
                    except OSError:
                        pass

    def _handle_broadcast(self, sock, data, sender):
        """a broadcast reaches every device: all of them answer discovery
        from their own control socket, so the consumer sees each one at
        its own address."""
        cmd = P.parse_cmd(data)
        if cmd is None or cmd.code != P.DISCOVERY_CMD:
            return
        for device in self.devices:
            reply = device.handle_gvcp(data, sender)
            if reply:
                try:
                    device._control_socket.sendto(reply, sender)
                except OSError:
                    pass

    # ------------------------------------------------------------
    # GVSP sender threads
    # ------------------------------------------------------------

    def _send_stream(self, device, data):
        destination = device.stream_destination()
        if destination is None:
            return
        try:
            device._stream_socket.sendto(data, destination)
        except OSError:
            pass

    def _sender_loop(self, device):
        while self._running:
            try:
                payload = device.frames.get(timeout=0.25)
            except Exception:
                continue
            destination = device.stream_destination()
            if destination is None:
                continue
            packets = device.build_packets(payload)
            delay = device.registers.read_u32(R.SCPD)
            sock = device._stream_socket
            for packet in packets:
                try:
                    sock.sendto(packet, destination)
                except OSError:
                    break
                if delay:
                    time.sleep(delay / 1_000_000.0)

    # ------------------------------------------------------------
    # render thread
    # ------------------------------------------------------------

    def process(self):
        """called once per frame by the engine loop, on the render
        thread. the only place frames are rendered."""
        now = time.monotonic()
        for device in self.devices:
            if device.wants_frame(now):
                device.capture(self.engine, now)

    def info(self):
        return [device.info() for device in self.devices]

    def close(self):
        self._running = False
        for sock in self._sockets:
            try:
                self._selector.unregister(sock)
            except (KeyError, ValueError):
                pass
            sock.close()
