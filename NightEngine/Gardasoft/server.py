# server.py
# Network front end for the emulated Gardasoft CC320: the ASCII command
# channel on TCP/UDP 30313, UDP discovery on 30311, and an optional status
# web page.
#
# Threading mirrors GigE/server.py: socket threads never touch OpenGL. They
# execute commands against the controller under a lock; the render thread
# calls process() once per frame to advance time and push output state into
# the scene.

import threading
import selectors
import socket
import time

from NightEngine.Gardasoft import protocol as P
from NightEngine.Gardasoft.controller import NightGardasoftCC320, MODEL, FIRMWARE


class NightGardasoftServer:
    """hosts one emulated CC320 on the network."""

    def __init__(self, engine, controller=None, ip="127.0.0.1", serial=12345,
                 mac="00:0B:75:01:80:99", http_port=80, verbose=True,
                 bind_any=False):
        self.engine = engine
        self.controller = controller or NightGardasoftCC320(ip=ip, serial=serial,
                                                            mac=mac)
        self.ip = self.controller.ip
        self.verbose = verbose
        self.lock = threading.Lock()
        self._running = True
        self._sockets = []
        self._selector = selectors.DefaultSelector()
        self._recent_enquiry = {}          # sender -> time, to dedup broadcasts

        bind_ip = "0.0.0.0" if bind_any else self.ip

        # ---------------- command channel ---------------- #

        self._tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._tcp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self._tcp.bind((bind_ip, P.TCP_PORT))
        except OSError as error:
            from NightEngine.netutil import bind_failure_message
            raise OSError(bind_failure_message(
                "NightGardasoftServer", bind_ip, P.TCP_PORT, error)) from error
        self._tcp.listen(4)
        threading.Thread(target=self._accept_loop, daemon=True,
                         name="gardasoft-tcp").start()

        self._udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._udp.bind((bind_ip, P.TCP_PORT))
        self._selector.register(self._udp, selectors.EVENT_READ, "command")
        self._sockets.append(self._udp)

        # ---------------- discovery ---------------- #

        self._discovery = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._discovery.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._discovery.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._discovery.bind(("0.0.0.0", P.DISCOVERY_PORT))
        self._selector.register(self._discovery, selectors.EVENT_READ, "discovery")
        self._sockets.append(self._discovery)

        threading.Thread(target=self._udp_loop, daemon=True,
                         name="gardasoft-udp").start()

        # a real controller broadcasts its identity at power-up
        self._announce()

        # ---------------- optional web page ---------------- #

        self._http = None
        if http_port:
            self._start_http(bind_ip, http_port)

        if self.verbose:
            print(f"NightGardasoftServer: {MODEL} {FIRMWARE} at {self.ip} "
                  f"(commands TCP/UDP {P.TCP_PORT}, discovery UDP "
                  f"{P.DISCOVERY_PORT}, serial {self.controller.serial:06d})")

    # ------------------------------------------------------------
    # discovery
    # ------------------------------------------------------------

    def _discovery_reply(self):
        return P.format_discovery_reply(MODEL, self.controller.serial,
                                       self.controller.mac, self.ip)

    def _announce(self):
        """broadcast the identity, as a real unit does on power-up."""
        payload = self._discovery_reply().encode("ascii")
        for target in ("255.255.255.255", self._subnet_broadcast()):
            if not target:
                continue
            try:
                self._discovery.sendto(payload, (target, P.DISCOVERY_REPLY_PORT))
            except OSError:
                pass

    def _subnet_broadcast(self):
        try:
            octets = self.ip.split(".")
            return ".".join(octets[:3] + ["255"])
        except Exception:
            return None

    # ------------------------------------------------------------
    # socket threads
    # ------------------------------------------------------------

    def _accept_loop(self):
        while self._running:
            try:
                connection, address = self._tcp.accept()
            except OSError:
                break
            threading.Thread(target=self._tcp_client, args=(connection,),
                             daemon=True).start()

    def _tcp_client(self, connection):
        """reads CR-terminated lines. A real unit closes the connection after
        10s idle, so clients heartbeat with VR; emulate that."""
        connection.settimeout(P.TCP_IDLE_TIMEOUT)
        buffer = ""
        try:
            while self._running:
                try:
                    chunk = connection.recv(4096)
                except socket.timeout:
                    break                      # idle: close, like the hardware
                except OSError:
                    break
                if not chunk:
                    break
                buffer += chunk.decode("ascii", errors="ignore")
                while "\r" in buffer or "\n" in buffer:
                    index = min((buffer.index(c) for c in "\r\n" if c in buffer))
                    line, buffer = buffer[:index], buffer[index + 1:]
                    if not line.strip():
                        continue
                    reply = self._execute(line)
                    connection.sendall(reply.encode("ascii"))
        finally:
            connection.close()

    def _udp_loop(self):
        while self._running:
            try:
                events = self._selector.select(timeout=0.25)
            except OSError:
                break
            for key, _mask in events:
                try:
                    data, sender = key.fileobj.recvfrom(65535)
                except OSError:
                    continue
                if key.data == "discovery":
                    if P.is_discovery_request(data):
                        # the discovery socket is bound to all interfaces, and
                        # Windows delivers a broadcast once per interface, so
                        # collapse the near-simultaneous duplicates rather than
                        # answering each copy
                        now = time.monotonic()
                        last = self._recent_enquiry.get(sender)
                        self._recent_enquiry[sender] = now
                        if last is not None and now - last < 0.25:
                            continue
                        payload = self._discovery_reply().encode("ascii")
                        for port in (P.DISCOVERY_REPLY_PORT, sender[1]):
                            try:
                                key.fileobj.sendto(payload, (sender[0], port))
                            except OSError:
                                pass
                    continue
                line = data.decode("ascii", errors="ignore")
                if line.strip():
                    reply = self._execute(line)
                    try:
                        key.fileobj.sendto(reply.encode("ascii"), sender)
                    except OSError:
                        pass

    def _execute(self, line):
        with self.lock:
            return self.controller.execute_line(line)

    # ------------------------------------------------------------
    # status page
    # ------------------------------------------------------------

    def _start_http(self, bind_ip, port):
        from http.server import BaseHTTPRequestHandler, HTTPServer

        server = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                with server.lock:
                    info = server.controller.info()
                rows = "".join(
                    f"<tr><td>OP{index}</td><td>{'ON' if level else 'off'}</td></tr>"
                    for index, level in info["outputs"].items())
                body = (f"<html><head><title>{MODEL}</title></head><body>"
                        f"<h1>Gardasoft {MODEL} {FIRMWARE}</h1>"
                        f"<p>Serial {info['serial']:06d} &mdash; {info['ip']}</p>"
                        f"<p>Trigger period {info['timer_period_ms']:.2f} ms, "
                        f"encoder mode {info['encoder_mode']}</p>"
                        f"<table border=1>{rows}</table>"
                        f"<p>NightEngine emulated controller</p>"
                        f"</body></html>").encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        try:
            self._http = HTTPServer((bind_ip, port), Handler)
        except OSError as error:
            if self.verbose:
                print(f"NightGardasoftServer: no web page on port {port} "
                      f"({error}); commands and discovery are unaffected")
            self._http = None
            return
        threading.Thread(target=self._http.serve_forever, daemon=True,
                         name="gardasoft-http").start()

    # ------------------------------------------------------------
    # render thread
    # ------------------------------------------------------------

    def process(self, now=None):
        """advance the controller and push output state into the scene.
        Called once per frame by the engine loop."""
        moment = time.monotonic() if now is None else now
        with self.lock:
            self.controller.evaluate(moment)
            if self.engine is not None:
                self.controller.apply_to_engine(self.engine)

    def trigger_input(self, channel=1):
        """convenience for scene scripts: pulse an input, exactly as MP does."""
        with self.lock:
            self.controller.mp(channel)

    def info(self):
        with self.lock:
            return self.controller.info()

    def close(self):
        self._running = False
        if self._http:
            self._http.shutdown()
        for sock in self._sockets:
            try:
                self._selector.unregister(sock)
            except (KeyError, ValueError):
                pass
            sock.close()
        self._tcp.close()
