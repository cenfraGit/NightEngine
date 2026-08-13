# netutil.py
# Small shared networking helpers for the emulated devices.
#
# Both the GigE Vision camera and the Gardasoft controller must bind to an
# address the machine actually owns -- Windows rejects anything else with
# WinError 10049 ("The requested address is not valid in its context"), which
# on its own tells you nothing useful. These helpers turn that into an
# actionable message.

import socket


def local_ipv4_addresses():
    """every IPv4 address bound to this machine, best effort."""
    found = ["127.0.0.1"]
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            address = info[4][0]
            if address not in found:
                found.append(address)
    except socket.gaierror:
        pass
    # getaddrinfo misses addresses on disconnected or secondary adapters, so
    # also probe by connecting a UDP socket (which assigns a source address
    # without sending anything)
    for probe in ("8.8.8.8", "169.254.1.1", "192.168.1.1"):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.connect((probe, 9))
                address = sock.getsockname()[0]
                if address not in found:
                    found.append(address)
        except OSError:
            pass
    return found


def is_local_address(address):
    if address in ("0.0.0.0", "127.0.0.1"):
        return True
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind((address, 0))
        return True
    except OSError:
        return False


def bind_failure_message(component, address, port, error, mask="255.255.0.0"):
    """a message that says what went wrong AND how to fix it."""
    available = local_ipv4_addresses()
    lines = [f"{component}: cannot bind {address}:{port} ({error})."]
    if not is_local_address(address):
        lines.append(f"  {address} is not assigned to any local interface.")
        lines.append("  Addresses this machine can bind right now:")
        for candidate in available:
            lines.append(f"    {candidate}")
        lines.append("  Either pass one of those, or add the address to an "
                     "adapter, e.g.:")
        lines.append(f'    netsh interface ipv4 add address "Ethernet" '
                     f"{address} {mask}")
        lines.append("  (the adapter's primary address must be static, not "
                     "DHCP, for aliases to be accepted)")
    else:
        lines.append(f"  {address} exists but port {port} is already in use. "
                     f"Another instance may still be running.")
    return "\n".join(lines)
