# GigE Vision device-side reference

Working notes for `NightEngine/GigE/`. The GigE Vision specification is paywalled behind A3
membership, so everything here was reconstructed from three independent sources —
the Aravis implementation, Wireshark's GVCP/GVSP dissectors, and a third C# implementation —
and then **checked against a real capture of MVTec HALCON talking to a Teledyne DALSA Genie
Nano M4020**.

Items marked ⭐ were *observed on the wire* from that known-good device. Prefer them over
anything else; several contradict what the open-source implementations do.

---

## Why HALCON must be on a different machine

Stemmer's CVB GEV Server — a commercial product doing exactly this job — documents that a GEV
device and its client cannot share one PC when a filter driver is in use, because Windows
short-circuits the packets in the IP stack before the driver sees them. HALCON's GigEVision2
interface uses a filter driver by default.

The failure mode is deceptive: **discovery succeeds, open succeeds, and zero frames arrive.**
Never architect around loopback. Loopback is fine for our own tests (see `tests/`), because our
test client is an ordinary socket program.

Two more hard constraints:

- **One IP per camera.** The standard fixes the device control port at UDP 3956, so consumers
  address devices by IP. N cameras need N distinct local addresses, not N ports.
- **The advertised IP and subnet mask must be in the consumer NIC's subnet.** HALCON will list a
  device outside its subnet, then mark it inaccessible and refuse to open it. The Nano ran
  link-local: device `169.254.5.50`, mask `255.255.0.0`, host `169.254.5.94`.

---

## GVCP

Device listens on **UDP 3956**. Acknowledges go back to the command's source address and port,
never to a fixed port. Discovery arrives as a broadcast to `255.255.255.255:3956` and/or the
subnet broadcast; also answer unicast discovery, which sidesteps Windows broadcast fan-out
weirdness entirely.

**Command header, 8 bytes, big-endian** (everything in GVCP/GVSP is big-endian):

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | message key = `0x42` |
| 1 | 1 | flags (`0x01` ack required; `0x10` = allow broadcast ack on DISCOVERY) |
| 2 | 2 | command code |
| 4 | 2 | payload length, **excluding** the header |
| 6 | 2 | request id, non-zero |

**Acknowledge header, 8 bytes.** There is *no* `0x42` key byte in an acknowledge — the first two
bytes are a 16-bit status:

| Offset | Size | Field |
|---|---|---|
| 0 | 2 | status (`0x0000` = success) |
| 2 | 2 | acknowledge code |
| 4 | 2 | payload length |
| 6 | 2 | acknowledge id = the command's request id |

**Codes:** DISCOVERY `0x0002`/`0x0003`, FORCEIP `0x0004`/`0x0005`, PACKETRESEND `0x0040`/`0x0041`,
READREG `0x0080`/`0x0081`, WRITEREG `0x0082`/`0x0083`, READMEM `0x0084`/`0x0085`,
WRITEMEM `0x0086`/`0x0087`, PENDING_ACK `0x0089`.

**Payloads.** READREG: N×u32 addresses → N×u32 values. WRITEREG: N×(u32 address, u32 value) →
u32 count written. READMEM: (u32 address, u16 reserved, u16 count) → u32 address + data, at most
512 data bytes per acknowledge. WRITEMEM: u32 address + bytes. DISCOVERY_ACK: a verbatim copy of
bootstrap `0x0000`–`0x00F7`, **248 bytes**.

⭐ **Error status is `0x8000 | code`.** The Nano answers `0x8003` (invalid address) for a READREG
of `PendingTimeout` (`0x0958`), and HALCON carries on without complaint. So returning `0x8003`
for registers we don't implement is both correct and safe.

---

## Bootstrap registers

Addresses and the ⭐ values below are what our device reports.

| Address | Size | Register | Our value |
|---|---|---|---|
| `0x0000` | 4 | Version | ⭐ `0x00010002` (GEV 1.2) |
| `0x0004` | 4 | Device Mode | ⭐ `0x80000001` — bit 31 **set** = big-endian device, charset UTF-8 |
| `0x0008` / `0x000C` | 4+4 | MAC high / low | 6 MAC bytes live at `0x000A` |
| `0x0010` / `0x0014` | 4 | Supported / current IP config | bit0 persistent, bit1 DHCP, bit2 LLA |
| `0x0024` / `0x0034` / `0x0044` | 4 | IP / subnet mask / gateway | must match the consumer's subnet |
| `0x0048` | 32 | Manufacturer name | |
| `0x0068` | 32 | Model name | |
| `0x0088` | 32 | Device version | |
| `0x00A8` | 48 | Manufacturer info | |
| `0x00D8` | 16 | Serial number | distinct per device |
| `0x00E8` | 16 | User-defined name | distinct per device |
| `0x0200` | 512 | First URL | ⭐ plain `Local:<name>.zip;<addr_hex>;<len_hex>` — no `///` |
| `0x0400` | 512 | Second URL | zeroed (the Nano puts an HTTP fallback here) |
| `0x0600` | 4 | Network interface count | 1 |
| `0x0900` | 4 | Message channel count | 0 (the Nano reports 1; we skip events) |
| `0x0904` | 4 | Stream channel count | 1 |
| `0x092C` | 4 | SCCAPS | ⭐ Nano `0x80000000` |
| `0x0930` | 4 | Message channel caps | ⭐ HALCON reads this |
| `0x0934` | 4 | GVCP capability | ⭐ Nano `0xd640004f`; ours omits the manifest table |
| `0x0938` | 4 | Heartbeat timeout, ms | ⭐ 3000 |
| `0x093C` / `0x0940` | 4 | Tick frequency high / low | ⭐ `0` / `1000000` — **1 MHz**, not 1 GHz |
| `0x0954` | 4 | GVCP configuration | ⭐ HALCON writes `4` and reads it back |
| `0x0A00` | 4 | CCP | ⭐ **HALCON writes `1`; Aravis writes `2`** |

Stream channel registers, stride `0x40` per channel: SCP `0x0D00` (low 16 bits = host port),
SCPS `0x0D04` (low 16 bits = packet size; bit31 fire test packet, bit30 do-not-fragment,
bit29 big-endian), SCPD `0x0D08`, SCDA `0x0D18`, SCSP `0x0D1C` (read-only, our source port),
SCC `0x0D20`, SCCFG `0x0D24`. ⭐ The Nano defaults SCPS to `0x400005dc` = 1500 with
do-not-fragment.

**Do not advertise capability bit 26 (manifest table).** Setting it makes HALCON read a manifest
at `0x9000` and chase multiple XML files.

---

## GVSP

**Standard header, 8 bytes:**

| Offset | Size | Field |
|---|---|---|
| 0 | 2 | status |
| 2 | 2 | block id (16-bit, **never 0**) |
| 4 | 4 | `bit31 extended-id | bits30..24 content type | bits23..0 packet id` |

Content types: `0x01` leader, `0x02` trailer, `0x03` payload, `0x04` all-in.

**Image leader payload, 36 bytes:** reserved u16, payload type u16 (`0x0001` = image), timestamp
high u32, timestamp low u32, pixel format u32, width u32, height u32, offset_x u32, offset_y u32,
padding_x **u16**, padding_y **u16**. Aravis writes the paddings as 32-bit and corrupts them.

**Image trailer payload, 8 bytes:** reserved u16, payload type u16, actual size_y u32.

**Packet discipline:** packet id 0 for the leader, 1..N for payload, N+1 for the trailer, reset
per block. Block id increments per frame and wraps `0xFFFF → 1`.

**Packet sizing:** SCPS is the whole IP datagram size, excluding the Ethernet header and FCS.
Image bytes per packet = `SCPS − 36` (20 IPv4 + 8 UDP + 8 GVSP). ⭐ The Nano's own GenApi formula
computes `PACKET_SIZE - 36`, independently confirming that figure. Do not pad the last packet.

**Pixel formats:** Mono8 `0x01080001`, Mono16 `0x01100007`, RGB8 `0x02180014`, BGR8 `0x02180015`.
Bits 23..16 are bits-per-pixel, which is how PayloadSize is computed.

⭐ **Fire test packet.** HALCON negotiates packet size by writing SCPS with bit 31 set
(`0xc00005dc`). The device replies with exactly one datagram of that IP-datagram size with a
zeroed GVSP header — observed as 1472 UDP bytes for SCPS 1500 — and the bit self-clears.

---

## ⭐ HALCON behaviours found in no specification

**Keep-alive datagrams.** HALCON sends plain ASCII datagrams *from its receive ports to our
source ports*, to prime the UDP path:

- `stream_keep_alive\0` (18 bytes) → our GVSP source port
- `events_keep_alive\0` (18 bytes) → our message channel source port

These appear in no spec, not in Aravis, and not in Wireshark's dissectors. **Silently ignore
anything unrecognised on the stream socket.**

**READMEM counts vary.** HALCON reads 512, 380, 64, 32, 20 and 8 bytes — all multiples of 4.
Handle arbitrary counts and zero-fill past the end.

**The open sequence it actually drives:** discovery → poll CCP → `WRITEREG CCP=1` → READMEM the
identity strings (`0x48`/32, `0x68`/32, `0x88`/32, `0xd8`/16, `0xa8`/48) → READREG Version,
DeviceMode, interface/stream/message counts, MAC, IP, mask, gateway, IP config, GVCP capability →
write and read back GVCP configuration and SCCFG → READREG heartbeat, link speed, tick frequency →
READMEM First URL → chunked XML fetch → READREG SCPS → write SCDA and SCP → READREG SCSP →
negotiate SCPS including the test packet → acquisition → heartbeat via repeated `READREG CCP` →
`WRITEREG CCP=0`.

---

## GenApi XML

The Nano's map is **1,210 feature-level nodes** in 764 KB (81 KB zipped) — 8× the "~150" figure
CVB quotes and 30× Aravis's fake camera, which HALCON rejects. Ours is deliberately focused
(~90 nodes, 26 KB → 3.4 KB zipped) and built on patterns copied verbatim from the Nano.

- Schema **`Version_1_1`**, `StandardNameSpace="GEV"`, real `ProductGuid`/`VersionGuid`. Bump
  `VersionGuid` whenever the XML changes or consumers serve a stale cached node map.
- **Mandatory:** `Root` category, `Device` port (which must **not** appear in the feature tree),
  and `TLParamsLocked`.
- ⭐ `TLParamsLocked` is a bare `<Integer>` with a literal `<Value>0</Value>`, `Visibility
  Invisible`, min 0 max 1 — **no register behind it**.
- ⭐ `PayloadSize` is an `<Integer>` whose `pValue` points at a computed node, not a raw register.
- ⭐ Feature nodes wrap a backing register node via `pValue`; stream parameters carry
  `<pIsLocked>TLParamsLocked</pIsLocked>`.
- ⭐ **`MaskedIntReg` bit numbering is MSB-first (bit 0 = most significant).** The Nano selects
  SCPS's *low* 16 bits with `<LSB>31</LSB><MSB>16</MSB>`, and its do-not-fragment flag with
  `<Bit>1</Bit>` (normal bit 30). `normal_bit = 31 - genapi_bit` for a 4-byte register.
- ⭐ 17 of 18 `Gev*` transport-layer nodes are present in the Nano, so **include them** — it was
  otherwise unclear whether HALCON needs them.
- ⭐ **Compression is signalled purely by a `.zip` filename** in the First URL. Serve zipped:
  READMEM moves ≤512 bytes per round trip, so it cuts device open time several fold.
- ⭐ XML `VendorName`/`ModelName` need **not** match the bootstrap strings — the Nano's differ
  (`TeledyneDALSA` vs `Teledyne DALSA`, `Nano` vs `Nano-M4020`).
- ⭐ The alphanumeric-name rule is narrower than reported: 427 of the Nano's 1,210 node names
  contain underscores and HALCON is fine with it. HALCON's 2023-08 error-5312 hazard applies to
  user-visible parameter names only. We keep everything alphanumeric anyway.

---

## Debugging checklist

Ordered by likelihood for this project.

0. **Device never appears in the consumer's list** → the discovery broadcast isn't reaching us.
   Check plain connectivity first (`ping` the device address from the consumer). Measured on
   Windows 11: binding a broadcast address is rejected with WinError 10049, but a socket bound to
   a **unicast** address does receive subnet-directed broadcasts, and HALCON sends discovery to
   both the subnet and global broadcast — so the default bind should work. If it doesn't, use
   `bind_any=True` (`gige_camera.py --bind-any`), which binds `0.0.0.0:3956`; that provably
   receives broadcasts. Single device only, since a wildcard socket cannot attribute a unicast
   command to one of several devices.
1. **Advertised IP/subnet doesn't match the consumer NIC's subnet** → discovered but inaccessible.
2. **Filter driver eats same-host GVSP** → opens fine, zero frames. Use a separate machine.
3. **Firewall allows 3956 but blocks the ephemeral stream port** → configurable but no video.
   Allow inbound UDP to the consumer application on any port, for all three profiles.
4. **GenApi XML rejected** → missing `Root`/`Device`/`TLParamsLocked`, a dangling `pValue`
   reference, or a schema-order violation. `tests/test_gige_device.py` checks every reference
   resolves; also validate with
   [GenICamXmlValidator](https://github.com/genicam/GenICamXmlValidator).
5. **`PayloadSize` ≠ bytes actually sent.**
6. **Packet size exceeds the smallest MTU on the path** → frames vanish. Default to 1500 and let
   the consumer negotiate up.
7. **Receiver buffer overflow.** A 640×480 mono frame is 212 back-to-back datagrams; the default
   ~64 KB socket receive buffer drops ~28 of them. Real consumers enlarge `SO_RCVBUF` (and this
   is what HALCON's filter driver is for). Our test client sets 16 MB.
8. **Heartbeat killed by a debugger breakpoint** — HALCON's heartbeat thread halts with the rest,
   so keep the timeout generous and don't tear down aggressively.

**Tools.** Wireshark has GVCP (UDP 3956) and GVSP dissectors; GVSP is heuristic, so use
*Decode As → GVSP* on the stream port. MSYS2 ships prebuilt Aravis for Windows
(`pacman -S mingw-w64-x86_64-aravis`) giving `arv-tool-0.8` (enumerate, dump the feature tree —
the fastest check that our XML parses) and `arv-camera-test-0.8`. Basler pylon Viewer and Pleora
eBUS Player are free vendor-agnostic consumers worth testing against before HALCON. HALCON's own
logging: `ESEN_GENTL_LOGGING=1`, `ESEN_LOG_LEVEL=99`, viewed with Sysinternals DebugView.

Recreate the analysis of a capture with `tshark -r session.pcapng` — the parsing scripts used to
produce the ⭐ facts above walked raw `udp.payload` bytes rather than trusting field names.
