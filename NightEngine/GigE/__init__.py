# GigE Vision device emulation for NightEngine.
#
# Presents NightVisionCameras as GigE Vision compliant network devices,
# so third-party machine-vision software (HALCON, pylon Viewer, eBUS
# Player, Aravis ...) can discover and stream from them with no plugin.
#
# Implemented from scratch: GVCP (control) + GVSP (streaming) + a
# GenApi XML node map. The byte layouts and the register values were
# verified against a real Teledyne DALSA Genie Nano session captured
# while HALCON was talking to it -- see docs/gige-vision-reference.md.

from NightEngine.GigE.device import NightGigEDevice
from NightEngine.GigE.server import NightGigEServer

__all__ = ["NightGigEDevice", "NightGigEServer"]
