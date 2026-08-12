# genapi.py
# builds the GenApi XML node map served to the consumer.
#
# authored against patterns copied verbatim from a Teledyne DALSA Genie
# Nano XML that HALCON accepts:
#   * schema Version_1_1, StandardNameSpace="GEV"
#   * feature nodes wrap a backing register node through <pValue>
#   * stream parameters carry <pIsLocked>TLParamsLocked</pIsLocked>
#   * TLParamsLocked is a bare <Integer> with a literal <Value>0</Value>
#     and no register behind it
#   * PayloadSize is an <Integer> whose pValue is a computed node
#   * MaskedIntReg bit numbering is MSB-FIRST: <LSB>31</LSB><MSB>16</MSB>
#     selects the LOW 16 bits, and <Bit>1</Bit> is normal bit 30. this
#     was confirmed against the Nano's own GevSCPSPacketSize node.
#
# element ORDER inside each node follows the GenApi schema sequence, so
# do not reorder children casually.

import zipfile
import io

from NightEngine.GigE import registers as R

SCHEMA_NS = "http://www.genicam.org/GenApi/Version_1_1"


def _int_reg(name, address, access="RW", length=4, cachable=None):
    extra = f"      <Cachable>{cachable}</Cachable>\n" if cachable else ""
    return f"""   <IntReg Name="{name}">
      <Address>0x{address:X}</Address>
      <Length>{length}</Length>
      <AccessMode>{access}</AccessMode>
      <pPort>Device</pPort>
{extra}      <Sign>Unsigned</Sign>
      <Endianess>BigEndian</Endianess>
   </IntReg>
"""


def _masked_reg(name, address, msb, lsb, access="RW"):
    """msb/lsb are MSB-first bit indices (0 = most significant)."""
    return f"""   <MaskedIntReg Name="{name}">
      <Address>0x{address:X}</Address>
      <Length>4</Length>
      <AccessMode>{access}</AccessMode>
      <pPort>Device</pPort>
      <Cachable>NoCache</Cachable>
      <LSB>{lsb}</LSB>
      <MSB>{msb}</MSB>
      <Sign>Unsigned</Sign>
      <Endianess>BigEndian</Endianess>
   </MaskedIntReg>
"""


def _bit_reg(name, address, bit, access="RW"):
    """bit is an MSB-first index (0 = most significant)."""
    return f"""   <MaskedIntReg Name="{name}">
      <Address>0x{address:X}</Address>
      <Length>4</Length>
      <AccessMode>{access}</AccessMode>
      <pPort>Device</pPort>
      <Cachable>NoCache</Cachable>
      <Bit>{bit}</Bit>
      <Sign>Unsigned</Sign>
      <Endianess>BigEndian</Endianess>
   </MaskedIntReg>
"""


def _string_reg(name, address, length):
    return f"""   <StringReg Name="{name}" NameSpace="Standard">
      <Address>0x{address:X}</Address>
      <Length>{length}</Length>
      <AccessMode>RO</AccessMode>
      <pPort>Device</pPort>
   </StringReg>
"""


def _integer(name, reg, visibility="Beginner", tooltip="", minimum=None,
             maximum=None, inc=None, locked=False, standard=True,
             representation=None, unit=None):
    ns = ' NameSpace="Standard"' if standard else ""
    out = f'   <Integer Name="{name}"{ns}>\n'
    if tooltip:
        out += f"      <ToolTip>{tooltip}</ToolTip>\n"
        out += f"      <Description>{tooltip}</Description>\n"
    out += f"      <Visibility>{visibility}</Visibility>\n"
    if locked:
        out += "      <pIsLocked>TLParamsLocked</pIsLocked>\n"
    out += f"      <pValue>{reg}</pValue>\n"
    if minimum is not None:
        out += f"      <Min>{minimum}</Min>\n"
    if maximum is not None:
        out += f"      <Max>{maximum}</Max>\n"
    if inc is not None:
        out += f"      <Inc>{inc}</Inc>\n"
    if representation:
        out += f"      <Representation>{representation}</Representation>\n"
    if unit:
        out += f"      <Unit>{unit}</Unit>\n"
    out += "   </Integer>\n"
    return out


def build_xml(vendor="NightEngine", model="NightEngineCam",
              version="1.0", width=640, height=480,
              pixel_formats=("Mono8", "RGB8"),
              product_guid="7b1c9a10-0000-4000-8000-000000000001",
              version_guid="7b1c9a10-0000-4000-8000-000000000002"):
    """returns the GenApi XML for one emulated camera as a string.

    width/height are the camera's fixed framebuffer size; they are
    exposed read-only because the offscreen buffer is allocated once.
    CVB's commercial GEV server documents the same limitation."""

    pf_values = {"Mono8": 0x01080001, "RGB8": 0x02180014, "BGR8": 0x02180015}

    parts = []
    parts.append(f"""<?xml version="1.0" encoding="UTF-8"?>
<RegisterDescription
    xmlns="{SCHEMA_NS}"
    xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xsi:schemaLocation="{SCHEMA_NS} GenApiSchema_Version_1_1.xsd"
    ModelName="{model}"
    VendorName="{vendor}"
    StandardNameSpace="GEV"
    SchemaMajorVersion="1"
    SchemaMinorVersion="1"
    SchemaSubMinorVersion="0"
    MajorVersion="1"
    MinorVersion="0"
    SubMinorVersion="0"
    ToolTip="NightEngine emulated GigE Vision camera"
    ProductGuid="{product_guid}"
    VersionGuid="{version_guid}">
""")

    # ------------------------- categories ------------------------- #
    # note: the Device port must NOT appear in the feature tree.

    parts.append("""   <Category Name="Root" NameSpace="Standard">
      <ToolTip>Root of the feature tree</ToolTip>
      <Visibility>Beginner</Visibility>
      <pFeature>DeviceControl</pFeature>
      <pFeature>ImageFormatControl</pFeature>
      <pFeature>AcquisitionControl</pFeature>
      <pFeature>TransportLayerControl</pFeature>
   </Category>
   <Category Name="DeviceControl" NameSpace="Standard">
      <Visibility>Beginner</Visibility>
      <pFeature>DeviceVendorName</pFeature>
      <pFeature>DeviceModelName</pFeature>
      <pFeature>DeviceVersion</pFeature>
      <pFeature>DeviceManufacturerInfo</pFeature>
      <pFeature>DeviceID</pFeature>
      <pFeature>DeviceUserID</pFeature>
      <pFeature>DeviceScanType</pFeature>
      <pFeature>DeviceTLType</pFeature>
      <pFeature>DeviceReset</pFeature>
   </Category>
   <Category Name="ImageFormatControl" NameSpace="Standard">
      <Visibility>Beginner</Visibility>
      <pFeature>SensorWidth</pFeature>
      <pFeature>SensorHeight</pFeature>
      <pFeature>WidthMax</pFeature>
      <pFeature>HeightMax</pFeature>
      <pFeature>Width</pFeature>
      <pFeature>Height</pFeature>
      <pFeature>OffsetX</pFeature>
      <pFeature>OffsetY</pFeature>
      <pFeature>PixelFormat</pFeature>
   </Category>
   <Category Name="AcquisitionControl" NameSpace="Standard">
      <Visibility>Beginner</Visibility>
      <pFeature>AcquisitionMode</pFeature>
      <pFeature>AcquisitionStart</pFeature>
      <pFeature>AcquisitionStop</pFeature>
      <pFeature>AcquisitionFrameRate</pFeature>
      <pFeature>ExposureTime</pFeature>
      <pFeature>Gain</pFeature>
      <pFeature>TriggerMode</pFeature>
      <pFeature>TriggerSoftware</pFeature>
   </Category>
   <Category Name="TransportLayerControl" NameSpace="Standard">
      <Visibility>Expert</Visibility>
      <pFeature>PayloadSize</pFeature>
      <pFeature>GevVersionMajor</pFeature>
      <pFeature>GevVersionMinor</pFeature>
      <pFeature>GevMACAddress</pFeature>
      <pFeature>GevCurrentIPAddress</pFeature>
      <pFeature>GevCurrentSubnetMask</pFeature>
      <pFeature>GevCurrentDefaultGateway</pFeature>
      <pFeature>GevHeartbeatTimeout</pFeature>
      <pFeature>GevTimestampTickFrequency</pFeature>
      <pFeature>GevCCP</pFeature>
      <pFeature>GevStreamChannelSelector</pFeature>
      <pFeature>GevSCPHostPort</pFeature>
      <pFeature>GevSCPSPacketSize</pFeature>
      <pFeature>GevSCPSDoNotFragment</pFeature>
      <pFeature>GevSCPSFireTestPacket</pFeature>
      <pFeature>GevSCPD</pFeature>
      <pFeature>GevSCDA</pFeature>
      <pFeature>GevSCSP</pFeature>
   </Category>
""")

    # ------------------------ device info ------------------------ #

    parts.append(_string_reg("DeviceVendorName", R.MANUFACTURER_NAME, 32))
    parts.append(_string_reg("DeviceModelName", R.MODEL_NAME, 32))
    parts.append(_string_reg("DeviceVersion", R.DEVICE_VERSION, 32))
    parts.append(_string_reg("DeviceManufacturerInfo", R.MANUFACTURER_INFO, 48))
    parts.append(_string_reg("DeviceID", R.SERIAL_NUMBER, 16))
    parts.append(f"""   <StringReg Name="DeviceUserID" NameSpace="Standard">
      <Address>0x{R.USER_DEFINED_NAME:X}</Address>
      <Length>16</Length>
      <AccessMode>RW</AccessMode>
      <pPort>Device</pPort>
   </StringReg>
""")

    # DeviceScanType / DeviceTLType are enumerations over in-memory
    # integers -- no device register needed.
    parts.append("""   <Integer Name="DeviceScanTypeValue">
      <Visibility>Invisible</Visibility>
      <Value>0</Value>
   </Integer>
   <Enumeration Name="DeviceScanType" NameSpace="Standard">
      <ToolTip>Scan type of the sensor</ToolTip>
      <Visibility>Expert</Visibility>
      <EnumEntry Name="Areascan" NameSpace="Standard">
         <Value>0</Value>
      </EnumEntry>
      <pValue>DeviceScanTypeValue</pValue>
   </Enumeration>
   <Integer Name="DeviceTLTypeValue">
      <Visibility>Invisible</Visibility>
      <Value>0</Value>
   </Integer>
   <Enumeration Name="DeviceTLType" NameSpace="Standard">
      <ToolTip>Transport layer type of the device</ToolTip>
      <Visibility>Expert</Visibility>
      <EnumEntry Name="GigEVision" NameSpace="Standard">
         <Value>0</Value>
      </EnumEntry>
      <pValue>DeviceTLTypeValue</pValue>
   </Enumeration>
""")
    parts.append(_int_reg("DeviceResetReg", R.REG_DEVICE_RESET, "WO"))
    parts.append("""   <Command Name="DeviceReset" NameSpace="Standard">
      <ToolTip>Resets the device</ToolTip>
      <Visibility>Guru</Visibility>
      <pValue>DeviceResetReg</pValue>
      <CommandValue>1</CommandValue>
   </Command>
""")

    # ----------------------- image format ----------------------- #

    parts.append(_int_reg("SensorWidthReg", R.REG_SENSOR_WIDTH, "RO"))
    parts.append(_int_reg("SensorHeightReg", R.REG_SENSOR_HEIGHT, "RO"))
    parts.append(_int_reg("WidthReg", R.REG_WIDTH, "RO"))
    parts.append(_int_reg("HeightReg", R.REG_HEIGHT, "RO"))
    parts.append(_int_reg("OffsetXReg", R.REG_OFFSET_X, "RO"))
    parts.append(_int_reg("OffsetYReg", R.REG_OFFSET_Y, "RO"))
    parts.append(_int_reg("PixelFormatReg", R.REG_PIXEL_FORMAT, "RW",
                          cachable="NoCache"))

    parts.append(_integer("SensorWidth", "SensorWidthReg", "Expert",
                          "Effective width of the sensor in pixels"))
    parts.append(_integer("SensorHeight", "SensorHeightReg", "Expert",
                          "Effective height of the sensor in pixels"))
    parts.append(_integer("WidthMax", "SensorWidthReg", "Expert",
                          "Maximum image width"))
    parts.append(_integer("HeightMax", "SensorHeightReg", "Expert",
                          "Maximum image height"))
    parts.append(_integer("Width", "WidthReg", "Beginner",
                          "Width of the image in pixels",
                          minimum=width, maximum=width, inc=1, locked=True,
                          representation="Linear"))
    parts.append(_integer("Height", "HeightReg", "Beginner",
                          "Height of the image in pixels",
                          minimum=height, maximum=height, inc=1, locked=True,
                          representation="Linear"))
    parts.append(_integer("OffsetX", "OffsetXReg", "Beginner",
                          "Horizontal offset of the region of interest",
                          minimum=0, maximum=0, inc=1, locked=True))
    parts.append(_integer("OffsetY", "OffsetYReg", "Beginner",
                          "Vertical offset of the region of interest",
                          minimum=0, maximum=0, inc=1, locked=True))

    entries = ""
    for pf in pixel_formats:
        entries += f"""      <EnumEntry Name="{pf}" NameSpace="Standard">
         <ToolTip>{pf}</ToolTip>
         <Value>0x{pf_values[pf]:08X}</Value>
      </EnumEntry>
"""
    parts.append(f"""   <Enumeration Name="PixelFormat" NameSpace="Standard">
      <ToolTip>Pixel format of the transmitted image</ToolTip>
      <Visibility>Beginner</Visibility>
      <pIsLocked>TLParamsLocked</pIsLocked>
{entries}      <pValue>PixelFormatReg</pValue>
   </Enumeration>
""")

    # ------------------------ acquisition ------------------------ #

    parts.append(_int_reg("AcquisitionModeReg", R.REG_ACQUISITION_MODE))
    parts.append(_int_reg("AcquisitionCommandReg", R.REG_ACQUISITION_COMMAND, "WO"))
    parts.append(_int_reg("FramePeriodReg", R.REG_FRAME_PERIOD_US))
    parts.append(_int_reg("ExposureTimeReg", R.REG_EXPOSURE_TIME_US))
    parts.append(_int_reg("GainReg", R.REG_GAIN_RAW))
    parts.append(_int_reg("TriggerModeReg", R.REG_TRIGGER_MODE))
    parts.append(_int_reg("TriggerSoftwareReg", R.REG_TRIGGER_SOFTWARE, "WO"))

    parts.append("""   <Enumeration Name="AcquisitionMode" NameSpace="Standard">
      <ToolTip>Acquisition mode of the device</ToolTip>
      <Visibility>Beginner</Visibility>
      <pIsLocked>TLParamsLocked</pIsLocked>
      <EnumEntry Name="Continuous" NameSpace="Standard">
         <Value>0</Value>
      </EnumEntry>
      <EnumEntry Name="SingleFrame" NameSpace="Standard">
         <Value>1</Value>
      </EnumEntry>
      <pValue>AcquisitionModeReg</pValue>
   </Enumeration>
   <Command Name="AcquisitionStart" NameSpace="Standard">
      <ToolTip>Starts the acquisition of images</ToolTip>
      <Visibility>Beginner</Visibility>
      <pValue>AcquisitionCommandReg</pValue>
      <CommandValue>1</CommandValue>
   </Command>
   <Command Name="AcquisitionStop" NameSpace="Standard">
      <ToolTip>Stops the acquisition of images</ToolTip>
      <Visibility>Beginner</Visibility>
      <pValue>AcquisitionCommandReg</pValue>
      <CommandValue>0</CommandValue>
   </Command>
   <Converter Name="AcquisitionFrameRate" NameSpace="Standard">
      <ToolTip>Frame rate in hertz</ToolTip>
      <Visibility>Beginner</Visibility>
      <FormulaTo>1000000/FROM</FormulaTo>
      <FormulaFrom>1000000/TO</FormulaFrom>
      <pValue>FramePeriodReg</pValue>
      <Slope>Decreasing</Slope>
   </Converter>
   <Converter Name="ExposureTime" NameSpace="Standard">
      <ToolTip>Exposure time in microseconds</ToolTip>
      <Visibility>Beginner</Visibility>
      <FormulaTo>FROM</FormulaTo>
      <FormulaFrom>TO</FormulaFrom>
      <pValue>ExposureTimeReg</pValue>
      <Slope>Increasing</Slope>
   </Converter>
   <Converter Name="Gain" NameSpace="Standard">
      <ToolTip>Gain applied to the image</ToolTip>
      <Visibility>Beginner</Visibility>
      <FormulaTo>FROM</FormulaTo>
      <FormulaFrom>TO</FormulaFrom>
      <pValue>GainReg</pValue>
      <Slope>Increasing</Slope>
   </Converter>
   <Enumeration Name="TriggerMode" NameSpace="Standard">
      <ToolTip>Controls whether the trigger is active</ToolTip>
      <Visibility>Beginner</Visibility>
      <EnumEntry Name="Off" NameSpace="Standard">
         <Value>0</Value>
      </EnumEntry>
      <EnumEntry Name="On" NameSpace="Standard">
         <Value>1</Value>
      </EnumEntry>
      <pValue>TriggerModeReg</pValue>
   </Enumeration>
   <Command Name="TriggerSoftware" NameSpace="Standard">
      <ToolTip>Generates an internal trigger</ToolTip>
      <Visibility>Beginner</Visibility>
      <pValue>TriggerSoftwareReg</pValue>
      <CommandValue>1</CommandValue>
   </Command>
""")

    # --------------------- transport layer --------------------- #
    # PayloadSize is computed, exactly the bytes we transmit per frame.

    parts.append("""   <IntSwissKnife Name="PayloadSizeValue">
      <Visibility>Invisible</Visibility>
      <pVariable Name="WIDTH">WidthReg</pVariable>
      <pVariable Name="HEIGHT">HeightReg</pVariable>
      <pVariable Name="PIXELFORMAT">PixelFormatReg</pVariable>
      <Formula>WIDTH * HEIGHT * ((PIXELFORMAT &gt;&gt; 16) &amp; 0xFF) / 8</Formula>
   </IntSwissKnife>
   <Integer Name="PayloadSize" NameSpace="Standard">
      <ToolTip>Number of bytes transferred for each image</ToolTip>
      <Visibility>Expert</Visibility>
      <pValue>PayloadSizeValue</pValue>
      <Unit>B</Unit>
   </Integer>
   <Integer Name="TLParamsLocked">
      <ToolTip>Locks critical features while acquisition is running</ToolTip>
      <Visibility>Invisible</Visibility>
      <Value>0</Value>
      <Min>0</Min>
      <Max>1</Max>
   </Integer>
   <Integer Name="GevStreamChannelSelector" NameSpace="Standard">
      <ToolTip>Selects the stream channel</ToolTip>
      <Visibility>Expert</Visibility>
      <Value>0</Value>
      <Min>0</Min>
      <Max>0</Max>
   </Integer>
""")

    # bootstrap-backed Gev* nodes. bit indices below are MSB-first.
    parts.append(_masked_reg("GevVersionMajorReg", R.VERSION, msb=0, lsb=15, access="RO"))
    parts.append(_masked_reg("GevVersionMinorReg", R.VERSION, msb=16, lsb=31, access="RO"))
    parts.append(_int_reg("GevMACAddressLowReg", R.MAC_LOW, "RO"))
    parts.append(_int_reg("GevCurrentIPAddressReg", R.CURRENT_IP, "RO"))
    parts.append(_int_reg("GevCurrentSubnetMaskReg", R.SUBNET_MASK, "RO"))
    parts.append(_int_reg("GevCurrentDefaultGatewayReg", R.GATEWAY, "RO"))
    parts.append(_int_reg("GevHeartbeatTimeoutReg", R.HEARTBEAT_TIMEOUT))
    parts.append(_int_reg("GevTimestampTickFrequencyReg", R.TICK_FREQ_LOW, "RO"))
    parts.append(_int_reg("GevCCPReg", R.CCP, "RW", cachable="NoCache"))
    # low 16 bits of SCP / SCPS  ->  MSB-first msb=16, lsb=31
    parts.append(_masked_reg("GevSCPHostPortReg", R.SCP, msb=16, lsb=31))
    parts.append(_masked_reg("GevSCPSPacketSizeReg", R.SCPS, msb=16, lsb=31))
    parts.append(_bit_reg("GevSCPSDoNotFragmentReg", R.SCPS, bit=1))
    parts.append(_bit_reg("GevSCPSFireTestPacketReg", R.SCPS, bit=0))
    parts.append(_int_reg("GevSCPDReg", R.SCPD))
    parts.append(_int_reg("GevSCDAReg", R.SCDA))
    parts.append(_int_reg("GevSCSPReg", R.SCSP, "RO"))

    parts.append(_integer("GevVersionMajor", "GevVersionMajorReg", "Expert",
                          "Major version of the GigE Vision specification"))
    parts.append(_integer("GevVersionMinor", "GevVersionMinorReg", "Expert",
                          "Minor version of the GigE Vision specification"))
    parts.append(_integer("GevMACAddress", "GevMACAddressLowReg", "Expert",
                          "MAC address of the network interface",
                          representation="MACAddress"))
    parts.append(_integer("GevCurrentIPAddress", "GevCurrentIPAddressReg", "Expert",
                          "Current IP address of the device",
                          representation="IPV4Address"))
    parts.append(_integer("GevCurrentSubnetMask", "GevCurrentSubnetMaskReg", "Expert",
                          "Current subnet mask of the device",
                          representation="IPV4Address"))
    parts.append(_integer("GevCurrentDefaultGateway", "GevCurrentDefaultGatewayReg",
                          "Expert", "Current default gateway of the device",
                          representation="IPV4Address"))
    parts.append(_integer("GevHeartbeatTimeout", "GevHeartbeatTimeoutReg", "Guru",
                          "Heartbeat timeout in milliseconds", unit="ms"))
    parts.append(_integer("GevTimestampTickFrequency", "GevTimestampTickFrequencyReg",
                          "Guru", "Timestamp tick frequency in ticks per second"))
    parts.append("""   <Enumeration Name="GevCCP" NameSpace="Standard">
      <ToolTip>Device access privilege of the application</ToolTip>
      <Visibility>Guru</Visibility>
      <EnumEntry Name="OpenAccess" NameSpace="Standard">
         <Value>0</Value>
      </EnumEntry>
      <EnumEntry Name="ExclusiveAccess" NameSpace="Standard">
         <Value>1</Value>
      </EnumEntry>
      <EnumEntry Name="ControlAccess" NameSpace="Standard">
         <Value>2</Value>
      </EnumEntry>
      <pValue>GevCCPReg</pValue>
   </Enumeration>
""")
    parts.append(_integer("GevSCPHostPort", "GevSCPHostPortReg", "Invisible",
                          "Port on the host that receives the stream",
                          minimum=0, maximum=65535, inc=1))
    parts.append(_integer("GevSCPSPacketSize", "GevSCPSPacketSizeReg", "Expert",
                          "Stream packet size in bytes",
                          minimum=512, maximum=16384, inc=4, locked=True,
                          representation="Linear"))
    parts.append("""   <Boolean Name="GevSCPSDoNotFragment" NameSpace="Standard">
      <ToolTip>Sets the do-not-fragment bit on stream packets</ToolTip>
      <Visibility>Invisible</Visibility>
      <pIsLocked>TLParamsLocked</pIsLocked>
      <pValue>GevSCPSDoNotFragmentReg</pValue>
      <OnValue>1</OnValue>
      <OffValue>0</OffValue>
   </Boolean>
   <Boolean Name="GevSCPSFireTestPacket" NameSpace="Standard">
      <ToolTip>Sends one test packet of the configured size</ToolTip>
      <Visibility>Invisible</Visibility>
      <pValue>GevSCPSFireTestPacketReg</pValue>
      <OnValue>1</OnValue>
      <OffValue>0</OffValue>
   </Boolean>
""")
    parts.append(_integer("GevSCPD", "GevSCPDReg", "Expert",
                          "Delay between stream packets, in timestamp ticks",
                          locked=True))
    parts.append(_integer("GevSCDA", "GevSCDAReg", "Invisible",
                          "Destination IP address for the stream channel",
                          locked=True, representation="IPV4Address"))
    parts.append(_integer("GevSCSP", "GevSCSPReg", "Invisible",
                          "Source port of the stream channel"))

    parts.append("""   <Port Name="Device" NameSpace="Standard">
      <ToolTip>Port through which the node map reaches the device</ToolTip>
      <Visibility>Invisible</Visibility>
   </Port>
</RegisterDescription>
""")

    return "".join(parts)


def zip_xml(xml_text, filename):
    """the consumer decides an XML is compressed purely from the .zip
    extension in the First URL, so this must pair with a .zip name.
    zipping matters: READMEM moves <=512 bytes per round trip, so
    compressing cuts device open time several fold."""
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w", zipfile.ZIP_DEFLATED) as archive:
        archive.writestr(filename, xml_text.encode("utf-8"))
    return buffer.getvalue()
