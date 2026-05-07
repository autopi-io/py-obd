# -*- coding: utf-8 -*-

########################################################################
#                                                                      #
# python-OBD: A python OBD-II serial module derived from pyobd         #
#                                                                      #
# Copyright 2004 Donour Sizemore (donour@uchicago.edu)                 #
# Copyright 2009 Secons Ltd. (www.obdtester.com)                       #
# Copyright 2009 Peter J. Creath                                       #
# Copyright 2016 Brendan Whitfield (brendan-w.com)                     #
# Copyright 2018 AutoPi.io ApS (support@autopi.io)                     #
#                                                                      #
########################################################################
#                                                                      #
# elm327.py                                                            #
#                                                                      #
# This file is part of python-OBD (a derivative of pyOBD)              #
#                                                                      #
# python-OBD is free software: you can redistribute it and/or modify   #
# it under the terms of the GNU General Public License as published by #
# the Free Software Foundation, either version 2 of the License, or    #
# (at your option) any later version.                                  #
#                                                                      #
# python-OBD is distributed in the hope that it will be useful,        #
# but WITHOUT ANY WARRANTY; without even the implied warranty of       #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the        #
# GNU General Public License for more details.                         #
#                                                                      #
# You should have received a copy of the GNU General Public License    #
# along with python-OBD.  If not, see <http://www.gnu.org/licenses/>.  #
#                                                                      #
########################################################################

import collections
import logging
import re
import serial
import time

from timeit import default_timer as timer
from ..protocols import *
from ..utils import BufferedSerialReader, OBDStatus, OBDError


logger = logging.getLogger(__name__)


########################################################################
# Protocol definitions
########################################################################

class SAE_J1850_PWM(LegacyProtocol):
    NAME = "SAE J1850 PWM"
    ID = "1"
    def __init__(self, lines_0100):
        LegacyProtocol.__init__(self, lines_0100)

class SAE_J1850_VPW(LegacyProtocol):
    NAME = "SAE J1850 VPW"
    ID = "2"
    def __init__(self, lines_0100):
        LegacyProtocol.__init__(self, lines_0100)

class ISO_9141_2(LegacyProtocol):
    NAME = "ISO 9141-2"
    ID = "3"
    def __init__(self, lines_0100):
        LegacyProtocol.__init__(self, lines_0100)

class ISO_14230_4_5baud(LegacyProtocol):
    NAME = "ISO 14230-4 (KWP 5BAUD)"
    ID = "4"
    def __init__(self, lines_0100):
        LegacyProtocol.__init__(self, lines_0100)

class ISO_14230_4_fast(LegacyProtocol):
    NAME = "ISO 14230-4 (KWP FAST)"
    ID = "5"
    def __init__(self, lines_0100):
        LegacyProtocol.__init__(self, lines_0100)

class ISO_15765_4_11bit_500k(CANProtocol):
    NAME = "ISO 15765-4 (CAN 11/500)"
    ID = "6"
    HEADER_BITS = 11
    def __init__(self, lines_0100):
        CANProtocol.__init__(self, lines_0100)

class ISO_15765_4_29bit_500k(CANProtocol):
    NAME = "ISO 15765-4 (CAN 29/500)"
    ID = "7"
    HEADER_BITS = 29
    def __init__(self, lines_0100):
        CANProtocol.__init__(self, lines_0100)

class ISO_15765_4_11bit_250k(CANProtocol):
    NAME = "ISO 15765-4 (CAN 11/250)"
    ID = "8"
    HEADER_BITS = 11
    def __init__(self, lines_0100):
        CANProtocol.__init__(self, lines_0100)

class ISO_15765_4_29bit_250k(CANProtocol):
    NAME = "ISO 15765-4 (CAN 29/250)"
    ID = "9"
    HEADER_BITS = 29
    def __init__(self, lines_0100):
        CANProtocol.__init__(self, lines_0100)

class SAE_J1939(CANProtocol):
    NAME = "SAE J1939 (CAN 29/250)"
    ID = "A"
    HEADER_BITS = 29
    def __init__(self, lines_0100):
        CANProtocol.__init__(self, lines_0100)


########################################################################
# Interface implementation
########################################################################

class ELM327Error(OBDError):

    def __init__(self, *args, **kwargs):
        self.code = kwargs.pop("code", None)
        super(ELM327Error, self).__init__(*args, **kwargs)

class ELM327(object):
    """
    Handles communication with the ELM327 adapter.
    """

    PROMPT = b"\r>"
    INTERRUPT = b"\x7F"
    OK = b"OK"
    ERRORS = {
        "?":                  "Invalid/unsupported command",
        "BUS BUSY":           "Too much activity on bus",
        "BUS ERROR":          "Invalid signal detected on bus",
        "CAN ERROR":          "CAN sending or receiving failed",
        "DATA ERROR":         "Incorrect response from vehicle",
        "FB ERROR":           "Problem with feedback signal",
        "UNABLE TO CONNECT":  "Unable to connect because no supported protocol found",
        "NO DATA":            "No data received from vehicle within timeout",
        "BUFFER FULL":        "Internal RS232 transmit buffer is full",
        "ACT ALERT":          "No RS232 or OBD activity for some time",
        "LV RESET":           "Low voltage reset",
        "LP ALERT":           "Low power (standby) mode in 2 seconds",
        "STOPPED":            "Operation interrupted by a received RS232 character",
    }

    SUPPORTED_PROTOCOLS = collections.OrderedDict({
        #"0" : None, # Automatic Mode. This isn't an actual protocol. If the
                     # ELM reports this, then we don't have enough
                     # information. see auto_protocol()
        "1" : SAE_J1850_PWM,
        "2" : SAE_J1850_VPW,
        "3" : ISO_9141_2,
        "4" : ISO_14230_4_5baud,
        "5" : ISO_14230_4_fast,
        "6" : ISO_15765_4_11bit_500k,
        "7" : ISO_15765_4_29bit_500k,
        "8" : ISO_15765_4_11bit_250k,
        "9" : ISO_15765_4_29bit_250k,
        "A" : SAE_J1939,
        #"B" : None, # user defined 1
        #"C" : None, # user defined 2
    })

    # Used as a fallback, when ATSP0 doesn't cut it
    TRY_PROTOCOL_ORDER = [
        "6", # ISO_15765_4_11bit_500k
        "8", # ISO_15765_4_11bit_250k
        "1", # SAE_J1850_PWM
        "7", # ISO_15765_4_29bit_500k
        "9", # ISO_15765_4_29bit_250k
        "2", # SAE_J1850_VPW
        "3", # ISO_9141_2
        "4", # ISO_14230_4_5baud
        "5", # ISO_14230_4_fast
        "A", # SAE_J1939
    ]

    # 38400, 9600 are the possible boot bauds (unless reprogrammed via
    # PP 0C).  19200, 38400, 57600, 115200, 230400, 500000 are listed on
    # p.46 of the ELM327 datasheet.
    #
    # Once pyserial supports non-standard baud rates on platforms other
    # than Linux, we'll add 500K to this list.
    #
    # We check the two default baud rates first, then go fastest to
    # slowest, on the theory that anyone who's using a slow baud rate is
    # going to be less picky about the time required to detect it.
    TRY_BAUDRATES = [38400, 9600, 230400, 115200, 57600, 38400, 19200]

    # OBD-II functional address
    OBDII_HEADER = "7DF" 

    # Programmable parameter IDs
    PP_ATH = "01"
    PP_ATE = "09"

    # State of command interface
    STATE_INTERACTIVE = "INTERACTIVE"
    STATE_MONITOR = "MONITOR"
    STATE_MONITOR_ALL = "MONITOR_ALL"


    def __init__(self, port, timeout=None, status_callback=None):
        """
        Initializes interface instance.
        """

        self._status              = OBDStatus.NOT_CONNECTED
        self._status_callback     = status_callback
        self._protocol            = UnknownProtocol([])
        self._state               = self.STATE_INTERACTIVE

        # Settings controlled via programmable parameters
        self._echo_off            = False
        self._print_headers       = False

        # Cached settings that have been changed runtime
        self._runtime_settings    = {}

        # Settings related to serial connection
        self._default_timeout = timeout if timeout != None else 10  # Seconds
        self._port = serial.Serial(parity   = serial.PARITY_NONE,
                                   stopbits = 1,
                                   bytesize = 8,
                                   timeout  = self._default_timeout)
        self._port.port = port

        self._at_command_mappings = [

            # Immutable
            (re.compile("^ATE(?P<value>[0-1])$", re.IGNORECASE),          lambda val: self._immutable_setting(not self._echo_off, int(val))),
            #(re.compile("^ATH(?P<value>[0-1])$", re.IGNORECASE),          lambda val: self._immutable_setting(self._print_headers, int(val))),

            # Mutable
            (re.compile("^ATAT(?P<value>[0-2])$", re.IGNORECASE),         lambda val: self.set_adaptive_timing(int(val))),
            (re.compile("^ATR(?P<value>[0-1])$", re.IGNORECASE),          lambda val: self.set_expect_responses(int(val))),
            (re.compile("^ATD$", re.IGNORECASE),                          self.restore_defaults),
            (re.compile("^ATWS$", re.IGNORECASE),                         self.warm_reset),
            (re.compile("^ATZ$", re.IGNORECASE),                          self.warm_reset),  # NOTE: Do not perform hard reset
            (re.compile("^ATSH(?P<value>[0-9A-F]+)$", re.IGNORECASE),     self.set_header),
            (re.compile("^ATSP(?P<value>[0-9A-C])$", re.IGNORECASE),      lambda val: self.set_protocol(None if val == "0" else val, verify=False)),
            (re.compile("^ATST(?P<value>[0-9A-F]{1,2})$", re.IGNORECASE), lambda val: self.set_response_timeout(int(val, 16))),
            (re.compile("^ATCAF(?P<value>[0-1])$", re.IGNORECASE),        lambda val: self.set_can_auto_format(int(val))),
            (re.compile("^ATCEA(?P<value>[0-9A-F]*)$", re.IGNORECASE),    self.set_can_extended_address),
            (re.compile("^ATCP(?P<value>[0-9A-F]{1,2})$", re.IGNORECASE), self.set_can_priority),
            (re.compile("^ATS(?P<value>[0-1])$", re.IGNORECASE),          lambda val: self.set_print_spaces(int(val))),
        ]

        self._read_line_buffer = None


    def open(self, baudrate, protocol=None, echo_off=True, print_headers=True):
        """
        Opens serial connection and initializes ELM327 interface.
        """

        logger.info("Opening interface connection: Port={:}, Baudrate={:}, Protocol={:}".format(
            self._port.port,
            "auto" if baudrate is None else baudrate,
            "auto" if protocol is None else protocol
        ))

        # Open serial connection
        try:
            self._port.open()
        except:
            logger.exception("Failed to open serial connection")

            # Remember to report back status
            self._trigger_status_callback()

            raise

        # Set the ELM's baudrate
        try:
            self.set_baudrate(baudrate)
        except:
            logger.exception("Failed to set baudrate of serial connection")

            self.close()

            raise

        # Configure ELM settings
        try:

            # Check if ready
            res = self.send("ATI", delay=1, raw_response=True)  # Wait 1 second for ELM to initialize
            # Return data can be junk, so don't bother checking

            # Determine if echo is on or off
            res = self.send("ATI", raw_response=True)
            self._echo_off = not self._has_message(res, "ATI")

            # Load current settings from programmable parameters
            params = self._get_pps()

            has_changed = False

            # Set echo on/off
            if self._ensure_pp(params[self.PP_ATE], "FF" if echo_off else "00", default="00"):
                has_changed = True
            # Echo has maybe been changed manually (using ATE0/1) - reset to load setting from PP
            elif self._echo_off != echo_off:
                has_changed = True

            # Enable/disable printing of headers
            if self._ensure_pp(params[self.PP_ATH], "00" if print_headers else "FF", default="FF"):
                has_changed = True

            if has_changed:
                logger.info("Changes have been made to programmable parameter(s)")
            
            # Always perform soft reset to:
            #   - Restore all default runtime settings
            #   - Reload changed programmable parameter(s)
            #   - Avoid any continuing protocol detection/verification problems
            self.warm_reset()

            # Finally update setting variables with possible new values
            self._echo_off = echo_off
            self._print_headers = print_headers

        except:
            logger.exception("Failed to configure ELM settings")

            self.close()

            raise

        # By now, we've successfuly communicated with the ELM, but not the car
        self._status = OBDStatus.ITF_CONNECTED

        # Remember to report back status
        self._trigger_status_callback()

        # Try to communicate with the car, and load the correct protocol parser
        try:
            prot = protocol if isinstance(protocol, dict) else {"id": protocol}
            self.set_protocol(prot.pop("id", None), **prot)
        except ELM327Error as err:
            logger.warning(str(err))

            return

        logger.info("Connected successfully to vehicle: Port={:}, Baudrate={:}, Protocol={:}".format(
            self._port.port,
            self._port.baudrate,
            self._protocol.ID
        ))


    def close(self):
        """
        Closes connection to interface.
        """

        if self._status == OBDStatus.NOT_CONNECTED:
            return

        self._status = OBDStatus.NOT_CONNECTED
        
        if self._port is not None:
            logger.info("Closing port connection")

            self._port.close()

        # Report status changed
        self._trigger_status_callback()


    def reopen(self):
        """
        Closes and opens connection again to interface.
        """

        self.close()

        self.open(self._port.baudrate if self._port else None,
            protocol={
                "id": getattr(self._protocol, "ID", None),
                "baudrate": getattr(self._protocol, "baudrate", None)
            } if not getattr(self._protocol, "autodetected", True) else None
        )


    def restore_defaults(self):
        """
        Set the options to their default (or factory) settings, as when power is first applied.
        """

        res = self.send("ATD")
        if self._is_ok(res):
            logger.info("Default settings restored")

            # Clear any cached runtime settings
            self._runtime_settings.clear()


    def warm_reset(self):
        """
        Soft reset that keeps the user selected baud rate.
        """

        try:
            self.send("ATWS")
        finally:

            # Clear any cached runtime settings
            self._runtime_settings.clear()


    def reset(self):
        """
        Full reset. Serial connection is closed and re-opened.
        """

        self.send("ATZ")

        # Clear any cached runtime settings
        self._runtime_settings.clear()

        self.reopen()


    def connection(self):
        return self._port


    def status(self):
        return self._status


    def runtime_settings(self):
        return self._runtime_settings


    @classmethod
    def supported_protocols(cls):
        return cls.SUPPORTED_PROTOCOLS


    def protocol(self, verify=True):

        # Verify protocol if requested and not already unknown
        if verify and not isinstance(self._protocol, UnknownProtocol):
            try:
                self._verify_protocol(self._protocol.ID)
            except ELM327Error as err:
                self._unknown_protocol()

                logger.warning(str(err))
            except:
                self._unknown_protocol()

                raise

        return self._protocol


    def ecus(self):
        return self._protocol.ecu_map.values() if self._protocol else []


    def set_protocol(self, ident, **kwargs):

        # Validate protocol identifier if given
        if ident != None and ident not in self.supported_protocols():
            raise ELM327Error("Unsupported protocol '{:}'".format(ident))

        try:

            # Autodetect protocol
            if ident == None:
                self._protocol = self._auto_protocol(**kwargs)
                self._protocol.autodetected = True

                logger.info("Protocol '{:}' set automatically: {:}".format(self._protocol.ID, self._protocol))

            # Set explicit protocol
            else:
                self._protocol = self._manual_protocol(ident, **kwargs)
                self._protocol.autodetected = False

                logger.info("Protocol '{:}' set manually: {:}".format(self._protocol.ID, self._protocol))

            # Update overall status
            self._status = OBDStatus.BUS_CONNECTED

            # Report status changed
            self._trigger_status_callback(protocol=self._protocol)

        except:
            self._unknown_protocol()

            raise

        return self._protocol


    def set_baudrate(self, baudrate):
        if baudrate == None:
            
            # When connecting to pseudo terminal, don't bother with auto baud
            if self._port.portstr.startswith("/dev/pts"):
                logger.warning("Detected pseudo terminal, skipping baudrate setup")
                return
            
            # Autodetect baudrate using default choices list
            self._auto_baudrate(self.TRY_BAUDRATES)

        elif isinstance(baudrate, list):

            # Autodetect baudrate using given choices list
            self._auto_baudrate(baudrate)

        else:

            # Create a list of choices with given baudrate as first entry
            choices = list(self.TRY_BAUDRATES)
            choices.remove(baudrate)
            choices.insert(0, baudrate)

            self._auto_baudrate(choices)


    def set_expect_responses(self, value):
        """
        Turn responses on or off. Default is True.
        """

        if value == self._runtime_settings.get("expect_responses", True):
            return

        try:
            res = self.send("ATR{:d}".format(value))
        except ELM327Error as err:
            raise ELM327Error("Unable to set expect responses '{:}': {:}".format(value, err), code=err.code)

        if not self._is_ok(res):
            raise ELM327Error("Invalid response when setting expect responses '{:}': {:}".format(value, res), code=self._last(res))

        if logger.isEnabledFor(logging.DEBUG):
            logger.debug("Changed responses from '{:}' to '{:}'".format(self._runtime_settings.get("expect_responses", None), value))

        self._runtime_settings["expect_responses"] = value


    def set_response_timeout(self, value):
        """
        Set timeout to value x 4 ms. Default is 50 (hex 32) giving a time of approximately 200 msec.
        """

        if value == self._runtime_settings.get("response_timeout", 50):
            return

        try:
            res = self.send("ATST{:X}".format(value))
        except ELM327Error as err:
            raise ELM327Error("Unable to set response timeout '{:}': {:}".format(value, err), code=err.code)

        if not self._is_ok(res):
            raise ELM327Error("Invalid response when setting response timeout '{:}': {:}".format(value, res), code=self._last(res))

        if logger.isEnabledFor(logging.DEBUG):
            logger.debug("Changed response timeout from '{:}' to '{:}'".format(self._runtime_settings.get("response_timeout", None), value))

        self._runtime_settings["response_timeout"] = value


    def set_adaptive_timing(self, value):
        """
        Set adaptive timing mode. Default is 1.
        Sometimes, a single OBD requests results in multiple response frames.
        The time between frames varies significantly depending on the vehicle year, make, and model – from as low as 5 ms up to 100 ms.

        Mode options:
            0 = Adaptive timing off (fixed timeout).
            1 = Adaptive timing on, normal mode. This is the default option.
            2 = Adaptive timing on, aggressive mode. This option may increase throughput on slower connections, at the expense of slightly increasing the risk of missing frames.
        """

        if value == self._runtime_settings.get("adaptive_timing", 1):
            return

        try:
            res = self.send("ATAT{:d}".format(value))
        except ELM327Error as err:
            raise ELM327Error("Unable to set adaptive timing '{:}': {:}".format(value, err), code=err.code)

        if not self._is_ok(res):
            raise ELM327Error("Invalid response when setting adaptive timing '{:}': {:}".format(value, res), code=self._last(res))

        if logger.isEnabledFor(logging.DEBUG):
            logger.debug("Changed adaptive timing from '{:}' to '{:}'".format(self._runtime_settings.get("adaptive_timing", None), value))

        self._runtime_settings["adaptive_timing"] = value


    def set_header(self, value):
        """
        Set header value to use when sending request(s). For OBD-II CAN protocols default is '7DF'.
        """

        default = self.OBDII_HEADER if isinstance(self._protocol, CANProtocol) else None

        value = default if value == None else str(value).upper()
        if value == self._runtime_settings.get("header", default):
            return
        elif value == None:
            logger.warning("Performing warm reset to force default header")
            self.warm_reset()

            return

        try:
            res = self.send("ATSH" + value)
        except ELM327Error as err:
            raise ELM327Error("Unable to set header '{:}': {:}".format(value, err), code=err.code)

        if not self._is_ok(res):
            raise ELM327Error("Invalid response when setting header '{:}': {:}".format(value, res), code=self._last(res))

        if logger.isEnabledFor(logging.DEBUG):
            logger.debug("Changed header from '{:}' to '{:}'".format(self._runtime_settings.get("header", None), value))

        self._runtime_settings["header"] = value


    def set_can_auto_format(self, value):
        """
        Enable/disable CAN automatic formatting. Default is True.
        """

        if value == self._runtime_settings.get("can_auto_format", True):
            return

        try:
            res = self.send("ATCAF{:d}".format(value))
        except ELM327Error as err:
            raise ELM327Error("Unable to set CAN automatic formatting '{:}': {:}".format(value, err), code=err.code)

        if not self._is_ok(res):
            raise ELM327Error("Invalid response when setting CAN automatic formatting '{:}': {:}".format(value, res), code=self._last(res))

        if logger.isEnabledFor(logging.DEBUG):
            logger.debug("Changed CAN automatic formatting from '{:}' to '{:}'".format(self._runtime_settings.get("can_auto_format", None), value))

        self._runtime_settings["can_auto_format"] = value


    def set_can_extended_address(self, value):
        """
        Use CAN extended address.
        """

        if value == self._runtime_settings.get("can_extended_address", None):
            return

        if value:
            try:
                res = self.send("ATCEA{:s}".format(value))
            except ELM327Error as err:
                raise ELM327Error("Unable to set CAN extended address '{:}': {:}".format(value, err), code=err.code)
        else:
            try:
                res = self.send("ATCEA")
            except ELM327Error as err:
                raise ELM327Error("Unable to clear CAN extended addresses: {:}".format(err), code=err.code)
            
        if not self._is_ok(res):
            raise ELM327Error("Invalid response when setting CAN extended address '{:}': {:}".format(value, res), code=self._last(res))

        if logger.isEnabledFor(logging.DEBUG):
            logger.debug("Changed CAN extended address from '{:}' to '{:}'".format(self._runtime_settings.get("can_extended_address", None), value))

        self._runtime_settings["can_extended_address"] = value


    def set_can_priority(self, value):
        """
        Set CAN priority bits of a 29-bit CAN ID. This command sets the five most significant bits of transmitted frames.
        """

        if value == self._runtime_settings.get("can_priority", None):
            return

        try:
            res = self.send("ATCP{:s}".format(value))
        except ELM327Error as err:
            raise ELM327Error("Unable to set CAN priority '{:}': {:}".format(value, err), code=err.code)

        if not self._is_ok(res):
            raise ELM327Error("Invalid response when setting CAN priority '{:}': {:}".format(value, res), code=self._last(res))

        if logger.isEnabledFor(logging.DEBUG):
            logger.debug("Changed CAN priority from '{:}' to '{:}'".format(self._runtime_settings.get("can_priority", None), value))

        self._runtime_settings["can_priority"] = value


    def set_print_spaces(self, value):
        """
        Turn printing of spaces in OBD responses on or off. To get better performance, turn spaces off. Default is True.
        """

        if value == self._runtime_settings.get("print_spaces", True):
            return

        try:
            res = self.send("ATS{:d}".format(value))
        except ELM327Error as err:
            raise ELM327Error("Unable to set print spaces '{:}': {:}".format(value, err), code=err.code)

        if not self._is_ok(res):
            raise ELM327Error("Invalid response when setting print spaces '{:}': {:}".format(value, res), code=self._last(res))

        if logger.isEnabledFor(logging.DEBUG):
            logger.debug("Changed print spaces from '{:}' to '{:}'".format(self._runtime_settings.get("print_spaces", None), value))

        self._runtime_settings["print_spaces"] = value


    def _normalize_cmd(self, cmd):
        if isinstance(cmd, bytearray):
            try:
                return bytes(cmd).decode("ascii")
            except UnicodeDecodeError:
                raise TypeError("Command must be ASCII bytes or str")

        # In Python 2, bytes is an alias of str, so only decode true bytes objects.
        if isinstance(cmd, bytes) and not isinstance(cmd, str):
            try:
                return cmd.decode("ascii")
            except UnicodeDecodeError:
                raise TypeError("Command must be ASCII bytes or str")

        return cmd


    def query(self, cmd, header=None, parse=True, read_timeout=None):
        """
        Used to service all OBDCommands.

        Sends the given command string, and if requested
        parses the response lines with the protocol object.

        An empty command string will re-trigger the previous command.

        Returns a list of parsed Message objects or raw response lines.
        """

        cmd = self._normalize_cmd(cmd)

        if not cmd.startswith("AT"):  # Special commands do not need this (ELM_VERSION, ELM_VOLTAGE)

            # Ensure header is set
            self.set_header(header)

            # Ensure CAN automatic formatting is enabled
            self.set_can_auto_format(True)

            # Ensure responses are turned on
            self.set_expect_responses(True)

        lines = self.send(cmd, read_timeout=read_timeout, raw_response=not parse)

        # Parse using protocol if requested
        if parse:
            messages = self._protocol(lines)
            return messages
        
        return lines


    def relay(self, cmd, raw_response=False):
        """
        Ralays any command to the interface.
        """

        cmd = self._normalize_cmd(cmd)

        try:

            # If an AT command try to find a matching mapping
            if cmd[:2].upper() in ["AT"]:
                for regex, func in self._at_command_mappings:
                    match = regex.match(cmd.strip().replace(" ", ""))
                    if match:
                        func(*match.groups())

                        # All good, return a single OK line
                        return [self.OK]

                if logger.isEnabledFor(logging.DEBUG):
                    logger.debug("No mapping found for AT command '{:}' - sending it directly to interface".format(cmd))

            lines = self.send(cmd, raw_response=raw_response)

        except Exception as ex:
            logger.exception("Failed to relay command '{:}' to interface".format(cmd))

            if not raw_response:
                raise
            else:

                # Try to extract 'code' from ELM327Error exceptions
                lines = [getattr(ex, "code", None) or "FAIL"]

        return lines


    def send(self, cmd, delay=None, read_timeout=None, interrupt_delay=None, raw_response=False):
        """
        Low-level send of a raw command string.

        Will write the given string, no questions asked.
        Returns read result (a list of line strings) after an optional delay.
        """

        cmd = self._normalize_cmd(cmd)

        self._write(cmd)

        if delay is not None:

            if logger.isEnabledFor(logging.DEBUG):
                logger.debug("Wait %d seconds" % delay)

            time.sleep(delay)

        lines = self._read(timeout=read_timeout, interrupt_delay=interrupt_delay)

        if not lines:
            if logger.isEnabledFor(logging.DEBUG):
                logger.debug("Got no response on command: {:}".format(cmd))

            return lines

        # Return raw response
        if raw_response:
            return lines

        # Filter out echo if present
        if not self._echo_off:

            # Sanity check if echo matches sent command
            if cmd != lines[0]:
                logger.warning("Sent command does not match echo: '{:}' != '{:}'".format(cmd, lines[0]))
            else:
                lines = lines[1:]

        # Check for errors
        for line in lines:
            if line in self.ERRORS:
                raise ELM327Error(self.ERRORS[line], code=line)

        return lines


    def _auto_baudrate(self, choices):
        """
        Detect the baud rate at which a connected ELM32x interface is operating.
        """

        # Before we change the timout, save the "normal" value
        timeout = self._port.timeout
        self._port.timeout = 0.1  # We're only talking with the ELM, so things should go quickly

        try:
            for baudrate in choices:
                self._port.baudrate = baudrate
                self._port.flushInput()
                self._port.flushOutput()

                # Send a nonsense command to get a prompt back from the scanner
                # (an empty command runs the risk of repeating a dangerous command)
                # The first character might get eaten if the interface was busy,
                # so write a second one (again so that the lone CR doesn't repeat
                # the previous command)
                self._port.write(b"\x7F\x7F\r")
                self._port.flush()

                res = self._port.read(1024)

                if logger.isEnabledFor(logging.DEBUG):
                    logger.debug("Response from baudrate choice '%d': %s" % (baudrate, repr(res)))

                # Watch for the prompt character
                if res.endswith(self.PROMPT):
                    logger.info("Choosing baudrate '%d'" % baudrate)

                    return

            raise ELM327Error("Unable to automatically find baudrate from given choices")
        finally:
            self._port.timeout = timeout  # Reinstate our original timeout


    def _verify_protocol(self, ident, test=False):
        ret = []

        ignore_lines = ["SEARCHING...", "NO DATA"]
        for line in self.query("0100", parse=False, read_timeout=10):

            # Skip ignore lines
            if line in ignore_lines:
                continue

            # Check if valid hex
            try:
                int(line.replace(" ", ""), 16)
            except ValueError:
                err = self.ERRORS.get(line, "Invalid non-hex response: {:}".format(line))

                msg = "Unable to verify connectivity of protocol '{:}': {:}".format(ident, err)
                if test:
                    logger.warning(msg)

                    return []
                else:
                    raise ELM327Error(msg, code=line)

            ret.append(line)

        if not ret:

            msg = "No data received when trying to verify connectivity of protocol '{:}'".format(ident)
            if test:
                logger.warning(msg)

                return []
            else:
                raise ELM327Error(msg)

        return ret


    def _manual_protocol(self, ident, verify=True, **kwargs):

        # Change protocol
        res = self.send("ATTP" + ident)
        if not self._is_ok(res):
            raise ELM327Error("Invalid response when manually changing to protocol '{:}': {:}".format(ident, res), code=self._last(res))

        # Verify protocol connectivity
        res_0100 = self._verify_protocol(ident, test=not verify)

        # Verify protocol changed
        res = self.send("ATDPN")
        if not self._has_message(res, ident):
            raise ELM327Error("Manually changed protocol '{:}' does not match currently active protocol '{:}'".format(ident, res))

        # Initialize protocol parser
        return self.supported_protocols()[ident](res_0100)


    def _auto_protocol(self, verify=True, **kwargs):
        """
        Attempts communication with the car.
        Upon success, the appropriate protocol parser is loaded.
        """

        # Set auto protocol mode
        res = self.send("ATSP0")
        if not self._is_ok(res):
            raise ELM327Error("Invalid response when setting auto protocol mode: {:}".format(res), code=self._last(res))

        # Search for protocol and verify connectivity
        res_0100 = self._verify_protocol("auto", test=not verify)

        # Get protocol number
        res = self.send("ATDPN")
        if len(res) != 1:
            logger.error("Invalid response when getting protocol number: {:}".format(res))
            raise ELM327Error("Failed to retrieve current protocol after searching for protocol automatically")

        ident = res[0]  # Grab the first (and only) line returned
        # Suppress any "automatic" prefix
        ident = ident[1:] if (len(ident) > 1 and ident.startswith("A")) else ident

        # Check if the protocol is supported
        if not ident in self.supported_protocols():
            raise ELM327Error("Automatically detected protocol '{:}' is not supported".format(ident))

        # Instantiate the corresponding protocol parser
        return self.supported_protocols()[ident](res_0100)


    def _unknown_protocol(self):
        self._protocol = UnknownProtocol([])

        # Update overall status
        if self._status == OBDStatus.BUS_CONNECTED:
            self._status = OBDStatus.BUS_DISCONNECTED

            # Report status changed
            self._trigger_status_callback()


    def _trigger_status_callback(self, **kwargs):
        if self._status_callback:
            try:
                self._status_callback(self._status, **kwargs)
            except:
                logger.exception("Failed to trigger status callback")


    def _get_pps(self):
        """
        Retrieves all programmable parameters.
        """

        ret = {}

        lines = self.send("ATPPS")
        for line in lines:
            for param in line.split("  "):
                match = re.match("^(?P<id>[0-9A-F]{2}):(?P<value>[0-9A-F]{2}) (?P<state>[N|F]{1})$", param)
                if match:
                    group = match.groupdict()
                    ret[group["id"]] = group
                else:
                    raise ELM327Error("Unable to parse programmable parameter: {:}".format(param))

        return ret


    def _set_pp(self, id, value, enable=None):
        """
        Sets value of a programmable parameter and enables it if requested.
        """

        res = self.send("ATPP{:s} SV{:s}".format(id, value))
        if self._is_ok(res):
            logger.info("Updated programmable parameter '{:}' value '{:}'".format(id, value))
        else:
            raise ELM327Error("Failed to set programmable parameter '{:}' value '{:}': {:}".format(id, value, res), code=self._last(res))

        if enable is not None:
            res = self.send("ATPP{:s} {:s}".format(id, "ON" if enable else "OFF"))
            if self._is_ok(res):
                logger.info("{:} programmable parameter '{:}'".format("Enabled" if enable else "Disabled", id))
            else:
                raise ELM327Error("Failed to {:} programmable parameter '{:}': {:}".format("enable" if enable else "disable", id, res), code=self._last(res))


    def _ensure_pp(self, param, value, default=None):
        """
        Ensures a programmable parameter value is set if required.
        Returns True of False indicating if changes have been made.
        """

        # Check if default value and state is already fulfilled
        if default != None and default == value and default == param["value"] and param["state"] == "F":
            return False

        # Check if value is changed
        if value == param["value"] and param["state"] == "N":
            return False

        # Go ahead and update parameter
        if default != None and default == value:
            self._set_pp(param["id"], default, enable=False)
        else:
            self._set_pp(param["id"], value, enable=True)

        return True


    def _interrupt(self, ready_wait=True):
        if logger.isEnabledFor(logging.DEBUG):
            logger.debug("Write to interrupt: " + repr(self.INTERRUPT))

        self._port.flushInput()  # Dump everything in the input buffer
        self._port.write(self.INTERRUPT)  # Write an interrupt character
        self._port.flush()  # Wait for the output buffer to finish transmitting

        # Wait for ready prompt
        if ready_wait:
            self._read()


    def _write(self, cmd):
        """
        Low-level function to write a string to the port.
        """

        if not self._port or not self._port.is_open:
            raise ELM327Error("Cannot write when serial connection is not open")

        # Ensure in interactive state before writing any command
        if self._state != self.STATE_INTERACTIVE:
            try:
                self._interrupt()
            finally:

                # Always go to interactive state
                self._state = self.STATE_INTERACTIVE

        cmd += "\r"  # Terminate with carriage return in accordance with ELM327 and STN11XX specifications
        
        if logger.isEnabledFor(logging.DEBUG):
            logger.debug("Write: " + repr(cmd))

        self._port.flushInput()  # Dump everything in the input buffer
        self._port.write(cmd.encode())  # Turn the string into bytes and write
        self._port.flush()  # Wait for the output buffer to finish transmitting


    def _read(self, timeout=None, interrupt_delay=None):
        """
        Low-level read function.

        Accumulates characters until the prompt character is seen.
        Returns a list of [/r/n] delimited strings.
        """

        if not self._port or not self._port.is_open:
            raise ELM327Error("Cannot read when serial connection is not open")

        # Override default timeout if requested
        if timeout != None and self._port.timeout != timeout:
            self._port.timeout = timeout

        try:

            buffer = bytearray()
            start = timer()

            while True:

                # Retrieve as much data as possible
                data = self._port.read(self._port.in_waiting or 1)

                # If nothing was recieved
                if not data:
                    logger.error("No more data received on serial port within timeout of {:} second(s) - maybe ELM327 response timeout is set to a higher value, or the connection itself may be unstable due to high baud rate".format(self._port.timeout))
                    logger.warning("Partial data received until timeout occurred: {:}".format(repr(buffer)))

                    # Only break if no interrupt character is pending
                    if interrupt_delay == None:
                        break

                buffer.extend(data)

                # End on chevron + carriage return (ELM prompt character)
                if buffer.endswith(self.PROMPT):
                    break

                # Check if it is time to send an interrupt character
                if interrupt_delay != None and (timer() - start) >= interrupt_delay:
                    interrupt_delay = None

                    self._interrupt(ready_wait=False)

            # Log, and remove the "bytearray(   ...   )" part
            if logger.isEnabledFor(logging.DEBUG):
                logger.debug("Read: " + repr(buffer)[10:-1])

            # Clean out any null characters
            buffer = re.sub(b"\x00", b"", buffer)

            # Remove the prompt characters
            if buffer.endswith(self.PROMPT):
                buffer = buffer[:-len(self.PROMPT)]

            # Convert bytes into a standard string
            string = buffer.decode()

            # Splits into lines while removing empty lines and trailing spaces
            lines = [s.strip() for s in re.split("[\r\n]", string) if bool(s)]

            return lines

        finally:

            # Restore default timeout if changed
            if self._port.timeout != self._default_timeout:
                self._port.timeout = self._default_timeout


    def _read_line(self, wait=True, buffer_size=2048):
        """
        Low-level function to read a single line. The return value is an array of bytes.
        """

        if not self._port or not self._port.is_open:
            raise ELM327Error("Cannot read line when serial connection is not open")

        # Prepare read buffer if not already initialized
        if not self._read_line_buffer:
            self._read_line_buffer = BufferedSerialReader(self._port, size=buffer_size)

            if logger.isEnabledFor(logging.DEBUG):
                logger.debug("Created buffered serial reader of size {:}".format(buffer_size))

        # Decide to skip or continue and wait if no data pending
        if not wait and not self._read_line_buffer.in_waiting:
            if logger.isEnabledFor(logging.DEBUG):
                logger.debug("Skipping read line because no data is currently pending on serial port")

            return

        # Wait for read of entire line or until timeout
        res = self._read_line_buffer.read_until(b"\r")
        if not res:
            if logger.isEnabledFor(logging.DEBUG):
                logger.debug("No line could be read on serial port within timeout of {:} second(s)".format(self._port.timeout))

            return

        if logger.isEnabledFor(logging.DEBUG):
            logger.debug("Read line: " + repr(res))

        if res == self.PROMPT:
            logger.warning("No more lines available to read on serial port - interface returned to interactive state, if not already")

            # We know for sure that we are in interactive state because of the prompt character
            self._state = self.STATE_INTERACTIVE

            return

        return res  # NOTE: Returns byte array


    def _is_ok(self, lines, expect_echo=False):
        if not lines:
            return False

        if not self._echo_off or expect_echo:
            return self._has_message(lines, self.OK) == self.OK

        return len(lines) == 1 and lines[0] == self.OK


    def _last(self, lines):
        if not lines:
            return None

        return lines[-1]


    def _has_message(self, lines, *args):
        for line in lines:
            for arg in args:
                if arg in line:
                    return arg

        return None


    def _immutable_setting(self, actual, wanted):
        if actual != wanted:
            raise ELM327Error("Immutable setting has actual value '{:}' but '{:}' is wanted".format(actual, wanted), code="IMMUTABLE")

