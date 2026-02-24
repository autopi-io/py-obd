# -*- coding: utf-8 -*-

########################################################################
#                                                                      #
# python-OBD: A python OBD-II serial module derived from pyobd         #
#                                                                      #
# Copyright 2004 Donour Sizemore (donour@uchicago.edu)                 #
# Copyright 2009 Secons Ltd. (www.obdtester.com)                       #
# Copyright 2009 Peter J. Creath                                       #
# Copyright 2016 Brendan Whitfield (brendan-w.com)                     #
#                                                                      #
########################################################################
#                                                                      #
# OBDCommand.py                                                        #
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

from .utils import *
from .protocols import ECU
from .OBDResponse import OBDResponse

import logging


logger = logging.getLogger(__name__)


class OBDCommand():
    def __init__(self,
                 name,
                 desc,
                 command,
                 _bytes,
                 decoder,
                 ecu=ECU.ALL,
                 fast=False,
                 frames=None,
                 strict=False):
        self.name      = name        # human readable name (also used as key in commands dict)
        self.desc      = desc        # human readable description
        self.command   = command     # command string
        self.bytes     = _bytes      # number of bytes expected in return
        self.decode    = decoder     # decoding function
        self.ecu       = ecu         # ECU ID from which this command expects messages from
        self.fast      = fast        # can an extra digit be added to the end of the command? (to make the ELM return early)
        self.frames    = frames      # Number of frames expected in return
        self.strict    = strict      # Perform strict validation of returned bytes and frames count

    def clone(self):
        return OBDCommand(self.name,
                          self.desc,
                          self.command,
                          self.bytes,
                          self.decode,
                          self.ecu,
                          self.fast,
                          self.frames,
                          self.strict)

    @property
    def mode(self):
        if len(self.command) >= 2 and \
           isHex(self.command.decode()):
            return int(self.command[:2], 16)
        else:
            return None

    @property
    def pid(self):
        if len(self.command) > 2 and \
           isHex(self.command.decode()):
            return int(self.command[2:], 16)
        else:
            return None


    def __call__(self, messages, filtering=True):

        if filtering:

            # filter for applicable messages (from the right ECU(s))
            for_us = lambda m: (self.ecu & m.ecu) > 0
            messages = list(filter(for_us, messages))

        if self.frames != None and self.frames != len(messages):
            if self.strict:
                raise OBDError("Command '{:}' expected {:} frames returned but got {:}".format(self, self.frames, len(messages)))

            logger.warning("Command '{:}' expected {:} frames returned but got {:}".format(self, self.frames, len(messages)))

        # guarantee data size for the decoder
        for m in messages:
            self.__constrain_message_data(m)

        # create the response object with the raw data recieved
        # and reference to original command
        r = OBDResponse(self, messages)
        if messages:
            r.value = self.decode(messages)
        else:
            logger.warning("Command '{:}' did not receive any acceptable messages".format(self))

        return r


    def __constrain_message_data(self, message):
        """ pads or chops the data field to the size specified by this command """
        if self.bytes > 0:
            if len(message.data) > self.bytes:
                if self.strict:
                    raise OBDError("Command '{:}' response message is longer than expected ({:}/{:})".format(self, len(message.data), self.bytes))

                # chop off the right side
                message.data = message.data[:self.bytes]

                logger.warning("Command '{:}' response message is longer than expected - trimmed message: {:}".format(self, repr(message.data)))
            elif len(message.data) < self.bytes:
                if self.strict:
                    raise OBDError("Command '{:}' response message is shorter than expected ({:}/{:})".format(self, len(message.data), self.bytes))

                # pad the right with zeros
                message.data += (b'\x00' * (self.bytes - len(message.data)))
                
                logger.warning("Command '{:}' response message is shorter than expected - padded message: {:}".format(self, repr(message.data)))


    def __str__(self):
        return "%s: %s" % (self.command, self.desc)

    def __hash__(self):
        # needed for using commands as keys in a dict (see _async.py)
        return hash(self.command)

    def __eq__(self, other):
        if isinstance(other, OBDCommand):
            return (self.command == other.command)
        else:
            return False
