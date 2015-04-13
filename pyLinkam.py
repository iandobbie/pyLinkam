# coding: utf-8
"""pyLinkam

Copyright 2014-2015 Mick Phillips (mick.phillips at gmail dot com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
=============================================================================

python wrapper for Linkam's T95 interfacing DLL.

Requires .NET Framework 4.5.1 Full.

Note that return values are cast as native python types to 
enable remote calls over Pyro from python instances without
support for .NET.
"""

import clr
import sys
import ctypes
import time
import Pyro4
import threading
import distutils.version

if (distutils.version.LooseVersion(Pyro4.__version__) >=
    distutils.version.LooseVersion('4.22')):
    Pyro4.config.SERIALIZERS_ACCEPTED.discard('serpent')
    Pyro4.config.SERIALIZERS_ACCEPTED.add('pickle')
Pyro4.config.SERIALIZER = 'pickle'

CONFIG_NAME = 'linkam'
DLL_PATH = r"C:\b24\pyLinkam\linkam"

sys.path.append(DLL_PATH)
clr.AddReference(r"LinkamCommsLibrary.dll")
import LinkamCommsDll


class _IntParser(object):
    """A class that parses bits in a (u)Int to attributes of itself."""
    # Mapping of bit numbers to attribute names. Override.
    _bitFields = {}
    def __init__(self):
        # Initialise empty attributes.
        for key, bit in self._bitFields.iteritems():
            setattr(self, key, None)


    def update(self, uInt):
        """Update attributes with new values."""
        self._value = long(uInt)
        for key, bit in self._bitFields.iteritems():
            setattr(self, key, self._value & 2**bit > 0)


class _StageConfig(_IntParser):
    # stageTypes and bitFields from SDK docs, appear to be correct.
    _stageTypes = {2**0: 'standard',
                   2**1: 'high temp',
                   2**2: 'peltier',
                   2**3: 'graded',
                   2**4: 'tensile',
                   2**5: 'dsc',
                   2**6: 'warm',
                   2**7: 'ito',
                   2**8: 'css450',
                   2**9: 'correlative',}

    _bitFields = {'motorisedX': 49,
                  'motorisedY': 50}


    def update(self, u64):
        super(_StageConfig, self).update(u64)
        self.stageType = self._stageTypes[u64 & 0b1111111111]
        

class _StageStatus(_IntParser):
    # These bitFields are as per the documentation, but they are incorrect.
    _bitFields = {'errorState': 0,
                  'heater1RampAtSetPoint': 1,
                  'heater1Started': 2,
                  'heater2RampAtSetPoint': 3,
                  'heater2Started': 4,
                  'vacuumRampAtSetPoint': 5,
                  'vacuumControlStarted': 6,
                  'vacuumValveTAtMaxVac': 7,
                  'vacuumValveAtAtmos': 8,
                  'humidityRampAtSetPoint': 9, 
                  'humidityStarted': 10,
                  'coolingPumpPumping': 11,
                  'coolingPumpInAutoMode': 12,
                  'Ramp1': 13,
                  'xMotorAtMinTravel': 41,
                  'xMotorAtMaxTravel': 42,
                  'xMotorStopped': 43,
                  'yMotorAtMinTravel': 44,
                  'yMotorAtMaxTravel': 45,
                  'yMotorStopped': 46,
                  'zMotorAtMinTravel': 47,
                  'zMotorAtMaxTravel': 48,
                  'zMotorStopped': 49,
                  'heater1SampleCalibrationApplied': 50,
                  }


class LinkamStage(object):
    """An interface to Linkam's .net assembly for their stages."""
    # Enumerated value types for SetValue and GetValue.
    eVALUETYPE = LinkamCommsDll.Comms.eVALUETYPE
    XMOTOR_BIT = 2**45
    YMOTOR_BIT = 2**48


    def __init__(self):
        # Comms link to the hardware.
        self.stage = LinkamCommsDll.Comms()
        # Reported stage configuration.
        self.stageConfig = _StageConfig()
        # Stage status.
        self.status = _StageStatus()
        # A flag to show if the stage has been connected.
        self.connected = False
        # A flag to show that the motors have been homed.
        self.motorsHomed = False
		# A handler to detect stage disconnection events.
        self.stage.ControllerDisconnected += self.disconnectEventHandler


    def disconnectEventHandler(self, sender, eventArgs):
        """Handles stage disconnection events."""
        # Indicate that the stage is no longer connected.
        self.connected = False
        # Indicated that the motors are not homed.
        self.motorsHomed = False


    def connect(self, reconnect=False):
        if self.connected and not reconnect:
            return None
        result = self.stage.OpenComms(True, 0, 0)
        self.connected = result & 0x0001
        if self.connected:
            self.stageConfig.update(self.stage.GetStageConfig())
            self.status.update(self.stage.GetStatus())
            return True
        else:
            return dict(connected = bool(connected),
                    commsNotResponding = long(result) & 0b0010,
                    commsFailedToSendConfigData = long(result) & 0b0100,
                    commsSerialPortError = long(result) & 0b1000)


    def getConfig(self):
        configWord = self.stage.GetStageConfig()
        self.stageConfig.update(configWord)
        return self.stageConfig


    def getPosition(self):
        """Fetch and return the stage's current position as (x, y)."""
        ValueIDs = (self.eVALUETYPE.u32XMotorPosnR.value__,
                    self.eVALUETYPE.u32YMotorPosnR.value__)
        return tuple((float(self.stage.GetValue(id)) for id in ValueIDs))


    def getStatus(self):
        statusWord = self.stage.GetStatus()
        self.status.update(statusWord)
        return self.status


    def homeMotors(self):
        """Home the motors.
        From CMS196.exe CMS196.frmMotorControl.btnIndex_Click.
        """
        self.moveToXY(-12000., -4000.)
        # Set this here, but would be better to wait until the
        # homing operation has finished by, say, watching the
        # stage position until it reaches (0, 0).
        self.motorsHomed = True


    def moveToXY(self, x=None, y=None):
        """Move stage motors to position (x, y)

        If either x or y is None, the position on that axis is unchanged.
        Return as soon as motion is started to avoid timeouts when called
        remotely. Use isMoving() to check movement status.
        """
        xValueID = self.eVALUETYPE.u32XMotorLimitRW.value__
        yValueID = self.eVALUETYPE.u32YMotorLimitRW.value__
        if x:
            self.stage.SetValue(xValueID, x)
            self.stage.StartMotors(True, 0)
        if y:
            self.stage.SetValue(yValueID, y)
            self.stage.StartMotors(True, 1)


    def isMoving(self):
        """Factored out from moveToXY.
        Blocking until move is completed will cause timeout errors when
        called remotely.
        """
        stoppedCount = 0
        maxCount = 3
        while stoppedCount < maxCount:
            status = self.stage.GetStatus()
            if status & self.XMOTOR_BIT and status & self.YMOTOR_BIT:
                stoppedCount += 1
            else:
                return True
            time.sleep(0.001)
        return False


class Server(object):
    def __init__(self):
        self.object = None
        self.daemon_thread = None
        self.config = None
        self.run_flag = True

    def __del__(self):
        self.run_flag = False
        if self.daemon_thread:
            self.daemon_thread.join()


    def run(self):
        import readconfig
        config = readconfig.config

        host = config.get(CONFIG_NAME, 'ipAddress')
        port = config.getint(CONFIG_NAME, 'port')

        self.object = LinkamStage()

        daemon = Pyro4.Daemon(port=port, host=host)

        # Start the daemon in a new thread.
        self.daemon_thread = threading.Thread(
            target=Pyro4.Daemon.serveSimple,
            args = ({self.object: CONFIG_NAME},),
            kwargs = {'daemon': daemon, 'ns': False}
            )
        self.daemon_thread.start()

        # Wait until run_flag is set to False.
        while self.run_flag:
            time.sleep(1)

        # Do any cleanup.
        daemon.shutdown()

        self.daemon_thread.join()


    def stop(self):
        self.run_flag = False


def main():
    s = Server()
    s.run()


if __name__ == '__main__':
    main()