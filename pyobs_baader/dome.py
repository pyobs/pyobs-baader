import math
import threading
import serial
import logging

from pyobs.modules import timeout
from pyobs.utils.threads import LockWithAbort

from pyobs.interfaces import IAltAz, IMotion
from pyobs.mixins import FollowMixin
from pyobs.modules.roof import BaseDome

log = logging.getLogger(__name__)


class BaaderDome(FollowMixin, BaseDome):
    """A pyobs module for a Baader dome."""

    def __init__(self, port: str = '/dev/ttyUSB0', baud_rate: int = 9600, byte_size: int = 8, parity: str = 'N',
                 stop_bits: int = 1, timeout: int = 180, tolerance: float = 2, follow: str = None, *args, **kwargs):
        """Initializes a new Baader dome.
        
        Args:
            port: COM port to use.
            baud_rate: Baud rate for connection.
            byte_size: Byte size for connection.
            parity: Parity (Y/N).
            stop_bits: Number of stop bits.
            timeout: Connection timeout in seconds.
            tolerance: Tolerance for azimuth.
            follow: Name of other device (e.g. telescope) to follow.
        """
        BaseDome.__init__(self, *args, **kwargs)

        # store it
        self._port = port
        self._baud_rate = baud_rate
        self._byte_size = byte_size
        self._stop_bits = stop_bits
        self._parity = parity
        self._timeout = timeout
        self._tolerance = tolerance

        # next command
        self._command_lock = threading.RLock()
        self._next_command = None

        # move locks
        self._lock_shutter = threading.RLock()
        self._abort_shutter = threading.Event()
        self._lock_move = threading.RLock()
        self._abort_move = threading.Event()
        
        # status
        self._shutter = None
        self._altitude = 0
        self._azimuth = 0

        # start thread
        self._add_thread_func(self._communication)

        # mixins
        FollowMixin.__init__(self, device=follow, interval=10, tolerance=tolerance, mode=IAltAz)

    @timeout(1200000)
    def init(self, *args, **kwargs):
        """Open dome.

        Raises:
            ValueError if dome cannot be opened.
        """

        # acquire lock
        with LockWithAbort(self._lock_shutter, self._abort_shutter):
            # log
            log.info('Opening roof...')
            self._change_motion_status(IMotion.Status.INITIALIZING)
    
            # set next command
            with self._command_lock:
                self._next_command = 'd#opeshut'
    
            # wait for it
            while self._shutter != 'd#shutope':
                # abort?
                if self._abort_shutter.is_set():
                    log.warning('Opening roof aborted.')
                    return
                
                # wait a little
                self._abort_shutter.wait(1)
                
            # set new status
            self._change_motion_status(IMotion.Status.POSITIONED)

    @timeout(1200000)
    def park(self, *args, **kwargs):
        """Close dome.

        Raises:
            ValueError if dome cannot be opened.
        """

        # acquire lock
        with LockWithAbort(self._lock_shutter, self._abort_shutter):
            # log
            log.info('Closing roof...')
            self._change_motion_status(IMotion.Status.PARKING)

            # set next command
            with self._command_lock:
                self._next_command = 'd#closhut'

            # wait for it
            while self._shutter != 'd#shutclo':
                # abort?
                if self._abort_shutter.is_set():
                    log.warning('Closing roof aborted.')
                    return

                # wait a little
                self._abort_shutter.wait(1)

            # set new status
            self._change_motion_status(IMotion.Status.PARKED)

    @timeout(1200000)
    def move_altaz(self, alt: float, az: float, *args, **kwargs):
        """Moves to given coordinates.

        Args:
            alt: Alt in deg to move to.
            az: Az in deg to move to.

        Raises:
            ValueError: If device could not move.
        """

        # acquire lock
        with LockWithAbort(self._lock_move, self._abort_move):
            # store altitude directory
            self._altitude = alt
            self._change_motion_status(IMotion.Status.SLEWING)
    
            # Baader measures azimuth as West of South, so we need to convert it
            azimuth = BaaderDome._adjust_azimuth(az)

            # set next command
            with self._command_lock:
                # format command, which is of format 'd#azi0900' with the number in units of 1/10 degree
                self._next_command = 'd#azi%04d' % int(math.floor(azimuth * 10))

            # wait for it
            while 180 - abs(abs(az - self._azimuth) - 180) > self._tolerance:
                # abort?
                if self._abort_shutter.is_set():
                    return

                # wait a little
                self._abort_shutter.wait(1)

            # set new status
            self._change_motion_status(IMotion.Status.POSITIONED)

    def get_altaz(self, *args, **kwargs) -> (float, float):
        """Returns current Alt and Az.

        Returns:
            Tuple of current Alt and Az in degrees.
        """
        return self._altitude, self._azimuth

    def stop_motion(self, device: str = None, *args, **kwargs):
        """Stop the motion.

        Args:
            device: Name of device to stop, or None for all.
        """

        # not supported, but don't want to raise an exception
        pass

    def is_ready(self, *args, **kwargs) -> bool:
        """Returns the device is "ready", whatever that means for the specific device.

        Returns:
            Whether device is ready
        """

        # check that motion is not in one of the states listed below
        return self.get_motion_status() not in [IMotion.Status.PARKED, IMotion.Status.INITIALIZING,
                                                IMotion.Status.PARKING, IMotion.Status.ERROR, IMotion.Status.UNKNOWN]

    def _communication(self):
        """Thread method for communicating with the dome."""

        # run forever
        while not self.closing.is_set():
            # get next command
            with self._command_lock:
                command = self._next_command
                self._next_command = None

            # got a command?
            if command is not None:
                # execute it!
                self._send_command(command)

            else:
                # nothing to do, so update status
                self._update_status()

            # sleep a little
            self.closing.wait(1)

    def _update_status(self):
        """Update status from dome."""

        # get shutter status, might be one of d#shutclo, d#shutrun, or d#shutope
        self._shutter = self._send_command('d#getshut')
        if self._shutter == 'd#shutclo':
            self._change_motion_status(IMotion.Status.PARKED)

        # get azimuth
        az_response = self._send_command('d#getazim')

        # parse azimuth
        if not az_response.startswith('d#azi'):
            # wrong response!
            self._azimuth = None
        else:
            # response is of form d#azi1800, so take lats four characters, parse to float and divide by 10
            az = float(az_response[5:]) / 10.

            # Baader measures azimuth as West of South, so we need to convert it
            self._azimuth = BaaderDome._adjust_azimuth(az)

    @staticmethod
    def _adjust_azimuth(az: float) -> float:
        """Baader measures azimuth as West of South, so we need to convert it. This works both ways.

        Args:
            az: Azimuth.

        Returns:
            Converted azimuth.
        """
        az += 180
        if az >= 360:
            az -= 360
        return az

    def _send_command(self, command: str, attempts: int = 5, wait: int = 5) -> str:
        """Executes command.

        Args:
            command: Baader command to send to the dome.
            attempts: Number of attempts for sending command.
            wait: Wait time in seconds between attempts.

        Returns:
            Reply from the dome.
        """

        # length of command must be 9
        if len(command) != 9:
            raise ValueError('Invalid command length.')

        # catch errors
        try:
            # connect to dome
            with serial.Serial(self._port, self._baud_rate, bytesize=self._byte_size, parity=self._parity,
                               stopbits=self._stop_bits, timeout=self._timeout) as ser:
                # send command
                ser.write(bytes(command, 'utf-8'))

                # read reply and decode it
                reply = ser.read(9).decode('utf-8')

        except serial.SerialException:
            # something went wrong
            reply = None

        # got error?
        if reply is None or reply == 'd#comerro':
            # another attempt?
            if attempts > 0:
                # wait a little
                self.closing.wait(wait)

                # retry
                self._send_command(command, attempts-1, wait)

            else:
                # no success
                raise ValueError('Could not send command.')

        # return response
        return reply


__all__ = ['BaaderDome']
