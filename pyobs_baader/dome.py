import math
import threading
from queue import Queue
import serial
import logging

from pyobs.modules import timeout
from pyobs.utils.threads import LockWithAbort

from pyobs.interfaces import IAltAz, IMotion
from pyobs.mixins import FollowMixin
from pyobs.modules.roof import BaseDome

log = logging.getLogger(__name__)


class Command:
    def __init__(self, command: str):
        self.command = command
        self.response = None
        self.finished = threading.Event()

    def set(self, response: str):
        """Set response and trigger finished.

        Args:
            response: Reponse from command
        """
        self.response = response
        self.finished.set()

    def wait(self, timeout: int = 10) -> str:
        """Wait for command to finish.

        Args:
            timeout: Timeout in seconds.

        Returns:
            Response of command.

        Raises:
            TimeoutError if wait timed out.
        """
        self.finished.wait(timeout)
        if not self.finished.is_set():
            raise TimeoutError
        return self.response


class OpenCommand(Command):
    def __init__(self):
        Command.__init__(self, command='d#opeshut')


class CloseCommand(Command):
    def __init__(self):
        Command.__init__(self, command='d#closhut')


class SlewCommand(Command):
    def __init__(self, azimuth: float):
        Command.__init__(self, command='d#azi%04d' % int(math.floor(azimuth * 10)))


class GetShutCommand(Command):
    def __init__(self):
        Command.__init__(self, command='d#getshut')


class GetAzimCommand(Command):
    def __init__(self):
        Command.__init__(self, command='d#getazim')


class BaaderDome(FollowMixin, BaseDome):
    """A pyobs module for a Baader dome."""

    def __init__(self, port: str = '/dev/ttyUSB0', baud_rate: int = 9600, byte_size: int = 8, parity: str = 'N',
                 stop_bits: int = 1, timeout: int = 180, tolerance: float = 3, park_az: float = 180, follow: str = None,
                 *args, **kwargs):
        """Initializes a new Baader dome.
        
        Args:
            port: COM port to use.
            baud_rate: Baud rate for connection.
            byte_size: Byte size for connection.
            parity: Parity (Y/N).
            stop_bits: Number of stop bits.
            timeout: Connection timeout in seconds.
            tolerance: Tolerance for azimuth.
            park_az: Azimuth for park position.
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
        self._park_az = park_az

        # next command
        self._next_commands = Queue()

        # move locks
        self._lock_shutter = threading.RLock()
        self._abort_shutter = threading.Event()
        self._lock_move = threading.RLock()
        self._abort_move = threading.Event()
        
        # status
        self._shutter = None
        self._altitude = 0
        self._azimuth = 0
        self._set_az = 0

        # start thread
        self._add_thread_func(self._communication)
        self._add_thread_func(self._update_status)

        # init status to IDLE
        self._change_motion_status(IMotion.Status.IDLE)

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
            log.info('Opening dome...')
            self._change_motion_status(IMotion.Status.INITIALIZING)
    
            # execute command
            command = self._queue_command(OpenCommand())
            if command.wait() != 'd#gotmess':
                raise ValueError('Got invalid response from dome.')

            # wait for it
            while self._shutter != 'd#shutope':
                # abort?
                if self._abort_shutter.is_set():
                    log.warning('Opening dome aborted.')
                    return
                
                # wait a little
                self._abort_shutter.wait(1)
                
            # set new status
            log.info('Dome opened.')
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
            log.info('Closing dome...')
            self._change_motion_status(IMotion.Status.PARKING)

            # send command for closing shutter
            command = self._queue_command(CloseCommand())
            if command.wait() != 'd#gotmess':
                raise ValueError('Got invalid response from dome.')

            # move to park position and wait for it
            log.info('Moving to park position...')
            self._move(self._park_az, self._abort_move)

            # finally, wait for shutter to close
            while self._shutter != 'd#shutclo':
                # abort?
                if self._abort_shutter.is_set():
                    log.warning('Closing dome aborted.')
                    return

                # wait a little
                self._abort_shutter.wait(1)

            # set new status
            log.info('Dome closed.')
            self._change_motion_status(IMotion.Status.PARKED)

    def _move(self, az: float, abort: threading.Event):
        """Move the roof and wait for it.

        Args:
            az: Azimuth to move to.
            abort: Abort event.
        """

        # Baader measures azimuth as West of South, so we need to convert it
        azimuth = BaaderDome._adjust_azimuth(az)

        # execute command
        command = self._queue_command(SlewCommand(azimuth))
        if command.wait() != 'd#gotmess':
            raise ValueError('Got invalid response from dome.')

        # wait for it
        log_timer = 0
        while 180 - abs(abs(az - self._azimuth) - 180) > self._tolerance:
            # abort?
            if abort.is_set():
                return

            # log?
            if log_timer == 0:
                log.info('Moving dome from current az=%.2f° to %.2f° (%.2f° left)...',
                         self._azimuth, az, 180 - abs(abs(az - self._azimuth) - 180))
            log_timer += 1
            if log_timer == 10:
                log_timer = 0

            # wait a little
            abort.wait(1)

        # finished
        log.info('Moved to az=%.2f.', az)

    @timeout(1200000)
    def move_altaz(self, alt: float, az: float, *args, **kwargs):
        """Moves to given coordinates.

        Args:
            alt: Alt in deg to move to.
            az: Az in deg to move to.

        Raises:
            ValueError: If device could not move.
        """

        # only move, when ready
        if not self.is_ready():
            log.warning('Dome not ready, ignoring slew command.')
            return

        # destination az already set?
        if az == self._set_az:
            return
        self._set_az = az

        # is this a larger move?
        large_move = abs(az - self._azimuth) > 2. * self._tolerance

        # decide, whether we're tracking or just slewing
        tracking = self.is_following and not large_move

        # acquire lock
        with LockWithAbort(self._lock_move, self._abort_move):
            # store altitude
            self._altitude = alt

            # change status to TRACKING or SLEWING, depending on whether we're tracking
            self._change_motion_status(IMotion.Status.TRACKING if tracking else IMotion.Status.SLEWING)
    
            # move dome
            self._move(az, self._abort_move)

            # change status to TRACKING or POSITIONED, depending on whether we're tracking
            self._change_motion_status(IMotion.Status.TRACKING if self.is_following else IMotion.Status.POSITIONED)

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

    def _queue_command(self, command: Command) -> Command:
        """Queues a command and returns it.

        Args:
            command: Command to queue.

        Returns:
            The command.
        """
        self._next_commands.put(command)
        size = self._next_commands.qsize()
        if size > 10:
            log.warning('Command queue is getting long with %d items!', size)
        return command

    def _communication(self):
        """Thread method for communicating with the dome."""

        # run forever
        while not self.closing.is_set():
            # no new command?
            if self._next_commands.empty():
                self.closing.wait(1)
                continue

            # get command
            cmd: Command = self._next_commands.get()

            # execute it
            response = self._send_command(cmd.command)
            cmd.set(response)

    def _update_status(self):
        """Update status from dome."""

        # loop forever
        while not self.closing.is_set():
            # get shutter status, might be one of d#shutclo, d#shutrun, or d#shutope
            command = self._queue_command(GetShutCommand())
            self._shutter = command.wait()

            # if shutter is closed and current motion status is not INITIALIZING, set it to PARKET
            if self._shutter == 'd#shutclo' and self.get_motion_status() != IMotion.Status.INITIALIZING:
                self._change_motion_status(IMotion.Status.PARKED)

            # get azimuth
            command = self._queue_command(GetAzimCommand())
            az_response = command.wait()

            # parse azimuth
            if az_response is None or not az_response.startswith('d#azi'):
                # wrong response!
                self._azimuth = None
            else:
                # response is of form d#azi1800, so take lats four characters, parse to float and divide by 10
                az = float(az_response[5:]) / 10.

                # Baader measures azimuth as West of South, so we need to convert it
                self._azimuth = BaaderDome._adjust_azimuth(az)

            # sleep a little
            self.closing.wait(2)

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
