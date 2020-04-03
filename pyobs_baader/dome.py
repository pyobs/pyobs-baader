import threading
import serial

from pyobs.interfaces import IDome
from pyobs import PyObsModule


class BaaderDome(PyObsModule, IDome):
    """A pyobs module for a Baader dome."""

    def __init__(self, port: str = '/dev/ttyUSB0', baud_rate: int = 9600, byte_size: int = 8, parity: str = 'N',
                 stop_bits: int = 1, timeout: int = 180, *args, **kwargs):
        """Initializes a new Baader dome.
        
        Args:
            port: COM port to use.
            baud_rate: Baud rate for connection.
            byte_size: Byte size for connection.
            parity: Parity (Y/N).
            stop_bits: Number of stop bits.
            timeout: Connection timeout in seconds.
        """
        PyObsModule.__init__(self, *args, **kwargs)

        # store it
        self._port = port
        self._baud_rate = baud_rate
        self._byte_size = byte_size
        self._stop_bits = stop_bits
        self._parity = parity
        self._timeout = timeout

        # next command
        self._command_lock = threading.RLock()
        self._next_command = None

        # status
        self._shutter = None
        self._azimuth = None

        # start thread
        self._add_thread_func(self._communication)

    def init(self, *args, **kwargs):
        """Open dome.

        Raises:
            ValueError if dome cannot be opened.
        """
        with self._command_lock:
            self._next_command = 'd#opeshut'

    def park(self, *args, **kwargs):
        """Close dome.

        Raises:
            ValueError if dome cannot be opened.
        """
        with self._command_lock:
            self._next_command = 'd#closhut'

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

        # get azimuth
        az_response = self._send_command('d#getazim')

        # parse azimuth
        if not az_response.startswith('d#azi'):
            # wrong response!
            self._azimuth = None
        else:
            # response is of form d#azi1800, so take lats four characters, parse to float and divide by 10
            self._azimuth = float(az_response[5:]) / 10.

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
                reply = ser.read(6).decode('utf-8')

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
