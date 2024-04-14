# Copyright (C) Meridian Innovation Ltd. Hong Kong, 2020. All rights reserved.
import numpy as np
import logging
import time
from pprint import pformat
from senxor.mi48 import get_reg_name
from senxor.utils import cksum

# the following dependency is only for get_serial
import serial
import serial.tools.list_ports

logger = logging.getLogger(__name__)

class I2C_Interface:
    """I2C interface object to access a connected device"""
    def __init__(self, i2c_bus, chip_addr):
        self.device = i2c_bus
        self.chip_addr = chip_addr

    def open(self):
        self.device.open()

    def regread(self, register_addr, regname=""):
        byte = self.device.read_byte_data(self.chip_addr, register_addr)
        return byte


    def regwrite(self, register_addr, register_value, regname=""):
        byte = register_value  # no need to .encode()
        self.device.write_byte_data(self.chip_addr, register_addr, byte)
        return None

    def reset_input_buffer(self):
        try:
            self.device.reset_input_buffer()
        except AttributeError:
            # device may not support that
            pass

    def reset_output_buffer(self):
        try:
            self.device.reset_output_buffer()
        except AttributeError:
            # device may not support that
            pass

    def close(self):
        self.device.close()


class SPI_Interface:
    """SPI interface object to access a connected device"""
    def __init__(self, spi_device, xfer_size):
        self.device = spi_device
        # host system would typically have a buffer that is
        # smaller than the entire frame
        self.xfer_size = xfer_size

    def open(self):
        self.device.open()

    def read(self, length_in_words):
        # MI48 operates as a full duplex device and requires
        # a dummy write byte for every byte read back
        length_in_bytes = 2 * length_in_words
        dummy_bytes = [0,] * self.xfer_size
        xfer_size_words = int(self.xfer_size / 2)
        length_in_words = int(length_in_bytes / 2)
        # create a couple of buffers for storage of intermediate
        # transfer and for the return buffer
        data = np.zeros(length_in_words, dtype=np.uint16)
        # make up a counter of how many words we have received
        n_words = 0
        # loop until we receive the required number of words
        while n_words < length_in_words:
            i0 = n_words
            i1 = n_words + xfer_size_words
            # Keep the CS asserted throughout the transfer
            # This should be a property of device.xfer.
            # If device is an instance of spidev on rpi, this seems to be
            # true for both xfer and xfer2 routines.
            # For the sake of generality, keep this as xfer
            response = self.device.xfer(dummy_bytes)
            # interpret the response as array of uint8
            buffer = np.array(response).astype('u1')
            n_words += int(len(buffer) / 2.)
            # The MI48 assumes 16 bit word transfer with MSbit first.
            # But we are reading with 8-bit word transfers on the RPI,
            # and storing the MSB to lower location than the LSB.
            # Hence we end up with big-endian data of unsigned 2-byte ints.
            _data = np.ndarray(shape=(int(len(buffer) / 2),),
                               buffer=buffer, dtype='>u2')
            try:
                data[i0: i1] = _data
            except IndexError:
                # depending on xfer_size, the last transfer may be shorter
                # print(i0, 2*i0, length_in_words, len(_data), len(buffer))
                data[i0:] = _data[:length_in_words - i0]
        return data

    def reset_input_buffer(self):
        try:
            self.device.reset_input_buffer()
        except AttributeError:
            # device may not support that
            pass

    def reset_output_buffer(self):
        try:
            self.device.reset_output_buffer()
        except AttributeError:
            # device may not support that
            pass

    def close(self):
        self.device.close()


# --------------------------
# USB Vendor ID
# --------------------------
MI_VID = 1046       # 0x0416

# --------------------------
# USB Product ID
# --------------------------
MI_PID_EVK = 45058  # 0xB002 EVK
MI_PID_XPRO = 45088  # 0xB002 XPro
MI_PIDs = [MI_PID_EVK, MI_PID_XPRO]

# ------------------------------------------
# USB header specific field lengths in bytes
# ------------------------------------------
USB_CMD_LEN = 4
USB_ACK_LEN = 4
USB_CKS_LEN = 4  # check sum
USB_HDR_LEN = 320

class USB_Interface:
    """USB interface object to access a connected device"""

    def __init__(self, port):
        self.port = port
        self.log = logger

    def open(self):
        self.port.open()
        self.topen = time.time()

    def close(self):
        self.port.close()

    def reset_input_buffer(self):
        self.port.reset_input_buffer()

    def reset_output_buffer(self):
        self.port.reset_output_buffer()

    def regread(self, reg, regname=""):
        """Read a control/status register via USB protocol"""
        result = None
        while result is None:
            cmd = 'RREG{:02X}XXXXXX'.format(reg)
            cmd = '   #{:04X}{}'.format(len(cmd), cmd)
            cmd_name = 'GET_{}'.format(regname)
            result = usb_command(self.port, cmd, cmd_name)
            if result is None: return
            if not isinstance(result, int):
                # a non int would be a GFRA coming back before the RREG
                result = None
        return result

    def regwrite(self, reg, value, regname=""):
        """Write to a control register via USB protocol"""
        cmd = 'WREG{:02X}{:02X}XXXX'.format(reg, value)
        cmd = '   #{:04X}{}'.format(len(cmd), cmd)
        cmd_name = 'SET_{}'.format(regname)
        usb_command(self.port, cmd, cmd_name)
        return None

    def read(self, size_in_words):
        """Read a GFRA acknowledge, remove USB header, and return data frame.

        The returned data frame is a 1-D numpy array of unsigned int16.
        """
        cmd, data = usb_acknowledge(self.port)
        if cmd == 'GFRA':
            # data is a sequence (1-d array) of 16-bit unsigned ints
            # here we drop the USB header 
            return data[-size_in_words:]
        else:
            self.log.warning('read returned {} acknowledge.'.format(cmd))
            return None


def usb_command(port, cmd: str, cmd_name='', verbose=True):
    """send command to MI48 via USB and return its acknowledge"""
    _cmd = ''
    while _cmd != cmd[8:12]:
        # host command
        port.write(cmd.encode())
        # device ack
        _cmd, data = usb_acknowledge(port)
        if _cmd != cmd[8:12]:
            if verbose:
                logger.debug('Expected ACK: {}, rcvd: {}'.
                             format(cmd[8:12], _cmd))
                logger.debug('Resetting input buffer')
            port.reset_input_buffer()
    if _cmd == 'RREG':
        assert isinstance(data, int)
    # report
    if verbose: logger.debug('{}'.format(fmt_usb_cmd(cmd, data)))
    return data

def usb_acknowledge(port):
    """Receive the EVK acknowledge and parse it"""
    ack = None
    # this loop will make the program hang if ser.read()
    # has no timeout configured!
    while ack is None:
        # if *.decode() yields UnicodeDecodeError
        # drop the ACK and wait for the next one
        ack = usb_get_ack(port)
        if ack is None:
            #logger.warning('None ACK received. Resetting input buffer.')
            port.reset_input_buffer()
    parsed = usb_parse_ack(*ack)
    return parsed

def usb_parse_ack(cmd:str, data:bytes):
    """
    Parse command and return the command string and a data item.

    The data item depends on the type of acknowledge:

    * 'GFRA' -- a 1-D array of 16-bit unsigned integers.
    * 'RREG' -- an integer
    * 'WREG' -- a None value
    * 'SERR' -- decoded data field
    """
    cmd = cmd.decode()
    if cmd == 'WREG':
        # An acknowledge to a register-write contains no data
        return cmd, None
    if cmd == 'RREG':
        # read command returns only a register value
        return cmd, int(data.decode(), base=16)
    if cmd == 'SERR':
        # I have no info on what SERR contains... undocumented
        return cmd, data.decode()
    if cmd == 'GFRA':
        # Frame acknowledge contains unencoded unsigned 16-bit ints
        data = np.frombuffer(data, dtype='u2')
        return cmd, data

def usb_get_ack(port):
    """
    Obtain an acknowledge to a command sent to a virtual serial port

    Ack has the following format:
    | '   #' | 4B length(LenCmdDat) | 4B command | data (lenth - 8B) | 4B CKS |

    Return bytes.
    """
    res = ''
    while res != '   #':
        res = port.read(4)
        if res is None:
            # likely the result of interface.read timeout (e.g. for USB)
            return None
        try:
            res = res.decode()
        except UnicodeDecodeError:
            # This will happen if we're draining USB buffer from GFRA ack.
            # so we ignore till we reach the beginning of the next frame.
            res = ''

    # Read the length field and start check sum calculation
    _len = port.read(USB_ACK_LEN)
    cs = cksum(_len)
    try:
        ack_len = int(_len.decode(), base=16)
    except ValueError:
        return None
    data_len = ack_len - USB_ACK_LEN - USB_CMD_LEN
    # Read the data part of the payload and update the checksum
    cmd = port.read(USB_CMD_LEN)
    cs = cksum(cmd, cs)
    data = port.read(data_len)
    if data_len > 0:
        cs = cksum(data, cs)
    cs = cs & 0xFFFF
    # Read the check sum field
    cks = port.read(USB_CKS_LEN)
    try:
        cks.decode()
    except UnicodeDecodeError:
        print('USB check sum decode error: {}'.format(cks))
        return None
    try:
        cks = int(cks, base=16)
    except ValueError:
        # if host too slow, we get invalid literals here
        logger.error('Bad USB check sum literals for {}: {}'.format(cmd, cks))
        return None
    if cs != cks:
        logger.error('Check sum mismatch: calculated {}, received {}'.
                    format(hex(cs), hex(cks)))
        return None
    return cmd, data

def fmt_usb_cmd(cmd, data):
    """Command is a string already; here we return a more informative one"""
    s = []
#    s.append(cmd[:8])     # len
    s.append(cmd[8:12])    # type
    s.append(cmd[12:14])   # addr
    s.append('{:16s}'.format(get_reg_name(int(cmd[12:14], 16))))

    if cmd[8:12] == 'WREG':
        val = int(cmd[14:16], 16)
        s.append('0x{:02X}'.format(val))

    if cmd[8:12] == 'RREG':
        assert isinstance(data, int)
        s.append('0x{:02X}'.format(data))  # value
#        s.append('(0x{:02X})'.format(data))

    return ' '+' '.join(s)

def get_serial(open_ports=None, comport=None, verbose=True):
    """Open a serial port to which the MI48 is attached.

    Raise UnboundLocalError if no serial port is successfully open
    """
    for p in list(serial.tools.list_ports.comports()):
        if p.vid == MI_VID and p.pid in MI_PIDs:
            # check it is the comport we want and skip if not
            # if we did not specify description/name then get the 
            # first that we find to match Meridian's devices
            logger.info(f'Senxor detected: {p.description}')
            if comport is not None and comport not in p.description:
                continue
            try:
                ser = serial.Serial(p.device)
            except serial.SerialException:
                # assume it is open
                logger.info('Failed opening port:\n{}'.format(p))
                # do not raise, but check the next port in the list
                continue
            # here we already have gotten a serial device; set it up
            ser.baudrate = 115200
            ser.rtscts = True
            ser.dsrdtr = True
            ser.timeout = 0.5
            ser.write_timeout = 0.5
            logger.info('Opened USB port:\n{}\n'.format(pformat(ser)))
            break
    # the following return statement will generate UnboundLocalError
    # if no serial was successfully opened
    return ser

