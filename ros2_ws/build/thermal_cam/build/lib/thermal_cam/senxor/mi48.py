# Copyright (C) Meridian Innovation Ltd. Hong Kong, 2020. All rights reserved.
#
import sys
import logging
import functools
import time
import struct
import array
import numpy as np
from senxor.utils import crc16

def logger_wrapper(name, level, msg, exc_info=None, logger=None):
    _msg = '{:12s} {}'.format(name, msg)
    if logger == None:
        logging.log(level, _msg, exc_info=exc_info)
    else:
        logger.log(level, _msg, exc_info=exc_info)

# =======================
# MI48xx specific objects
# =======================
# Reference for temperature conversion K to C
KELVIN_0 = -273.15  # in Celsius
T_OFFSET_UNIT = 0.05  # increment unit for OFFSET_CORR register in K

# Word index in SPI header field indexing referenced to SPI header base index
SPIHDR_FRCNT = 0
SPIHDR_SXVDD = 1
SPIHDR_SXTA  = 2
SPIHDR_TIME  = 3   # two words
SPIHDR_MAXV  = 5
SPIHDR_MINV  = 6
SPIHDR_CRC   = 7
SPIHDR_LOW_NETD_PX_XY  = 8
SPIHDR_LOW_NETD_PX  = 9

DEFAULT_CTRL_STAT = {
    'FRAME_MODE': 0x20,
    'STATUS':     0x00,
    'FRAME_RATE': 0x04,
    'POWER_DOWN_1': 0x00,
    'POWER_DOWN_2': 0x02,
    "DISTANCE_CORR" : 0x70,  # read from camera module; depends on calibration
    'EMISSIVITY': 0x5F,
    'OFFSET_CORR': 0x00,
    'FILTER_CTRL': 0x00,
    'FILTER_1_LSB': 0x32,
    'FILTER_1_MSB': 0x00,
    'FILTER_2': 0x04,
    # reduced NETD (high sensitivity line/pixel)
    "NETD_CONFIG"   : 0x00,  # RW Configuration for reduced NETD row/pixel
    "NETD_FACTOR"   : 0x14,  # RW NETD Reduction factor
    "NETD_PIXEL_X"  : 0x00,  # RW Column coordinate of reduced NETD
    "NETD_PIXEL_Y"  : 0x00,  # RW Row    coordinate of reduced NETD
}


# MI48Ax register map
regmap = {
    "EVK_TEST"      : 0x00,  # 
    "FRAME_MODE"    : 0xB1,  # RW Control of capture and readout
    "FW_VERSION_1"  : 0xB2,  # R  Firmware Version (Major, Minor)
    "FW_VERSION_2"  : 0xB3,  # R  Firmware Version (Build)
    "FRAME_RATE"    : 0xB4,  # RW Frame rate delivery through the SPI interface
    "POWER_DOWN_1"  : 0xB5,  # RW Control of power down parameters
    "STATUS"        : 0xB6,  # R  Status of the MI48
    "POWER_DOWN_2"  : 0xB7,  # RW Control of power down parameters
    "SENXOR_TYPE"   : 0xBA,  # R  Type of the attached camera module
    "DISTANCE_CORR" : 0xC2,  # RW Temperature vs distance correction factor
    "EMISSIVITY"    : 0xCA,  # RW Emissivity value for conversion to [K]
    "OFFSET_CORR"   : 0xCB,  # RW Temperature offset correction per frame
    'FILTER_CTRL'   : 0xD0,  # RW Control of filters
    'FILTER_1_LSB'  : 0xD1,  # RW Settings of filter 1 (temporal LSB)
    'FILTER_1_MSB'  : 0xD2,  # RW Settings of filter 1 (temporal MSB)
    'FILTER_2'      : 0xD3,  # RW Settings of filter 2 (rolling average)
    "NETD_CONFIG"   : 0xD4,  # RW Configuration for reduced NETD row/pixel
    "NETD_FACTOR"   : 0xD5,  # RW NETD reduction factor
    "NETD_PIXEL_X"  : 0xD6,  # RW Column coordinate of reduced NETD
    "NETD_PIXEL_Y"  : 0xD7,  # RW Row    coordinate of reduced NETD
    "FLASH_CTRL"    : 0xD8,  # RW Control read/write to MI48 flash-memory
    "SENXOR_ID"     : 0xE0,  # R  Serial number of the attached camera module
    "SENXOR_ID_0"   : 0xE0,  # R  Serial number of the attached camera module
    "SENXOR_ID_1"   : 0xE1,  # R  Serial number of the attached camera module
    "SENXOR_ID_2"   : 0xE2,  # R  Serial number of the attached camera module
    "SENXOR_ID_3"   : 0xE3,  # R  Serial number of the attached camera module
    "SENXOR_ID_4"   : 0xE4,  # R  Serial number of the attached camera module
    "SENXOR_ID_5"   : 0xE5,  # R  Serial number of the attached camera module
}

MI48_FRAME_MODE    = 0xB1  # RW Control the capture and readout of thermal data 
MI48_FW_VERSION_1  = 0xB2  # R  Firmware Version (Major, Minor)
MI48_FW_VERSION_2  = 0xB3  # R  Firmware Version (Build)
MI48_FRAME_RATE    = 0xB4  # RW Frame rate delivery through the SPI interface
MI48_POWER_DOWN_1  = 0xB5  # RW Control of power down parameters
MI48_STATUS        = 0xB6  # R  Status of the attached camera module and MI48A1 interface operations
MI48_POWER_DOWN_2  = 0xB7  # RW Control of power down parameters
MI48_SENXOR_TYPE   = 0xBA  # R  Type of the attached camera module
MI48_DISTANCE_CORR = 0xC2  # RW Temperature vs distance correction factor
MI48_EMISSIVITY    = 0xCA  # RW Emissivity value for conversion of SenXorTM data to temperature
MI48_OFFSET_CORR   = 0xCB  # RW Temperature offset correction for the entire data frame
MI48_FILTER_CTRL   = 0xD0  # RW Control of filters
MI48_FILTER_1_LSB  = 0xD1  # RW Settings of filter 1 (temporal LSB)
MI48_FILTER_1_MSB  = 0xD2  # RW Settings of filter 1 (temporal MSB)
MI48_FILTER_2      = 0xD3  # RW Settings of filter 2 (rolling average)
MI48_NETD_CONFIG   = 0xD4  # RW Configuration for reduced NETD row/pixel
MI48_NETD_FACTOR   = 0xD5  # RW NETD reduction factor
MI48_NETD_PIXEL_X  = 0xD6  # RW Column coordinate of reduced NETD pixel
MI48_NETD_PIXEL_Y  = 0xD7  # RW Row    coordinate of reduced NETD row
MI48_FLASH_CTRL    = 0xD8  # RW Control of RW to MI48 flash-memory
MI48_SENXOR_ID     = 0xE0  # R  Serial number of the attached camera module
MI48_SENXOR_ID_0   = 0xE0  # R  Serial number of the attached camera module
MI48_SENXOR_ID_1   = 0xE1  # R  Serial number of the attached camera module
MI48_SENXOR_ID_2   = 0xE2  # R  Serial number of the attached camera module
MI48_SENXOR_ID_3   = 0xE3  # R  Serial number of the attached camera module
MI48_SENXOR_ID_4   = 0xE4  # R  Serial number of the attached camera module
MI48_SENXOR_ID_5   = 0xE5  # R  Serial number of the attached camera module
MI48_SENXOR_ID_LEN = 6     # number of bytes of the SENXOR_ID

# STATUS Register Flags Masks
READOUT_TOO_SLOW = 0x02
SENXOR_IF_ERROR = 0x04
SXIF_ERROR = 0x04     # alias
CAPTURE_ERROR = 0x08
DATA_READY = 0x10
BOOTING_UP = 0x20

# FRAME_MODE Register Flags Masks
GET_SINGLE_FRAME = 0x01
CONTINUOUS_STREAM = 0x02
READOUT_MODE = 0x1C         # bits 4-2
NO_HEADER = 0x20            # skip header HEADER data
LOW_NETD_ROW_IN_HEADER = 0x40   # include data with reduced NETD in the header

# camera module type and FPA
SENXOR_NAME = {
    0: 'MI0801 non-MP',  # non-MP modules
    1: 'MI0801',  # MP modules
    2: 'MI0301',
    3: 'MI0802',
}

FPA_SHAPE = {
    'MI0801 non-MP': (80, 62),
    'MI0801': (80, 62),
    'MI0802': (80, 62),
    'MI0301': (32, 32),
    'bobcat': (80, 62),
    'bobcat-2': (80, 62),
    'lynx': (32, 32),
    'cougar': (80, 62),
    'panther': (160, 120),
    0: (80, 62),
    1: (80, 62),
    2: (32, 32),
    3: (80, 62),
    8: (160, 120),
}


class MI48:
    """
    MI48xx abstraction
    """
    def __init__(self, interfaces:list, fps=None, name="MI48",
                reset_handler=None, data_ready=None, read_raw=False):
        """Initialise with a serial port"""
        # logging stuff
        self.name = name
        self.log = functools.partial(logger_wrapper, self.name, logger=None)
        # interface handles
        self.interfaces = interfaces
        # note that this will potentially clear only the host
        # interface buffers; meanwhile, the MI48 buffers would
        # require different handling, if the MI48 was left in
        # an unknown state
        self.clear_interface_buffers()
        self.reset = reset_handler
        if self.reset is not None: self.reset()
        if data_ready is not None:
            self.data_ready = data_ready
        # this should be read from the camera module
        self.fpa_shape = None
        # At this stage check that MI48 is not streaming already,
        # which may happen if termination of last stream was not handled 
        # cleanly. If we do not stop the MI48 here, the status handling
        # during boot-up and error-handling will be messed up.
        mode = self.get_mode()
        if mode & (GET_SINGLE_FRAME | CONTINUOUS_STREAM):
#            # if the MI48 is streaming, it's obviously booted up before
            self.stop_capture()
        #
        # check what camera we have
        self.camera_info = self.get_camera_info()
        # do not parse frame header if MI48 is not on the core dev board
        self.parse_header = self.is_on_cdb()
        # check status register and raise relevant flags
        status, mode = self.bootup(verbose=True)
        # may need to handle ValueError from above call:
            # happens if USB is still streaming when we restart
            # and instead of RREG (int) we get GFRA acknowledge (array)
        self.capture_no_header = mode & NO_HEADER
        self.with_low_netd_row_in_header = mode & LOW_NETD_ROW_IN_HEADER
        bootup_error =\
                (mode & GET_SINGLE_FRAME) or (mode & CONTINUOUS_STREAM)\
                or (status & READOUT_TOO_SLOW) or (status & DATA_READY)\
                or (status & CAPTURE_ERROR) or (status & SXIF_ERROR)
        # see if we need to react to any errors
        if bootup_error:
            status, mode = self.error_handler(status, mode, verbose=True)
            self.log(logging.DEBUG, 'Status: {}'.format(hex(status)))
            self.log(logging.DEBUG, 'Mode  : {}'.format(hex(mode)))
        self.capture_no_header = mode & NO_HEADER
        # reset crc.error
        self.crc_error = False
        # set FPS
        if fps is not None:
            self.set_fps(fps)
        # set the format of the returned data
        self.read_raw = read_raw

    def bootup(self, verbose=False):
        """Ensure bootup of the mi48 is complete, returning MODE and STATUS.

        Return all flags raised at any one point while looping and waiting for
        boot-up to complete, so that any error handling can be done after
        boot-up. Exception is boot_in_progress flag, as we're handling it here.
        This is necessary because error handling will likely require register
        write, which is allowed only once that bootup is comlete.
        """
        # NOTE on the timeout (in seconds) below:
        # On WINDOWS the clock has poor resolution, which probably
        # depends on CPU frequency too.
        timeout = max(0.025, time.get_clock_info('monotonic').resolution)
        self.check_ctrl_stat_regs()
        t0 = time.monotonic()
        status = self.get_status(verbose=verbose)
        mode = self.get_mode(verbose=verbose)
        boot_in_progress = status & BOOTING_UP
#        no_header = mode & NO_HEADER
        while boot_in_progress:
            status = self.get_status(verbose=True)
            mode = self.get_mode(verbose=True)
            boot_in_progress = status & BOOTING_UP
            time.sleep(timeout)
        t1 = time.monotonic()
        self.log(logging.DEBUG, 'Bootup complete in {:.0f} ms'.
                format(1.e3 * (t1-t0)))
        # clear boot in progress flag as we're done with it
        status = status & (~BOOTING_UP & 0xFF)
        self.log(logging.DEBUG, 'Status: {}'.format(hex(status)))
        self.log(logging.DEBUG, 'Mode  : {}'.format(hex(mode)))
#        mode = mode & no_header
        return status, mode

    def error_handler(self, status, mode, verbose=False):
        """Attempt to bring the MI48 to a clean state.

        Specifically, attempt to stop capture and clear
        the STATUS (upon read) and FRAME_MODE registers, also
        clean the MI48 output frame buffer (read and dump the frame).
        If a reset handler is provided to self, then use it if an
        error between camera module and MI48 is detected
        """
        if mode & (GET_SINGLE_FRAME | CONTINUOUS_STREAM):
            # Stop MI48 capture
            self.log(logging.WARNING, 'Attempting MI48.stop_capture()')
            mode = self.stop_capture(verbose)
        if (status & READOUT_TOO_SLOW):
            self.log(logging.WARNING, 'Ignoring Readout Too Slow flag')
            # ignore readout_too_slow for the moment.
            # if we try to dump a frame based on Readout_too_slow or
            # Capture_error without seeing data_ready, then in the 
            # case of a USB interface we will hang forever, as the 
            # Ack will never come. For the SPI it doesn't matter, since
            # it is a full duplex, and we get data as long as we, as a 
            # master, push out zeros on the bus.
        if (status & DATA_READY):
            # Attempt to clear output buffer of the SPI slave interface
            # If USB interface, that would have been tackled by MI48 FW.
            # This should not be seen at boot up for USB interface.
            self.log(logging.DEBUG, 'Trying to dump a frame')
            data, header = self.read()
            self.log(logging.WARNING, 'Dumping residual frame:')
            self.log(logging.WARNING, header)
            self.log(logging.WARNING, data[:20])
        # status and mode will be clean by here from the above
        if (status & CAPTURE_ERROR):
            self.log(logging.ERROR,
                'Capture ERROR: typically due to bad FPC connection.\n'
                '               Check FPC and connectors.\n'
                '               If problem percist, try to prepare reproducible\n',
                '               example and call/email Meridian.')

        if (status & SXIF_ERROR):
            try:
                self.log(logging.ERROR,
                    'SenXor Interface ERROR: Attempting SW reset of MI48')
                self.reset()
            except TypeError:
                # no reset handle provided
                self.log(logging.ERROR,
                    '                        SW Reset handle not available.\n'
                    '                        Please press HW reset of MI48\n')
                raise RuntimeError
        self.log(logging.DEBUG, 'Trying to get status again')
        status = self.get_status(verbose=True)
        return status, mode

    def regread(self, reg):
        """Read a control/status register; Allow hex or str for reg"""
        if isinstance(reg, str):
            regname = reg
            # Try to get the address value from the register map, but
            # if the address refers to the internal flash, there will be
            # no entry in the regmap dictionary; use value directly
            try:
                reg = regmap[reg]
            except KeyError:
                # assume hex string
                reg = int(reg)
        else:
            # assume integer; make up the hex representation for logging
            regname = f'0x{reg:02X}'
        return self.interfaces[0].regread(reg, regname)

    def regwrite(self, reg, value):
        """Write to a control register"""
        if isinstance(reg, str):
            regname = reg
            reg = regmap[regname]
        else:
            regname = ""
        return self.interfaces[0].regwrite(reg, value, regname)


    def read(self):
        """Read a data frame

        Return the temperature data or (data, header), where the
        header is a dictionary.
        The returned data is a 2D array of np.float16 representing the
        temperature in Celsius.
        Header values if requested are also decoded from bytes.
        """
        # figure out how many words to get; recall 2 bytes per pixel
        data_size = np.prod(self.fpa_shape)
        size_in_words = data_size
        if not self.capture_no_header:
            size_in_words += self.cols
            if self.with_low_netd_row_in_header:
                size_in_words += self.cols
        # print('Reading {} words'.format(size_in_words))

        # The spi device must provide read(number-of-bytes) function
        response = self.interfaces[1].read(size_in_words)

        # Obtain the data but do NOT convert to degrees C yet,
        # because we have to calculate CRC on it first.
        # Assume the interfaces[1].read() returns 16-bit integers
        # Recall that the temperature data frame is after the
        # optional header
        try:
            data = response[-data_size:]
        except TypeError:
            # if interface.read() yields None we've got an error
            return None, None

        # Parse the optional header; else return the data
        # If the MI48 is not on the core-development board, do not parse
        if self.capture_no_header or not self.parse_header:
            header = None
        else:
            _header = response[:-data_size]
            header = self.parse_frame_header(_header, self.with_low_netd_row_in_header)
            self.crc_error = False
            # check crc
            # note that MI48 implements CRC-16/CCITT-FALSE which
            # must be initialised with 0xFFFF
            _crc = crc16(data)
            if not header['crc'] == hex(_crc):
                self.crc_error = True
                self.log(logging.ERROR, 'Frame CRC error. '+
                    'Header CRC: {}, Data CRC: {}'.\
                    format(header['crc'], hex(_crc)))

        # Once we have done the CRC check, convert to degrees C
        # unless raw numbers are requested
        if self.read_raw:
            return data, header
        else:
            data = data / 10. + KELVIN_0
            return data.astype(np.float64), header

    def is_on_cdb(self):
        """Check if MI48 is on a core dev board or user system"""
        res = self.regread(0x00)
        mi48cb = res == 0xFF
        return mi48cb

    def get_status(self, verbose=False):
        """Read status register; log if non-zero status in verbose mode"""
        status = self.regread('STATUS')
        if verbose and status != 0:
            self.log(logging.WARNING,'Non-zero STATUS: 0x{:02X}'.
                    format(status))
            self.log(logging.WARNING, ', '.join(self.parse_status(status)))
        return status

    def parse_status(self, regvalue):
        """Return a list of strings corresponding to set flags"""
        s = []
        if regvalue & 0x02: s.append('Readout too slow')
        if regvalue & 0x04: s.append('SenXor interface ERROR')
        if regvalue & 0x08: s.append('Capture ERROR')
        if regvalue & 0x10: s.append('Data ready')
        if regvalue & 0x20: s.append('Boot up in progress')
        return s

    def get_mode(self, verbose=False):
        mode = self.regread('FRAME_MODE')
        # it seems that if there is a SIGINT, serial interface may 
        # close immediately, and a timeout will return None here.
        if mode is None: return None
        if verbose and (mode & 0x03)!= 0x00:
            self.log(logging.WARNING,'Capture in progress: 0x{:02X}'.
                    format(mode))
            self.log(logging.WARNING, ', '.join(self.parse_mode(mode)))
        return mode

    def parse_mode(self, regvalue):
        """Return a list of strings corresponding to set flags"""
        s = []
        if regvalue & 0x01: s.append('Single capture in progress')
        if regvalue & 0x02: s.append('Continuous streaming')
        if regvalue & 0x10: s.append('No frame header')
        return s

    def get_pm1(self):
        return self.regread('POWER_DOWN_1')

    def get_pm2(self):
        return self.regread('POWER_DOWN_2')

    def get_frame_rate(self):
        return self.regread('FRAME_RATE')

    def get_emissivity(self):
        return self.regread('EMISSIVITY')

    def get_dist_corr(self):
        return self.regread('DISTANCE_CORR')

    def get_offset_corr_regvalue(self):
        """Read the value of OFFSET_CORR register"""
        return self.regread('OFFSET_CORR')

    def get_offset_corr_K(self):
        """Get the temperature offset corresponding to OFFSET_CORR register"""
        unit = T_OFFSET_UNIT
        n = self.regread('OFFSET_CORR')
        if n < 128:
            return n * unit
        else:
            return (n - 256) * unit

    def get_filter_ctrl(self):
        """Return the value of Filter Control Register"""
        return self.regread('FILTER_CTRL')

    def get_filter_1(self):
        """Return the strength of the temporal filter"""
        lsb = self.regread('FILTER_1_LSB')
        msb = self.regread('FILTER_2_LSB')
        res = (msb << 8) + lsb
        return res

    def get_filter_2(self):
        """Return the depth of the Rolling Average filter"""
        return self.regread('FILTER_2')

    def get_camera_info(self):
        """Get camera info: senxor type/ID, maxFPS, FW version"""
        try:
            return self.camera_info
        except AttributeError:
            # if we haven't yet read the info from camera module
            pass
        # read camera module info
        res = {}
        self.camera_info = res
        res['CAMERA_TYPE'] = self.get_camera_type()
        uid, uid_hex, uid_hexsn = self.get_camera_id()
        res['CAMERA_ID'] = uid
        res['CAMERA_ID_HEX'] = uid_hex
        res['CAMERA_ID_HEXSN'] = uid_hexsn
        res['SN'] = 'SN'+uid_hex
        res['FW_VERSION'] = self.get_fw_version()
        self.camera_type = res['CAMERA_TYPE']
        self.camera_name = SENXOR_NAME[self.camera_type]
        self.fpa_shape = FPA_SHAPE[self.camera_type]
        self.cols = self.fpa_shape[0]
        self.rows = self.fpa_shape[1]
        self.camera_id = res['CAMERA_ID']
        self.camera_id_hex = res['CAMERA_ID_HEX']
        self.camera_id_hexsn = res['CAMERA_ID_HEXSN']
        self.sn = res['SN'].upper()
        self.fw_version = res['FW_VERSION']
        res['MAX_FPS'] = self.get_max_fps()
        self.maxfps = res['MAX_FPS']
        # note that current FPS requires self.maxfps, 
        # becuase we can only read the divisor
        res['Current FPS'] = self.get_fps()
        return res

    def get_ctrl_stat_regs(self):
        """Read all registers, return a dictionary {'RegName': 0xValue}"""
        res = {}
        self.log(logging.DEBUG, 'Reading Control and Status Regs:')
        for reg in list(DEFAULT_CTRL_STAT.keys()):
            res[reg] = self.regread(reg)
        return res

    def check_ctrl_stat_regs(self, expect=None):
        """Check control and statuts registers as expected"""
        self.log(logging.DEBUG, 'Checking Control and Status Regs:')
        if expect is None:
            expect = DEFAULT_CTRL_STAT
        res = self.get_ctrl_stat_regs()
        for reg, val in res.items():
            log_level = logging.DEBUG
            if reg in expect.keys():
                _exp = expect.get(reg)
                if val != _exp:
                    log_level = logging.WARNING
                    self.log(log_level, '{}: {} (expected {})'.
                             format(reg, hex(val), hex(_exp)))
                    continue
            self.log(log_level, '{}: {}'.format(reg, val))

    def get_max_fps(self):
        """Get some frames in continuous mode and establish max FPS"""
        # TODO: real implementation of burst capture 250 frames, and 
        #       determine average FPS.
        #       Or at least map maxfps to corresponding FW of the MI48
        #       and the camera type.
        if self.camera_type in [0,1]:
            maxfps = 25.5  # this is true for Bobcat with latest MI48Ax
            return maxfps
        if self.camera_type in [2]:
            maxfps = 28.57 # lynx
            return maxfps
        maxfps = 30.0  # aspirational
        return maxfps

    def get_fps(self):
        """Get current FPS [1/s]"""
        divisor = self.get_frame_rate()
        try:
            return float(self.maxfps) / divisor
        except ZeroDivisionError:
            return self.maxfps

    def set_frame_rate(self, fps_divisor:int):
        """Set the frame rate divisor register (integer)"""
        self.regwrite('FRAME_RATE', fps_divisor)

    def set_fps(self, fps):
        """Set the desired FPS [1/s] or the closest possible"""
        try:
            fps_divisor = int(round(float(self.maxfps) / fps))
        except ZeroDivisionError:
            fps_divisor = 32
        self.log(logging.DEBUG, 'FPS target {}, divisor {}, actual {}'.
                 format(fps, fps_divisor, self.maxfps/fps_divisor))
        self.regwrite('FRAME_RATE', fps_divisor)
        return None

    def set_emissivity(self, emissivity):
        """Set emissivity, given in integer % (1-100) or float (0-1)"""
        if emissivity > 100 or emissivity <= 0:
            raise ValueError('Emissivity must be 0 to 1 (float) or 1 to 100 (int, %)')
        #
        if emissivity <= 1:
            # assume a fraction; but MI48 accepts only integer %
            emissivity = int(emissivity * 100)
        #
        self.log(logging.DEBUG, 'Setting emissivity to {} %'.
                 format(emissivity))
        self.regwrite('EMISSIVITY', emissivity)
        return None

    def enable_low_netd(self, factor=DEFAULT_CTRL_STAT['NETD_FACTOR'],
                        row=None, col=None, with_low_netd_row_in_frame=False):
        """Configure enhanced NETD"""
        if row is None:
            row = int(self.rows/2.) - 1
        if col is None:
            col = int(self.cols/2.) - 1
        netd_config = 0x01
        if with_low_netd_row_in_frame:
            netd_config += 0x02
        self.regwrite('NETD_FACTOR', factor)
        self.log(logging.DEBUG, 'NETD_FACTOR {}'.format(
                 hex(self.get_netd_factor())))
        self.regwrite('NETD_PIXEL_Y', row)
        self.regwrite('NETD_PIXEL_X', col)
        self.regwrite('NETD_CONFIG', netd_config)
        self.log(logging.DEBUG, 'NETD_CONFIG {}'.format(
                 hex(self.get_netd_config())))
        self.with_low_netd_row_in_frame = with_low_netd_row_in_frame
        return None

    def disable_low_netd(self):
        """Disable enhanced NETD"""
        netd_config = 0x00
        self.regwrite('NETD_CONFIG', netd_config)
        self.with_low_netd_row_in_frame = False
        self.log(logging.DEBUG, 'NETD_CONFIG {}'.format(
                 hex(self.get_netd_config())))
        return None

    def get_netd_factor(self):
        """Read the value of NETD Reduction Factor register"""
        return self.regread('NETD_FACTOR')

    def get_netd_config(self):
        """Read the value of NETD Reduction Factor register"""
        return self.regread('NETD_CONFIG')

    def enable_filter(self, f1=False, f2=False, f3=False, f3_ks_5=False):
        """
        Enable filters: f1-temporal, f3-median, f2-rolling average.

        Implement a read-modify-write operation, so that filters may
        be toggled independently.
        """
        while True:
            try:
                fctrl = self.regread('FILTER_CTRL')
                break
            except:
                pass
        #fctrl = 0x00
        if f1:
            # enable and initialise filter 1
            fctrl |= 0x03  # bit 0 and 1
        if f2:
            fctrl |= 0x04  # bit 3
        if f3:
            fctrl |= 0x40  # bit 6
        if f3_ks_5:
            fctrl |= 0x20  # bit 5
        msg = "Enabling"
        if fctrl & 0x01:
            fset1 = self.get_filter_1()
            msg += ' Filter 1 ({})'.format(hex(fset1))
        if fctrl & 0x04:
            fset2 = self.get_filter_2()
            msg += ' Filter 2 ({})'.format(hex(fset2))
        if fctrl & 0x40:
            msg += ' Filter 3 ({})'.format(hex(fctrl & 0x20))
        self.log(logging.DEBUG, msg)
        self.regwrite('FILTER_CTRL', fctrl)
        time.sleep(40.e-3)
        self.log(logging.DEBUG, 'FILTER_CONTROL {}'.format(
                 hex(self.get_filter_ctrl())))
        #return self.regread('FILTER_CTRL')
        return None

    def disable_filter(self, f1=True, f2=True, f3=True):
        fctrl = self.regread('FILTER_CTRL')
        if f1:
            # disable filter 1
            fctrl &= 0xFC  # clear bit 0 and 1 (1111_1100)
        if f2:
            # disable filter 2
            fctrl &= 0xFB  # clear bit 2 (1111_1011)
        if f3:
            # disable filter 3
            fctrl &= 0xBF  # clear bit 6 (1011_1111)
        msg = "Disabling"
        if f1:
            msg += ' Filter 1'
        if f2:
            msg += ' Filter 2'
        if f3:
            msg += ' Filter 3'
        self.log(logging.DEBUG, msg)
        self.regwrite('FILTER_CTRL', fctrl)
        self.log(logging.DEBUG, 'FILTER_CONTROL {}'.format(
                 hex(self.get_filter_ctrl())))
        return None

    def get_filter_1(self):
        lsb = self.regread('FILTER_1_LSB')
        msb = self.regread('FILTER_1_MSB')
        res = (msb << 8) + lsb
        return res

    def set_filter_1(self, setting=None):
        if None:
            lsb = DEFAULT_CTRL_STAT['FILTER_1_LSB']
            msb = DEFAULT_CTRL_STAT['FILTER_1_MSB']
        lsb = setting & 0xFF
        msb = (setting & 0xFF00) >> 8
        self.regwrite('FILTER_1_LSB', lsb)
        self.regwrite('FILTER_1_MSB', msb)
        return None

    def set_filter_2(self, setting=DEFAULT_CTRL_STAT['FILTER_2']):
        self.regwrite('FILTER_2', setting)
        return None

    def set_distance_corr(self, correction_factor):
        """Set a distance correction factor; must be tabulated vs distance in the host"""
        regval = correction_factor
        self.log(logging.DEBUG, 'Setting distance corection factor to {}'.
                 format(correction_factor))
        self.regwrite('DISTANCE_CORR', regval)
        return None

    def set_offset_corr(self, offset_in_Kelvin):
        """Set an offset across entire frame in Kelvin; in increment of 0.05 K"""
        assert offset_in_Kelvin <= 6.35 and offset_in_Kelvin >= -6.4
        n = int(round(offset_in_Kelvin / T_OFFSET_UNIT))
        if n < 0:
            regval = 256 - abs(n)
        else:
            regval = n
        self.log(logging.DEBUG, 'Setting temperature offset, [K]: {}, regvalue: {}'.
                 format(offset_in_Kelvin, regval))
        self.regwrite('OFFSET_CORR', regval)
        return None

    def get_camera_type(self):
        """Read SenXor_Type register"""
        return self.regread('SENXOR_TYPE')

    def get_camera_id(self):
        """Read SenXor_ID register; Return string Year.Week.Fab.SerNum
        """
        uid = []
        for i in range(0, MI48_SENXOR_ID_LEN):
            uid.append(self.regread('SENXOR_ID_{}'.format(i)))
        uid_hex = bytearray(uid).hex()
        year = 2000 + uid[0]
        week = uid[1]
        fab  = uid[2]
        sernum_hex = bytearray(uid[3:]).hex()
        sernum = (uid[3] << 16) + (uid[4] << 8) + uid[5]
        uid = '{}.{}.{}.{}'.format(year, week, fab, sernum)
        uid_hexsn = '{}.{}.{}.{}'.format(year, week, fab, sernum_hex)
        return uid, uid_hex, uid_hexsn

    def get_fw_version(self):
        """Get maj.min.build of EVK FW; return as a string"""
        fwv = self.regread('FW_VERSION_1')
        fwb = self.regread('FW_VERSION_2')
        fwv_major = (fwv >> 4) & 0xF
        fwv_minor = fwv & 0xF
        fwv_build = fwb
        return '{}.{}.{}'.format(fwv_major, fwv_minor, fwv_build)

    def enable_user_flash(self):
        self.regwrite('FLASH_CTRL', 0x01)

    def disable_user_flash(self):
        self.regwrite('FLASH_CTRL', 0x00)

    def get_compensation_params(self, npar=3, base_addr=0):
        """
        Read the compensation parameters stored in the MI48 flash.

        Return a list of `npar` floats, where `npar` is the number of
        parameters. 
        The parameters are stored at `base_addr` in the user
        flash space, using little-endian order, i.e.  LSB to 0x00 etc.,
        in the form of 4--byte IEEE-754 numbers.
        """
        params = []
        for i in range(npar):
            # Parameters are stored as IEEE-754 floats, i.e. 4-bytes,
            # little endian.
            # When we read the MI48 we get back unsigned int for each
            # byte.
            int_list = []
            for j in range(4):
                flash_addr = base_addr + 4 * i + j
                int_list.append(self.regread(flash_addr))
                time.sleep(1)
            byte_array = array.array('B', int_list)
            params.append(struct.unpack('<f', byte_array)[0])
        return params

    def store_compensation_params(self, params, base_addr=0, timeout=0.5):
        """
        Write compensation parameters to user space of MI48 flash.

        `params` is a list of floats. Each float is translated to
        a 4-byte IEEE-754 representation and stored in sequence,
        starting from `base_addr` in the user flash space, using
        little-endian order, i.e.  LSB to `base_addr`
        """
        for i, p in enumerate(params):
            byte_array = struct.pack('<f', p)
            int_list = list(byte_array)
            assert len(list(byte_array)) == 4
            for j, uint8 in enumerate(int_list):
                flash_addr = base_addr + 4 * i + j
                self.regwrite(flash_addr, uint8)
                # writing to a flash memory; not sure of the speed
                time.sleep(timeout)

    def parse_frame_header(self, header: list, with_low_netd_row=False):
        """
        Return a dictionary with parsed head items in appropriate type.

        Assume header is already a list of 16 bit unsigned int or similar
        """
        result = {}
        result['frame_counter']         = header[SPIHDR_FRCNT]
        result['senxor_vdd']            = header[SPIHDR_SXVDD] / 1.0e4
        result['senxor_temperature']    = header[SPIHDR_SXTA] / 100. + KELVIN_0
        result['timestamp']             = (header[SPIHDR_TIME + 1] << 16) +\
                                          header[SPIHDR_TIME]
        result['pixel_max']             = header[SPIHDR_MAXV] / 10. + KELVIN_0
        result['pixel_min']             = header[SPIHDR_MINV] / 10. + KELVIN_0
        result['crc']                   = hex(header[SPIHDR_CRC])
        # low NETD
        result['low_netd_pixel_x']     = header[SPIHDR_LOW_NETD_PX_XY] & 0x00FF
        result['low_netd_pixel_y']     = header[SPIHDR_LOW_NETD_PX_XY] >> 8
        _low_netd_px = header[SPIHDR_LOW_NETD_PX]
        result['low_netd_pixel']        = _low_netd_px / 10. + KELVIN_0
        if with_low_netd_row:
            _row_len = int(len(header) / 2)
            result['low_netd_row']      = header[_row_len:] / 10. + KELVIN_0
        return result

    def start(self, stream=True, with_header=True, with_low_netd_row_in_header=False):
        """
        Start capture.
        """
        mode = 0
        if stream:
            self.log(logging.DEBUG, 'Entering continuous capture mode.')
            mode = CONTINUOUS_STREAM
        else:
            self.log(logging.DEBUG, 'Capturing a single frame.')
            mode = GET_SINGLE_FRAME
        if not with_header:
            # set the NO_HEADER bit
            mode = mode | NO_HEADER
            self.log(logging.DEBUG, 'Capture without frame header.')
        if with_low_netd_row_in_header:
            # Enable LOW_NETD_ROW_IN_HEADER bit
            mode = mode | LOW_NETD_ROW_IN_HEADER
            self.log(logging.DEBUG, 'Insert low NETD row in  header.')
        # Set flags based on which to know how to interpret the header
        self.capture_no_header = (not with_header)
        self.with_low_netd_row_in_header = with_low_netd_row_in_header
        #
        self.regwrite('FRAME_MODE', mode)
        return None

    def stop_capture(self, verbose=True, poll_timeout=0.1,
                     stop_timeout=0.3):
        """Stop capture; currently clears the FRAME_MODE register."""
        # Attempt to stop capture; do not tamper with other bits except
        # the ones for initiating/stopping data acquisition
        mode = self.get_mode()
        if mode is None:
            self.log(logging.DEBUG, 'Lost access to camera interface.')
            return None
        _mode = mode & (~(GET_SINGLE_FRAME | CONTINUOUS_STREAM) & 0xFF)
        # self.log(logging.DEBUG, 'Writing 0x{:02X}'.format(_mode))
        self.regwrite('FRAME_MODE', _mode)
        t0 = time.time()
        delay = 0
        while mode & (GET_SINGLE_FRAME | CONTINUOUS_STREAM) != 0x00:
            mode = self.get_mode(verbose)
            if mode is None:
                self.log(logging.DEBUG, 'Lost access to camera interface.')
                return None
            time.sleep(poll_timeout)  # in ms
            delay = time.time() - t0
            if delay > stop_timeout:  # in ms
                self.log(logging.DEBUG,
                         'Camera module failed to stop in {:.0f} ms'.\
                         format(1.e3 * stop_timeout))
                return mode
        self.log(logging.DEBUG, 'Camera module stopped in {:.0f} ms.'.
            format(1.e3 * delay))
        return mode

    def clear_interface_buffers(self):
        """may need to overload this"""
        for intface in self.interfaces:
            intface.reset_input_buffer()
            intface.reset_output_buffer()

    def close_interfaces(self):
        """may need to overaload this"""
        for intface in self.interfaces:
            intface.close()

    def stop(self, poll_timeout=0.1, stop_timeout=0.5):
        """Stop capture and close ports to device"""
        # stop external device first
        self.log(logging.DEBUG, 'Stopping camera module')
        self.stop_capture(poll_timeout=poll_timeout,
                          stop_timeout=stop_timeout)
        # close the interfaces
        # purge interface buffers to make sure we don't
        # accidentally read/write something old next time we start
        self.log(logging.DEBUG, 'Closing host interfaces')
        self.clear_interface_buffers()
        self.close_interfaces()
        return None

    def __repr__(self):
        _s = []
        _s.append('Camera Type {} (type {}), resolution {}, max FPS {}'.
                  format(self.camera_name, self.camera_type,
                         self.fpa_shape, self.maxfps))
        _s.append('FW version {}'.format(self.fw_version))
        _s.append('SenXor ID {}'.format(self.camera_id))
        return '\n'.join(_s)

def get_reg_name(addr):
    """Given a register address, return its name"""
    for key, val in regmap.items():
        if val == addr: return key
    return 'Unknown reg: 0x{:02X}'.format(addr)

def format_header(hdr, with_low_netd_row=False):
    """Format frame header to represent in log messages"""
    s = "FID{:6d}  time{:8d}  V_dd {:5.3f}  T_SX {:5.2f}".\
        format(hdr['frame_counter'], hdr['timestamp'],\
               hdr['senxor_vdd'], hdr['senxor_temperature'])
    try:
        s = ' '.join([s, "low NETD ({},{}) {:4.1f}".format(
            hex(hdr['low_netd_pixel_y']), hex(hdr['low_netd_pixel_x']),
            hdr['low_netd_pixel'])])
    except KeyError:
        pass
    if with_low_netd_row:
        s = '\n'.join([s, hdr['low_netd_row'].__repr__()])
        s += '\n'
    return s

def format_framestats(data):
    """Format data frame stats to represent in log messages"""
    s = "Min {:6.1f}   Max {:6.1f}  Avg {:5.1f}  Std {:3.1f}".\
            format(data.min(), data.max(), data.mean(),
                   data.astype(np.float64).std())
    return s

    
