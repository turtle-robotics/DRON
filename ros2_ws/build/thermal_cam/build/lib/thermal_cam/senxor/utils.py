# Copyright (C) Meridian Innovation Ltd. Hong Kong, 2020. All rights reserved.

# For CRC reference start with http://crcmod/sourceforge.net/crcmod.predefined.html
# The MI48 implements the CRC-16/CCITT-FALSE
# polynomial = 0x11021, init=0xFFFF, reversed=False, xor-out=0x0000,
# check=0x29B1 (for input of b'123456789)
import time
import math
import itertools
from functools import partial
import operator
import crcmod.predefined
import numpy as np
import cv2 as cv
import cmapy

list_ironbow_b = [0,6,12,18,27,38,49,59,64,68,73,78,82,86,90,94,98,102,105,109,112,115,119,122,124,127,129,132,134,136,138,140,142,145,147,148,150,151,152,153,154,155,157,158,159,160,161,163,163,164,165,166,166,167,167,167,167,167,166,166,166,165,165,165,165,164,164,164,163,162,161,160,160,160,158,157,156,155,153,152,151,150,148,147,146,145,143,142,141,140,138,136,134,132,130,127,125,123,121,119,118,116,114,112,110,108,106,104,102,100,98,96,94,92,90,88,86,84,82,80,78,75,73,71,69,67,65,63,61,59,57,55,53,51,49,48,46,44,42,40,38,36,34,32,31,29,27,25,24,22,21,20,18,17,16,15,13,12,11,9,8,7,6,4,3,2,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,2,3,5,6,7,9,10,12,13,14,16,17,20,23,26,28,31,34,37,39,42,45,48,50,53,56,59,62,66,70,74,78,82,86,91,96,101,106,111,115,120,125,130,135,140,146,152,158,164,171,178,185,192,201,210,219,229,237,243,248,251,254]
list_ironbow_g = [0,0,0,0,0,0,0,0,0,1,2,3,4,3,3,2,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,2,2,2,2,3,3,3,4,5,6,7,8,9,10,11,12,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,30,31,32,33,34,35,36,37,39,40,42,43,45,47,48,50,51,53,54,56,58,59,61,62,64,65,67,69,70,72,73,75,76,78,80,81,83,84,86,88,89,91,93,95,96,98,100,102,103,105,107,109,110,112,114,116,117,119,121,122,124,126,128,129,131,133,134,136,138,139,141,143,145,146,148,150,151,153,155,156,158,160,161,163,165,167,168,170,172,173,175,177,178,180,182,184,185,187,188,190,191,193,194,196,197,199,200,202,203,205,206,208,209,211,212,214,215,216,217,219,220,221,223,224,225,227,228,229,231,232,233,235,235,236,236,237,238,239,240,241,242,243,244,245,246,247,248,249,249,250,251,252,253,254,255,255,255,255,255,254,254,254,254,254]
list_ironbow_r = [0,0,0,0,0,0,0,0,0,0,0,0,0,2,5,9,12,16,19,23,26,29,33,36,39,43,46,49,52,54,57,60,63,66,69,71,74,77,80,83,85,88,91,94,96,99,102,105,107,110,112,115,117,120,122,124,127,129,131,133,136,138,140,142,145,147,149,151,154,156,158,160,161,163,165,167,169,170,172,174,176,178,179,181,183,185,187,189,190,192,194,195,196,198,199,201,202,204,205,206,208,209,211,212,213,214,215,216,217,218,219,221,222,223,224,225,226,227,228,229,230,231,232,234,235,236,237,238,239,240,241,242,243,243,244,245,245,246,247,248,248,249,250,250,251,251,252,253,253,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,254,254,254,253,253,252,252,252,251,251,250,250,250,250,250,249,248,247,246,246,245,245,245,246,247,249,251,254]
lut_ironbow = np.zeros((256, 1, 3), dtype=np.uint8)
lut_ironbow[:,:,0] = np.array(list_ironbow_b).reshape(256,1)
lut_ironbow[:,:,1] = np.array(list_ironbow_g).reshape(256,1)
lut_ironbow[:,:,2] = np.array(list_ironbow_r).reshape(256,1)

list_rainbow2 = [ 1, 3, 74, 0, 3, 74, 0, 3, 75, 0, 3, 75, 0, 3, 76, 0, 3, 76, 0, 3, 77, 0, 3, 79, 0, 3, 82, 0, 5, 85, 0, 7, 88, 0, 10, 91, 0, 14, 94, 0, 19, 98, 0, 22, 100, 0, 25, 103, 0, 28, 106, 0, 32, 109, 0, 35, 112, 0, 38, 116, 0, 40, 119, 0, 42, 123, 0, 45, 128, 0, 49, 133, 0, 50, 134, 0, 51, 136, 0, 52, 137, 0, 53, 139, 0, 54, 142, 0, 55, 144, 0, 56, 145, 0, 58, 149, 0, 61, 154, 0, 63, 156, 0, 65, 159, 0, 66, 161, 0, 68, 164, 0, 69, 167, 0, 71, 170, 0, 73, 174, 0, 75, 179, 0, 76, 181, 0, 78, 184, 0, 79, 187, 0, 80, 188, 0, 81, 190, 0, 84, 194, 0, 87, 198, 0, 88, 200, 0, 90, 203, 0, 92, 205, 0, 94, 207, 0, 94, 208, 0, 95, 209, 0, 96, 210, 0, 97, 211, 0, 99, 214, 0, 102, 217, 0, 103, 218, 0, 104, 219, 0, 105, 220, 0, 107, 221, 0, 109, 223, 0, 111, 223, 0, 113, 223, 0, 115, 222, 0, 117, 221, 0, 118, 220, 1, 120, 219, 1, 122, 217, 2, 124, 216, 2, 126, 214, 3, 129, 212, 3, 131, 207, 4, 132, 205, 4, 133, 202, 4, 134, 197, 5, 136, 192, 6, 138, 185, 7, 141, 178, 8, 142, 172, 10, 144, 166, 10, 144, 162, 11, 145, 158, 12, 146, 153, 13, 147, 149, 15, 149, 140, 17, 151, 132, 22, 153, 120, 25, 154, 115, 28, 156, 109, 34, 158, 101, 40, 160, 94, 45, 162, 86, 51, 164, 79, 59, 167, 69, 67, 171, 60, 72, 173, 54, 78, 175, 48, 83, 177, 43, 89, 179, 39, 93, 181, 35, 98, 183, 31, 105, 185, 26, 109, 187, 23, 113, 188, 21, 118, 189, 19, 123, 191, 17, 128, 193, 14, 134, 195, 12, 138, 196, 10, 142, 197, 8, 146, 198, 6, 151, 200, 5, 155, 201, 4, 160, 203, 3, 164, 204, 2, 169, 205, 2, 173, 206, 1, 175, 207, 1, 178, 207, 1, 184, 208, 0, 190, 210, 0, 193, 211, 0, 196, 212, 0, 199, 212, 0, 202, 213, 1, 207, 214, 2, 212, 215, 3, 215, 214, 3, 218, 214, 3, 220, 213, 3, 222, 213, 4, 224, 212, 4, 225, 212, 5, 226, 212, 5, 229, 211, 5, 232, 211, 6, 232, 211, 6, 233, 211, 6, 234, 210, 6, 235, 210, 7, 236, 209, 7, 237, 208, 8, 239, 206, 8, 241, 204, 9, 242, 203, 9, 244, 202, 10, 244, 201, 10, 245, 200, 10, 245, 199, 11, 246, 198, 11, 247, 197, 12, 248, 194, 13, 249, 191, 14, 250, 189, 14, 251, 187, 15, 251, 185, 16, 252, 183, 17, 252, 178, 18, 253, 174, 19, 253, 171, 19, 254, 168, 20, 254, 165, 21, 254, 164, 21, 255, 163, 22, 255, 161, 22, 255, 159, 23, 255, 157, 23, 255, 155, 24, 255, 149, 25, 255, 143, 27, 255, 139, 28, 255, 135, 30, 255, 131, 31, 255, 127, 32, 255, 118, 34, 255, 110, 36, 255, 104, 37, 255, 101, 38, 255, 99, 39, 255, 93, 40, 255, 88, 42, 254, 82, 43, 254, 77, 45, 254, 69, 47, 254, 62, 49, 253, 57, 50, 253, 53, 52, 252, 49, 53, 252, 45, 55, 251, 39, 57, 251, 33, 59, 251, 32, 60, 251, 31, 60, 251, 30, 61, 251, 29, 61, 251, 28, 62, 250, 27, 63, 250, 27, 65, 249, 26, 66, 249, 26, 68, 248, 25, 70, 248, 24, 73, 247, 24, 75, 247, 25, 77, 247, 25, 79, 247, 26, 81, 247, 32, 83, 247, 35, 85, 247, 38, 86, 247, 42, 88, 247, 46, 90, 247, 50, 92, 248, 55, 94, 248, 59, 96, 248, 64, 98, 248, 72, 101, 249, 81, 104, 249, 87, 106, 250, 93, 108, 250, 95, 109, 250, 98, 110, 250, 100, 111, 251, 101, 112, 251, 102, 113, 251, 109, 117, 252, 116, 121, 252, 121, 123, 253, 126, 126, 253, 130, 128, 254, 135, 131, 254, 139, 133, 254, 144, 136, 254, 151, 140, 255, 158, 144, 255, 163, 146, 255, 168, 149, 255, 173, 152, 255, 176, 153, 255, 178, 155, 255, 184, 160, 255, 191, 165, 255, 195, 168, 255, 199, 172, 255, 203, 175, 255, 207, 179, 255, 211, 182, 255, 216, 185, 255, 218, 190, 255, 220, 196, 255, 222, 200, 255, 225, 202, 255, 227, 204, 255, 230, 206, 255, 233, 208 ]
lut_rainbow2 = np.zeros((256, 1, 3), dtype=np.uint8)
lut_rainbow2[:,:,0] = np.array(list_rainbow2[2::3]).reshape(256,1)
lut_rainbow2[:,:,1] = np.array(list_rainbow2[1::3]).reshape(256,1)
lut_rainbow2[:,:,2] = np.array(list_rainbow2[0::3]).reshape(256,1)


colormaps = {
    'autumn': cv.COLORMAP_AUTUMN,
    'bone': cv.COLORMAP_BONE,
    'jet': cv.COLORMAP_JET,
    'winter': cv.COLORMAP_WINTER,
    'rainbow': cv.COLORMAP_RAINBOW,
    'ocean': cv.COLORMAP_OCEAN,
    'summer': cv.COLORMAP_SUMMER,
    'spring': cv.COLORMAP_SPRING,
    'cool': cv.COLORMAP_COOL,
    'hsv': cv.COLORMAP_HSV,
    'pink': cv.COLORMAP_PINK,
    'hot': cv.COLORMAP_HOT,
    'parula': cv.COLORMAP_PARULA,
    'magma': cv.COLORMAP_MAGMA,
    'inferno': cv.COLORMAP_INFERNO,
    'plasma': cv.COLORMAP_PLASMA,
    'viridis': cv.COLORMAP_VIRIDIS,
    'cividis': cv.COLORMAP_CIVIDIS,
    'twilight': cv.COLORMAP_TWILIGHT,
    'twilight_shifted': cv.COLORMAP_TWILIGHT_SHIFTED,
# the following is available in 4.2 or later only
    # 'turbo': cv.COLORMAP_TURBO,
    'rainbow2': lut_rainbow2,
    'ironbow': lut_ironbow[-256:],
}

crc16 = crcmod.predefined.mkCrcFun('crc-ccitt-false')

def cksum(data, sum=0):
    """Calculate simple sum over data, allowing for non-zero init"""
    sum = sum
    for byte in data:
        sum += byte
    #
    return sum

def data_to_frame(data, array_shape, hflip=False):
    """
    Convert 1D array into nH x nV 2D array corresponding to the FPA.

    Use this func to change orientation to forward looking camera with `hflip`.
    """
    # Note that the data coming for the EVK is stored as a 1D array.
    # the data.reshape() reconstructs the 2D FPA array shape; 
    # Note the data ordering is 'F' (fortran-like).
    nc, nr = array_shape
    if hflip:
        # The flipping below realises horisontal flip, assuming that 
        # the USB port faces the ceiling or the sky, to correct for 
        # left/right flip in the camera, if necessary.
        frame = np.flip(data.reshape(array_shape, order='F').T, 1)
    else:
        frame = data.reshape(array_shape, order='F').T
    return frame.copy()

def remap(data, new_range=(0, 255), curr_range=None, to_uint8=True):
    """
    Remap data from one range to another; return float16.

    This function is critical for working with temperature data and
    maintaining accuracy.

    The mapping is a linear transformation:

        l1, h1 = curr_range

        l2, h2 = new_range

        x = (data - l1) / (h1 - l1)

        out = l2 + x * (h2 - l1)

    If `curr_range` is not specified, assume it is defined by the data limits.
    If `to_uint8` is true, return an uint8, instead of float16. This is
    useful in conjuction with `new_range` being (0, 255), to prepare for
    many OpneCV routines which accept only uint8.
    """
    lo2, hi2 = new_range
    #
    if curr_range is None:
        lo1 = np.min(data)
        hi1 = np.max(data)
    else:
        lo1, hi1 = curr_range
    #
    # The relpos below represents the relative position of _data in the
    # current range.
    # We could potentially manipulate relpos by some function to 
    # realise non-linear remapping
    relpos = (data - lo1) / float(hi1 - lo1)
    out = lo2 + relpos * (hi2 - lo2)
    #
    if to_uint8:
        return out.astype('uint8')
    else:
        return out.astype('float16')

def distance_correction(dist):
    """
    Return the distance correction factor for a given distance

    This is likely to change for mass production version, if we find that
    for a large sample a different set of coefficient fit the data better.
    """
    return 1.0445 + 0.0875 * dist

def get_default_outfile(src_id=None, ext='dat'):
    """Yield a timestamped filename with specified extension."""
    ts = time.strftime('%Y%m%d-%H%M%S', time.localtime())
    if src_id is not None:
        filename = "senxor-{}-{}.{}".format(src_id, ts, ext)
    else:
        filename = "senxor-{}.{}".format(ts, ext)
    return filename

def cv_render(data, title='', resize=(800, 620), colormap='jet',
              interpolation=cv.INTER_CUBIC, display=True):
    """
    Render and display a 2D numpy array data of type uint8, using OpenCV.
    
    Color the image using any of the supported OpenCV colormaps.
    Resize the image, ensuring the aspect ratio is maintained.
    Use cubic interpolation when upsizing.
    
    If `display` is true, render the image in an OpenCV-controled window.
    Else, return the OpenCV image object.
    """
    try:
        # use defualt opencv maps or explicitly defined above
        cmap = colormaps[colormap]
    except KeyError:
        # use matplotlib
        cmap = cmapy.cmap(colormap)
    cvcol = cv.applyColorMap(data, cmap)
    if isinstance(resize, tuple) or isinstance(resize, list):
        cvresize =  cv.resize(cvcol, dsize=resize,
                            interpolation=interpolation)
    else:
        cvresize =  cv.resize(cvcol, dsize=None, fx=resize, fy=resize,
                            interpolation=interpolation)
    if display:
        cv.imshow(title, cvresize)
    return cvresize

def cv_filter(data, parameters=None, use_median=True, use_bilat=True,
                     use_nlm=False):
    """
    Spatial filtering based on a sequence of Meidan, Bilateral and NLM.

    Requires uint8 and returns uint8 data.

    For best results, set both Bilateral and Non-Local Means
    flags to true.

    See OpenCV for significance and values of the optional parameters
    """
    # default median filter parameters
    par = {'blur_ks': 5}
    # default bilateral filter parameters
    par.update({'d': 7, 'sigmaColor': 23, 'sigmaSpace': 23})
    # default nlmeans filter parameters
    par.update({'h': 5, 'templateWindowSize': 5, 'searchWindowSize': 11})
    # update parameters from caller
    if parameters is not None:
        par.update(parameters)
    # do the filtering
    filtered = data
    if use_median:
        #t0 = time.time()
        filtered = cv.medianBlur(filtered, par['blur_ks'])
        #print('Median cost [ms]: {:8.4f}'.format(time.time() - t0))
    if use_bilat:
        #t0 = time.time()
        filtered = cv.bilateralFilter(filtered, d=par.get('d'),
                                    sigmaColor=par.get('sigmaColor'),
                                    sigmaSpace=par.get('sigmaSpace'))
        #print('Bilateral cost [ms]: {:8.4f}'.format(time.time() - t0))

    # nlmeans works only on uint8; the result is also uint8
    # so we must normalise and then renormalise
    if use_nlm:
        t0 = time.time()
        filtered = cv.fastNlMeansDenoising(filtered, None, h=par.get('h'),
                        templateWindowSize=par.get('templateWindowSize'),
                        searchWindowSize=par.get('searchWindowSize'))
        print('NLMeans cost [ms]: {:8.4f}'.format(time.time() - t0))
    return filtered


class RollingAverageFilter:

    def __init__(self, N=4):
        """
        Rolling average filter over ``N`` frames.

        Usage:

            # establish rolling average over 20 frames
            min_filter = RollingAverageFilter(N=20)
            ...

            min_temp = min_filter(measured_min)
        """
        self.N = N
        self.count = 0
        self.av = 0
        self.update = self._update_0

    def _update_0(self, new):
        self.count += 1
        self.av += 1. / self.count * (new - self.av)
        if not self.count < self.N:
            self.update = self._update_1

    def _update_1(self, new):
        self.av += 1. / self.N * (new - self.av)

    def clear(self):
        self.__init__(self.N)

    def __call__(self, new):
        self.update(new)
        return self.av


class FibonacciAverageFilter:

    fib = [0, 1, 2, 3, 5, 8, 13, 21, 34, 55]

    def __init__(self, initial, N=6, i_start=1):
        """
        Fibonacci-weighted average filter over ``N`` frames.

        Usage:

            # establish rolling average over 20 frames
            min_filter = RollingAverageFilter(initial_min, N=20)
            ...

            min_temp = min_filter(measured_min)
        """
        self.N = N
        self.s = i_start
        self.p = i_start + N
        self.frames = [initial] * self.N
        w = np.array(self.fib[self.s: self.p])
        self.weights = np.asarray(w)/np.sum(w)

    def __call__(self, new):
        """Update the rolling average estimate"""
        self.frames = self.frames[1:] + [new]
        print(len(self.frames))
        self.frames[-1] = np.sum([w*f for w,f in
                            zip(self.weights, self.frames)], axis=0)

        print(self.frames[-1].shape)
        return self.frames[-1]


class KeyboardHandler:
    """
    Add a handler for a specific key to allow interactive change of
    parameters which are stored in a dictionary.
    """
    def __init__(self, pardict):
        """
        Keyboard handler that allows for interactive change of parameters.

        pardict (dict): a dictionary with parameter key-val pairs to be
                        affected
        """
        self.parameters = pardict
        self.actions = {}
        self.triggers = {}

    def register(self, key, parname, action='toggle', bounds=None, trigger=None):
        """
        Associate a key-press with certain modification of a parameter.

        Parameters:

            key: key (toggle) or a pair of keys (increment/decrement)
            parname:  name of parameter from ``pardict`` to be affected
            action: 'toggle' by default; if integer, then treat as delta
                    by which to increment/decrement by pair of keys
            bounds: limits for increment/decrement action
            trigger: a function that is passed the updated parameter value
        """
        if action=='toggle':
            self.actions[key] = (parname, partial(operator.not_), bounds)
        else:
            delta = -action
            self.actions[key[0]] = (parname, partial(operator.add, delta), bounds)
            delta = action
            self.actions[key[1]] = (parname, partial(operator.add, delta), bounds)
        if trigger is not None:
            try:
                for k in key:
                    self.triggers[k] = partial(trigger[0], *trigger[1:])
            except AttributeError:
                    self.triggers[key] = partial(trigger[0], *trigger[1:])

    def __call__(self, key):
        """Execute the action associated with pressing of the given key"""
        try:
            pname, func, bounds = self.actions[key]
            pval = self.parameters[pname]
            newval = func(pval)
            if bounds is None:
                self.parameters[pname] = newval
            else:
                self.parameters[pname] = newval
                if newval < bounds[0]:
                    self.parameters[pname] = bounds[0]
                if newval > bounds[1]:
                    self.parameters[pname] = bounds[1]
            # do whatever requested after the parameter update
            try:
                self.triggers[key](self.parameters[pname])
            except KeyError:
                # no further action triggered by the pressed key
                pass
        except KeyError:
            # wrong key pressed
            pass


class TestData:
    nc, nr = 80, 62
    nh = 80
    def __init__(self):
        """Create a dictionary to store all data.
        
        The dictionary key is decided upon adding items.
        When adding data, we can pass either a tupple (Vdd, Tsx, Frames), or
        a 2D array of shape N_frames, n_header+n_col*n_rows.
        In the latter case, the assumption is that header[:, 1] and header[:, 2]
        are Vdd and Tsx. These are parsed to produce the correct units (V, and degC)
        (Vdd, Tsx, frame) is the stored dictionary value.
        """
        self.data = {}
        
    def update(self, key, data):
        """Add data as a tupple (Vdd, Tsx, Frames) or a 2D array from np.loadtxt"""
        try:
            Vdd, Tsx, frames = data
        except ValueError:
            frames = data[:, -self.nc * self.nr:]
            Vdd = data[:, 2]   # * 1.e-4
            Tsx = data[:, 3]   # 100 + KELVIN0
        self.data[key] = Vdd, Tsx, frames
        
    def get(self, key):
        """Retrieve data for a given key"""
        return self.data[key]


def quick_segment(data, param=None):
    """
    Perform a quick hot-on-cold segmentation and return the contour lines,
    the mask of the hot contours, and their statistics, as a 3-tuple.

    Input `data` must be a 2D frame.
    Parameters control the bilateral filtering, median bluring for
    smoothing the contours, and how to do the hot-on-cold thresholding.
    """
    p = {
        'use_bilat': True,
        'bilat_d': 7,
        'bilat_sigmaColor': 23,
        'bilat_sigmaSpace': 23,
        #
        'use_median': True,
        'median_ksize': 3,
        #
        'thresholding': 'adaptive',  # or Otsu?
        'adaptth_blockSize': 97,
        'adaptth_C': -29,
        #
        'contour_minArea': -9,
    }
    if param is not None: p.update(param)
    img = data.copy()
    if p['use_bilat']:
        # reduce noise
        img = cv.bilateralFilter(img.astype(np.float32), d=p['bilat_d'],
                                  sigmaColor=p['bilat_sigmaColor'],
                                  sigmaSpace=p['bilat_sigmaSpace'])
    # abserr = np.abs(bilat - bilat.mean())
    # segment the image
    img = cv.adaptiveThreshold(remap(img), 1, cv.ADAPTIVE_THRESH_GAUSSIAN_C,
                                cv.THRESH_BINARY,
                                blockSize=p['adaptth_blockSize'],
                                C=p['adaptth_C'])
    if p['use_median']:
        # smooth the contours of the segments
        img = cv.medianBlur(img, ksize=p['median_ksize'])

    # extract the contours and return their line, mask and statistics
    contours, hierarchy = cv.findContours(img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    cntr_stats = get_contour_stats(data, contours, minArea=p['contour_minArea'])
    contours, masks, cntrstats = zip(*cntr_stats)
    return contours, masks, cntrstats


def get_contour_stats(data, contours, minArea=None, min_sdev=None,
                      mean_range=None, sortby='mean'):
    """
    Return a list of tupples: [(contour, mask, metrics)].

    The 'metrics' is a dictionary with the following keys:
        centroid, area, mean, median, sdev, min, max, ptp.
    """
    output = []
    # first identify warm-on-cold contours that have at least
    # minArea number of pixels
    for i, c in enumerate(contours):
        # rough estimate of area with sign, to distinguish b/w
        # hot on cold (-ve) and cold on hot (+ve)
        area = cv.contourArea(c, oriented=True)
        # work only with hot on cold contours, assuming hot is foreground
        if minArea is None or area < minArea:
            # create a filled mask for the current contour
            mask = np.zeros(data.shape, dtype='uint8')
            cv.drawContours(mask, contours, i, 1, cv.FILLED)
            # get some metrics of the data within the current contour
            metrics = {}
            M = cv.moments(c)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            centroid = (cx, cy)
            metrics['centroid'] = centroid
            metrics['area'] = int(math.copysign(len( mask[mask != 0]), area))
            metrics['mean'] = data[mask != 0].mean()
            metrics['median'] = np.median(data[mask != 0])
            metrics['sdev'] = data[mask != 0].astype(np.float32).std()
            metrics['min'] = data[mask != 0].min()
            metrics['max'] = data[mask != 0].max()
            metrics['spread'] = np.ptp(data[mask != 0])
            output.append((c, mask, metrics))
    # now check other features of the contours and exclude them
    # if not matching
    exclude = []
    if output:
        for i, (c, mask, metrics) in enumerate(output):
            if min_sdev is not None and metrics['sdev'] < min_sdev:
                exclude.append(i)
            if mean_range is not None and\
               (mean_range[0] > metrics['mean'] or\
                mean_range[1] < metrics['mean']):
                exclude.append(i)
    if exclude:
        output = [output[i] for i in range(len(output)) if i not in exclude]
    output = sorted(output, key=lambda L: L[2][sortby], reverse=True)
    return output


def get_ipx_1D(icol_irow, n=9, ncols=80):
    """
    Return the 1-D vector indexes of the `n` pixels centered on `icol_irow`
    pixel of the 2-D frame.
    """
    ipc, ipr = icol_irow
    ipx = ncols * ipr + ipc-1
    
    # special cases first
    if n == 1:
        ipx = [ipx]
        return ipx
    if n == 3:
        ipx = [ipx, ipx-1, ipx+1]
        return ipx
    if n == 5:
        ipx = [ipx, ipx-1, ipx+1, ipx-ncols, ipx+ncols]
        return ipx
    if n == 6:
        ipx = [ipx, ipx-ncols-1, ipx-ncols+1, ipx+ncols-1, ipx+ncols+1]
        return ipx

    # if n == (2q+1)^2
    q, r = int(np.sqrt(n) // 2), int(np.sqrt(n) % 2)
    assert r == 1
    offs = range(-q, q+1)
    ix_offs = [coloffs + ncols * rowoffs for (rowoffs, coloffs)
               in itertools.product(offs, offs)]
    ipx = [ipx + offs for offs in ix_offs]
    return ipx

get_ipx_1D((40,31), n=9)

def stptime2float(x, fmt="%Y-%m-%dT%H:%M:%S.%f%z"):
    """
    Convert the time string into a numpy float.

    This function may be used as a converter, when reading e.g. output 
    from the SenXorViewer file to a numpy array, via np.loadtxt.
    However, this is not recommended.
    Instead, read the frame data separately:
        data = np.loadtext(filename, usecols=range(n, n+4960), delimiter=',')
    and construct a pandas dataframe for the n columns of header related stuff
    plus select pixels as necessary.
    """
    dt = datetime.datetime.strptime(x, fmt)
    return np.datetime64(dt).astype(float)
