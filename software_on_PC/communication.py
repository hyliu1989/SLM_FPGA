# -*- coding: utf-8 -*-
"""
Python wrapper for the JTAG Atlantic communiation library

Adapted from http://alterawiki.com/wiki/High_Speed_Image_Download_Demo
"""
import ctypes
import numpy as np

# requiring jtag_atlantic.dll and jtag_client.dll put in the same folder as this notebook
dll_path = 'jtag_atlantic.dll'

atlantic_dll = ctypes.cdll.LoadLibrary(dll_path)


"""
Major DLL functions extraction
"""
## JTAGATLANTIC * jtagatlantic_open (
##     const char * chain,
##     int device_index,
##     int link_instance,
##     const char * app_name);
JAOpen = getattr(atlantic_dll,'?jtagatlantic_open@@YAPEAUJTAGATLANTIC@@PEBDHH0@Z')  # mangled name obtained from dllwalker
JAOpen.argtypes = [ctypes.c_char_p, ctypes.c_int64, ctypes.c_int64, ctypes.c_char_p]
JAOpen.restype = ctypes.c_void_p


## int  jtagatlantic_get_error (const char * * other_info);
JAGetError = getattr(atlantic_dll,'?jtagatlantic_get_error@@YA?AW4JATL_ERROR@@PEAPEBD@Z')
JAGetError.argtypes = [ctypes.c_void_p]
JAGetError.restype = ctypes.c_int64


## void jtagatlantic_close (JTAGATLANTIC * link);
JAClose = getattr(atlantic_dll,'?jtagatlantic_close@@YAXPEAUJTAGATLANTIC@@@Z')
JAClose.argtypes = [ctypes.c_void_p]
JAClose.restype = None


## int  jtagatlantic_write (JTAGATLANTIC * link, const char * data, unsigned int count);
JAWrite = getattr(atlantic_dll,'?jtagatlantic_write@@YAHPEAUJTAGATLANTIC@@PEBDI@Z')
JAWrite.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint64]
JAWrite.restype = ctypes.c_int64


## int  jtagatlantic_flush (JTAGATLANTIC * link);
JAFlush = getattr(atlantic_dll,'?jtagatlantic_flush@@YAHPEAUJTAGATLANTIC@@@Z')
JAFlush.argtypes = [ctypes.c_void_p]
JAFlush.restype = ctypes.c_int64


## int  jtagatlantic_read (JTAGATLANTIC * link, char * buffer, unsigned int buffsize);
JARead = getattr(atlantic_dll,'?jtagatlantic_read@@YAHPEAUJTAGATLANTIC@@PEADI@Z')
JARead.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint64]
JARead.restype = ctypes.c_int64


def getLink(cable_name=b"DE-SoC [USB-1]", device_index=2, link_instance=0):
    """
    Get the pointer to the JTAG instance
    
    The information of cable_name, device_index and link_instance can be ubtained 
    by running nios2-terminal.exe.
    
    cable_name: related to the board you are using
    device_index: related to the onboard chips that are on JTAG links. 
                  0: to let the DLL determine itself. 
                  1: usually the onboard FPGA "writer" chip.
                  2: the FPGA chip
    link_instance: -1: to let the DLL determine the JTAG instance in the chip. 
                    0: in my case I have only 1 instance of JTAG-UART so it is indexed by 0.
    """
    link = JAOpen(
        ctypes.c_char_p(cable_name),
        ctypes.c_int64(device_index),
        ctypes.c_int64(link_instance),
        ctypes.c_char_p(b'')
    )
    return link


def _writeBody(link, cmd, data_bytes, ack=True, verbose=True):
    assert len(cmd) == 1

    # to_send = cmd + data_bytes + b'\xFE\x01'

    total_bytes = 0
    n_bytes_written = 0
    
    # write command
    total_bytes += 1
    n_bytes_written += JAWrite(link, ctypes.c_char_p(cmd), 1)
    JAFlush(link)
    
    # write image data
    if isinstance(data_bytes, bytes):
        total_bytes += len(data_bytes)
        for i in range(0, len(data_bytes), 10000):
            to_send_chunk = data_bytes[i:i+10000]
            n_bytes_written += JAWrite(link, ctypes.c_char_p(to_send_chunk), len(to_send_chunk))
            JAFlush(link)
    else:
        for seg in data_bytes:
            if not isinstance(seg,bytes):
                JAWrite(link, ctypes.c_char_p(b'\xFE\x00'), 2)
                raise TypeError('data_bytes contains nonbytes item')
            total_bytes += len(seg)
            for i in range(0, len(seg), 10000):
                to_send_chunk = seg[i:i+10000]
                n_bytes_written += JAWrite(link, ctypes.c_char_p(to_send_chunk), len(to_send_chunk))
                JAFlush(link)
    
    # write ack command
    if ack:
        total_bytes += 2
        n_bytes_written += JAWrite(link, ctypes.c_char_p(b'\xFE\x01'), 2)
        JAFlush(link)

    if verbose:
        print('to send',total_bytes,'bytes')
        print('bytes written:', n_bytes_written)
    return n_bytes_written == total_bytes

def _imagesToMyBytes(images):
    images = images.ravel()
    assert images.dtype == np.uint8

    segments = []
    i_seg_head = 0
    for i in range(images.size):
        if images[i] == 0xFE:
            segments.append(images[i_seg_head:i+1].tobytes())  # including images[i]
            segments.append(b'\xFE')
            i_seg_head = i+1
    
    if i_seg_head == images.size:  # the last pixel is 0xFE and is processed
        pass
    else:  # the last pixel has not been processed
        segments.append(images[i_seg_head:].tobytes())

    return segments


"""
Main Utility functions
"""
def sendImages(link, images, verbose=True):
    assert len(images.shape) == 3
    num_images = images.shape[0]
    assert images.shape[1] == 1024
    assert images.shape[2] == 1024
    assert images.dtype == np.uint8
    assert 1 <= num_images and num_images <= 64

    cmd = bytes([0x80+num_images-1])
    img_bytes_segs = _imagesToMyBytes(images)
    
    return _writeBody(link, cmd, img_bytes_segs, ack=True, verbose=verbose)


def sendOneImage(link, image, frame_id, verbose=True):
    assert len(image.shape) == 2
    assert image.shape[0] == 1024
    assert image.shape[1] == 1024
    assert image.dtype == np.uint8
    assert 0 <= frame_id and frame_id <= 63

    cmd = bytes([0x40+frame_id])
    img_bytes_segs = _imagesToMyBytes(image)
    
    return _writeBody(link, cmd, img_bytes_segs, ack=True, verbose=verbose)


def sendInterrupt(link):
    to_send = b'\xFE\x00'
    n_bytes_written = JAWrite(link, ctypes.c_char_p(to_send), len(to_send))
    JAFlush(link)
    return n_bytes_written == 2


def sendNumFrames(link, num_frames, verbose=True):
    if num_frames <= 0 or num_frames > 64:
        raise ValueError('The value of number of frames can only be within 1 to 64.')
    num_frames = np.uint8(num_frames)
    data_bytes=bytes([num_frames])
    return _writeBody(link, cmd=b'\x07', data_bytes=data_bytes, ack=True, verbose=verbose)


def sendOffsetX(link, offset_x_value=0, verbose=True):
    if abs(offset_x_value) > 128:
        raise ValueError('The absolute value of offset should not exceed 128')
    data_bytes = bytes([abs(offset_x_value), (offset_x_value<0)])
    return _writeBody(link, cmd=b'\x01', data_bytes=data_bytes, ack=True, verbose=verbose)
 

def sendOffsetY(link, offset_y_value=0, verbose=True):
    if abs(offset_y_value) > 128:
        raise ValueError('The absolute value of offset should not exceed 128')
    data_bytes = bytes([abs(offset_y_value), (offset_y_value<0)])
    return _writeBody(link, cmd=b'\x02', data_bytes=data_bytes, ack=True, verbose=verbose)


def sendCyclesOfDisplay(link, n_cycles=1, verbose=True):
    if abs(n_cycles) >= (1<<16):
        raise ValueError('The value should not exceed 65535')
    n_cycles = np.uint16(n_cycles)
    n_cycles_u = np.uint8((n_cycles & 0xFF00) >> 8)
    n_cycles_l = np.uint8((n_cycles & 0x00FF)     )
    data_bytes = bytes([n_cycles_l, n_cycles_u])
    return _writeBody(link, cmd=b'\x03', data_bytes=data_bytes, ack=True, verbose=verbose)


def triggerSequencing(link, verbose=True):
    return _writeBody(link, cmd=b'\x04', data_bytes=b'', ack=True, verbose=verbose)


def triggerSequencingWithGalvo(link, verbose=True):
    return _writeBody(link, cmd=b'\x05', data_bytes=b'', ack=True, verbose=verbose)


def sendGalvoNumPositions(link, value, verbose=True):
    if value >= (1 << 32):
        raise ValueError('The value should not exceed %d' % ((1<<32)-1))
    value = np.uint32(value)
    value_0 = np.uint8((value & 0x000000FF)     )
    value_1 = np.uint8((value & 0x0000FF00) >> 8)
    value_2 = np.uint8((value & 0x00FF0000) >> 16)
    value_3 = np.uint8((value & 0xFF000000) >> 24)
    data_bytes = bytes([value_0,value_1,value_2, value_3])
    return _writeBody(link, cmd=b'\x06', data_bytes=data_bytes, ack=True, verbose=verbose)

def sendStaticDisplayFrameId(link, frame_id, verbose=True):
    if not (0 <= frame_id and frame_id <= 63):
        raise ValueError('The value should be 0~63 (inclusive)')
    
    frame_id = np.uint8(frame_id)
    data_bytes = bytes([frame_id])
    return _writeBody(link, cmd=b'\x08', data_bytes=data_bytes, ack=True, verbose=verbose)


def closeConnection(link):
    return JAClose(link)

