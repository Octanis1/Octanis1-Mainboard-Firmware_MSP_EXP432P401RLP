import unittest
import struct
import msgpack
from flash_log import *

def make_entry_data(time, name, data):
    return msgpack.packb([time, name, data], use_bin_type=True)

def make_entry(data):
    return struct.pack('B', len(data)) + struct.pack('B', crc8(0, data)) + data

class FlashLogTestCase(unittest.TestCase):
    def test_single_entry(self):
        data = make_entry('hello'.encode('ascii'))
        entries = [e for e in log_entries(data)]
        self.assertEqual(['hello'.encode('ascii')], entries)

    def test_can_distinguish_entry_from_garbage(self):
        data = b'\xDE\xAD' + make_entry('hello'.encode('ascii')) + b'\xDE\xAD'
        entries = [e for e in log_entries(data)]
        self.assertEqual(['hello'.encode('ascii')], entries)

    def test_can_unpack_entry_data(self):
        data = make_entry_data(1234, "foo", 42)
        entry = log_entry_unpack(data)
        self.assertEqual({'name': 'foo', 'time': 1234, 'data': 42}, entry)