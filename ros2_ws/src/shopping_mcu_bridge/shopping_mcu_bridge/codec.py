"""Version-1 fixed frames: A5 id length payload CRC16-CCITT-FALSE (little endian)."""
import binascii
import struct


class Codec:
    def __init__(self, contract):
        self.specs = {k: v for k, v in contract['topics'].items() if 'wire_id' in v}
        self.ids = {v['wire_id']: k for k, v in self.specs.items()}
        self.buffer = bytearray()

    def encode(self, key, values):
        spec = self.specs[key]
        payload = struct.pack('<'+spec['format'], *values)
        body = bytes((spec['wire_id'], len(payload)))+payload
        return b'\xa5'+body+struct.pack('<H', binascii.crc_hqx(body, 0xffff))

    def feed(self, data):
        self.buffer.extend(data)
        result = []
        while self.buffer:
            if self.buffer[0] != 0xa5:
                del self.buffer[0]
                continue
            if len(self.buffer) < 3:
                break
            key = self.ids.get(self.buffer[1])
            if key is None or self.buffer[2] != struct.calcsize('<'+self.specs[key]['format']):
                del self.buffer[0]
                continue
            size = self.buffer[2]+5
            if len(self.buffer) < size:
                break
            body = self.buffer[1:size-2]
            crc = struct.unpack('<H', self.buffer[size-2:size])[0]
            if crc != binascii.crc_hqx(body, 0xffff):
                del self.buffer[0]
                continue
            result.append((key, struct.unpack('<'+self.specs[key]['format'], body[2:])))
            del self.buffer[:size]
        return result
