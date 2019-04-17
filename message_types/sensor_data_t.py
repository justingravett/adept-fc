"""ZCM type definitions
This file automatically generated by zcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class sensor_data_t(object):
    __slots__ = ["x", "y", "z", "u", "v", "w", "a_x", "a_y", "a_z", "p", "q", "r", "phi", "theta", "psi", "Vmag", "alpha", "beta", "lat", "lon", "alt", "t_sent"]

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.u = 0.0
        self.v = 0.0
        self.w = 0.0
        self.a_x = 0.0
        self.a_y = 0.0
        self.a_z = 0.0
        self.p = 0.0
        self.q = 0.0
        self.r = 0.0
        self.phi = 0.0
        self.theta = 0.0
        self.psi = 0.0
        self.Vmag = 0.0
        self.alpha = 0.0
        self.beta = 0.0
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.t_sent = 0.0

    def encode(self):
        buf = BytesIO()
        buf.write(sensor_data_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">dddddddddddddddddddddd", self.x, self.y, self.z, self.u, self.v, self.w, self.a_x, self.a_y, self.a_z, self.p, self.q, self.r, self.phi, self.theta, self.psi, self.Vmag, self.alpha, self.beta, self.lat, self.lon, self.alt, self.t_sent))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != sensor_data_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return sensor_data_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = sensor_data_t()
        self.x, self.y, self.z, self.u, self.v, self.w, self.a_x, self.a_y, self.a_z, self.p, self.q, self.r, self.phi, self.theta, self.psi, self.Vmag, self.alpha, self.beta, self.lat, self.lon, self.alt, self.t_sent = struct.unpack(">dddddddddddddddddddddd", buf.read(176))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if sensor_data_t in parents: return 0
        tmphash = (0x794879f655918b74) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + ((tmphash>>63)&0x1)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if sensor_data_t._packed_fingerprint is None:
            sensor_data_t._packed_fingerprint = struct.pack(">Q", sensor_data_t._get_hash_recursive([]))
        return sensor_data_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)
