# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from champion_nav_msgs/ChampionNavLaserScan.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class ChampionNavLaserScan(genpy.Message):
  _md5sum = "850289c5c3fc67d7436158ddcfd96432"
  _type = "champion_nav_msgs/ChampionNavLaserScan"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """Header header
float32 angle_min
float32 angle_max
float32 scan_time
float32 time_increment
float32 range_min
float32 range_max
float32[] ranges
float32[] angles
float32[] intensities

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
"""
  __slots__ = ['header','angle_min','angle_max','scan_time','time_increment','range_min','range_max','ranges','angles','intensities']
  _slot_types = ['std_msgs/Header','float32','float32','float32','float32','float32','float32','float32[]','float32[]','float32[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,angle_min,angle_max,scan_time,time_increment,range_min,range_max,ranges,angles,intensities

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ChampionNavLaserScan, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.angle_min is None:
        self.angle_min = 0.
      if self.angle_max is None:
        self.angle_max = 0.
      if self.scan_time is None:
        self.scan_time = 0.
      if self.time_increment is None:
        self.time_increment = 0.
      if self.range_min is None:
        self.range_min = 0.
      if self.range_max is None:
        self.range_max = 0.
      if self.ranges is None:
        self.ranges = []
      if self.angles is None:
        self.angles = []
      if self.intensities is None:
        self.intensities = []
    else:
      self.header = std_msgs.msg.Header()
      self.angle_min = 0.
      self.angle_max = 0.
      self.scan_time = 0.
      self.time_increment = 0.
      self.range_min = 0.
      self.range_max = 0.
      self.ranges = []
      self.angles = []
      self.intensities = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_6f().pack(_x.angle_min, _x.angle_max, _x.scan_time, _x.time_increment, _x.range_min, _x.range_max))
      length = len(self.ranges)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.Struct(pattern).pack(*self.ranges))
      length = len(self.angles)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.Struct(pattern).pack(*self.angles))
      length = len(self.intensities)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.Struct(pattern).pack(*self.intensities))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 24
      (_x.angle_min, _x.angle_max, _x.scan_time, _x.time_increment, _x.range_min, _x.range_max,) = _get_struct_6f().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.ranges = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.angles = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.intensities = s.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_6f().pack(_x.angle_min, _x.angle_max, _x.scan_time, _x.time_increment, _x.range_min, _x.range_max))
      length = len(self.ranges)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.ranges.tostring())
      length = len(self.angles)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.angles.tostring())
      length = len(self.intensities)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.intensities.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 24
      (_x.angle_min, _x.angle_max, _x.scan_time, _x.time_increment, _x.range_min, _x.range_max,) = _get_struct_6f().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.ranges = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.angles = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.intensities = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_6f = None
def _get_struct_6f():
    global _struct_6f
    if _struct_6f is None:
        _struct_6f = struct.Struct("<6f")
    return _struct_6f