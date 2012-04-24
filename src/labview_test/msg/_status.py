"""autogenerated by genmsg_py from status.msg. Do not edit."""
import roslib.message
import struct


class status(roslib.message.Message):
  _md5sum = "7763077d645b748607239d785522621f"
  _type = "labview_test/status"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """#Values for status voltages in volts
float64 five_v
float64 twelve_v
float64 twentyeight_v
float64 fourtyeight_v

#Values for status temperatures in F
float64 temp_a
float64 temp_b
float64 temp_c
float64 temp_d
float64 temp_crio

"""
  __slots__ = ['five_v','twelve_v','twentyeight_v','fourtyeight_v','temp_a','temp_b','temp_c','temp_d','temp_crio']
  _slot_types = ['float64','float64','float64','float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       five_v,twelve_v,twentyeight_v,fourtyeight_v,temp_a,temp_b,temp_c,temp_d,temp_crio
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(status, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.five_v is None:
        self.five_v = 0.
      if self.twelve_v is None:
        self.twelve_v = 0.
      if self.twentyeight_v is None:
        self.twentyeight_v = 0.
      if self.fourtyeight_v is None:
        self.fourtyeight_v = 0.
      if self.temp_a is None:
        self.temp_a = 0.
      if self.temp_b is None:
        self.temp_b = 0.
      if self.temp_c is None:
        self.temp_c = 0.
      if self.temp_d is None:
        self.temp_d = 0.
      if self.temp_crio is None:
        self.temp_crio = 0.
    else:
      self.five_v = 0.
      self.twelve_v = 0.
      self.twentyeight_v = 0.
      self.fourtyeight_v = 0.
      self.temp_a = 0.
      self.temp_b = 0.
      self.temp_c = 0.
      self.temp_d = 0.
      self.temp_crio = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_9d.pack(_x.five_v, _x.twelve_v, _x.twentyeight_v, _x.fourtyeight_v, _x.temp_a, _x.temp_b, _x.temp_c, _x.temp_d, _x.temp_crio))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      _x = self
      start = end
      end += 72
      (_x.five_v, _x.twelve_v, _x.twentyeight_v, _x.fourtyeight_v, _x.temp_a, _x.temp_b, _x.temp_c, _x.temp_d, _x.temp_crio,) = _struct_9d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_9d.pack(_x.five_v, _x.twelve_v, _x.twentyeight_v, _x.fourtyeight_v, _x.temp_a, _x.temp_b, _x.temp_c, _x.temp_d, _x.temp_crio))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 72
      (_x.five_v, _x.twelve_v, _x.twentyeight_v, _x.fourtyeight_v, _x.temp_a, _x.temp_b, _x.temp_c, _x.temp_d, _x.temp_crio,) = _struct_9d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_9d = struct.Struct("<9d")