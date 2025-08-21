# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mainspace:msg/EncoderSpeed.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_EncoderSpeed(type):
    """Metaclass of message 'EncoderSpeed'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('mainspace')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'mainspace.msg.EncoderSpeed')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__encoder_speed
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__encoder_speed
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__encoder_speed
            cls._TYPE_SUPPORT = module.type_support_msg__msg__encoder_speed
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__encoder_speed

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class EncoderSpeed(metaclass=Metaclass_EncoderSpeed):
    """Message class 'EncoderSpeed'."""

    __slots__ = [
        '_fl',
        '_fr',
        '_rl',
        '_rr',
    ]

    _fields_and_field_types = {
        'fl': 'int32',
        'fr': 'int32',
        'rl': 'int32',
        'rr': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.fl = kwargs.get('fl', int())
        self.fr = kwargs.get('fr', int())
        self.rl = kwargs.get('rl', int())
        self.rr = kwargs.get('rr', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.fl != other.fl:
            return False
        if self.fr != other.fr:
            return False
        if self.rl != other.rl:
            return False
        if self.rr != other.rr:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def fl(self):
        """Message field 'fl'."""
        return self._fl

    @fl.setter
    def fl(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'fl' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'fl' field must be an integer in [-2147483648, 2147483647]"
        self._fl = value

    @builtins.property
    def fr(self):
        """Message field 'fr'."""
        return self._fr

    @fr.setter
    def fr(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'fr' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'fr' field must be an integer in [-2147483648, 2147483647]"
        self._fr = value

    @builtins.property
    def rl(self):
        """Message field 'rl'."""
        return self._rl

    @rl.setter
    def rl(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'rl' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'rl' field must be an integer in [-2147483648, 2147483647]"
        self._rl = value

    @builtins.property
    def rr(self):
        """Message field 'rr'."""
        return self._rr

    @rr.setter
    def rr(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'rr' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'rr' field must be an integer in [-2147483648, 2147483647]"
        self._rr = value
