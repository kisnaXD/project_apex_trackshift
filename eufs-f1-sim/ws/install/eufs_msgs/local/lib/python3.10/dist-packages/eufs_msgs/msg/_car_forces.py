# generated from rosidl_generator_py/resource/_idl.py.em
# with input from eufs_msgs:msg/CarForces.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CarForces(type):
    """Metaclass of message 'CarForces'."""

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
            module = import_type_support('eufs_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'eufs_msgs.msg.CarForces')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__car_forces
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__car_forces
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__car_forces
            cls._TYPE_SUPPORT = module.type_support_msg__msg__car_forces
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__car_forces

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CarForces(metaclass=Metaclass_CarForces):
    """Message class 'CarForces'."""

    __slots__ = [
        '_front_left_lateral',
        '_front_left_longitudinal',
        '_front_left_vertical',
        '_front_right_lateral',
        '_front_right_longitudinal',
        '_front_right_vertical',
        '_rear_left_lateral',
        '_rear_left_longitudinal',
        '_rear_left_vertical',
        '_rear_right_lateral',
        '_rear_right_longitudinal',
        '_rear_right_vertical',
    ]

    _fields_and_field_types = {
        'front_left_lateral': 'double',
        'front_left_longitudinal': 'double',
        'front_left_vertical': 'double',
        'front_right_lateral': 'double',
        'front_right_longitudinal': 'double',
        'front_right_vertical': 'double',
        'rear_left_lateral': 'double',
        'rear_left_longitudinal': 'double',
        'rear_left_vertical': 'double',
        'rear_right_lateral': 'double',
        'rear_right_longitudinal': 'double',
        'rear_right_vertical': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.front_left_lateral = kwargs.get('front_left_lateral', float())
        self.front_left_longitudinal = kwargs.get('front_left_longitudinal', float())
        self.front_left_vertical = kwargs.get('front_left_vertical', float())
        self.front_right_lateral = kwargs.get('front_right_lateral', float())
        self.front_right_longitudinal = kwargs.get('front_right_longitudinal', float())
        self.front_right_vertical = kwargs.get('front_right_vertical', float())
        self.rear_left_lateral = kwargs.get('rear_left_lateral', float())
        self.rear_left_longitudinal = kwargs.get('rear_left_longitudinal', float())
        self.rear_left_vertical = kwargs.get('rear_left_vertical', float())
        self.rear_right_lateral = kwargs.get('rear_right_lateral', float())
        self.rear_right_longitudinal = kwargs.get('rear_right_longitudinal', float())
        self.rear_right_vertical = kwargs.get('rear_right_vertical', float())

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
        if self.front_left_lateral != other.front_left_lateral:
            return False
        if self.front_left_longitudinal != other.front_left_longitudinal:
            return False
        if self.front_left_vertical != other.front_left_vertical:
            return False
        if self.front_right_lateral != other.front_right_lateral:
            return False
        if self.front_right_longitudinal != other.front_right_longitudinal:
            return False
        if self.front_right_vertical != other.front_right_vertical:
            return False
        if self.rear_left_lateral != other.rear_left_lateral:
            return False
        if self.rear_left_longitudinal != other.rear_left_longitudinal:
            return False
        if self.rear_left_vertical != other.rear_left_vertical:
            return False
        if self.rear_right_lateral != other.rear_right_lateral:
            return False
        if self.rear_right_longitudinal != other.rear_right_longitudinal:
            return False
        if self.rear_right_vertical != other.rear_right_vertical:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def front_left_lateral(self):
        """Message field 'front_left_lateral'."""
        return self._front_left_lateral

    @front_left_lateral.setter
    def front_left_lateral(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'front_left_lateral' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'front_left_lateral' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._front_left_lateral = value

    @builtins.property
    def front_left_longitudinal(self):
        """Message field 'front_left_longitudinal'."""
        return self._front_left_longitudinal

    @front_left_longitudinal.setter
    def front_left_longitudinal(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'front_left_longitudinal' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'front_left_longitudinal' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._front_left_longitudinal = value

    @builtins.property
    def front_left_vertical(self):
        """Message field 'front_left_vertical'."""
        return self._front_left_vertical

    @front_left_vertical.setter
    def front_left_vertical(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'front_left_vertical' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'front_left_vertical' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._front_left_vertical = value

    @builtins.property
    def front_right_lateral(self):
        """Message field 'front_right_lateral'."""
        return self._front_right_lateral

    @front_right_lateral.setter
    def front_right_lateral(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'front_right_lateral' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'front_right_lateral' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._front_right_lateral = value

    @builtins.property
    def front_right_longitudinal(self):
        """Message field 'front_right_longitudinal'."""
        return self._front_right_longitudinal

    @front_right_longitudinal.setter
    def front_right_longitudinal(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'front_right_longitudinal' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'front_right_longitudinal' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._front_right_longitudinal = value

    @builtins.property
    def front_right_vertical(self):
        """Message field 'front_right_vertical'."""
        return self._front_right_vertical

    @front_right_vertical.setter
    def front_right_vertical(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'front_right_vertical' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'front_right_vertical' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._front_right_vertical = value

    @builtins.property
    def rear_left_lateral(self):
        """Message field 'rear_left_lateral'."""
        return self._rear_left_lateral

    @rear_left_lateral.setter
    def rear_left_lateral(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rear_left_lateral' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'rear_left_lateral' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._rear_left_lateral = value

    @builtins.property
    def rear_left_longitudinal(self):
        """Message field 'rear_left_longitudinal'."""
        return self._rear_left_longitudinal

    @rear_left_longitudinal.setter
    def rear_left_longitudinal(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rear_left_longitudinal' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'rear_left_longitudinal' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._rear_left_longitudinal = value

    @builtins.property
    def rear_left_vertical(self):
        """Message field 'rear_left_vertical'."""
        return self._rear_left_vertical

    @rear_left_vertical.setter
    def rear_left_vertical(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rear_left_vertical' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'rear_left_vertical' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._rear_left_vertical = value

    @builtins.property
    def rear_right_lateral(self):
        """Message field 'rear_right_lateral'."""
        return self._rear_right_lateral

    @rear_right_lateral.setter
    def rear_right_lateral(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rear_right_lateral' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'rear_right_lateral' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._rear_right_lateral = value

    @builtins.property
    def rear_right_longitudinal(self):
        """Message field 'rear_right_longitudinal'."""
        return self._rear_right_longitudinal

    @rear_right_longitudinal.setter
    def rear_right_longitudinal(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rear_right_longitudinal' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'rear_right_longitudinal' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._rear_right_longitudinal = value

    @builtins.property
    def rear_right_vertical(self):
        """Message field 'rear_right_vertical'."""
        return self._rear_right_vertical

    @rear_right_vertical.setter
    def rear_right_vertical(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rear_right_vertical' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'rear_right_vertical' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._rear_right_vertical = value
