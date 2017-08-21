# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: messages_tracker_geometry.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='messages_tracker_geometry.proto',
  package='',
  serialized_pb=_b('\n\x1fmessages_tracker_geometry.proto\"\x8e\x03\n\x19TRACKER_GeometryFieldSize\x12\x12\n\nline_width\x18\x01 \x02(\x05\x12\x14\n\x0c\x66ield_length\x18\x02 \x02(\x05\x12\x13\n\x0b\x66ield_width\x18\x03 \x02(\x05\x12\x16\n\x0e\x62oundary_width\x18\x04 \x02(\x05\x12\x15\n\rreferee_width\x18\x05 \x02(\x05\x12\x12\n\ngoal_width\x18\x06 \x02(\x05\x12\x12\n\ngoal_depth\x18\x07 \x02(\x05\x12\x17\n\x0fgoal_wall_width\x18\x08 \x02(\x05\x12\x1c\n\x14\x63\x65nter_circle_radius\x18\t \x02(\x05\x12\x16\n\x0e\x64\x65\x66\x65nse_radius\x18\n \x02(\x05\x12\x17\n\x0f\x64\x65\x66\x65nse_stretch\x18\x0b \x02(\x05\x12#\n\x1b\x66ree_kick_from_defense_dist\x18\x0c \x02(\x05\x12)\n!penalty_spot_from_field_line_dist\x18\r \x02(\x05\x12#\n\x1bpenalty_line_from_spot_dist\x18\x0e \x02(\x05\"\xcd\x02\n!TRACKER_GeometryCameraCalibration\x12\x11\n\tcamera_id\x18\x01 \x02(\r\x12\x14\n\x0c\x66ocal_length\x18\x02 \x02(\x02\x12\x19\n\x11principal_point_x\x18\x03 \x02(\x02\x12\x19\n\x11principal_point_y\x18\x04 \x02(\x02\x12\x12\n\ndistortion\x18\x05 \x02(\x02\x12\n\n\x02q0\x18\x06 \x02(\x02\x12\n\n\x02q1\x18\x07 \x02(\x02\x12\n\n\x02q2\x18\x08 \x02(\x02\x12\n\n\x02q3\x18\t \x02(\x02\x12\n\n\x02tx\x18\n \x02(\x02\x12\n\n\x02ty\x18\x0b \x02(\x02\x12\n\n\x02tz\x18\x0c \x02(\x02\x12\x1f\n\x17\x64\x65rived_camera_world_tx\x18\r \x01(\x02\x12\x1f\n\x17\x64\x65rived_camera_world_ty\x18\x0e \x01(\x02\x12\x1f\n\x17\x64\x65rived_camera_world_tz\x18\x0f \x01(\x02\"p\n\x10TRACKER_Geometry\x12)\n\x05\x66ield\x18\x01 \x02(\x0b\x32\x1a.TRACKER_GeometryFieldSize\x12\x31\n\x05\x63\x61lib\x18\x02 \x03(\x0b\x32\".TRACKER_GeometryCameraCalibration')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_TRACKER_GEOMETRYFIELDSIZE = _descriptor.Descriptor(
  name='TRACKER_GeometryFieldSize',
  full_name='TRACKER_GeometryFieldSize',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='line_width', full_name='TRACKER_GeometryFieldSize.line_width', index=0,
      number=1, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='field_length', full_name='TRACKER_GeometryFieldSize.field_length', index=1,
      number=2, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='field_width', full_name='TRACKER_GeometryFieldSize.field_width', index=2,
      number=3, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='boundary_width', full_name='TRACKER_GeometryFieldSize.boundary_width', index=3,
      number=4, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='referee_width', full_name='TRACKER_GeometryFieldSize.referee_width', index=4,
      number=5, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='goal_width', full_name='TRACKER_GeometryFieldSize.goal_width', index=5,
      number=6, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='goal_depth', full_name='TRACKER_GeometryFieldSize.goal_depth', index=6,
      number=7, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='goal_wall_width', full_name='TRACKER_GeometryFieldSize.goal_wall_width', index=7,
      number=8, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='center_circle_radius', full_name='TRACKER_GeometryFieldSize.center_circle_radius', index=8,
      number=9, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='defense_radius', full_name='TRACKER_GeometryFieldSize.defense_radius', index=9,
      number=10, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='defense_stretch', full_name='TRACKER_GeometryFieldSize.defense_stretch', index=10,
      number=11, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='free_kick_from_defense_dist', full_name='TRACKER_GeometryFieldSize.free_kick_from_defense_dist', index=11,
      number=12, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='penalty_spot_from_field_line_dist', full_name='TRACKER_GeometryFieldSize.penalty_spot_from_field_line_dist', index=12,
      number=13, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='penalty_line_from_spot_dist', full_name='TRACKER_GeometryFieldSize.penalty_line_from_spot_dist', index=13,
      number=14, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=36,
  serialized_end=434,
)


_TRACKER_GEOMETRYCAMERACALIBRATION = _descriptor.Descriptor(
  name='TRACKER_GeometryCameraCalibration',
  full_name='TRACKER_GeometryCameraCalibration',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='camera_id', full_name='TRACKER_GeometryCameraCalibration.camera_id', index=0,
      number=1, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='focal_length', full_name='TRACKER_GeometryCameraCalibration.focal_length', index=1,
      number=2, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='principal_point_x', full_name='TRACKER_GeometryCameraCalibration.principal_point_x', index=2,
      number=3, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='principal_point_y', full_name='TRACKER_GeometryCameraCalibration.principal_point_y', index=3,
      number=4, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='distortion', full_name='TRACKER_GeometryCameraCalibration.distortion', index=4,
      number=5, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='q0', full_name='TRACKER_GeometryCameraCalibration.q0', index=5,
      number=6, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='q1', full_name='TRACKER_GeometryCameraCalibration.q1', index=6,
      number=7, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='q2', full_name='TRACKER_GeometryCameraCalibration.q2', index=7,
      number=8, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='q3', full_name='TRACKER_GeometryCameraCalibration.q3', index=8,
      number=9, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='tx', full_name='TRACKER_GeometryCameraCalibration.tx', index=9,
      number=10, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ty', full_name='TRACKER_GeometryCameraCalibration.ty', index=10,
      number=11, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='tz', full_name='TRACKER_GeometryCameraCalibration.tz', index=11,
      number=12, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='derived_camera_world_tx', full_name='TRACKER_GeometryCameraCalibration.derived_camera_world_tx', index=12,
      number=13, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='derived_camera_world_ty', full_name='TRACKER_GeometryCameraCalibration.derived_camera_world_ty', index=13,
      number=14, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='derived_camera_world_tz', full_name='TRACKER_GeometryCameraCalibration.derived_camera_world_tz', index=14,
      number=15, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=437,
  serialized_end=770,
)


_TRACKER_GEOMETRY = _descriptor.Descriptor(
  name='TRACKER_Geometry',
  full_name='TRACKER_Geometry',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='field', full_name='TRACKER_Geometry.field', index=0,
      number=1, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='calib', full_name='TRACKER_Geometry.calib', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=772,
  serialized_end=884,
)

_TRACKER_GEOMETRY.fields_by_name['field'].message_type = _TRACKER_GEOMETRYFIELDSIZE
_TRACKER_GEOMETRY.fields_by_name['calib'].message_type = _TRACKER_GEOMETRYCAMERACALIBRATION
DESCRIPTOR.message_types_by_name['TRACKER_GeometryFieldSize'] = _TRACKER_GEOMETRYFIELDSIZE
DESCRIPTOR.message_types_by_name['TRACKER_GeometryCameraCalibration'] = _TRACKER_GEOMETRYCAMERACALIBRATION
DESCRIPTOR.message_types_by_name['TRACKER_Geometry'] = _TRACKER_GEOMETRY

TRACKER_GeometryFieldSize = _reflection.GeneratedProtocolMessageType('TRACKER_GeometryFieldSize', (_message.Message,), dict(
  DESCRIPTOR = _TRACKER_GEOMETRYFIELDSIZE,
  __module__ = 'messages_tracker_geometry_pb2'
  # @@protoc_insertion_point(class_scope:TRACKER_GeometryFieldSize)
  ))
_sym_db.RegisterMessage(TRACKER_GeometryFieldSize)

TRACKER_GeometryCameraCalibration = _reflection.GeneratedProtocolMessageType('TRACKER_GeometryCameraCalibration', (_message.Message,), dict(
  DESCRIPTOR = _TRACKER_GEOMETRYCAMERACALIBRATION,
  __module__ = 'messages_tracker_geometry_pb2'
  # @@protoc_insertion_point(class_scope:TRACKER_GeometryCameraCalibration)
  ))
_sym_db.RegisterMessage(TRACKER_GeometryCameraCalibration)

TRACKER_Geometry = _reflection.GeneratedProtocolMessageType('TRACKER_Geometry', (_message.Message,), dict(
  DESCRIPTOR = _TRACKER_GEOMETRY,
  __module__ = 'messages_tracker_geometry_pb2'
  # @@protoc_insertion_point(class_scope:TRACKER_Geometry)
  ))
_sym_db.RegisterMessage(TRACKER_Geometry)


# @@protoc_insertion_point(module_scope)