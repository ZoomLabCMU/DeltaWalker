# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: delta_trajectory.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x16\x64\x65lta_trajectory.proto\"u\n\x0c\x44\x65ltaMessage\x12\n\n\x02id\x18\x01 \x01(\x05\x12\x12\n\ntrajectory\x18\x02 \x03(\x02\x12\x1a\n\x12request_done_state\x18\x03 \x01(\x08\x12\x1a\n\x12request_joint_pose\x18\x04 \x01(\x08\x12\r\n\x05reset\x18\x05 \x01(\x08\x62\x06proto3')



_DELTAMESSAGE = DESCRIPTOR.message_types_by_name['DeltaMessage']
DeltaMessage = _reflection.GeneratedProtocolMessageType('DeltaMessage', (_message.Message,), {
  'DESCRIPTOR' : _DELTAMESSAGE,
  '__module__' : 'delta_trajectory_pb2'
  # @@protoc_insertion_point(class_scope:DeltaMessage)
  })
_sym_db.RegisterMessage(DeltaMessage)

if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _DELTAMESSAGE._serialized_start=26
  _DELTAMESSAGE._serialized_end=143
# @@protoc_insertion_point(module_scope)
