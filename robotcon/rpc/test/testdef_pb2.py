# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: testdef.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='testdef.proto',
  package='',
  syntax='proto3',
  serialized_options=None,
  serialized_pb=_b('\n\rtestdef.proto\" \n\x0cHelloRequest\x12\x10\n\x08greeting\x18\x01 \x01(\t\"\x1e\n\rHelloResponse\x12\r\n\x05reply\x18\x01 \x01(\t2;\n\x0cHelloService\x12+\n\x08SayHello\x12\r.HelloRequest\x1a\x0e.HelloResponse\"\x00\x62\x06proto3')
)




_HELLOREQUEST = _descriptor.Descriptor(
  name='HelloRequest',
  full_name='HelloRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='greeting', full_name='HelloRequest.greeting', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=17,
  serialized_end=49,
)


_HELLORESPONSE = _descriptor.Descriptor(
  name='HelloResponse',
  full_name='HelloResponse',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='reply', full_name='HelloResponse.reply', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=51,
  serialized_end=81,
)

DESCRIPTOR.message_types_by_name['HelloRequest'] = _HELLOREQUEST
DESCRIPTOR.message_types_by_name['HelloResponse'] = _HELLORESPONSE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

HelloRequest = _reflection.GeneratedProtocolMessageType('HelloRequest', (_message.Message,), dict(
  DESCRIPTOR = _HELLOREQUEST,
  __module__ = 'testdef_pb2'
  # @@protoc_insertion_point(class_scope:HelloRequest)
  ))
_sym_db.RegisterMessage(HelloRequest)

HelloResponse = _reflection.GeneratedProtocolMessageType('HelloResponse', (_message.Message,), dict(
  DESCRIPTOR = _HELLORESPONSE,
  __module__ = 'testdef_pb2'
  # @@protoc_insertion_point(class_scope:HelloResponse)
  ))
_sym_db.RegisterMessage(HelloResponse)



_HELLOSERVICE = _descriptor.ServiceDescriptor(
  name='HelloService',
  full_name='HelloService',
  file=DESCRIPTOR,
  index=0,
  serialized_options=None,
  serialized_start=83,
  serialized_end=142,
  methods=[
  _descriptor.MethodDescriptor(
    name='SayHello',
    full_name='HelloService.SayHello',
    index=0,
    containing_service=None,
    input_type=_HELLOREQUEST,
    output_type=_HELLORESPONSE,
    serialized_options=None,
  ),
])
_sym_db.RegisterServiceDescriptor(_HELLOSERVICE)

DESCRIPTOR.services_by_name['HelloService'] = _HELLOSERVICE

# @@protoc_insertion_point(module_scope)
