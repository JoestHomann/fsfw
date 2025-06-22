import struct


# Commands
TEST_COMMAND_0 = struct.pack("!I", 1)
TEST_COMMAND_1 = struct.pack("!I", 2)

TEST_COMMAND_1_PARAM_1 = bytearray([0xBA, 0xB0])
TEST_COMMAND_1_PARAM_2 = bytearray([0x00, 0x00, 0x00, 0x52, 0x4F, 0x42, 0x49, 0x4E])

DUMMY_COMMAND_3 = bytearray([0xBA, 0xDE, 0xAF, 0xFE])
