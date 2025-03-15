import struct
from enum import Enum

from framework.serial_helper import read_string
from framework.readable.readable import Readable

class CommunicationChannel(Enum):
  SPI_0 = 0
  SPI_1 = 1
  I2C_0 = 2
  I2C_1 = 3
  NONE = 33

class FaultDefinition:
    fault_name: str
    fault_bit: int

    is_communication_channel: bool = False
    communication_channel: CommunicationChannel

    is_faulted: bool

    @staticmethod
    def read_definition(readable: Readable, all_faults: int) -> 'FaultDefinition':
      new_def = FaultDefinition()
      new_def.fault_bit, comm_channel_id = struct.unpack('<2B', readable.read(2))
      new_def.fault_name = read_string(readable)

      new_def.communication_channel = CommunicationChannel(comm_channel_id)

      new_def.is_communication_channel = (new_def.fault_bit & 0x80)> 0
      new_def.fault_bit &= 0x7F

      new_def.is_faulted = ((all_faults >> new_def.fault_bit) & 0x01) > 0

      return new_def