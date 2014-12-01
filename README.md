# Printable Modular Robot (PMR) operation system

## Installation
For each module, set at least the Module Parameters befor flashing the board:
- master = true; Determines whether this module is master or not.
- adress = 0..255; This modules adress, 0 for the master, a unique number for each other module.

## Commands
The robot offers two communication layers. The internal communication is based on a simple protocol that uses only 2 bytes for encoding address, command and data. Two additional bytes are used to separate commmands with a starting and ending signature.

### Internal

`| 0xFE | addressAndCommand | data | 0xFF |`

The four most significant bits of the `adressAndCommand` encode the address the command will be sent to. This allows for 16 different addresses. We decided to use `0x0` as master address and `0xF` for broadcasting of messages.  
The four lower bits specify the command. Since we use `0xFE` and `0xFF` as signature there are 14 lower bits left for encoding the command. The following list summarizes all available internal commands.

value | command
--- | ---
0x0 | TOPOLOGY
0x1 | HEARTBEAT
0x2 | SET_ANGLE
0x3 | GET_ANGLE
0x4 | ENGAGE
0x5 | MESSAGE
0x6 | CALIB_SERVO
0x7 | PING
0x8 | CONNECT
0x9 | OSCILLATE
0xA | SET_FREQUENCY
0xB | SET_AMPLITUDE
0xC | SET_PHASE
0xD | OSCILLATOR_CLOCK_RESET

### External
