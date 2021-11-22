import sys
from enum import Enum

build_flags = {
    # Wrist lengths in mm
    "LENGTH1": 10,
    "LENGTH2": 15,
    "KP": 1,
    "KV": 0
}

class Process(Enum):
    PLATFORMIO_BUILD = 0
    MATLAB = 1

# Strings need extra quotes to compile properly
def string_value(value):
    return "\'\"%s\"\'" % value

def main(source):
    if source == Process.PLATFORMIO_BUILD:
        for name, value in build_flags.items():
            definition_value = string_value(value) if isinstance(value, str) else str(value)
            print("-D" + name + "=" + definition_value)
    elif source == Process.MATLAB:
        for name, value in build_flags.items():
            print(name + ' ' + str(value))

    return 0

if __name__ == '__main__':
    if len(sys.argv) == 1:
        process = 0
    else:
        process = int(sys.argv[1])
    sys.exit(main(Process(process)))