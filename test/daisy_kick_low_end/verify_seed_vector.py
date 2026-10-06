"""Check the linked firmware, not just a successfully compiled ISR symbol."""
import struct
import subprocess
import sys
from pathlib import Path
root = Path(__file__).resolve().parents[2]
build = root / 'daisy-kick/build'
symbols = subprocess.check_output(['arm-none-eabi-nm', str(build/'midi_oled_monitor.elf')], text=True)
address = next(int(line.split()[0],16) for line in symbols.splitlines() if line.endswith(' T KickMidiRxIrq'))
vector = struct.unpack_from('<I', (build/'midi_oled_monitor.bin').read_bytes(), (16+39)*4)[0]
assert vector == address|1, f'USART3 vector {vector:#x} does not reach the MIDI ISR {address:#x}'
print(f'USART3 vector verified: {vector:#x} -> KickMidiRxIrq (Thumb)')
