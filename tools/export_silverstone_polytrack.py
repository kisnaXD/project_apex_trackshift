import sys
import os
import zlib
import struct
import numpy as np

# Ensure project root is on PYTHONPATH
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from f1sim.engine.world import SilverstoneCircuit

DA = [
    "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P",
    "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z", "a", "b", "c", "d", "e", "f",
    "g", "h", "i", "j", "k", "l", "m", "n", "o", "p", "q", "r", "s", "t", "u", "v",
    "w", "x", "y", "z", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9"
]

def polytrack_ba_encode(data: bytes) -> str:
    """Bit-packing encoder matching BA() in Polytrack main.bundle.js."""
    bit_pos = 0
    total_bits = 8 * len(data)
    out = []

    def get_bits(pos):
        byte_idx = pos // 8
        bit_idx = pos % 8
        b0 = data[byte_idx]
        b1 = data[byte_idx + 1] if byte_idx + 1 < len(data) else 0
        word = (b0 | (b1 << 8)) >> bit_idx
        return word & 0x3F

    while bit_pos < total_bits:
        val = get_bits(bit_pos)
        if (30 & ~val) != 0:
            r = val
            bit_pos += 6
        else:
            r = 31 & val
            bit_pos += 5
        out.append(DA[r])

    return "".join(out)

def generate_silverstone_track(output_path="polytrack/0.5.0/tracks/official/silverstone.track"):
    world = SilverstoneCircuit()
    s_steps = np.linspace(0, world.track_length, 450, endpoint=False)
    
    # Scale from 2D sim coordinates to Polytrack grid tiles
    scale = 0.16
    
    # Environment ID (0=Summer), Sun Angle (45)
    payload = bytearray([0, 45])
    
    prev_gx, prev_gz = None, None
    block_count = 0
    
    for s in s_steps:
        x, y, psi = world.frenet_to_cartesian(s, 0.0)
        gx = int(np.round(x * scale))
        gz = int(np.round(y * scale))
        gy = 2  # Ground level
        
        if (gx, gz) == (prev_gx, prev_gz):
            continue
        prev_gx, prev_gz = gx, gz
        
        heading_quad = int(np.round((psi / (np.pi / 2.0)))) % 4
        rot_byte = (heading_quad & 0x3)
        
        payload.extend(struct.pack("<iiiB", gx, gy, gz, rot_byte))
        block_count += 1

    name = "Silverstone GP".encode("utf-8")
    author = "Apex2D".encode("utf-8")
    header = bytearray([len(name)]) + bytearray(name) + bytearray([len(author)]) + bytearray(author)

    # Double deflate compression
    comp1 = zlib.compressobj(level=9, method=zlib.DEFLATED, wbits=9, memLevel=9)
    pass1_bytes = comp1.compress(header + payload) + comp1.flush()
    pass1_str = polytrack_ba_encode(pass1_bytes)

    comp2 = zlib.compressobj(level=9, method=zlib.DEFLATED, wbits=15, memLevel=9)
    pass2_bytes = comp2.compress(pass1_str.encode("utf-8")) + comp2.flush()
    pass2_str = polytrack_ba_encode(pass2_bytes)

    final_track_file = "PolyTrack1" + pass2_str
    
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w") as f:
        f.write(final_track_file)
        
    print(f"Successfully generated {output_path} with {block_count} track nodes.")

if __name__ == "__main__":
    generate_silverstone_track()
