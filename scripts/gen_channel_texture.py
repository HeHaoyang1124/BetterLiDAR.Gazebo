import math
import numpy as np
import OpenEXR
import Imath

CHANNELS = 128
OUTPUT = "AT128_channel.lut.exr"

# Horizontal offsets in degrees
HZ_OFFSETS_DEG = (2.4, -0.65, -2.4, 0.65)

# Distance limits in meters
R_MIN_NEAR = 0.5
R_MIN_DEFAULT = 4.5
R_MAX_FAR = 260.0
R_MAX_DEFAULT = 90.0

# Far channel range indices (0-based)
FAR_RANGE_START = 32
FAR_RANGE_END = 95


def group_of(i: int) -> int:
    block = i // 16
    rem = i % 4

    return rem if block % 2 == 0 else (2, 0, 3, 1)[rem]


if __name__ == "__main__":
    pixels = np.zeros((CHANNELS, 4), dtype=np.float32)
    for i in range(CHANNELS):
        g = group_of(i)
        r_min = R_MIN_NEAR if i % 4 == 2 else R_MIN_DEFAULT
        r_max = R_MAX_FAR if FAR_RANGE_START <= i <= FAR_RANGE_END else R_MAX_DEFAULT
        hz_offset = math.radians(HZ_OFFSETS_DEG[g])
        pixels[i] = (r_min, r_max, hz_offset, float(g))

    FLOAT = Imath.PixelType(Imath.PixelType.FLOAT)
    header = OpenEXR.Header(1, CHANNELS)
    header['channels'] = {
        'R': Imath.Channel(FLOAT),
        'G': Imath.Channel(FLOAT),
        'B': Imath.Channel(FLOAT),
        'A': Imath.Channel(FLOAT),
    }

    exr = OpenEXR.OutputFile(OUTPUT, header)
    exr.writePixels({
        'R': pixels[:, 0].tobytes(),
        'G': pixels[:, 1].tobytes(),
        'B': pixels[:, 2].tobytes(),
        'A': pixels[:, 3].tobytes(),
    })
    exr.close()

    print(f"[gen_channel_texture] channels={CHANNELS} -> {OUTPUT}")