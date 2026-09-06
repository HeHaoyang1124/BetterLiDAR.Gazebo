import struct
import math

CHANNELS = 128
OUTPUT = "AT128.ctex"

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


def build_channels():
    rows = []
    for i in range(CHANNELS):
        group = group_of(i)

        r_min = R_MIN_NEAR if i % 4 == 2 else R_MIN_DEFAULT
        r_max = R_MAX_FAR if FAR_RANGE_START <= i <= FAR_RANGE_END else R_MAX_DEFAULT
        hz_offset = math.radians(HZ_OFFSETS_DEG[group])

        rows.append((r_min, r_max, hz_offset, float(group)))
    return rows


if __name__ == "__main__":
    rows = build_channels()

    with open(OUTPUT, "wb") as f:
        for r_min, r_max, hz_offset, group in rows:
            f.write(struct.pack("4f", r_min, r_max, hz_offset, group))

    print(f"[gen_channel_texture] channels={CHANNELS} "
          f"bytes={CHANNELS * 4 * 4} -> {OUTPUT}")