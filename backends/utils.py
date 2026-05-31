# utils.py

def compute_checksum(frame_bytes):
    chk = 0
    for b in frame_bytes:
        chk ^= b
    return chk & 0xFF


def build_write_frame(addr, value):
    data_h = (value >> 8) & 0xFF
    data_l = value & 0xFF

    frame = [0xAA, 0x01, addr, data_h, data_l]
    chk = compute_checksum(frame)

    frame.append(chk)
    return bytes(frame)


def build_stream_frame(period_ms):
    data_h = (period_ms >> 8) & 0xFF
    data_l = period_ms & 0xFF

    frame = [0xAA, 0x03, 0x04, data_h, data_l]
    chk = compute_checksum(frame)

    frame.append(chk)
    return bytes(frame)


def build_stop_frame():
    frame = [0xAA, 0x04, 0x00, 0x00, 0x00]
    chk = compute_checksum(frame)

    frame.append(chk)
    return bytes(frame)


def parse_telemetry(data):
    if len(data) != 8:
        return None

    if data[0] != 0xFF:
        return None

    chk = compute_checksum(data[:-1])
    if chk != data[-1]:
        return None

    speed = (data[1] << 8) | data[2]
    current = (data[3] << 8) | data[4]
    vcmd = (data[5] << 8) | data[6]

    # Convert signed 16-bit
    def to_signed(val):
        return val - 65536 if val > 32767 else val

    return {
        "speed_raw": to_signed(speed),
        "current_raw": to_signed(current),
        "vcmd_raw": to_signed(vcmd)
    }
def build_read_frame(addr):
    frame = [0xAA, 0x02, addr, 0x00, 0x00]
    frame.append(compute_checksum(frame))
    return bytes(frame)

