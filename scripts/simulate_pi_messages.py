import time
from src.common.protocol import make_message, encode_message


def main():
    cases = [("FAR", 3.0, 0.95, False), ("CAUTION", 1.5, 0.92, True),
             ("DANGER", 0.7, 0.96, True), ("CAUTION", None, 0.8, True)]
    for seq, (zone, distance, confidence, pedestrian) in enumerate(cases, start=1):
        message = make_message(seq, int(time.monotonic() * 1000), zone, distance, confidence, pedestrian)
        print(encode_message(message).decode().rstrip())
        time.sleep(0.1)


if __name__ == "__main__":
    main()
