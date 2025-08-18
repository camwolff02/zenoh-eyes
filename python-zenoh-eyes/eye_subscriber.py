from zenoh import Config, open as zenoh_open
from electric_eyes import EyeCommand
import time

def main():
    config = Config()
    session = zenoh_open(config)
    key = "robot/eye_command"

    def listener(sample):
        # Convert ZBytes -> bytes for FlatBuffers
        buf = bytes(sample.payload)
        eye = EyeCommand.EyeCommand.GetRootAsEyeCommand(buf, 0)

        print("[Received EyeCommand]")
        print(f"  look_ud = {eye.LookUd()} deg")
        print(f"  look_lr = {eye.LookLr()} deg")
        print(f"  eye_sep = {eye.EyeSep()} deg/mm")
        print(f"  blink   = {eye.Blink()}")

    session.declare_subscriber(key, listener)
    print(f"Subscribed to '{key}'")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down...")

    session.close()

if __name__ == "__main__":
    main()
