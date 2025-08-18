from zenoh import Config, open as zenoh_open
import flatbuffers
from electric_eyes import EyeCommand

def main():
    # Create Zenoh config and open session
    config = Config()
    session = zenoh_open(config)

    # Build EyeCommand with FlatBuffers
    builder = flatbuffers.Builder(0)
    EyeCommand.EyeCommandStart(builder)
    EyeCommand.EyeCommandAddLookUd(builder, 30)
    EyeCommand.EyeCommandAddLookLr(builder, -10)
    EyeCommand.EyeCommandAddEyeSep(builder, 45)
    EyeCommand.EyeCommandAddBlink(builder, True)
    eye_cmd = EyeCommand.EyeCommandEnd(builder)
    builder.Finish(eye_cmd)

    payload = builder.Output()
    key = "robot/eye_command"

    # Publish data
    session.put(key, payload)
    print(f"Published EyeCommand to '{key}'")

    session.close()

if __name__ == "__main__":
    main()
