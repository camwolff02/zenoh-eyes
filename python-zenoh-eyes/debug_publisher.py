#!/usr/bin/env python3
import time
import zenoh

def main():
    conf = zenoh.Config()  

    # If your router is not localhost, set the connect locator:
    # Uncomment for TCP
    # conf.insert_json5("connect/endpoints", '["tcp/localhost:7447"]')
    session = zenoh.open(conf)

    key = "robot/eye_command"
    print(f"Declaring publisher on '{key}'")
    pub = session.declare_publisher(key)

    count = 0
    try:
        while True:
            payload = f"cmd_{count}"
            print(f"Publishing: {payload}")
            pub.put(payload.encode("utf-8"))   # payload must be bytes
            count += 1
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("Interrupted — exiting.")
    finally:
        session.close()

if __name__ == "__main__":
    main()
