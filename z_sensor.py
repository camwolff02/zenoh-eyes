import zenoh, random, time

random.seed()

def read_temp():
    return random.randint(15, 30)

if __name__ == "__main__":
    with zenoh.open(zenoh.Config()) as session:
        topic = 'myhome/kitchen/temp'
        pub = session.declare_publisher(topic)

        while True:
            t = read_temp()
            buf = f"{t}"
            print(f"Putting Data ('{topic}': '{buf}')...")
            pub.put(buf)
            time.sleep(1)