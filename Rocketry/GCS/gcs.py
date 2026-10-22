import sys, time, threading, datetime
import serial                                    # pip install pyserial

TEAM = 7
HEADER = ("TEAM_ID,TIME_S,PACKET_COUNT,ALTITUDE_M,PRESSURE_PA,TEMP_C,VOLTAGE_V,"
          "GNSS_TIME,GNSS_LAT,GNSS_LON,GNSS_ALT_M,GNSS_SATS,"
          "ACCEL_X,ACCEL_Y,ACCEL_Z,GYRO_SPIN_DPS,FLIGHT_STATE")
NUM_COLUMNS = HEADER.count(",") + 1              # 17

COMMANDS = {                                     # name -> opcode (cmd_opcode_t in types.h)
    "start": 0x01, "stop": 0x02, "cal_baro": 0x03, "cal_gyro": 0x04, "cal_accel": 0x05,
    "test": 0x06, "pad": 0x07, "sep": 0x09, "payload": 0x0A, "chute": 0x0B,
}

ser = serial.Serial(sys.argv[1], 115200, timeout=0.2)    # e.g. COM5 or /dev/ttyUSB0
time.sleep(2)                                    # opening the port resets the board: let it start

stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
csv_path = f"flight_{stamp}.csv"
csv = open(csv_path, "w", newline="")
csv.write(HEADER + "\n"); csv.flush()
raw = open(f"raw_{stamp}.log", "w")              # every packet with its signal strength

def reader():
    while True:
        text = ser.readline().decode("ascii", errors="replace").strip()
        if not text:
            continue
        if not text.startswith("RX|"):
            print("[bridge]", text)               # INFO / ERR lines from the bridge board
            continue
        _, rssi, snr, payload = text.split("|", 3)
        raw.write(f"{time.time():.1f},{rssi},{snr},{payload}\n"); raw.flush()
        if payload.startswith("ACK,"):
            print(f"[ACK] {payload}   (RSSI {rssi} dBm, SNR {snr} dB)")
        elif payload.count(",") == NUM_COLUMNS - 1:           # a telemetry line
            csv.write(payload + "\n"); csv.flush()
            f = payload.split(",")
            print(f"[TLM #{f[2]}] t={f[1]}s alt={f[3]}m state={f[16]}   (RSSI {rssi} dBm, SNR {snr} dB)")
        else:
            print(f"[??] unrecognised: {payload}")

threading.Thread(target=reader, daemon=True).start()

seq = 0

def send_command(opcode, arg=0, repeats=3):
    global seq
    seq = (seq + 1) % 256
    body = [TEAM, seq, opcode, arg]
    xor = 0
    for b in body:
        xor ^= b
    frame = " ".join(f"{b:02X}" for b in body + [xor])
    for _ in range(repeats):          # the rocket is deaf while it transmits: repeat with the same sequence number
        ser.write(f"CMD {frame}\n".encode())
        time.sleep(0.4)
    print(f"sent {frame}  x{repeats}")

def switch_channel(backup):
    send_command(0x08, 1 if backup else 0)
    time.sleep(1.0)                   # give the rocket time to retune
    ser.write(("CH BACKUP\n" if backup else "CH MAIN\n").encode())

print(f"Saving to {csv_path}")
print("Commands:", ", ".join(COMMANDS), ", backup, main, quit")
while True:
    word = input("> ").strip().lower()
    if word == "quit":
        break
    elif word == "backup":
        switch_channel(True)
    elif word == "main":
        switch_channel(False)
    elif word in COMMANDS:
        send_command(COMMANDS[word])
    elif word:
        print("unknown command")
csv.close(); raw.close()