import serial
import csv
import datetime
import msvcrt  # Windows-specific, replaces select.select

PORT = "COM5"
BAUD = 115200
OUTPUT_FILE = "tof_log.csv"

ser = serial.Serial(PORT, BAUD, timeout=1)
print(f"Connected to {PORT}")
print("Commands: START, STOP, MOTOR_ON, MOTOR_OFF, QUIT")

cmd_buffer = ""

with open(OUTPUT_FILE, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["timestamp", "millis", "distance_mm"])

    while True:
        # Windows-compatible keyboard input (non-blocking)
        if msvcrt.kbhit():
            ch = msvcrt.getwche()
            if ch in ('\r', '\n'):
                cmd = cmd_buffer.strip().upper()
                cmd_buffer = ""
                print()  # newline after Enter
                if cmd == "QUIT":
                    break
                print(cmd)
                ser.write((cmd + "\n").encode())
            else:
                cmd_buffer += ch

        # Read from Arduino
        print(cmd_buffer)
        if ser.in_waiting:
            line = ser.readline().decode(errors="ignore").strip()
            print(line)
            if line.startswith("DATA:"):
                parts = line[5:].split(",")
                if len(parts) == 2:
                    ts = datetime.datetime.now().isoformat()
                    writer.writerow([ts, parts[0], parts[1]])
                    f.flush()
                    print(f"[{ts}] {parts[1]} mm")

            elif line.startswith("MSG:"):
                print(f"Arduino: {line[4:]}")

ser.close()
print("Disconnected.")