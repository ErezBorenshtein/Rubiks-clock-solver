import serial
import time

# Define the serial port name and baud rate
serial_port = "COM4"  # Change this to match your Arduino's serial port
baud_rate = 9600

# Open the serial port
try:
    ser = serial.Serial(serial_port, baud_rate)
    print(f"Serial port {serial_port} opened successfully.")
except serial.SerialException:
    print(f"Error: Could not open serial port {serial_port}.")
    exit()

# Define the line to send over serial
line_to_send = "p011100 r+00001 r-11110 p001100 r+00011 r+11100 p000100 r+01000 r+10111 p010100 r+11100 r-10011 p010000 r+01000 r+00111 p110000 r-21100 r-10011 p110100 r-11110 r+20001\n"
#line_to_send = "p011100 r+00001 r-11110 \n"

# Convert the line to bytes and send it over serial
try:
    x = ser.readline().decode().strip()
    print(x)
    while(x != "ready"):
        x = ser.readline().decode().strip()
        print(x)
    print("Arduino is ready to receive data.")
    ser.write(line_to_send.encode())
    ser.flush()
    #ser.write("end".encode())
    #ser.flush()
    print("Data sent over serial successfully.")
except serial.SerialException:
    print("Error: Failed to send data over serial.")

while True:
    # Read data from the serial port
    data = ser.readline().decode().strip()
    print(data)
    time.sleep(0.01)

# Close the serial port
ser.close()
print("Serial port closed.")
