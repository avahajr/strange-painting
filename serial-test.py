import serial
import time

# Set up the serial connection (adjust '/dev/ttyS0' to your serial port)
ser = serial.Serial('/dev/ttyACM0', 115200)
ser.reset_input_buffer()

# Main loop for bidirectional communication
try:
    while True:
        # Sending a message to ESP32
        written = ser.write(b'Hello ESP32!\n')
        print(f"wrote {written} bytes: Hello ESP32!")


        if (ser.in_waiting > 0):
            # Receiving message from ESP32
            received_message = ser.readline().decode('utf-8').rstrip()
            print("Received from ESP32:", received_message)
       


except KeyboardInterrupt:
    print("Program terminated!")

finally:
    ser.close()