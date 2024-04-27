import serial
import time
import threading

def send_msg(ser, interval):
    try: 
        while True:
            ser.write(b'Hello ESP32!\n')
            if (ser.in_waiting > 0):
                # Receiving message from ESP32
                received_message = ser.readline().decode('utf-8').rstrip()
                print("Received from ESP32:", received_message)
            time.sleep(interval)
    except Exception as e:
        print("error:", e)
    

ser = serial.Serial('/dev/ttyACM0', 115200)
ser.reset_input_buffer()

# Main loop for bidirectional communication
try:
    thread = threading.Thread(target=send_msg, args=(ser, 0.1))
    thread.start()
    while True:
        print("main program is running")
        time.sleep(1)
       


except KeyboardInterrupt:
    print("Program terminated!")

finally:
    ser.close()