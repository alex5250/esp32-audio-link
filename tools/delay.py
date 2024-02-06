import serial
import time

# Configure the UART connection
uart = serial.Serial("/dev/ttyUSB0", 115200)  # Replace with the correct serial port and baud rate
output = open("data.csv","a")
try:
    while True:
        # Read a line from UART (assumes messages end with newline '\n')
        message = uart.readline().decode('utf-8')

        # Get the current timestamp
        current_time = time.time()

        # Print the received message and the time elapsed since the last message
        if message:
            print(f"Received message: {message.strip()}")
            if 'last_timestamp' in locals():
                elapsed_time = (current_time - last_timestamp)*1000
                print(f"Time since last message: {elapsed_time:.10f} miliseconds")
                output.write(f"{elapsed_time:.10f}\n")
            last_timestamp = current_time

except KeyboardInterrupt:
    pass

# Close the UART connection when done
uart.close()
