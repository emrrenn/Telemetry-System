import serial
from datetime import datetime
import time
import csv

def connect_serial(port, baudrate):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"zaman_ms;hiz_kmh;T_bat_C;T_tank_C;V_bat_C;kalan_enerji_Wh")
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return None

def create_csv(filename):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["zaman_ms;hiz_kmh;T_bat_C;T_tank_C;V_bat_C;kalan_enerji_Wh"])

def listen_and_write(ser, csv_filename):
    with open(csv_filename, mode='a+', newline='') as file:
        writer = csv.writer(file)
        file_number = 0

        while True:

            try:
                if ser.in_waiting > 0:
                    data = ser.read(11)
                    timestamp = datetime.now().strftime('%f')[:-3]
                    decimal_data = ' '.join(str(byte) for byte in data[4:9]).split()

                    print(f"{timestamp};     {decimal_data[0]};     {decimal_data[1]};     {decimal_data[2]};      {decimal_data[3]};     {decimal_data[4]}")

                    writer.writerow([f"{timestamp};     {decimal_data[0]};     {decimal_data[1]};     {decimal_data[2]};      {decimal_data[3]};     {decimal_data[4]}"])

            except serial.SerialException as e:

                print(f"Serial connection error: {e}")
                time.sleep(2)
                ser = connect_serial(ser.port, ser.baudrate)
                if ser:
                    file.close()
                    file_number+1
                    csv_filename = f"data_{file_number}"
                    create_csv(csv_filename)
                    file = open(csv_filename, mode='a+', newline='')
                    writer = csv.writer(file)
                else:
                    print("Reconnection failed. Retrying...")

if __name__ == "__main__":
    port = '/dev/tty.usbserial-110'
    baudrate = 9600
    csv_filename = "data.csv"

    ser = connect_serial(port, baudrate)
    if ser:
        create_csv(csv_filename)
        try:
            listen_and_write(ser, csv_filename)
        except KeyboardInterrupt:
            print("Stopping UART listener")
        finally:
            ser.close()
            print("Serial port closed")
    else:
        print("Failed to establish initial serial connection.")