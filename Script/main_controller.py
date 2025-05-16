import serial
import threading
import os
import subprocess
import signal

# Настройка COM-порта
ser = serial.Serial(
    port='COM3',  # Укажи нужный порт
    baudrate=921600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE
)

output_file = "adc_output.txt"
reading = True
running = True

# Запуск viewer.py и сохранение процесса
viewer_proc = subprocess.Popen(
    ["python", "viewer.py"],
    creationflags=subprocess.CREATE_NEW_CONSOLE
)

# Поток чтения из порта
def read_from_serial():
    global reading
    with open(output_file, "w"):  # очистить файл
        pass

    with open(output_file, "a") as f:
        while running:
            if reading:
                try:
                    lsb = ser.read(1)
                    msb = ser.read(1)
                    value = int.from_bytes(lsb + msb, byteorder='little')
                    f.write(f"{value}\n")
                    f.flush()
                except:
                    break

def main():
    global running, reading
    threading.Thread(target=read_from_serial, daemon=True).start()

    print("Команды: 0x..., stop, continue, clear, close")
    while True:
        try:
            user_input = input().strip()

            if user_input.lower() == "stop":
                reading = False
            elif user_input.lower() == "continue":
                reading = True
            elif user_input.lower() == "clear":
                with open(output_file, "w"):
                    pass
            elif user_input.lower() == "close":
                running = False
                ser.close()
                # Закрыть viewer по PID
                viewer_proc.terminate()
                break
            elif user_input.startswith("0x"):
                hex_str = user_input[2:]
                if len(hex_str) % 2 != 0:
                    continue
                try:
                    command_bytes = bytes.fromhex(hex_str)
                    ser.write(command_bytes)
                except ValueError:
                    pass
        except Exception:
            break

if __name__ == "__main__":
    main()
