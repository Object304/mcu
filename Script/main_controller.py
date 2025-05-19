import serial
import threading
import os
import subprocess
import datetime

ser = serial.Serial(
    port='COM3',
    baudrate=921600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE
)

output_file = "adc_output.txt"
reading = True
running = True
convert_to_temp = False

viewer_proc = subprocess.Popen(
    ["python", "viewer.py"],
    creationflags=subprocess.CREATE_NEW_CONSOLE
)

def read_from_serial():
    global reading, convert_to_temp
    with open(output_file, "w"):
        pass
    with open(output_file, "a") as f:
        while running:
            if reading:
                try:
                    lsb = ser.read(1)
                    msb = ser.read(1)
                    raw = int.from_bytes(lsb + msb, byteorder='little')

                    if convert_to_temp:
                        voltage = (raw * 3.3) / 4095.0
                        temp = ((1.43 - voltage) * 1000.0 / 4.3) + 25.0
                        f.write(f"{temp:.2f}\n")
                    else:
                        f.write(f"{raw}\n")

                    f.flush()
                except:
                    break

def build_command(cmd_str, data_strs):
    try:
        cmd = int(cmd_str, 16)
        data = [int(d, 16) for d in data_strs if d.strip() != ""]
    except ValueError:
        print("Ошибка: неверный формат байтов (ожидается формат '00')")
        return None, None

    weight = 0x01 if len(data) >= 128 else 0x00

    now = datetime.datetime.now()
    time1 = now.minute
    time2 = now.second

    command = [0xAA, weight, cmd, time1, time2] + data

    xor_val = 0
    for b in command:
        xor_val ^= b

    command.append(xor_val)
    command.append(0xC0)

    return bytes(command), cmd

def send_to_viewer_control(cmd):
    with open("viewer_control.txt", "w") as f:
        f.write(cmd)

def main():
    global running, reading, convert_to_temp
    threading.Thread(target=read_from_serial, daemon=True).start()

    print("Команды:")
    print("  <cmd> <data...>  — отправить hex-команду (например: 12 A1 00)")
    print("  stop             — остановить приём данных")
    print("  continue         — продолжить приём данных")
    print("  clear            — очистить файл adc_output.txt")
    print("  close            — закрыть оба окна")
    print("  refresh          — обновить viewer вручную")
    print("  refresh_en       — включить автообновление viewer")
    print("  refresh_dis      — отключить автообновление viewer")

    while True:
        try:
            user_input = input("> ").strip()

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
                viewer_proc.terminate()
                break
            elif user_input.lower() in ("refresh", "refresh_en", "refresh_dis"):
                send_to_viewer_control(user_input.lower())
            else:
                tokens = user_input.split()
                if not tokens:
                    continue
                cmd_str = tokens[0]
                data = tokens[1:] if len(tokens) > 1 else []
                packet, cmd_val = build_command(cmd_str, data)
                if packet:
                    convert_to_temp = (cmd_val == 0x05)
                    ser.write(packet)
                    print(f"Отправлено: {[hex(b) for b in packet]}")
        except Exception as e:
            print(f"Ошибка: {e}")
            break

if __name__ == "__main__":
    main()
