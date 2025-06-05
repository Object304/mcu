import serial
import threading
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
big_data_mode = False
sample_counter = 0

viewer_proc = subprocess.Popen(
    ["python", "viewer.py"],
    creationflags=subprocess.CREATE_NEW_CONSOLE
)

def send_to_viewer_control(cmd):
    with open("viewer_control.txt", "w") as f:
        f.write(cmd)

def read_from_serial():
    global reading, convert_to_temp, sample_counter
    with open(output_file, "w"):
        pass
    with open(output_file, "a") as f:
        buffer = bytearray()
        while running:
            if reading:
                try:
                    byte = ser.read(1)
                    if not byte:
                        continue
                    b = byte[0]

                    if b != 0xAA:
                        continue

                    buffer = bytearray()
                    buffer.append(b)

                    len_bytes = ser.read(2)
                    if len(len_bytes) < 2:
                        continue
                    buffer.extend(len_bytes)
                    num_values = int.from_bytes(len_bytes, byteorder='little')

                    data_bytes = ser.read(2 * num_values)
                    if len(data_bytes) < 2 * num_values:
                        continue
                    buffer.extend(data_bytes)

                    xor_byte = ser.read(1)
                    if not xor_byte:
                        continue
                    buffer.append(xor_byte[0])

                    xor = 0
                    for b in buffer:
                        xor ^= b
                    if xor != 0:
                        f.write("Данные повреждены\n")
                        f.flush()
                        continue

                    for i in range(num_values):
                        raw = int.from_bytes(data_bytes[i*2:i*2+2], byteorder='little')
                        if convert_to_temp:
                            voltage = (raw * 3.3) / 4095.0
                            temp = ((1.43 - voltage) * 1000.0 / 4.3) + 25.0
                            f.write(f"{sample_counter} {temp:.2f}\n")
                        else:
                            f.write(f"{sample_counter} {raw}\n")
                        sample_counter += 1
                    f.flush()
                    send_to_viewer_control("refresh")
                except Exception as e:
                    f.write(f"Ошибка чтения: {e}\n")
                    f.flush()
                    continue

def build_command(cmd_str, data_strs):
    try:
        cmd = int(cmd_str, 16)
        data = [int(d, 16) for d in data_strs if d.strip() != ""]
    except ValueError:
        print("Ошибка: неверный формат байтов (ожидается формат '00')")
        return None, None

    while len(data) < 5:
        data.append(0x00)
    data = data[:5]

    now = datetime.datetime.now()
    time1 = now.minute
    time2 = now.second

    packet = [0xAA, cmd, time1, time2] + data

    xor_val = 0
    for b in packet:
        xor_val ^= b
    packet.append(xor_val)

    return bytes(packet), cmd

def main():
    global running, reading, convert_to_temp, big_data_mode, sample_counter
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
    print("  big_data_dis     — отключить режим передачи большого объёма данных")

    while True:
        try:
            user_input = input("> ").strip()

            if not user_input:
                continue

            if user_input.lower() == "stop":
                reading = False
            elif user_input.lower() == "continue":
                reading = True
            elif user_input.lower() == "clear":
                with open(output_file, "w"):
                    pass
                sample_counter = 0
                send_to_viewer_control("clear")
                send_to_viewer_control("refresh")
            elif user_input.lower() == "close":
                running = False
                ser.close()
                viewer_proc.terminate()
                break
            elif user_input.lower() in ("refresh", "refresh_en", "refresh_dis"):
                send_to_viewer_control(user_input.lower())
            elif user_input.lower() == "big_data_dis":
                big_data_mode = False
                print("Режим передачи большого объёма данных отключён.")
            else:
                tokens = user_input.split()
                if not tokens:
                    continue

                if big_data_mode:
                    try:
                        raw_bytes = bytes(int(t, 16) for t in tokens)
                        ser.write(raw_bytes)
                        print(f"Отправлено: {[hex(b) for b in raw_bytes]}")
                    except ValueError:
                        print("Ошибка: в big_data_mode можно вводить только байты в формате hex")
                else:
                    cmd_str = tokens[0]
                    data = tokens[1:] if len(tokens) > 1 else []
                    packet, cmd_val = build_command(cmd_str, data)
                    if packet:
                        if cmd_val == 0x06:
                            big_data_mode = True
                            print("Передача большого объёма данных началась.")
                        convert_to_temp = (cmd_val == 0x05)
                        ser.write(packet)
                        print(f"Отправлено: {[hex(b) for b in packet]}")
        except Exception as e:
            print(f"Ошибка: {e}")
            break

if __name__ == "__main__":
    main()
