import serial
import threading
import datetime

ser = serial.Serial(
    port='COM3',
    baudrate=256000,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE
)

output_file = "adc_output.txt"
reading = True
running = True

expected_samples = 0
bit_width = 12
sample_counter = 0
receiving_data = False
data_buffer = bytearray()

def parse_samples(buffer, bits_per_sample, expected_count):
    samples = []
    i = 0
    total_bits = len(buffer) * 8
    needed_bits = expected_count * bits_per_sample
    if needed_bits > total_bits:
        print("Ошибка: недостаточно данных для парсинга")
        return None

    bitstream = int.from_bytes(buffer, 'little')
    mask = (1 << bits_per_sample) - 1

    for n in range(expected_count):
        value = (bitstream >> (n * bits_per_sample)) & mask
        samples.append(value)

    return samples

def read_from_serial():
    global reading, running, expected_samples, bit_width, sample_counter, receiving_data, data_buffer
    with open(output_file, "w"):
        pass
    with open(output_file, "a") as f:
        while running:
            if not reading:
                continue
            try:
                header = ser.read(10)
                if len(header) < 10:
                    print("Команда неполная")
                    continue

                # Всегда выводим команду
                print("Получено:", ' '.join(f'{b:02X}' for b in header))

                xor_check = header[-1]
                calc_xor = 0
                for b in header[:9]:  # Включая 0xAA
                    calc_xor ^= b

                if xor_check != calc_xor:
                    if header[1] == 0x00:
                        print("Команда о начале передачи не принята (ошибка XOR)")
                    elif header[1] == 0x01:
                        print("Команда о конце передачи не принята (ошибка XOR)")
                    else:
                        print("Команда повреждена (ошибка XOR)")
                    continue

                if header[0] != 0xAA:
                    print("Некорректная команда: нет sync-бита")
                    continue

                cmd = header[1]

                if cmd == 0x00:
                    # Команда начала передачи
                    expected_samples = header[4] | (header[5] << 8)
                    bit_width = {
                        0x00: 12,
                        0x01: 10,
                        0x02: 8,
                        0x03: 6
                    }.get(header[6], 12)

                    sample_counter = 0
                    data_buffer = bytearray()
                    receiving_data = True
                    print(f"Начало передачи: {expected_samples} отсчётов, {bit_width} бит")

                elif cmd == 0x01:
                    # Команда конца передачи
                    if not receiving_data:
                        print("Команда о конце передачи получена без начала")
                        continue

                    print("Конец передачи")
                    samples = parse_samples(data_buffer, bit_width, expected_samples)
                    if samples is None or len(samples) != expected_samples:
                        print("Ошибка чтения данных")
                        continue

                    for i, val in enumerate(samples):
                        f.write(f"{i} {val}\n")
                    f.flush()

                    receiving_data = False
                    print(f"Успешно записано {len(samples)} отсчётов в файл")

                elif receiving_data:
                    data_buffer.extend(header)

            except Exception as e:
                print(f"Ошибка: {e}")
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
    global running, reading
    threading.Thread(target=read_from_serial, daemon=True).start()

    print("Команды:")
    print("  <cmd> <data...>  — отправить hex-команду (например: 01 05 AA)")
    print("  stop             — остановить приём данных")
    print("  continue         — продолжить приём данных")
    print("  clear            — очистить файл adc_output.txt")
    print("  close            — завершить работу")

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
                print("Файл очищен.")
            elif user_input.lower() == "close":
                running = False
                ser.close()
                break
            else:
                tokens = user_input.split()
                cmd_str = tokens[0]
                data = tokens[1:] if len(tokens) > 1 else []
                packet, cmd_val = build_command(cmd_str, data)
                if packet:
                    ser.write(packet)
                    print(f"Отправлено: {[hex(b) for b in packet]}")
        except Exception as e:
            print(f"Ошибка: {e}")
            break

if __name__ == "__main__":
    main()
