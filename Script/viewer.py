import time
import os

output_file = "adc_output.txt"
auto_refresh = False
last_content = ""

def print_file(force=False):
    global last_content
    if os.path.exists(output_file):
        with open(output_file, "r") as f:
            content = f.read()
            if force or content != last_content:
                os.system('cls' if os.name == 'nt' else 'clear')
                print("Содержимое файла adc_output.txt\n")
                print(content.strip())
                last_content = content

def main():
    global auto_refresh
    print("Содержимое файла adc_output.txt\n")
    while True:
        try:
            if os.path.exists("viewer_control.txt"):
                with open("viewer_control.txt", "r") as f:
                    command = f.read().strip()
                os.remove("viewer_control.txt")

                if command == "refresh":
                    print_file()
                elif command == "refresh_en":
                    auto_refresh = True
                elif command == "refresh_dis":
                    auto_refresh = False
                elif command == "clear":
                    print_file(force=True)  # одна принудительная очистка + обновление

            if auto_refresh:
                print_file()
            time.sleep(0.2)
        except KeyboardInterrupt:
            break
        except Exception:
            pass

if __name__ == "__main__":
    main()
