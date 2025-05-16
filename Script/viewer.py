import time
import os

output_file = "adc_output.txt"

def main():
    print("Содержимое файла adc_output.txt:\n")
    while True:
        try:
            if os.path.exists(output_file):
                with open(output_file, "r") as f:
                    content = f.read()
                    os.system('cls' if os.name == 'nt' else 'clear')
                    print("Содержимое файла adc_output.txt:\n")
                    print(content.strip())
            time.sleep(1)
        except KeyboardInterrupt:
            break
        except Exception:
            pass

if __name__ == "__main__":
    main()
