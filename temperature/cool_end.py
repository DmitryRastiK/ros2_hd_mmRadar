import requests
from urllib.parse import urljoin
import time
import csv
import os

BASE_URL = "http://192.168.1.51"
FILENAME = "cool_end_1.csv"

# Ожидание 5 минут, потом 10 изсерений
# Получилось так: 5 + 5 + 5 + 20 минут

def write_first_n_temps(n: int = 10):
    endpoint = "/cgi_diag.cgi"
    url = urljoin(BASE_URL, endpoint)

    script_dir = os.path.dirname(__file__)
    csv_path = os.path.join(script_dir, FILENAME)

    start_monotonic = time.monotonic()
    written_count = 0

    file_exists = os.path.exists(csv_path)
    write_header = not file_exists or (file_exists and os.path.getsize(csv_path) == 0)

    with open(csv_path, mode="a", newline="") as csv_file:
        writer = csv.writer(csv_file)
        if write_header:
            writer.writerow(["elapsed_seconds", "cpuTemp", "cpuTime"])  # header

        while written_count < n:
            elapsed = time.monotonic() - start_monotonic

            cpu_temp = None
            cpu_time = None

            try:
                response = requests.get(url, timeout=5)
                response.raise_for_status()
                data = response.json()
                cpu_temp = data.get("cpuTemp")
                cpu_time = data.get("cpuTime")
            except Exception:
                # Ignore errors and keep polling until we collect N temperature packets
                pass

            # Записываем только если есть температура
            if cpu_temp is not None:
                writer.writerow([f"{elapsed:.6f}", cpu_temp, cpu_time])
                csv_file.flush()
                written_count += 1
                print(f"{elapsed:.6f}s -> записано {written_count}/{n}")
            else:
                print(f"{elapsed:.6f}s -> нет значения температуры, пропуск")

            try:
                time.sleep(1)
            except KeyboardInterrupt:
                print("\nStopped by user.")
                break


if __name__ == "__main__":
    write_first_n_temps(10)


