import requests
from urllib.parse import urljoin
import time
import csv
import os
from datetime import datetime

BASE_URL = "http://192.168.1.50"
FILENAME = "USRR_30V_1666mA_to_LRR.csv"

def poll_cpu_metrics():
    endpoint = "/cgi_diag.cgi"
    url = urljoin(BASE_URL, endpoint)

    script_dir = os.path.dirname(__file__)
    csv_path = os.path.join(script_dir, FILENAME)

    start_monotonic = time.monotonic()

    with open(csv_path, mode="w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["elapsed_seconds", "cpuTemp", "cpuTime"])  # header

        while True:
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
                # Suppress console errors; still log row with None values
                pass

            writer.writerow([f"{elapsed:.6f}", cpu_temp, cpu_time])
            csv_file.flush()

            # Console output: only elapsed time since start
            print(f"{elapsed:.6f}s")

            try:
                time.sleep(1)
            except KeyboardInterrupt:
                print("\n Stopped by user.")
                break


if __name__ == "__main__":
    poll_cpu_metrics()
