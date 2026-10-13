#!/usr/bin/env python3
"""ST-LINK sanal COM port (LPUART1, 115200 8N1) logger.

- Gelen her satırı ekrana basar ve zaman damgasıyla logs/serial_<tarih>.txt dosyasına yazar.
- Terminale bir şey yazıp Enter'a basarsan log'a NOT olarak eklenir
  (ör. "vcc kablosunu oynatiyorum") -> olayları çıktıyla eşleştirmek kolaylaşır.
- Port başka programda açıksa (VS Code seri monitörü) bekler: iki program aynı
  portu okursa byte'lar ikisine bölünür, iki çıktı da bozulur.
- USB kopup gelirse kendiliğinden yeniden bağlanır. Çıkış: Ctrl+C.
"""
import datetime
import glob
import os
import subprocess
import sys
import threading
import time

import serial

BAUD = 115200
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logs")


def ts():
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    return ports[0] if ports else None


def other_users(port):
    """Portu (cu.* ya da tty.* adıyla) açmış başka process'lerin PID'leri."""
    tty = port.replace("/dev/cu.", "/dev/tty.")
    out = subprocess.run(["lsof", "-t", port, tty], capture_output=True, text=True).stdout
    return [p for p in out.split() if p != str(os.getpid())]


def main():
    os.makedirs(LOG_DIR, exist_ok=True)
    path = os.path.normpath(os.path.join(
        LOG_DIR, datetime.datetime.now().strftime("serial_%Y%m%d_%H%M%S.txt")))
    log = open(path, "a", buffering=1)          # satır satır diske yaz
    lock = threading.Lock()

    def write(line):
        s = f"{ts()} {line}"
        with lock:
            print(s, flush=True)
            log.write(s + "\n")

    def notes():
        for line in sys.stdin:
            if line.strip():
                write(f">>> NOT: {line.strip()}")

    print(f"Log dosyasi: {path}")
    threading.Thread(target=notes, daemon=True).start()

    last_status = None
    def status(msg):                            # aynı durumu tekrar tekrar basma
        nonlocal last_status
        if msg != last_status:
            write(msg)
            last_status = msg

    while True:
        port = find_port()
        if not port:
            status("[port yok: ST-LINK USB takili mi?]")
            time.sleep(1)
            continue
        others = other_users(port)
        if others:
            status(f"[{port} baska programda acik (pid {', '.join(others)}): "
                   f"VS Code seri monitorunu kapat]")
            time.sleep(1)
            continue
        try:
            with serial.Serial(port, BAUD, timeout=0.5) as ser:
                status(f"[baglandi: {port} @ {BAUD}]")
                buf = b""
                while True:
                    buf += ser.read(256)
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        write(line.rstrip(b"\r").decode("ascii", errors="replace"))
        except (serial.SerialException, OSError) as e:
            status(f"[baglanti koptu: {e}] tekrar deneniyor...")
            time.sleep(1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nCikis.")
