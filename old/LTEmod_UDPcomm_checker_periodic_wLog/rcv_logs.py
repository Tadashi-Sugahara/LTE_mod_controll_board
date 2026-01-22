
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32-C6 LittleFS /logs からログファイルを一覧取得・ダウンロードする受信スクリプト
- プロトコル:
  PC -> ESP32 : "#LIST"
  ESP32 -> PC : "LIST:<path>:<size>" ... (複数行)
  PC -> ESP32 : "#GET <path>"
  ESP32 -> PC : "FILEBEGIN <path> <size>" + <size bytes> + "FILEEND"
  ESP32 -> PC : "ERR:<message>" (必要時)

使い方:
  python recv_logs.py --port COM5 --baud 115200 --out ./downloaded_logs
  1) 一覧取得
  2) 任意のファイルを選択してダウンロード or まとめてダウンロード(--all)
"""

import argparse
import os
import sys
import time
from typing import List, Tuple, Optional

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("pyserial が必要です。 pip install pyserial を実行してください。")
    sys.exit(1)

CRLF = b"\r\n"
READ_LINE_TIMEOUT_S = 5.0
READ_FILE_TIMEOUT_S = 60.0

def auto_detect_port() -> Optional[str]:
    ports = list_ports.comports()
    for p in ports:
        # ESP32のUSB-CDCっぽいものを優先（ベンダIDなどで改善可能）
        if "CP210" in p.description or "USB" in p.device or "ttyUSB" in p.device or "ttyACM" in p.device:
            return p.device
    return ports[0].device if ports else None

def open_serial(port: str, baud: int) -> serial.Serial:
    ser = serial.Serial(port=port, baudrate=baud, timeout=0.2)
    # 受信バッファクリア
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.3)
    return ser

def read_line(ser: serial.Serial, timeout_s: float = READ_LINE_TIMEOUT_S) -> Optional[str]:
    """CRLFで終端されるテキスト行を読み取る（タイムアウトあり）"""
    deadline = time.time() + timeout_s
    buf = bytearray()
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        buf += b
        if buf.endswith(CRLF):
            try:
                return buf[:-2].decode("utf-8", errors="replace")
            except Exception:
                return buf[:-2].decode(errors="replace")
    return None

def send_cmd(ser: serial.Serial, cmd: str):
    data = (cmd + "\r\n").encode("utf-8")
    ser.write(data)
    ser.flush()

def request_list(ser: serial.Serial) -> List[Tuple[str, int]]:
    """#LIST を送り、LIST:<path>:<size> の列を受け取る"""
    send_cmd(ser, "#LIST")
    files = []
    # 1秒間無応答なら終了、以降は連続でLIST行が来る間受信
    start = time.time()
    while True:
        ln = read_line(ser, timeout_s=2.0)
        if ln is None:
            # 初回までに受信が無い場合 1秒程度は待ってみる
            if time.time() - start < 1.0:
                continue
            else:
                break
        if ln.startswith("LIST:"):
            # LIST:/logs/xxx.txt:1234
            try:
                # 形式: LIST:<path>:<size>
                _, rest = ln.split("LIST:", 1)
                path, size_s = rest.rsplit(":", 1)
                size = int(size_s)
                files.append((path.strip(), size))
            except Exception:
                print(f"[WARN] 不明なLIST行: {ln}")
        elif ln.startswith("ERR:"):
            print(f"[ESP32 ERROR] {ln}")
            break
        else:
            # それ以外の行が来たら一覧終了とみなす
            break
    return files

def request_get(ser: serial.Serial, path: str, out_dir: str) -> bool:
    """#GET <path> を送り、FILEBEGIN <path> <size> → バイナリ受信 → FILEEND を処理"""
    send_cmd(ser, f"#GET {path}")
    hdr = read_line(ser, timeout_s=READ_LINE_TIMEOUT_S)
    if hdr is None:
        print(f"[ERROR] ヘッダ未受信: {path}")
        return False
    if hdr.startswith("ERR:"):
        print(f"[ESP32 ERROR] {hdr}")
        return False
    if not hdr.startswith("FILEBEGIN "):
        print(f"[ERROR] 予期せぬヘッダ: {hdr}")
        return False

    # 例: FILEBEGIN /logs/atlog_20251218_112345.txt 1234
    try:
        _, recv_path, size_s = hdr.split(" ", 2)
        size = int(size_s.strip())
    except Exception:
        print(f"[ERROR] ヘッダ解析失敗: {hdr}")
        return False

    # 受信
    os.makedirs(out_dir, exist_ok=True)
    fname = os.path.basename(recv_path.strip())
    out_path = os.path.join(out_dir, fname)
    print(f"[INFO] {recv_path} ({size} bytes) → {out_path}")

    remaining = size
    deadline = time.time() + READ_FILE_TIMEOUT_S
    with open(out_path, "wb") as wf:
        while remaining > 0 and time.time() < deadline:
            # できるだけ大きめに読む（残量に合わせる）
            chunk = ser.read(min(remaining, 4096))
            if not chunk:
                continue
            wf.write(chunk)
            remaining -= len(chunk)

        wf.flush()

    if remaining != 0:
        print(f"[ERROR] 受信サイズ不足（残り {remaining} bytes）")
        return False

    # フッタ確認
    footer = read_line(ser, timeout_s=5.0)
    if footer is None or not footer.startswith("FILEEND"):
        print(f"[ERROR] FILEEND 未受信 or 異常: {footer}")
        return False

    print("[OK] 受信完了")
    return True

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=str, default=None, help="シリアルポート名（未指定なら自動検出）")
    ap.add_argument("--baud", type=int, default=115200, help="ボーレート（ESP32側UART設定に合わせる）")
    ap.add_argument("--out",  type=str, default="./downloaded_logs", help="保存先ディレクトリ")
    ap.add_argument("--all",  action="store_true", help="一覧の全ファイルをまとめて取得")
    args = ap.parse_args()

    port = args.port or auto_detect_port()
    if not port:
        print("シリアルポートが見つかりません。--port で指定してください。")
        return

    print(f"[INFO] Open {port} @ {args.baud}")
    try:
        ser = open_serial(port, args.baud)
    except Exception as e:
        print(f"[ERROR] シリアルオープン失敗: {e}")
        return

    # 一覧
    files = request_list(ser)
    if not files:
        print("[INFO] /logs にファイルが見つかりませんでした。")
        return

    print("\n=== /logs ファイル一覧 ===")
    for i, (p, s) in enumerate(files):
        print(f" {i}: {p} ({s} bytes)")
    print("==========================\n")

    targets: List[Tuple[str,int]] = []
    if args.all:
        targets = files
    else:
        try:
            sel = input("ダウンロードする番号をスペース区切りで指定（例: 0 2 3）> ").strip()
            idxs = [int(x) for x in sel.split()] if sel else []
            targets = [files[i] for i in idxs if 0 <= i < len(files)]
        except Exception:
            print("[WARN] 入力解析に失敗したため中止します。")
            return

    if not targets:
        print("[INFO] 対象がありません。終了します。")
        return

    ok = 0
    for path, size in targets:
        if request_get(ser, path, args.out):
            ok += 1
        else:
            print(f"[FAIL] {path} の取得に失敗")

    print(f"\n[SUMMARY] 成功 {ok} / {len(targets)} 件")
    ser.close()

if __name__ == "__main__":
    main()
