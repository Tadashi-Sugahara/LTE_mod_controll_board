# Softbankに数回アタッチしに行く、全文失敗した場合、KDDIへ数回アタッチしにいく
import serial
import time

# シリアルポートの設定
SERIAL_PORT = 'COM4'  # 使用するポートに変更してください（例: '/dev/ttyUSB0'）
BAUD_RATE = 115200       # ボーレートを設定します

N = 0 #送信回数
M = 0 #受信成功回数

def send_commands_from_file(file_path):
    global M
    # シリアルポートをオープン
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        # テキストファイルを読み込み
        with open(file_path, 'r', encoding="cp932", errors="ignore") as file:
            for line in file:
                command = line.strip()  # 行の前後の空白を削除
                if line.startswith('#'):
                    continue
                
                if command:  # 空行をスキップ
                    # 1.ATコマンドを送信
                    full_command = command + '\r'  # ATコマンドは通常 '\r' で終了
                    print(f'Sending: {full_command.strip()}')
                    ser.write(full_command.encode('utf-8'))  # コマンドを送信
                    # 最初のレスポンス（エコー）を無視
                    first_response = ser.readline().decode('utf-8').strip()

                    # 2.レスポンスを受信
                    start_time = time.time()  # タイマーを開始
                    response_received = False  # レスポンス受信フラグ
                    time.sleep(0.5)
                    while True:
                        response = ser.readline().decode('utf-8').strip()
                        if "A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5" in response:
                            M = M + 1
                        if response in ["OK", "ERROR"]:
                            print(f'Response: {response}')
                            response_received = True
                            break  # "OK" または "ERROR" が受信されたらループを抜ける
                        elif response:  # 受信したが "OK" または "ERROR" ではない場合
                            print(f'Unexpected response: {response}')

                        # 2秒以上レスポンスがない場合、次のコマンドに移動
                        if time.time() - start_time > 10:
                            print("No response received within 10 seconds, moving to the next command.")
                            break
                    if not response_received:
                        print("Response not received.")



if __name__ == '__main__':
    file_path = './1_Start.txt'  # 読み込むテキストファイルのパス
    send_commands_from_file(file_path)
    while True:
        if N == 500:
            break
        N = N + 1
        print(f'------------Uart Commnication Start: {N}---------------')
        file_path = './2_UART.txt'  # 読み込むテキストファイルのパス
        send_commands_from_file(file_path)
        print(f'------------Uart Commnication OK Times: {M}---------------')
    file_path = './3_Done.txt'  # 読み込むテキストファイルのパス
    send_commands_from_file(file_path)    
    
    Result = M/N * 100
    print(f'!!!!!!!!![最終試験結果：------------{Result}%-------------]')