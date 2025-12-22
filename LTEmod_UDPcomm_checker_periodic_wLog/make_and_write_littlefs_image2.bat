REM 1) イメージ作成
C:\Users\torat\AppData\Local\Arduino15\packages\esp32\tools\mklittlefs\4.0.2-db0513a\mklittlefs.exe -c data -b 4096 -p 256 -s 0x160000 littlefs.bin

REM 2) 書き込み（C6）
C:\Users\torat\AppData\Local\Arduino15\packages\esp32\tools\esptool_py\5.1.0\esptool.exe --chip esp32c6 --port COM4 --baud 921600 write_flash 0x290000 littlefs.bin

pause