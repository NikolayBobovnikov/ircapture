openocd.exe -f ../openocd.cfg -s D:\Software\OpenOCD-0.9.0-Win32\share\openocd\scripts -c "init" -c "reset halt" -c "flash write_image erase build/vivetracker" -c "reset run" -c "shutdown"
pause

