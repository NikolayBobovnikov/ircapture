set auto-load safe-path / 
target remote localhost:3331
monitor reset halt
load
monitor reset halt
break main
continue
