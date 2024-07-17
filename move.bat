arm-none-eabi-objcopy -O binary "rtthread.elf"  "save.bin"
copy save.bin X:\Aqualarm_firmware\%1.bin
del save.bin