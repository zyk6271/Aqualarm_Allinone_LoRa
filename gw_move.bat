arm-none-eabi-objcopy -O binary "rtthread.elf"  "save.bin"
..\hex_output\srec_cat.exe ..\hex_output\Bootloader.bin -Binary -offset 0x8000000 -o boot.hex -Intel
..\hex_output\srec_cat.exe save.bin -Binary -offset 0x800C000 -o app.hex -Intel
copy /b ..\Debug\boot.hex + ..\Debug\app.hex X:\Aqualarm_firmware\%1.hex
del boot.hex
del app.hex
del save.bin
