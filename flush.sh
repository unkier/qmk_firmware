arm-none-eabi-gdb -nx --batch -ex 'target extended-remote /dev/ttyACM0' -x black_magic_probe_flash.scr .build/redox_test_default.elf