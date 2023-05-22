make -j isoimage && /sbin/qemu-system-x86_64 -smp 2 -m 2048 -vga std -serial stdio -boot d -cdrom nautilus.iso \
-drive id=disk,file=disk.img,if=none    \
-device ahci,id=ahci                    \
-device ide-hd,drive=disk,bus=ahci.0