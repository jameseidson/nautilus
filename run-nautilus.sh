# AHCI DEVICE:
# qemu-system-x86_64 -drive id=disk,file=disk_image,if=none -device ich9-ahci,id=ahci -device ide-drive,drive=disk,bus=ahci.0
# FROM: https://wiki.archlinux.org/title/QEMU

make -j isoimage && /sbin/qemu-system-x86_64 -smp 2 -m 2048 -vga std -serial stdio -cdrom nautilus.iso -device ich9-ahci,id=ahci