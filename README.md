# Embedded Smart Pill Dispenser

Small embedded C project for an automated pill dispenser used as a course project.

Contents
- `pill_disp.c` — main source file in this repo

Quick start

1. Build (host machine, for quick syntax check or testing):

   gcc -Wall -Wextra -std=c11 -o pill_disp pill_disp.c

2. For an embedded target, use the appropriate cross-toolchain (for example
   arm-none-eabi-gcc) and your board's flashing tools. Example (pseudo):

   arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O2 -ffunction-sections -fdata-sections \
     -T your_linker_script.ld -o pill_disp.elf pill_disp.c

   # then convert/flash with objcopy / openocd / vendor tools

Repository status
- CI: basic GitHub Actions syntax check for `pill_disp.c` (see `.github/workflows/ci.yml`)

Contributing
- See `CONTRIBUTING.md` for a short guide.

License
- This project is released under the MIT License. See `LICENSE`.
