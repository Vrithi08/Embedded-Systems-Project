This repository contains an embedded C firmware example for a small automated pill dispenser built on an STM32 microcontroller. The implementation is intentionally compact and uses register-level access (no HAL) so students can learn how peripherals (I2C, timers, GPIO) are configured.

Table of contents
- Overview
- Hardware & wiring (detailed)
- Firmware architecture and code walkthrough
- Key constants and how to change behavior
- Build, debug and flash examples
- Troubleshooting and FAQ
- Safety and testing
- Extending the project
- Contributing & license

Overview
--------
The firmware reads time from a DS3231 RTC over I2C and uses a 16x2 I2C LCD to show the current time and short messages. A servo motor (driven by TIM1 CH1 on PA8) rotates to dispense tablets into a slot; a buzzer (PA5) provides audible notifications. The demo uses a short test schedule so you can run the dispenser logic on a bench quickly.

Hardware & wiring (detailed)
---------------------------
Required components
- STM32F4-series MCU or compatible board (Nucleo or custom board). The source uses register-level names from `stm32f4xx.h`.
- DS3231 RTC module (I2C)
- I2C character LCD with PCF8574 backpack (address 0x27 typical)
- Small hobby servo (standard 3-wire)
- Active buzzer or buzzer + transistor driver

Pin mapping used in `pill_disp.c`
- I2C1: PB6 = SCL (AF4), PB7 = SDA (AF4)
- LCD I2C address: 0x27
- RTC I2C address: 0x68
- Buzzer: PA5 (simple GPIO output)
- Servo PWM: PA8 (TIM1_CH1)

Wiring notes
- Power: Use a shared ground between MCU, RTC, LCD, servo power supply and buzzer. If the servo requires 5V, supply 5V to the servo VCC but keep grounds common.
- I2C: connect SDA to PB7 and SCL to PB6. If you use a module with pull-ups built-in, do not add extra pull-ups. If not present, add 4.7k pull-ups to 3.3V.
- LCD VCC: many backpacks accept 5V but can run at 3.3V. Verify your module. The MCU I2C pins are 3.3V.
- Buzzer: for louder beeps or if the buzzer draws >20 mA, drive it through a small NPN transistor with a base resistor and a flyback diode if inductive.

Firmware architecture and code walkthrough
----------------------------------------
Files
- `pill_disp.c` — single-file firmware with all logic and peripheral setup.

Initialization sequence (what happens at reset)
1. I2C1_Init() — enables GPIOB and I2C1 clocks, configures PB6/PB7 for AF4 I2C use and sets up timing registers (CR2, CCR, TRISE). This code assumes a 16 MHz peripheral clock for the I2C timing values used.
2. LCD_Init() — sends the 4-bit initialization sequence (HD44780) through the I2C backpack (PCF8574). The functions `lcd_send_nibbles`, `LCD_Cmd`, `LCD_Data` implement the nibble-level protocol.
3. Buzzer_Init() — configures PA5 as a push-pull output.
4. Servo_Init() — enables GPIOA and TIM1, configures PA8 for AF (TIM1_CH1), sets TIM1 prescaler and ARR to get a 20 ms period (50 Hz), sets CCR1 to 1500 (1.5 ms pulse center), and enables PWM output.
5. RTC_SetFromBuildTime() — convenience for development: sets the DS3231 time registers based on the build time macro so you don't need to set the RTC manually every debug session.

Main loop behavior
- The main loop reads the time (seconds, minutes, hours) every iteration using `RTC_ReadTime()` which calls `ds3231_read_regs()` to read registers via I2C and converts BCD to decimal.
- When a new second is detected, the LCD first prints the formatted time (e.g., TIME 12:34:56).
- Dispense flow (fast demo values):
   - MSG_INTERVAL = 15: when seconds % 15 == 0 the firmware shows "TAKE TABLET", turns the buzzer on and moves the servo by 60° from its current pill_count position. The code increments pill_count.
   - MSG_DURATION = 2: after 2 seconds the buzzer is turned off and the message cleared.
   - After three pills (pill_count >= 3) the servo is returned to home (0°). The firmware waits REFILL_DELAY seconds (3) then displays "REFILL TABLETS" and beeps 3 times (Buzzer_Beep(3,100,100)). After REFILL_DURATION seconds the refill message is cleared and pill_count resets to 0.

Key functions & low-level notes
- I2C helpers: `i2c_write_buffer()` sends I2C transmissions with the register reads/writes using SR1/SR2 flags and start/stop conditions. `ds3231_read_regs()` performs the standard write-register-then-read sequence used by many I2C RTCs.
- LCD: `lcd_send_nibbles()` writes two nibbles (high then low) and toggles the enable line via the backpack outputs. The code assumes the PCF8574 pins are wired in the backpack as usual (data pins on upper nibble).
- Servo: `Servo_SetAngle(angle)` maps 0..180 degrees to 1000..2000 timer compare units. The timer's PSC/ARR values were chosen assuming an APB2 timer clock that ends up at 1 MHz after PSC. Adjust PSC and CCR calculations if your board runs different clock speeds.

Key constants and how to change behavior
--------------------------------------
- I2C addresses: `LCD_7BIT` (0x27) and `RTC_7BIT` (0x68). Change these if your modules use different addresses.
- Pins: `BUZZER_PIN` (PA5) and `SERVO_PIN` (PA8). Update code if wiring to other pins.
- Timers and frequencies: `Servo_Init()` uses TIM1 PSC=15 and ARR=19999 to aim for a 1 MHz timer and 20 ms period. If your SystemCoreClock is different, recalculate PSC and ARR.
- Scheduling constants at top of `main()`:
   - `MSG_INTERVAL` — how often the dispense message triggers (seconds). Default 15 for demo; for real use set to 3600*X or use scheduled times.
   - `MSG_DURATION` — how long the TAKE TABLET message lasts (seconds).
   - `REFILL_DELAY` & `REFILL_DURATION` — timing for refill messaging.

Build, debug and flash examples
------------------------------
Host checks (syntax-only)
```powershell
gcc -fsyntax-only -Wall -Wextra pill_disp.c
```

Cross-compile example (adjust CPU/linker script/includes to match your board):
```bash
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O2 -ffunction-sections -fdata-sections \
   -I/path/to/stm32f4xx_headers -T your_linker_script.ld -Wl,--gc-sections -o pill_disp.elf pill_disp.c
arm-none-eabi-objcopy -O binary pill_disp.elf pill_disp.bin
```

Flash examples (choose the tool matching your hardware):
- Using OpenOCD + gdb (example; target config depends on your board):
   1. Start OpenOCD for your board.
   2. Connect with arm-none-eabi-gdb pill_disp.elf and run `monitor reset init` and `load`.
- Using ST-Link CLI (example):
   - st-flash write pill_disp.bin 0x8000000

Troubleshooting & FAQ
---------------------
- I2C devices not detected
   - Verify pull-ups on SDA/SCL (4.7k typical to 3.3V).
   - Confirm addresses: run an I2C scanner on your MCU or check with a logic analyzer.
   - Ensure the LCD backpack runs at the same voltage domain as the MCU I2C pins.
- LCD shows garbage or not initializing
   - Check that the PCF8574 pins mapping matches the `lcd_send_nibbles()` assumptions (some backpacks wire bits differently).
   - Increase startup delays in `LCD_Init()` if the module is slow to power up.
- Servo doesn't move or moves erratically
   - Ensure servo VCC is powered (often 5V). Use a separate servo power supply if necessary, but share ground with MCU.
   - Verify TIM1 is clocked and PSC/ARR values match your MCU clock.
- Buzzer silent
   - Check PA5 wiring and whether your buzzer is active-high or active-low. If it needs more current, use a transistor driver.

Safety and testing
------------------
- Keep body parts away from moving parts while testing.
- Use current-limited power supplies and fuse/protection when integrating motors.

Extending the project
---------------------
- Add buttons to acknowledge dispense events and pause reminders.
- Replace the demo timing with a schedule parser (store times in EEPROM/Flash).
- Add Wi-Fi/Bluetooth to sync schedules with a phone app.

Contributing & license
----------------------
- See `CONTRIBUTING.md` to submit patches and `LICENSE` for terms (MIT).



