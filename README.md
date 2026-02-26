# STM32F401 Intelligent Navigation Car

Complete firmware implementation based on the requirement document in the project root (`*.docx`).

## Implemented Features

- Core motion control (L298N): forward, backward, stop, left/right turn, 180 turn.
- OPB704 path mark detection (ADC + filtering + debounce):
  - mark edge triggers buzzer feedback
  - 7-seg count up/down real-time display
- 3-way obstacle detection (2Y0A21 front/left/right, ADC threshold).
- Full 5-scene navigation logic with front-priority rule.
- LED linkage:
  - obstacle LED follows obstacle status
  - mark LED follows OPB704 status
- Buzzer linkage:
  - mark detection beep
  - obstacle edge beep
  - completion signal pattern
- 7-segment common-cathode driver (0..9).
- HC-05 Bluetooth telemetry over USART2.
- 2x16 LCD1602 4-bit parallel mode:
  - line1: scene + motion state
  - line2: counter + obstacle flags
- PWM speed control for motors:
  - ENA: TIM3_CH2 (PC7)
  - ENB: TIM2_CH2 (PB3)

## Directory Layout

- `Core/Inc/app_config.h`: feature switches, thresholds, timing constants, speed setpoints.
- `Core/Inc/pin_map.h`: pin mapping and conflict-free LCD profile switch.
- `Core/Inc/*.h`: module interfaces.
- `Core/Src/main.c`: HAL init + peripheral init + scheduler loop.
- `Core/Src/navigation.c`: scene state machine and count behavior.
- `Core/Src/sensors.c`: ADC sampling/filtering/debounce logic.
- `Core/Src/motor.c`: H-bridge control and PWM speed output.
- `Core/Src/lcd1602.c`: LCD1602 4-bit driver.
- `Core/Src/bluetooth.c`: HC-05 report output.
- `Core/Src/seven_seg.c`, `Core/Src/buzzer.c`, `Core/Src/indicators.c`: peripheral drivers.
- `KSC-ARM/`: standalone Keil Studio Cloud project folder (`SongCloud.uvprojx`, STM32F401RE target).

## Pin Mapping Notes

- Requirement has a hard conflict for optional peripherals:
  - LCD D7 uses `PA8` but motor IN2 also uses `PA8`
  - LCD D4/D5 use `PA3/PA2` but USART2 also uses `PA3/PA2`

- To allow all features at once, default config uses:
  - `LCD_USE_CONFLICT_FREE_PINS = 1`
  - LCD remap:
    - D4 -> `PB6`
    - D5 -> `PB7`
    - D6 -> `PA10`
    - D7 -> `PB8`
  - RS/E stay as required (`PC10`/`PC12`).

- If you need strict document LCD pins, set `LCD_USE_CONFLICT_FREE_PINS = 0` and rewire carefully.

- Another requirement conflict exists:
  - OPB704 analog output uses `PA0`
  - red LED is also listed on `PA0`
  - default keeps OPB704 on `PA0` and moves red LED to `PC13`.

## Main Configuration

In `Core/Inc/app_config.h`:

- `ENABLE_BLUETOOTH` (default `1`)
- `ENABLE_LCD` (default `1`)
- `ENABLE_MOTOR_PWM` (default `1`)
- `LCD_USE_CONFLICT_FREE_PINS` (default `1`)
- motion timing and ADC thresholds
- PWM speed setpoints (`MOTOR_SPEED_*_PERCENT`)

## Keil Integration

1. Open `MDK-ARM/Blinky.uvprojx` (or convert it in Keil Studio Cloud).
2. The project now uses a single entry file: `MDK-ARM/main.cpp`.
   This file aggregates all app modules from `Core/Src/*` into one translation unit.
3. Ensure HAL modules are enabled:
   - GPIO, RCC, ADC, UART, TIM, PWR
4. Build and flash.
5. Calibrate in `Core/Inc/app_config.h`:
   - `MARK_ADC_THRESHOLD`
   - `OBSTACLE_ADC_THRESHOLD_25CM`
   - `TURN_90_MS`, `TURN_180_MS`, `REVERSE_LONG_MS`, `BACKOFF_SHORT_MS`
   - `MOTOR_SPEED_FORWARD_PERCENT`, `MOTOR_SPEED_REVERSE_PERCENT`, `MOTOR_SPEED_TURN_PERCENT`
