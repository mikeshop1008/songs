/*
 * Single-file application entry for Keil Studio Cloud conversion flow.
 * The actual module implementations are pulled in as one translation unit.
 */
extern "C" {
#include "../Core/Src/main.c"
#include "../Core/Src/motor.c"
#include "../Core/Src/sensors.c"
#include "../Core/Src/seven_seg.c"
#include "../Core/Src/buzzer.c"
#include "../Core/Src/indicators.c"
#include "../Core/Src/bluetooth.c"
#include "../Core/Src/navigation.c"
#include "../Core/Src/lcd1602.c"
#include "../Core/Src/stm32f4xx_it.c"
#include "../Core/Src/stm32f4xx_hal_msp.c"
}
