# Keil Studio Cloud Project (Standalone)

Use this folder as the dedicated Keil Studio Cloud project.

## Entry Project

- `SongCloud.uvprojx` (STM32F401RE target)

## Source Model

- `main.cpp` is a single-file entry that aggregates all app modules from `../Core/Src/*`.
- HAL and CMSIS sources are still compiled from `../Drivers/*`.
- Local RTE/startup files are under `KSC-ARM/RTE`.

## Keil Studio Cloud Steps

1. Open `KSC-ARM/SongCloud.uvprojx`.
2. Run `Convert uVision Project to Csolution`.
3. Build the generated `SongCloud.csolution.yml`.
4. Flash/debug with ST-LINK.
