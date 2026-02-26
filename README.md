# Keil Studio Cloud Project (Standalone)

This folder is fully self-contained for Keil Studio Cloud.

## Entry Project

- `SongCloud.uvprojx` (STM32F401RE target)

## Source Model

- All application logic is in a single file: `main.cpp`.
- No source files are referenced from `../Core` or `../Drivers`.
- Local runtime/config files are under `RTE/`.

## Required Platform Packs

- `Keil.STM32F4xx_DFP`
- `ARM::CMSIS`

These are resolved by Keil Studio Cloud pack management, not from repository files.

## Keil Studio Cloud Steps

1. Open `SongCloud.uvprojx`.
2. Run `Convert uVision Project to Csolution`.
3. Build the generated `SongCloud.csolution.yml`.
4. Flash/debug with ST-LINK.
