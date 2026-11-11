# SARC MASTER: STM32F103CBT6

- MCU: STM32F103CBT6, 128 KB Flash / 20 KB RAM.
- Programare: ST-Link prin SWD; nu se folosește bootloader USB.
- USB Device Full-Speed nativ pe PA11 (USB_DM) și PA12 (USB_DP), CDC.
- HSE 8 MHz, SYSCLK 72 MHz și USB 48 MHz.
- Ultimii 5 KB Flash sunt rezervați: CAL la `0x0801EC00`–`0x0801F3FF`, BM la `0x0801F400`–`0x0801FBFF`, SID la `0x0801FC00`–`0x0801FFFF`.

## Build Arduino CLI

Este instalat pachetul **STMicroelectronics Boards 2.12.0**. Placa verificată este **Generic STM32F1 series / Generic F103CBTx**, cu **USB support: CDC (no generic 'Serial')**, **USB speed: Low/Full Speed**, **U(S)ART support: Disabled** și **Upload method: STM32CubeProgrammer (SWD)**.

Din PowerShell, rulează scriptul din acest director:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\compile-cbt6.ps1
```

Scriptul selectează identificatorul `STMicroelectronics:stm32:GenF1:pnum=GENERIC_F103CBTX,usb=CDC,xusb=FS,xserial=disabled,upload_method=swdMethod` și setează `upload.maximum_size=125952`. Platforma folosește această valoare pentru `LD_MAX_SIZE` în linkerul său standard, deci regiunea FLASH alocabilă firmware-ului se termină la `0x0801EC00`. Cele două pagini CAL, două pagini BM și pagina SID nu pot fi alocate firmware-ului. Limita RAM rămâne 20480 B.

Scriptul doar compilează și exportă BIN-ul în `build/cbt6-sid`; nu programează placa. Fișierul `.bin` se încarcă la adresa Flash `0x08000000` numai cu autorizarea separată de programare.

USB CDC folosește PA11/PA12. Programarea selectată este ST-Link prin SWD. TIM2 și TIM3 rămân configurate în firmware pentru encodere.
