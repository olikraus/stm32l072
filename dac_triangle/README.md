# DAC Triangle Wave Output with the STM32L072Kx

 * USB UART Converter is connected to pins 19 and 20 of the STM32L072
 * Use a USB to 3.3V TTL UART converter (e.g. with CP2102) for programming with the STM32FLASH tool
 * A switch at pin 31 (BOOT0) selects between in-system programming and user code mode
 * A button at pin 4 (RESET) will reset the STM32L072 and either start the ISP bootloader or the user code (depending on the state at pin 4)
 * Triangle wave is generated with DAC1 (pin 10, PA4)
 * Triangle wave output via KA8602 

![https://raw.githubusercontent.com/olikraus/stm32l072/main/blink/stm32l072k_schematic.png](https://raw.githubusercontent.com/olikraus/stm32l072/main/blink/stm32l072k_schematic.png)

![https://raw.githubusercontent.com/olikraus/stm32l072/main/blink/stm32l072k_ka8602.jpg](https://raw.githubusercontent.com/olikraus/stm32l072/main/blink/stm32l072k_ka8602.jpg)

