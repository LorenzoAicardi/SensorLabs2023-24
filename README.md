# SensorLabs2023_24

Projects for the Sensor Systems course. The projects were made using the STM32 Cube IDE, and are designed to work on a NUCLEO-F401RE board.

Collaborators:
- Lorenzo Aicardi
- Sabrina Azzi
- Chiara Bordegari
- Riccardo Briccola
- Giovanni Codemo

# Project topics

- Homework 1a: Clap to turn on LED (LED: PA5, mic pin: PA8)
- Homework 1b: Have the LED blinking at 1 Hz frequency
- Homework 2a: Play a song (with HAL_delay()) (SPEAKER PIN: PA9, TIM1_CH2)
- Homework 2b: Play a song (without HAL_delay())
- Homework 3a: Use USART to transmit data to pc
- Homework 3b: Display names on LCD (LCD pins: PA4, PB1, PB2, PB12, 13, 14, 15 all GPIO_Output)
- Homework 4a: Acquire potentiometer voltage and send it via UART (POTENTIOMETER PIN: PA1)
- Homework 4b: Display potentiometer value on LCD
- Homework 5a: Acquire potentiometer, Vrefint, Temperature sensor every 1 second and transmit them via remote terminal using DMA
- Homework 5b: Acquire data from LDR (PIN: PA0)
- Homework 6a: I2C Temperature Sensor (I2C pins: PB8 I2C_clock, PB9 I2C_data)
- Homework 6b: I2C Accelerometer
- Homework 6c: I2C Accelerometer with autoincrement
- Homework 7a: SPI Led Matrix (PB6 + PA5 SPI_CLK, PA7 SPI_MOSI)
- Homework 7b: SPI Led Matrix, alternate letters (with timer interrupts)
- Homework 8: Encoder (PC6, PC7, TIM3-CH1 e TIM3-CH2, encoder mode in TIM3)
- Homework 9a: Keyboard matrix readout (scanning with hal delay) (PC2 PC3 PC12 PC13 input (column drivers) → shorting BJT to ground, PC8 PC9 PC10 PC11 output (row drivers) → I manually activate the transistor, shorting it to the column thus reading 0)
- Homework 9b: Keyboard matrix readout (with interrupts)
- Homework 10: IR Communication (IR LED: PB10 TIM2_CH3, IR receiver PA10 USART1_RX)
