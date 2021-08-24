# CAN Bus HEX data send and receive from/to the terminal via UART

# Motivation:
Normally we type some command in the terminal like putty or tera term. The command is interpreted as string not as a hex value. Offcourse there are some terminal setting options in some terminal like real term to set the command as hex value lets say aa bb cc 44 55 66 77 88. But here we want make our program in such a way that we dont need to use the help of terminal settings. We will give the hex value as a command and enable the microcontroller to intrepret the string  as a hex value string not as a normal string. This type of application is very useful when we have no choice of terminal settings in our real life industrial projects.

# Works so far done:
1. Microcontroller  receives the string in the interrupt mode via UART. This string contains the CAN_STD_ID and 8 bytes of HEX data.
2. after receiving the string convert the string at hex value string. 
3. Send the CAN_STD_ID and CAN HEX value data.
4. Receive the CAN_STD_ID and Hex value from the CAN receiver side and send the data towards the microcontroller
5. Then microcontroller sends the data(CAN_STD_ID and 8 bytes of Hex value) via UART to display on the terminal.

# Recuirements 
1. STM32 Nucleo board or Discovery board
2. STM32 Cube IDE
3. IXAAT USB to CAN Compact or something similar that works
4. CAN analyzer software

# Installations
1. Install the stm32 cube IDE and configure the project according to the microcontroller board requirements
2. Install CAN analyzer.
