# Modbus_Master_PIC24
A PIC24 module working as a master in a network of RTU in a system of water distribution

Most information are in Mcc generated files: TIMER2 and UART2. And also in main.c

This project is managed by interruptions in the paripherals mentioned before and using state machines for logic: like sending and receiving data using MODBUS.

There is a total of 12 Slaves, not all equals. Particulary, the 2nd is used as a HMI for change from manual to automatic and viceversa. 
## Usage 

	*Clone the repository
	*Open a projec using MPLAB

## Organization

	**mcc_generated_files** folder has the peripheral files, where the routines interrupts have been written.
	**main.c**	has the main tank's logic. 
	Futher macros and declarations have been made in **uart2.h**
	
	
	
## Further notes

Please collaborate with useful code, we are happy to view your commets and take it as a valuable thing.

Keep coding, keep learning, keep growing up!

# License

MIT License

Copyright (c) [2020] [Adrian Vazquez]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
