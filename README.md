# GPS-Derived-Frequency-Standard-2024

This page archives the firmware of an improved and modernized version of the "Simplified GPS-Derived Frequency Standard", which the author originally designed back in 2006, and that was documented in a [QEX Magazine article](https://ve2zaz.net/GPS_Std/Downloads/VE2ZAZ_GPS_Derived_Std_QEX_09_10_2006.pdf). Although this modernized version brings in several improvements, the principle of operation on this system is the same as on the original design. The operation can be characterized as running a Frequency-Locked Loop (FLL): 

- A 10 MHz adjustable "ovenized" crystal oscillator (OCXO) continuously increments a hardware 16-bit counter.
- A one Pulse-Per-Second signal derived from GPS captures values from the counter.
- The difference between captured counter values is read by the firmware every 10 seconds. Knowing that the ideal count difference for a 10.0000 MHz signal is 57600, an offset is computed.
- At the end of the sampling cycle, the firmware adjusts the OCXO frequency via a Digital-to-Analog-Converter (DAC) to compensate for the average of the frequency offsets, thus targeting the 10.0000 MHz ideal frequency.
- This cycle repeats...

The modernized version of GPS-Derived Frequency Standard offers the following improvements over the previous version:

- A 32-bit micro-controller, the STM32 Black Pill (STM32F401C) or Blue Pill (STM32F103C) platform (previously an 8-bit PIC micro-controller),
- A true 12/14/16-bit Digital-To-Analog-Converter (DAC) to generate the OCXO tuning voltage (previously a PWM output and some low-pass filtering),
- A real clock distribution chip with 50 Ohm outputs, to fan out the 10 MHz references (previously a CMOS gate chip),
- A faster 10MHz signal receiver/shaper/buffer chip. This provides a sharper and more accurate 10 MHz reference into the FLL.
- A 20-character, 4-line LCD display that provides the essential system status information (previously non-existent),
- A buffered 1-PPS output available for external usage (previously non-existent)
- Separate digital and analog +5V rails, for a lower noise performance (previously a single +5V rail),
- A firmware developed in C language on the Arduino IDE environment, offering easy code change/recompile by the user (previously coded in unintuitive PIC assembly language),
- A Proportional-Integral (PI) FLL loop, yielding a much finer frequency control and faster convergence (previously a constant step FLL loop),
- A three-stage FLL acquisition process: short, medium and long sampling cycles (previously a two-stage coarse-fine process). This provides a much faster convergence towards the ideal 10 MHz frequency, and a faster recovery from short or small GPS disturbances.
- A PCB integrating the GPS, the DAC and even a prototyping area which can, depending on the model used, hold the OCXO,
- A native USB 2.0 serial port on STM32 Black Pill (the STM32 Blue Pill requires an external UART-to-USB converter),
- A comprehensive VT-100 text console via the USB port for easy configuration and control of the system parameters, and for complete status monitoring (replaces the very basic text console and the Windows software).

For more information on this project (harware, configuration, operation, performance, etc.), please refer to [VE2ZAZ's web page](https://ve2zaz.net/GPS_Std_New/GPS_Std.htm)
