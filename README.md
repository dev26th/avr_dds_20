# avr_dds_20
Improvements for the AVR DDS Signal Generator V2.0 Project http://www.scienceprog.com/avr-dds-signal-generator-v20/

Changes:
* Sweep. Start frequency can be selected, step is minimal one, stop frequency is about 69 kHz.
* Additional 'options' menu
* Off-state is configurable
* PWM with configurable duty on the HS-output
* Modify frequency on-the-fly
* Persist all settings
* Maximal frequency increased to 100 kHz
* Speed-up the noise (x50)

Hardware modification, see [circuit](circuit.png) for details:
* the RESET button is disconnected from pin 9 (IC1) 
* 4 diodes (e.g. 1N4148) are added from pin 3 (IC1) to buttons RESET, UP, LEFT and DOWN

