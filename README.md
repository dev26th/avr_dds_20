# avr_dds_20
Improvements for the AVR DDS Signal Generator V2.0 Project http://www.scienceprog.com/avr-dds-signal-generator-v20/

Changes:
* Synchronization pulse of HS-output on start of sequence or for each period
* Sweep. Start, stop frequency and step is configurable.
* Additional 'options' menu
* Off-state is configurable
* PWM with configurable duty on the HS and analog outputs
* Modify frequency on-the-fly
* Persist all settings
* Maximal frequency increased to 250 kHz
* Frequency resolution is 0.1 Hz
* Speed-up the noise (~x50)
* Calibration of frequency
* One-pulse mode with configarable length on the HS and analog outputs
* Start trigger with configuable delay

Hardware modification, see [circuit](circuit.png) for details:
* the RESET button is disconnected from pin 9 and connected to pin 20
* 4 diodes (e.g. 1N4148) are added from pin 3 to buttons RESET, UP, LEFT and DOWN

