ESP8266 Walkie Talkie
=====================

This is a somewhat failed attempt to make a ESP8266 based two-way radio.
It fails due to ESP8266 causing a lot of radio noise in the audio signals.

Files
-----

* **alaw.c:** Implementation of A-law audio compression algorithm.
* **audio_task.c:** Main logic for audio capture and playback and network connection
* **fast_adc.c:** Fast ADC implementation (up to ~20kHz samplerate)
* **sigma_delta.c:** Sigma-delta modulation for audio playback
* **user_main.c:** Startup logic, volume control and wifi passwords.

Building
--------

To build the firmware, you need https://github.com/SuperHouse/esp-open-rtos and https://github.com/pfalcon/esp-open-sdk/ installed.

Make sure that esp-open-sdk is in your PATH environment variable.

If needed, edit ESP_OPEN_RTOS_DIR in Makefile.

Then just type `make` to build or `make flash` to also program the ESP8266.

Connections
-----------

See http://essentialscrap.com/esptalkie/ for more details, but for basic testing you can connect:

* GPIO3 / RXD to speaker
* TOUT / ADC0 to output from microphone amplifier (0..1V voltage level)

Testing with PC
---------------

There is a script in **tools/listen.py** that can act as the other end of the connection, playing back sound using PC speakers.
