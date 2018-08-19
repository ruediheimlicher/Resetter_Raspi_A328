# Resetter_Raspi
Watchdog mit ATtiny13 zur Überwachung des Raspberry Pi. Ein Cron-Script auf dem Raspi gibt in regelmässigen Abständen einen LO-Puls aus, der in der ISR von PCINT2 den Zähler resetcount zurücksetzt. Bleibt der Impuls aus, wird ein LO-Impuls gesetzt, der den Raspi neu startet
 Watchdog with ATtiny13 to surview a Raspberry Pi. 
 A cron-script on the Raspberry Pi gives an LO-Puls in a regulary interval to the PCINT2-input, resettig the counter 'resetcount' in the ISR(PCINT0_vect)-routine. If that puls is missing for a certain amount of time the Attiny gives a LO-Pulse to shutdown and restart the Raspberry Pi.
 
 http://www.mosaic-industries.com/embedded-systems/microcontroller-projects/raspberry-pi/on-off-power-controller
