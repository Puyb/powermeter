This is the complete code for a crank-based power meter, transmitting over BLE
to a bike computer. The whole project is copied from Steve Jarvis website/gitlab
and tested on 'Adafruit Bluefruit nRF52840 Feather Express' board 

IMPLEMENTED:
- Reading of load and gyro sensors (by  Steve Jarvis)
- Connecting and updating load and gyro info to bluetooth device (by Steve Jarvis)
- UART over bluetooth to enable connection via the Adafruit Bluefruit LE connect App (iOS/Android)
- Sleep + wakeup by gyro motion
- Upgrade to latest Adafruit nRF52 version (supporting BLE multi)
- Integrated calibration via Adafruit Bluefruit LE connect App (UART)

TODO: 
- It seemed my version only works when compiling for 'DEBUG'
- Persistant storage and retrieval of (calibration) settings via NVRAM

Thijs


Original text from Steve Jarvis:

Schematics and a part list for the hardware, as well as 
notes on calibrating, are on [the wiki](https://gitlab.com/sjarvis/powermeter/-/wikis/home).
In all, it's a full instruction set for building your own cycling power meter,
compatible with any bike computer that supports Bluetooth (i.e. any Wahoo
or Garmin).

This is a fun project and I trained with it just great for a year. It can handle the 
bumps and (minor) hits from the road, but I did struggle to get it any kind of 
water resistant. I'd say that was the biggest pain point. It's also not a super 
attractive form factor, but function > form, right?!

As far as accuracy, it's within a reasonable margin of error when compared with
commercial units, and usually on the low side. As far as training goes, it's for 
certain valuable, because consistency is most important. As far as ego goes, I 
guess it's not the friendliest. I tested these times and durations with about 
these results:

1. Wahoo Kickr, the programmed 20 min FTP test. This meter read 9% lower.
2. Computrainer, 40 min at ~190 watts. This meter read 6% higher.
3. Favero Assioma Duo, 40 min at ~190 watts. This meter read 6% lower.

Now you data heads out there might point out that the average power is not all that
matters, and you're right. I also tracked the variance in readings throughout 
rides, and this meter is "spikier" than commercial meters. I believe commercial meters
apply extra smoothing to the data (this code does very little, and only done in
software versus hardware). But maybe more importantly, commercial meters do an
exceptional job accounting for the position in the rotation when the reading is taken. 
For example, if the meter takes a reading during the heat of the downstroke in 
one revolution, and in the next takes a reading at the apex, that's going to result 
in very spiky power over time, when in reality it's a consistent effort. This project 
handles this issue in software, by calculating the time for a single revolution, 
and timing the readings such that they occur at as nearly the same positions in 
the pedal stroke each revolution. This change caused a huge improvement in short 
term (<5 second) consistency on power reporting. The hardware includes a gyroscope, 
which could be used to dial this in better still.

The license is GPL, because perhaps my most important take away is this stuff 
isn't rocket science and there's no reason these meters cost as much as the rest 
of the bike.
