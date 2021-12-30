Do It Yourself (DIY) Cycle Power Meter

This is the code for a crank-based power meter, transmitting over BLE to a bike computer. The whole project is based on Steve Jarvis website/gitlab with a couple of improvements (see gitlab link below) and tested on a 'Adafruit Bluefruit nRF52840 Feather Express' board.

Please find the complete DIY work-instruction here:

https://gitlab.com/tbressers/power/-/wikis/home

Best regards,
Thijs

---
The original text below from Steve Jarvis (probably) still holds, although the read-out
currently is interrupt-driven which should give slightly better results. Furthermore,
I've added the reading of the crank Z-tilt (via the monitor function), which is the
basis of a crank Z-tilt based read-out. Original text:

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
