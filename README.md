PLEASE VIEW IT IN CODE IF YOU WANT, AS IT LOOK HORRIBLE IN PREVIEW

This is where I put my own personal project that I just find interesting
For now, the project is a SparkFun GNSS board ubloc NEO-F10N connected to an esp32-s3
START OUT:
New to geo stuff; let's build up a bit. Today I tried messing with Python and Folium a bit, plotted some points, and made a helper that calculates the distance between them so I can see how much I've walked in a day and prove I am not that lazy.
|
UPDATE:
So had to remember where I have been and then get lat/lon for it; it is kinda annoying and also suffers from massive accuracy issues (probably underestimates the distance, aka it said that I am still lazy, which is untrue), so I did some research about GNSS boards and how it works and am probably gonna get one.
|
UPDATE:
After researching, I got a nice GNSS board, the ubloc MAX M10N and an ESP32 WROOM dev kit (familiarity from ZEUS PCB)
|
UPDATE:
The SparkFun board for that chip ran out, so I gotta get the NEO-F10N board; it's somewhat better, and for the ESP dev kit, I decided to upgrade to the ESP32-S3 dev board
|
UPDATE:
Got those boards delivered; I also got a power supply and breadboard. Had to cut the board UART trace to avoid an ESP32 UART conflict, and now it's wired up.
|
UPDATE:
Research about parsing, how C treats strings, NMEA messages, and checksums completed; now starting to code.
|
UPDATE:
After so long, I finally finished coding the parsing and am ready to test it out.
|
UPDATE:
Finish soldering pins for the board to connect to a breadboard both hardware and firmware are ready to test, lets go.
|
UPDATE:
I realize i need to get an active attenna for this (SMA) and also due to i severed the uart trace now i cant connect to ubloc center so gotta research about their config ubx frame, whyyyyyyyy.
|
UPDATE:
Had to implement a dynamic baud rate to accommodate the UBX frame (which is a nightmare, as I had to dig into some weird binary frame that looks like gibberish) and start up; kinda regret not plugging the board in first before cutting the trace.
|
UPDATE:
The code is now finished, but the signal seems to be very bad on testing, as the pps should be blinking but it just solid color for a long time, i dont know why; I regret cutting the trace even more now idk what wrong is it the ubx frame, or the board or it just bad signal.
|
UPDATE:
I figured it out; it just had a cold start. I gotta leave it outside for like idk 20 min, which is longggg, but it finally worked. But I can't receive any messages on the ESP monitor.
|
UPDATE:
So apparently the USB buffer to report data back got weird. I flushed and used the ESP log. There's a slight delay, but the coordinates are now showing on the monitor.
|
UPDATE:
Walking around with this is annoying, so I designed a 3d printed enclosure for it.
|
UPDATE:
Learned about Git today; I created a repo and linked the project and all my files to it, and pasted my old README into it too. Probably start including future goals/next milestones from now on.
|
UPDATE:
So the delay log problem is actually not buffer issue, it the mismatch bauld rate, so i did some research and the gnss board will remember it bauld rate as it have an onboard flash, so now i implemented a smarter way, still use dynamic bauld rate but now send an ack ubx and switching between the default and target bauld until the ack valid, that solved the issue so retry and never assume.
|
UPDATE:
Real-world testing at uni, and it's kinda weird; the data kept jumping around; it's horrible. I implemented a message that reports GNSS no fix rather than discard it- mystery.
|
UPDATE:
After more research, I see they actually have a dynamic platform, and it's supposed to use a virtual-assumption-based extended Kalman filter to get rid of pesky noise, so I implemented that too. I'll look into filtering or averaging techniques later; they seem to improve the signal.
|
UPDATE:
Designed a new enclosure for it, much more compact, and also rewired the board for better integrity.
|
NEWEST UPDATE:
With a random math vid I stumbled on YouTube, I now implemented an EMA filter for the data, just a tad better but still an improvement (might look into Kalman later). I now config the gnns board to 10Hz, added the timestamp to the log, and a code toggle for raw NMEA log in case I need it in the future (maybe I log NMEA to a micro SD card and parse later on laptop with Python)
FUTURE GOAL: - get a micro SD card module and the card itself, then write new code to log the parsed message to it in CSV form.
             - Implement a web host type to view it on a phone, aka display data wirelessly (maybe)
             - Implement an imu (maybe)
             - Fuse the two sensors (maybe)
