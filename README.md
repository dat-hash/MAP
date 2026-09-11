This is where I put my own personal project that I just find interesting in
For now, the project is a sparkfun gnss board ubloc NEO-F10N connect to an esp32-s3
START OUT:
new to geo stuff let build up abit, today i tried to mess with python and folium abit plot some points and make a helper that can calculate distance from those points to know how much i have walked a day to show that i am not that lazy.
|
UPDATE:
So had to remember where i have been and then get lat/lon for it is kinda annoying and also sacrifice from massive accuracy issue (propably underestimate the distance aka it said that i am still lazy which is untrue), so i did some research about gnss board with how it work and probably gonna get one.
|
UPDATE:
After researching I got a nice gnss board the ubloc MAX M10N and an esp32 wroom dev kit (familiarity from ZEUS PCB)
|
UPDATE:
The sparkfun board of that chip ran out so gotta get the NEO-F10N board somewhat better and esp dev kit i decide to upgrade to the esp32-s3 dev board
|
UPDATE:
Got those boards delivered, also i got a power supply and breadboard too, had to cut the board UART trace to avoid esp32 uart conflict and now wired it up.
|
UPDATE:
Research about parsing, how C treat string, NMEA message and checksum completed, now start to code.
|
UPDATE:
After so long i am finally finish coding for the parsing and ready to test it out.
|
UPDATE:
Finish soldering pins for the board to connect to a breadboard both hardware and firmware are ready to test, lets go.
|
UPDATE:
I realize i need to get an active attenna for this (SMA) and also due to i severed the uart trace now i cant connect to ubloc center so gotta research about their config ubx frame, whyyyyyyyy.
|
UPDATE:
Had to implement a dynamic bauld rate to accomodate the ubx frame (which is a nightmare as i had to dip into some weird binary frame that look like gibberish) and start up, kinda regret not plug the board in first before cutting the trace.
|
UPDATE:
The code is now finish, but the signal seem to be very bad on testing as the pps should be blinking but it just solid color for a long time, i dont know why regret cutting the trace even more now idk what wrong is it the ubx frame, or the board or it just bad signal.
|
UPDATE:
I figured it out, it just have cold start i just gotta left it outside for like idk 20 min which is longggg but it finally work but i cant received any message on the esp monitor.
|
UPDATE:
So apparently the USB buffer to report data back got weird i try to flush and also use esp log now and sure it take a bit delay but the coordinate are now shown on the monitor.
|
UPDATE:
Walking around with this is annoying so i designed a 3d printed enclosure for it.
|
NEWEST UPDATE:
Learned about GIT today, i created a repo and linked the project and all my file to it, paste my old readme into this too. Probably start to include future goal/next milestone from now on.
  FUTURE GOAL: - get an micro sd card module and also the card itself then write new code for it to log the parsed message into it in CSV form.
