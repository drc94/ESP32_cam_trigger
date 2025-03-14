#include <pgmspace.h>

////////////////////////////////

//61.3
const char config_txt[] PROGMEM = 
R"==x==(desklens  // camera name
13  // framesize  13=hd
10  // length of video in seconds
0  // interval - ms between recording frames 
1  // speedup - multiply framerate 
GMT  // timezone
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Above lines - 1 item per line followed by 2 spaces

desklens - camera name for files, mdns, etc
13 - framesize 13 is hd 720p 1280x720
 - for ov2640 camera 10=vga, 11=svga, 12=xga, 13=hd, 14=sxga, 15=uxga
 - for ov5640 camera add 16=fhd, 19=qxga, 20=qhd, 23=qsxga
1800 - length of video in seconds
0 - interval - millisecond between frames for recording
 - 0 is as fast as possible bound by camera speed or sd writing speed
 - 500 is 2 frames per second (subject to bounds)
 - 10000 is 0.1 frames per second or 10 seconds per frame
 - vga is max 25 fps, hd is max 12.5 fps, uxga at 12.5 fps etc
1 - speedup - multiply framerate to speed up playback
  - 1 is realtime
  - 24 will play a 1 fps recording at 24 fps on your computer
  - 300 will play 10 sec per frame at 30 fps on your computer
GMT - timezone for dates and times on your files
  - mountain: MST7MDT,M3.2.0/2:00:00,M11.1.0/2:00:00
  - eastern: EST5EDT,M3.2.0,M11.1.0
  - central europe: CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00
  - Ulaanbaatar, Mongolia: ULAT-8ULAST,M3.5.0/2,M9.5.0/2 
  - find timezone here: 
     https://sites.google.com/a/usapiens.com/opnode/time-zones

)==x==";
