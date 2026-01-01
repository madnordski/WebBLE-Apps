# WebBLE Apps as Front Ends for ESP32 Projects

## What's here.

- TrackVoltageCar is a simple version of ServoCar that measures voltage
on Lionel convensional control layouts.  If learning WebBLE, ServoTest
ServoCar will serve you best. TrackVoltageCar is for toy train
enthusiasts that have trains powered by conventional A/C transformers.

- ServoTest and ServoCar illustrate methods for controlling ESP32
connected servos with your phone using WebBLE.

- WebBLE_RNT_Example is the Random Nerd Tutorials example.

*Please note that this repository is different because it also hosts
these files over https.  In order to run these html scripts visit:*

   [front ends](https://madnordski.github.io/WebBLE-Apps/)
   
   `https://madnordski.github.io/WebBLE-Apps/`

**Running WebBLE scripts requires an https connection (or perhaps
localhost).  Here we use github to host these files in order to get an
https connection.**

---
![TrackVoltageCarAppSM](https://github.com/user-attachments/assets/3203d527-3934-4eaf-88c5-384bb3b61623)
![TrackVoltageCar_programmingSM](https://github.com/user-attachments/assets/ec1f691e-f645-49ab-ad0b-138ff6311ced)

---
For ServoTest and ServoCar be sure to read the comments in these
files. There are more than a couple of hard-won lessons in
there. ServoTest.html is meant to be used with ServoTest.ino and
ServoCar.html is meant to be used with ServoCar.ino running on an
ESP32. We have only ever run these scripts using the XIAO_ESP32-C3 and
XIAO_ESP32-S3.

Finally, be aware that WebBLE_RNT_Example is as was published by RNT
in the fall of 2024 and there is a bug in the file on their website
which is included here (reported in December).

The line:

   `device.addEventListener('gattservicedisconnected', onDisconnected);`

should be:

   `device.addEventListener('gattserverdisconnected', onDisconnected);`
