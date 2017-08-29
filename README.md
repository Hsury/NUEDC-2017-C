# NUEDC-2017
**NUEDC** is short for **National Undergraduate Electronic Design Contest(全国大学生电子设计竞赛)**.

### Rule
NUEDC 2017, Problem C: [PDF document](http://tinyurl.com/ydgn47fy)</br>
You can also go to [the official FTP site](http://tinyurl.com/yavwnahk) for rules of other problems and information in the early years.

### Hardware
- Raspberry Pi 3B
- USB/CSI Camera with a wide-angle lens
- Quadcopter which can get commands using serial port

### Software
- Raspbian Jessie(2017-07-05)
- Python 2.7

### Dependency
- numpy
- python-opencv
- raspberry-gpio-python
- pyserial

Note that by default Raspberry Pi 3B will use its on-board bluetooth module as the primary serial device, you can follow [this tutorial](https://raspberrypi.stackexchange.com/a/54766) to fix this.
