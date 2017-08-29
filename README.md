# NUEDC-2017
**NUEDC** is short for **National Undergraduate Electronic Design Contest(全国大学生电子设计竞赛)**.

### Rule
NUEDC 2017, Problem C: [PDF document](ftp://ftp.nuedc.com.cn/2017/exams/%CB%C4%D0%FD%D2%ED%D7%D4%D6%F7%B7%C9%D0%D0%C6%F7%CC%BD%B2%E2%B8%FA%D7%D9%CF%B5%CD%B3%A3%A8C%CC%E2%A3%A9.pdf)
You can also go to [the official FTP site](ftp://ftp.nuedc.com.cn/) for rules of other problems or information in early years.

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
