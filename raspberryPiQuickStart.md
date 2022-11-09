# Raspberry Pi Quick Start

## RaspberryPi 4 Model B

<br>

[Simple Led Control](#simple-led-control)

[more](#picture2)


#### Quick Start

![Quick Start Guide](./pics/quickstart1.png)

<br>

### Pinout

![Raspberry Pi 4 Model B Pins Configuration](./pics/GPIO-Pinout-Diagram-2.png)

Seconde Diagram: 

![Raspberry Pi 4 Model B Pins Configuration](./pics/R-Pi-4-GPIO-Pinout.jpg)

----

<br>

## How to create and activate a virtual environment, what is a virtual environment and why you are asked to create one for most codes? 
[You will find the answer HERE](https://realpython.com/python-virtual-environments-a-primer/#activate-it)

<br>

## Code BOX

<br>

<details><summary>install RPi.GPIO</summary>
<p>

***NOTE:*** install this module in virtual environment.
```
pip install RPi.GPIO
```

<br>

</p>
</details>

<br>

```py
 # Simple LED blink...

import RPi.GPIO as GPIO  
import time

led_pin = 23

GPIO.setmode(GPIO.BCM)
GPIO.setup(led_pin, GPIO.OUT)

blink_repeat = int(input("blink repeats >> "))

while(blink_repeat>0)
    GPIO.output(led_pin, GPIO.HIGH)
    time.sleep(1)
    GPIO.output(led_pin, GPIO.LOW)
    time.sleep(1)

    blink_repeat -= 1
```

<br>

<details><summary> what is BCM in GPIO.setmode(GPIO.BCM) </summary>
<p>
There are two kinds of Input and Output pin numbering for the Raspberry pi. One is the BCM and the other is BOARD. Basically these pin numberings are useful for writing python script for the Raspberry Pi. 

<br>

**GPIO BOARD**– This type of pin numbering refers to the number of the pin in the plug, i.e, the numbers printed on the board, for example, P1. The advantage of this type of numbering is, it will not change even though the version of board changes.

**GPIO BCM**– The BCM option refers to the pin by “Broadcom SOC Channel. They signify the Broadcom SOC channel designation. The BCM channel changes as the version number changes.

**Broadcom SOC Channel**– BCM refers to the “Broadcom SOC channel” number, which is the numbering inside the chip which is used on the Raspberry Pi. These numbers changed between board versions as you can see in the previous tables for the 26-pin header type 1 versus 2, and or not sequential. 


***NOTE:***
The BCM numbers changed between versions of the Pi1 Model B, and you’ll need to work out which one you have guide here. So it may be safer to use the BOARD numbers if you are going to use more than one Raspberry Pi in a project.

<br>

In a nutshell, BCM pins maybe differ in raspberrypi's boards but Board pins are the same.

[Further details](https://iot4beginners.com/difference-between-bcm-and-board-pin-numbering-in-raspberry-pi/) 

<br>
</p>
</details>

<br>

## Connecting Raspberry pi to Arduino

You can simply send data via USB cable.
![RPi to Arduino connection](./pics/raspberrypi_arduino_uno_serial_usb.png)
<br>

Or use GPIO pins.. BUT it's recommended to use USB port.
![RPi to Arduino connection using GPIO pins](./pics/raspberrypi_arduino_serial_gpio.png)
***NOTE:*** Raspbery pi operating at 3.3v, so if it's connected to Arduino a logic level converter should be used.

**Detecting Arduino Board:**
```
ls /dev/tty*
```
***NOTE:*** when Arduino is connected */dev/ttyACM0* or */dev/ttyUSB0* may appear in the list. BUT keep in mind that the number maybe different.

**To find ACM in the list of ports:**
```py
from os import system

print("Connected devices include 'ACM' are: ")
system("ls /dev/tty* | grep ACM")
```


[Further information](https://roboticsbackend.com/raspberry-pi-arduino-serial-communication/)


### picture2

fdfdfdf
