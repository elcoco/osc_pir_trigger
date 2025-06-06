# OSC PIR trigger


## Hardware

[Waveshare ESP32-S3-ETH](https://www.waveshare.com/wiki/ESP32-S3-ETH#Other_resource_link)  
[Schematics](https://files.waveshare.com/wiki/ESP32-S3-ETH/ESP32-S3-ETH-Schematic.pdf)

<img src="https://www.waveshare.com/w/upload/e/e0/ESP32-S3-ETH-details-15.jpg" width=400/>

## Default esp settings

    trigger ip: 10.0.0.100:8888
    target ip:  10.0.0.255:53000
    osc message: /go
    accepts integers (timeout) on: /trigger/timeout
    

## PIR module

Module reports high after a trigger for a duration of 1 second.  
Inbetween triggers there is a minimum 3 second delay.   

Looking at the module, facing buttons, dome up:  
- Right pot meter is sensitivity
- Left pot meter is trigger delay. (All the way to the left is minimum 3 sec)

## Debugging

https://github.com/72nd/osc-utility

    # Use 'osc-utility' to listen for OSC messages
    osc-utility server --port 53000 --host 0.0.0.0
