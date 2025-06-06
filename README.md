# OSC PIR trigger

## Default esp settings

    trigger ip: 10.0.0.100:8888
    target ip:  10.0.0.255:53000
    osc message: /go
    

## PIR module

Module reports high after a trigger for a duration of 1 second.  
Inbetween triggers there is a minimum 3 second delay.   

Looking at the module, facing buttons, dome up:  
- Right pot meter is sensitivity
- Left pot meter is trigger delay. (All the way to the left is minimum 3 sec)


