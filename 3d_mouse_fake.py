#!/usr/bin/env python

commands = ["1|0|0"
,"1.5|0|0"
,"1|0|0"
,".5|0|0"
,"0|0|0"
,"-.5|0|0"
,"-1|0|0"
,"-1.5|0|0"
,"-1|0|0"]

while True:
    for x in commands:
        raw_input()
        f = open("3d_mouse_input", 'w')
        f.write(x + "\n")
        f.close()
        print x
