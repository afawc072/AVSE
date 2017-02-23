#!/bin/bash
#This file is ran by the /etc/rc.d/local script to execute AVSE when the PI is booting up

#change directory to where we want to go... because of relative paths...

sleep 2

cd ~/git-reps/AVSE/

("bin/AVSE")
