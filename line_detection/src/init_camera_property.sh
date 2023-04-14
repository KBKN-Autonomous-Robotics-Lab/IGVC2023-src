#!bin/bash
#v4l2-ctl -d /dev/sensors/insta360_air -v width=3008,height=1504
v4l2-ctl -d /dev/sensors/insta360_air -v width=1472,height=736
v4l2-ctl -d /dev/sensors/insta360_air -c white_balance_temperature_auto=0
v4l2-ctl -d /dev/sensors/insta360_air -c white_balance_temperature=4000
