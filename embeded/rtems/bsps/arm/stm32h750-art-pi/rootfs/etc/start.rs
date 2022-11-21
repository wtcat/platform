#! joel -t JOEL -p 30 -s 8192

sleep 1

ln -s /home /media/mmcsd-0-0

#Put shell start parameters
shell -d /dev/console -p 30 -s 16384

# Starting blue-led
xecho "blink:0 100 1000" >> /dev/leds

exit