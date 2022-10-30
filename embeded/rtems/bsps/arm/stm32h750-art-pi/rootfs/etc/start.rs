#! joel -t JOEL -p 110 -s 8192

#Put shell start parameters
shell -d /dev/console -p 110 -s 4096

# Starting blue-led
xecho "blink:0 100 1000" >> /dev/leds
exit