#! joel -t JOEL -p 30 -s 8192

#Put shell start parameters
shell -d /dev/console -p 30 -s 4096

# Starting blue-led
xecho "blink:0 100 1000" >> /dev/leds
exit