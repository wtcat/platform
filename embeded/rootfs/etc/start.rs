#! joel -t JOEL -p 110 -s 8192

#Create ramdisk
mkdir /temp
mkrd -d /dev/ram -s 512 -n 2048
mount -t imfs /dev/ram /temp

#Starting media server
media -p 110 -s 4096

#Put shell start parameters
shell -d /dev/console -p 110 -s 8192
exit