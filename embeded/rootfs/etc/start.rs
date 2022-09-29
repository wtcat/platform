#! joel -t JOEL -p 110 -s 8192

#Create ramdisk
mkdir /temp
mkrd -d /dev/ram -s 512 -n 2048
mount -t imfs /dev/ram /temp
