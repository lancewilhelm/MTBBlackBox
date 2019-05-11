#!/bin/bash
sudo apt-get install -y i2c-tools

cd /tmp
wget https://lion.drogon.net/wiringpi-2.50-1.deb
sudo dpkg -i wiringpi-2.50-1.deb

cd ~/mtbblackbox
sudo cp ~/mtbblackbox/setup/modules /etc/
sudo cp ~/mtbblackbox/setup/config.txt /boot/

sudo rm /var/spool/cron/crontabs/pi
sudo cp ~/mtbblackbox/setup/crontabScript /var/spool/cron/crontabs/pi
sudo chmod 600 /var/spool/cron/crontabs/pi

sudo reboot now
