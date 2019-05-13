#!/bin/bash
sudo apt-get install -y i2c-tools

# Needed for LEDs and Buttons
cd /tmp
wget https://lion.drogon.net/wiringpi-2.50-1.deb
sudo dpkg -i wiringpi-2.50-1.deb

# For I2C stuff
cd ~/mtbblackbox
sudo cp ~/mtbblackbox/setup/modules /etc/
sudo cp ~/mtbblackbox/setup/config.txt /boot/

# For run from reboot
sudo rm /var/spool/cron/crontabs/pi
sudo cp ~/mtbblackbox/setup/crontabScript /var/spool/cron/crontabs/pi
sudo chmod 600 /var/spool/cron/crontabs/pi

# For GPS
# check the booksmarks later and put in here
sudo ln -s /lib/systemd/system/gpsd.service /etc/systemd/system/multi-user.target.wants/

sudo reboot now
