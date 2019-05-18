#!/bin/bash
sudo apt-get install -y i2c-tools

# Needed for LEDs and Buttons
cd /tmp
wget https://lion.drogon.net/wiringpi-2.50-1.deb
sudo dpkg -i wiringpi-2.50-1.deb

# Repo
git clone git@gitlab.com:planetarymotion/mtbblackbox.git

# For I2C stuff
cd ~/mtbblackbox
sudo cp ~/mtbblackbox/setup/modules /etc/
sudo cp ~/mtbblackbox/setup/config.txt /boot/

# For run from reboot
sudo cp /home/pi/mtbblackbox/bootup.sh /etc/init.d/mtbbb
sudo chmod +x /etc/init.d/mtbbb
sudo touch "/var/log/mtbbb.log" && sudo chown "pi" "/var/log/mtbbb.log"
sudo update-rc.d mtbbb defaults

# For GPS
# check the booksmarks later and put in here
sudo apt -y install gpsd gpsd-clients python-gps chrony libgps-dev
sudo systemctl stop gpsd.socket
sudo systemctl disable gpsd.socket
sudo ln -s /lib/systemd/system/gpsd.service /etc/systemd/system/multi-user.target.wants/



sudo reboot now
