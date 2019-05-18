#!/bin/bash

### BEGIN INIT INFO
# Provides:          mtbbb
# Required-Start:    $remote_fs $syslog $time gpsd
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: mtbbb service
# Description:       Run mtbbb service
### END INIT INFO

# Carry out specific functions when asked to by the system
case "$1" in
  start)
    echo "Starting mtbbb..."
    sudo -u pi bash -c 'sudo /home/pi/mtbblackbox/mtbbb &> log.txt'
    ;;
  stop)
    echo "Stopping mtbbb..."
    sudo -u pi bash -c 'cd /path/to/scripts/ && ./stop-mtbbb.sh'
    sleep 2
    ;;
  *)
    echo "Usage: /etc/init.d/mtbbb {start|stop}"
    exit 1
    ;;
esac

exit 0
