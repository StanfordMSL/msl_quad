=======================
Mission Control Station
=======================

<for all things involving ground based computers (relay,mocap,personal computer)>

The current main companion computer platform on mslquads is the Jetson TX2. Previous
generations used the Odroid-XU4. Also documented are the steps for an UP Board.

Odroid
~~~~~~~

TX2
~~~~

UP Board
~~~~~~~~~
0. Install Ubuntu 18.04.4 LTS
1. sudo apt update
2. sudo apt full-upgrade -y
3. sudo reboot
4. Proceed with the steps in https://wiki.up-community.org/Ubuntu#Ubuntu_18.04_installation_and_configuration.
   But look out for kernel errors. The last bit of the autoremove might fail where it refuses to remove the
   running kernel. If it does do:

    i. sudo updatedb
    ii. locate -e $(uname -r)
    iii. locate -be $(uname -r) -0 | xargs -0 sudo rm -r
    iv. if (iii.) fails, do: locate -be $(uname -r) -0 | xargs -d '\n' -0 sudo rm -r. Whatever it is, you can double check if the running kernel is fully removed by running the autoremove and see whether it throws the error.

5. Run the remainder of the instructions in 4)
6. Enable HAT functionality: https://wiki.up-community.org/Ubuntu#Enable_the_HAT_functionality_from_userspace_2
7. Install ROS, git, code and whatever else you need. Some useful commands/links:

    * sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras ros-melodic-vrpn-client-ros
    * sudo wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
    * ./install_geographiclib_datasets.sh
    * Terminator (see comments in https://askubuntu.com/questions/1006448/how-to-install-the-latest-version-of-terminator)
    * Visual Studio Code (https://code.visualstudio.com/download)
    * QGroundControl (https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html)
    * sudo apt install git
    * sudo apt install openssh-server
    * sudo apt install screen
    * pip install --upgrade pip
    * pip install scipy-stack

8. For python interaction with the HAT, install the adafruit libraries using the instructions in https://forum.up-community.org/discussion/3569/solved-getting-an-adafruit-servo-hat-to-work-on-an-up-board#latest
9. Connect telem2 to the rx/tx/gnd on the UpBoard. DO NOT CONNECT THE 5V pin.
10. Setup px4 side: https://dev.px4.io/master/en/companion_computer/pixhawk_companion.html
11. Setup upboard side: stty -F /dev/ttyS4 921600 cs8 -cstopb -parenb


