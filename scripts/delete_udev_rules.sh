#!/bin/bash
sudo rm   /etc/udev/rules.d/myarduino.rules
sudo service udev reload
sudo service udev restart
