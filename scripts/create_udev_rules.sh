#!/bin/bash
sudo cp myarduino.rules  /etc/udev/rules.d
sudo service udev reload
sudo service udev restart
