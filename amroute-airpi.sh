#!/bin/bash

## Install service
sudo cp amroute-airpi.service /etc/systemd/system

## reload service
sudo systemctl daemon-reload
sudo systemctl enable amroute-airpi.service
