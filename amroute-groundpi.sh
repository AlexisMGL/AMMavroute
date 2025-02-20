#!/bin/bash

## Install service
sudo cp amroute-groundpi.service /etc/systemd/system

## reload service
sudo systemctl daemon-reload
sudo systemctl enable amroute-groundpi.service
