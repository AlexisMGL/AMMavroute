
# AMMavroute

AMMavroute intelligently selects the best route for MAVLink telemetry between two Raspberry Pi nodes – AirPi (on board the UAV) and GroundPi (connected via Ethernet to the ground station). It works over four links, in the following priority:
- Wifi network
- RFD (radio modem)
- Skylink or Iridium GoExec satellite service
- Rockblock Modem

## Graphical Setup


## Installation

### Prerequisites

On both the AirPi and GroundPi, install the prerequisite packages:

```bash
sudo apt install python3-pip python3-yaml python3-serial python3-xml python3-netifaces python3-numpy python3-future
pip3 install pymavlink --user --break-system-packages
```

### AirPi

Run the following script to install and run the AirPi script:

```bash
./amroute-airpi.sh
```

### GroundPi

Run the following script to install and run the GroundPi script:

```bash
./amroute-groundpi.sh
```

## Link Selection

AMMavroute will select the links (if available) in the following priority:
1. Wifi
2. RFD
3. Skylink or Goexec Satellite (via a VPN or Public Static IP at the GroundPi)
4. Rockblock SBD modem (Iridium 9602/9603)

If a link receives no packets for 4 seconds, the next link in the above list will be activated. If a higher-ranked link is connected, AMMavroute will switch to that link instantly.

### Rockblock Modem

The Rockblock modem has a configurable 20-second delay time before it activates. To prevent accidental satellite usage, AMMavroute must be first connected via Wifi or RFD. After this initial connection, if both Wifi and RFD connections are lost, the Satellite link will be used.

## Running AMMavroute

After initial setup and configuration, AMMavroute will run automatically when the Raspberry Pi boots, using a systemd service file.

### Service Management

To check the service status:

```bash
sudo systemctl status ammavroute-air.service   # On the AirPi
sudo systemctl status ammavroute-ground.service  # On the GroundPi
```

To stop the service:

```bash
sudo systemctl stop ammavroute-air.service   # On the AirPi
sudo systemctl stop ammavroute-ground.service  # On the GroundPi
```

To start the service:

```bash
sudo systemctl start ammavroute-air.service   # On the AirPi
sudo systemctl start ammavroute-ground.service  # On the GroundPi
```

### Running Manually

To run AMMavroute manually for debugging, first stop the service, then run:

```bash
cd ./AMMavroute && ./amroute-airpi.py   # On the AirPi
cd ./AMMavroute && ./amroute-groundpi.py  # On the GroundPi
```

## Connecting with Mission Planner

If using Mission Planner, use the “UDPCI” connection type. The IP is the Ethernet IP of the GroundPi, port 15000. The GroundPi will send status updates every 10 seconds, viewable on the “Messages” tab in Mission Planner.

## Changing Settings

To change the network addresses or serial ports that AMMavroute uses, edit the following files:

- `~/AMMavroute/settings-airpi.yaml` on the AirPi
- `~/AMMavroute/settings-groundpi.yaml` on the GroundPi

After editing, restart the service as described in the "Running AMMavroute" section.

## Other Notes

### RFD900

If the RFD signal strength is shown as “N/A”, ensure the RFD900s are within range, powered on, running the latest firmware, and set to "MAVLink" in Mission Planner.

### Satellite Packet Loss

Due to packet filtering, Mission Planner will show a poor packet loss rate on the Satellite connection. This is expected.

### Rockblock Installation

The Rockblock modem is connected directly to the Cube Orange TELEM port. A Lua script within ArduPilot will control the Rockblock. Ensure the following ArduPilot parameters are set:

- `SCR_ENABLE = 1` (requires reboot)
- `SERIALn_PROTOCOL = 28` (where `n` is the TELEM port number)
- `RCK_FORCEHL = 2`
- `RCK_TIMEOUT = 15`

Using the Rockblock requires the “rockblock2mav” software to be installed and running on the GroundPi. See [rockblock2mav](https://github.com/stephendade/rockblock2mav) for details.

## Version History

- **V1.9 (28/08/2025)** - Add independant direct TCP connexion GCS/airpi
- **V1.8 (01/03/2025)** - Add multi drones support with independant connexions
- **V1.7 (17/02/2025)** - Add Rockblock support
- **V1.6 (12/06/2024)** - Add video streamer MAVLink connection
- **V1.5.1 (11/05/2024)** - Bug fixes for Iridium GoExec
- **V1.5 (08/04/2024)** - Added support for Iridium GoExec modem: activation and status reporting
- **V1.4 (03/11/2023)** - Removed Ping, added extra debugging text
- **V1.3 (26/03/2023)** - Fixed Skylink datarate calculation
- **V1.2 (14/03/2023)** - Added Skylink signal strength reporting
- **V1.1 (03/03/2023)** - Show local and remote RFD RSSI messages, improved telemetry mode string
- **V1.0** - Initial release

