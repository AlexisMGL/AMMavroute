#!/usr/bin/env python3
'''
Script to manage Wifi, RFD and Satellite telemetry from a flight controller
Written by Stephen Dade (stephen@rpanion.com)
'''

from pymavlink import mavutil
import yaml
import sys
import threading
import time
import serial
import netifaces as ni
from enum import Enum
import struct
# from pymavlink.generator.mavcrc import x25crc

exit_event = threading.Event()


def signal_handler(signum, frame):
    exit_event.set()


def set_sys_comp(conn, sysid, compid):
    conn.source_system = sysid
    conn.source_component = compid
    conn.mav.srcSystem = sysid
    conn.mav.srcComponent = compid


def makeUID(message):
    # Make hashable uniqure ID from MAVLink message
    return "{0}:{1}:{2}".format(message.get_seq(), message.get_msgId(), message.get_crc())


class CommsState(Enum):
    def __str__(self):
        if self.value == -1:
            return "Waiting for initial connection on Wifi or RFD"
        elif self.value == 0:
            return "Wifi Active"
        elif self.value == 1:
            return "RFD Active"
        elif self.value == 2:
            return "Satellite Active"
        elif self.value == 3:
            return "Rockblock SBD Active"
    WAITING_FOR_RFD_WIFI = -1
    ON_WIFI = 0
    ON_RFD = 1
    ON_SATCOM = 2
    ON_ROCKBLOCK = 3


if __name__ == '__main__':

    print("-----AMRoute GroundPi V1.7.0-----\nStarting...")

    settings = {}
    ip_wifi = None
    ip_eth = None

    conn_wifi = None
    conn_gcs = None
    conn_rfd = None
    conn_rockblock = None

    rx_packets_wifi = 0
    rx_packets_rfd = 0
    rx_packets_skylink = 0
    rx_packets_rockblock = 0

    ap_system = 0
    ap_component = 0

    m_wifi = None
    m_gcs = None
    m_skylink = None
    m_rfd = None
    m_rockblock = None

    rfd_sig = ""

    message_sent_time = {}

    msgbuf_for_last_2_seconds = {}

    time_since_last_report = time.time()

    time_since_last_wifi = 0
    time_since_last_rfd = 0
    time_since_last_satcom = 0

    heartbeat_time = time.time()

    connection_state = CommsState.WAITING_FOR_RFD_WIFI

    try:
        with open('settings-ground.yml', 'r') as file:
            settings = yaml.safe_load(file)
    except yaml.scanner.ScannerError:
        print("Error: Bad settings-ground.yml")
        sys.exit(1)

    # Get the IP address of the Wifi
    try:
        ip_wifi = ni.ifaddresses('wlo1')[ni.AF_INET][0]['addr']
        print("Wifi IP is {0}".format(ip_wifi))
    except (KeyError, ValueError):
        try:
            ip_wifi = ni.ifaddresses('wlan0')[ni.AF_INET][0]['addr']
            print("Wifi IP is {0}".format(ip_wifi))
        except (KeyError, ValueError):
            print("Error: Can't find Wifi")

    # Get the IP address of the Ethernet
    try:
        ip_eth = ni.ifaddresses('eno2')[ni.AF_INET][0]['addr']
        print("Ethernet IP is {0}".format(ip_eth))
    except (KeyError, ValueError):
        try:
            ip_eth = ni.ifaddresses('enx64c901e46a2a')[ni.AF_INET][0]['addr']
            print("Ethernet IP is {0}".format(ip_eth))
        except (KeyError, ValueError):
            try:
                ip_eth = ni.ifaddresses('eth0')[ni.AF_INET][0]['addr']
                print("Ethernet IP is {0}".format(ip_eth))
            except (KeyError, ValueError):
                print("Error: Can't find Ethernet")
    # Connect the GCS and don't wait for heartbeat
    conn_gcs = mavutil.mavlink_connection("udpin:{0}:{1}".format(ip_eth, settings['gcs_output_port']),
                                          autoreconnect=True,
                                          source_system=1, force_connected=False,
                                          source_component=mavutil.mavlink.MAV_COMP_ID_PERIPHERAL)

    # Connect the RFD, don't wait for heartbeat
    try:
        conn_rfd = mavutil.mavlink_connection(str(settings['rfd_port']), baud=settings['rfd_baud'], autoreconnect=True,
                                              source_system=1, force_connected=False,
                                              source_component=mavutil.mavlink.MAV_COMP_ID_PERIPHERAL)
    except serial.serialutil.SerialException:
        conn_rfd = None
        # print("Lost RFD")

    # Connect the Wifi (UDP Client), don't wait for heartbeat
    conn_wifi = mavutil.mavlink_connection("udpin:{0}:{1}".format(ip_wifi, settings['wifi_port']),
                                           autoreconnect=True,
                                           source_system=1, force_connected=False,
                                           source_component=mavutil.mavlink.MAV_COMP_ID_PERIPHERAL)

    # Connect the Skylink (UDP Client), don't wait for heartbeat
    conn_skylink = mavutil.mavlink_connection("udpin:{0}".format(settings['skylink_remote']), autoreconnect=True,
                                              source_system=1, force_connected=False,
                                              source_component=mavutil.mavlink.MAV_COMP_ID_PERIPHERAL)

    # Connect the Rockblock (UDP Client), don't wait for heartbeat
    conn_rockblock = mavutil.mavlink_connection("udpin:{0}".format(settings['rockblock_remote']), autoreconnect=True,
                                                source_system=1, force_connected=False,
                                                source_component=mavutil.mavlink.MAV_COMP_ID_PERIPHERAL)
    while True:
        # Process messages from all links
        try:
            if conn_gcs:
                m_gcs = conn_gcs.recv_msg()
            if conn_rfd:
                try:
                    m_rfd = conn_rfd.recv_msg()
                except serial.serialutil.SerialException:
                    m_rfd = None
                    # print("Lost RFD")
            if conn_wifi:
                m_wifi = conn_wifi.recv_msg()
            if conn_skylink:
                m_skylink = conn_skylink.recv_msg()
            if conn_rockblock:
                m_rockblock = conn_rockblock.recv_msg()
        except (BlockingIOError, KeyboardInterrupt):
            break

        if m_gcs is None and m_rfd is None and m_wifi is None and m_skylink and m_rockblock is None:
            time.sleep(0.01)

        # pass messages along. Note if we get duplicate messages on the RFD and Wifi coming in,
        # we only onsend 1 of those, to prevent duplicate messages being sent
        if m_rfd:
            if m_rfd.get_type() not in ["RADIO_STATUS", "BAD_DATA"]:
                # Don't forward radio status packet generated autmatically from the RFD
                if ap_system == 0:
                    print("Locked onto AP at {0}:{1}".format(m_rfd.get_srcSystem(), m_rfd.get_srcComponent()))
                    ap_system = m_rfd.get_srcSystem()
                    ap_component = m_rfd.get_srcComponent()
                    set_sys_comp(conn_gcs, ap_system, ap_component)
                if connection_state == CommsState.ON_RFD:
                    conn_gcs.write(m_rfd.get_msgbuf())
                time_since_last_rfd = time.time()
            elif m_rfd.get_type() == "RADIO_STATUS":
                # print(m_rfd)
                try:
                    rfd_sig = "RFD Signal: L:{0}/{1}, R:{2}/{3}".format(m_rfd.rssi, m_rfd.noise,
                                                                        m_rfd.remrssi, m_rfd.remnoise)
                    # if rfd_sig == "RFD Signal N/A":
                    #    conn_gcs.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, str("Regained RFD").encode())
                    #    rfd_sig = rfd_sig_new
                except IndexError:
                    conn_gcs.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, str("Lost RFD Signal").encode())
                    rfd_sig = "RFD Signal N/A"
        if m_wifi:
            # if m_wifi.get_type() == "HEARTBEAT":
            #    print("Got HB")
            if ap_system == 0:
                print("Locked onto AP at {0}:{1}".format(m_wifi.get_srcSystem(), m_wifi.get_srcComponent()))
                ap_system = m_wifi.get_srcSystem()
                ap_component = m_wifi.get_srcComponent()
                set_sys_comp(conn_gcs, ap_system, ap_component)
            if connection_state == CommsState.ON_WIFI:
                conn_gcs.write(m_wifi.get_msgbuf())
            time_since_last_wifi = time.time()
        if (m_skylink and (time.time() - time_since_last_rfd) > settings['hb_timeout'] and
           (time.time() - time_since_last_wifi) > settings['hb_timeout']):
            # Switch to SATCOM, if not already AND there's been no packets from RFD or Wifi for a bit
            if connection_state is not CommsState.ON_SATCOM:
                connection_state = CommsState.ON_SATCOM
                print("Going to Satellite")
                conn_gcs.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO,
                                             str("Changing to Satellite (2)").encode())
            if ap_system == 0:
                print("Locked onto AP at {0}:{1}".format(m_skylink.get_srcSystem(), m_skylink.get_srcComponent()))
                ap_system = m_skylink.get_srcSystem()
                ap_component = m_skylink.get_srcComponent()
                set_sys_comp(conn_gcs, ap_system, ap_component)
            conn_gcs.write(m_skylink.get_msgbuf())
            time_since_last_satcom = time.time()
        if m_rockblock:
            # Check if it not just a late packet from the aircraft, as the connection may
            # have been re-established whilst the rockblock packet was in transit
            if connection_state is not CommsState.ON_ROCKBLOCK:
                if (time.time() - time_since_last_rfd) > settings['rockblock_timeout'] and \
                   (time.time() - time_since_last_wifi) > settings['rockblock_timeout'] and \
                   (time.time() - time_since_last_satcom) > settings['rockblock_timeout']:
                    connection_state = CommsState.ON_ROCKBLOCK
                    print("Going to Rockblock")
                    conn_gcs.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, str("Going to Rockblock").encode())
                    conn_gcs.write(m_rockblock.get_msgbuf())
            else:
                conn_gcs.write(m_rockblock.get_msgbuf())

        # Check for inital connection on Wifi or RFD
        if time_since_last_wifi > 0 and connection_state == CommsState.WAITING_FOR_RFD_WIFI:
            connection_state = CommsState.ON_WIFI
            print("Initial connection on WIFI")
        elif time_since_last_rfd > 0 and connection_state == CommsState.WAITING_FOR_RFD_WIFI:
            connection_state = CommsState.ON_RFD
            print("Initial connection on RFD")

        # Switch RFD -> WIFI and back again. Bias to Wifi connection
        if ((time.time() - time_since_last_wifi) < settings['hb_timeout'] and
           connection_state == CommsState.ON_RFD):
            connection_state = CommsState.ON_WIFI
            print("Going to WIFI")
            conn_gcs.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, str("Changing to Wifi").encode())
        elif ((time.time() - time_since_last_wifi) > settings['hb_timeout'] and
              (time.time() - time_since_last_rfd) < settings['hb_timeout'] and
              connection_state == CommsState.ON_WIFI):
            connection_state = CommsState.ON_RFD
            print("Going to RFD")
            conn_gcs.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, str("Changing to RFD").encode())
        # Switch to SATCOM
        elif ((time.time() - time_since_last_rfd) > settings['hb_timeout'] and
              connection_state == CommsState.ON_RFD):
            connection_state = CommsState.ON_SATCOM
            print("Going to Satellite")
            conn_gcs.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, str("Changing to Satellite").encode())
        elif ((time.time() - time_since_last_wifi) > settings['hb_timeout'] and
              connection_state == CommsState.ON_WIFI):
            connection_state = CommsState.ON_SATCOM
            print("Going to Satellite")
            conn_gcs.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, str("Changing to Satellite").encode())
        # Switch back to RFD/Wifi from SATCOM/Rockblock
        elif (connection_state in [CommsState.ON_SATCOM, CommsState.ON_ROCKBLOCK] and
              (time.time() - time_since_last_rfd) < settings['hb_timeout']):
            connection_state = CommsState.ON_RFD
            print("Going to RFD")
            conn_gcs.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, str("Changing to RFD").encode())
        elif (connection_state in [CommsState.ON_SATCOM, CommsState.ON_ROCKBLOCK] and
              (time.time() - time_since_last_wifi) < settings['hb_timeout']):
            connection_state = CommsState.ON_WIFI
            print("Going to Wifi")
            conn_gcs.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, str("Changing to Wifi").encode())
        # Switch back to SATCOM from Rockblock
        elif (connection_state == CommsState.ON_ROCKBLOCK and
              (time.time() - time_since_last_satcom) < settings['rockblock_timeout']):
            connection_state = CommsState.ON_SATCOM
            print("Going to Satellite")
            conn_gcs.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, str("Changing to Satellite").encode())
        # Rockblock switching done on the aircraft side, so no need to put it here

        if m_gcs:
            if conn_rfd:
                try:
                    conn_rfd.write(m_gcs.get_msgbuf())
                except (struct.error, NotImplementedError):
                    pass
                except serial.serialutil.SerialException:
                    conn_rfd = None
                    # print("Lost RFD")
            if conn_wifi:
                try:
                    conn_wifi.write(m_gcs.get_msgbuf())
                except (struct.error, NotImplementedError):
                    pass
            try:
                if connection_state == CommsState.ON_SATCOM:
                    conn_skylink.write(m_gcs.get_msgbuf())
            except (struct.error, NotImplementedError):
                pass
            try:
                if connection_state == CommsState.ON_ROCKBLOCK:
                    conn_rockblock.write(m_gcs.get_msgbuf())
            except (struct.error, NotImplementedError):
                pass
            # print("Got {0} from FC".format(m_gcs.get_type()))

            # Try reconnecting any failed serial devices
            if conn_rfd is None:
                try:
                    conn_rfd = mavutil.mavlink_connection(str(settings['rfd_port']), baud=settings['rfd_baud'],
                                                          autoreconnect=True,
                                                          source_system=1, force_connected=False,
                                                          source_component=mavutil.mavlink.MAV_COMP_ID_PERIPHERAL)
                    print("Reconnected RFD")
                except serial.serialutil.SerialException:
                    conn_rfd = None
                    # print("Lost RFD")

        # send hb to airpi (on all non-Rockblock network links) once per hb_timeout
        # This ensures the links are tested for connectivity even if there's no GCS
        if time.time() - heartbeat_time > settings['hb_timeout']-1:
            heartbeat_time = time.time()
            if connection_state in [CommsState.WAITING_FOR_RFD_WIFI, CommsState.ON_RFD, CommsState.ON_WIFI]:
                if conn_rfd:
                    conn_rfd.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                                mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                                                0,
                                                0,
                                                0)
                if conn_wifi:
                    conn_wifi.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                                 mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                                                 0,
                                                 0,
                                                 0)
            elif connection_state == CommsState.ON_SATCOM:
                if conn_skylink:
                    conn_skylink.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                                    mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                                                    0,
                                                    0,
                                                    0)
            if conn_rockblock:
                conn_rockblock.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                                  mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                                                  0,
                                                  0,
                                                  0)
        # report data rate and system status once per 10 sec
        if time.time() - time_since_last_report > 10:
            if conn_gcs:
                try:
                    conn_gcs.mav.statustext_send(
                        mavutil.mavlink.MAV_SEVERITY_INFO, rfd_sig.encode())
                    time.sleep(0.001)
                    conn_gcs.mav.statustext_send(
                        mavutil.mavlink.MAV_SEVERITY_INFO, str(connection_state).encode())
                except (struct.error, NotImplementedError):
                    pass
                # and packet stats
                if conn_wifi:
                    delta_wifi = conn_wifi.mav_count - rx_packets_wifi
                else:
                    delta_wifi = 0
                if conn_rfd:
                    delta_rfd = conn_rfd.mav_count - rx_packets_rfd
                else:
                    delta_rfd = 0
                delta_skylink = conn_skylink.mav_count - rx_packets_skylink
                delta_rockblock = conn_rockblock.mav_count - rx_packets_rockblock
                stats_str = "GndRx last 10 sec: {0} Wifi, {1} RFD, {2} Sat, {3} Rck".format(delta_wifi,
                                                                                            delta_rfd,
                                                                                            delta_skylink,
                                                                                            delta_rockblock)
                if conn_wifi:
                    rx_packets_wifi = conn_wifi.mav_count
                else:
                    rx_packets_wifi = 0
                if conn_rfd:
                    rx_packets_rfd = conn_rfd.mav_count
                else:
                    rx_packets_rfd = 0
                rx_packets_skylink = conn_skylink.mav_count
                rx_packets_rockblock = conn_rockblock.mav_count
                try:
                    conn_gcs.mav.statustext_send(
                        mavutil.mavlink.MAV_SEVERITY_INFO, stats_str.encode())
                except (struct.error, NotImplementedError):
                    pass
            # reset RFD measurements
            rfd_sig = "RFD Signal N/A"
            time_since_last_report = time.time()

        if exit_event.is_set():
            break

    # Cleanup
    print("Exiting")
    if conn_gcs:
        conn_gcs.close()
    if conn_rfd:
        conn_rfd.close()
    if conn_wifi:
        conn_wifi.close()
    if conn_skylink:
        conn_skylink.close()
    if conn_rockblock:
        conn_rockblock.close()
