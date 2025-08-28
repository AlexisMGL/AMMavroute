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
from enum import Enum
import requests
import struct
import subprocess
import json

exit_event = threading.Event()


def signal_handler(signum, frame):
    exit_event.set()


def set_sys_comp(conn, sysid, compid):
    conn.source_system = sysid
    conn.source_component = compid
    conn.mav.srcSystem = sysid
    conn.mav.srcComponent = compid


def makeUID(message):
    # Make hashable unique ID from MAVLink message
    return "{0}:{1}:{2}".format(message.get_seq(), message.get_msgId(), message.get_crc())


def isFromGroundPi(message):
    # GroundPi sends hearbeats as a MAV_COMP_ID_PERIPHERAL
    if message.get_type() != "HEARTBEAT":
        return False
    if message.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_PERIPHERAL:
        return False
    return True


def ping(host):
    """
    Returns True if host (str) responds to a ping request.
    Remember that a host may not respond to a ping (ICMP) request even if the host name is valid.
    """

    # 1 ping, 0.2 sec timeout. Don't care about ping return - just want to keep link alive
    command = ['ping', '-c', '1', '-i', '0.2', host]

    return subprocess.call(command) == 0


class CommsState(Enum):
    def __str__(self):
        if self.value == -1:
            return "WAITING_FOR_RFD_WIFI"
        elif self.value == 0:
            return "ON_RFD_WIFI"
        else:
            return "ON_SATCOM"
    WAITING_FOR_RFD_WIFI = -1
    ON_RFD_WIFI = 0
    ON_SATCOM = 1


if __name__ == '__main__':

    print("-----AMRoute AirPi V1.9.0-----\nStarting...")

    settings = {}
    ip_wifi = None
    ap_system = 0
    ap_component = 0
    gcs_system = 0
    gcs_component = 0
    gcs_system_client = 0
    gcs_component_client = 0

    conn_wifi = None
    conn_wifi_client = None
    conn_fc = None
    conn_rfd = None
    conn_skylink = None
    conn_streamer = None

    rx_packets_wifi = 0
    rx_packets_rfd = 0
    rx_packets_skylink = 0

    m_wifi = None
    m_fc = None
    m_skylink = None
    m_rfd = None
    m_streamer = None

    message_sent_time = {}

    msgbuf_for_last_2_seconds = {}
    msgbuf_for_last_2_seconds_client = {}

    data_rate = 0
    time_since_last_report = time.time()

    time_since_last_rfd_wifi = 0

    connection_state = CommsState.WAITING_FOR_RFD_WIFI

    try:
        with open('settings-air.yml', 'r') as file:
            settings = yaml.safe_load(file)
    except yaml.scanner.ScannerError:
        print("Error: Bad settings-air.yml")
        sys.exit(1)

    # Connect the flight controller and wait for heartbeat
    try:
        conn_fc = mavutil.mavlink_connection(str(settings['fc_port']), baud=settings['fc_baud'], autoreconnect=True,
                                             source_system=1, force_connected=False,
                                             source_component=mavutil.mavlink.MAV_COMP_ID_PERIPHERAL)
        conn_streamer = mavutil.mavlink_connection("udpout:{0}".format(settings['streamer_ipport']), autoreconnect=True,
                                                   source_system=1, force_connected=False,
                                                   source_component=mavutil.mavlink.MAV_COMP_ID_PERIPHERAL)
        # wait for the heartbeat msg to find the system ID
        while True:
            mh = conn_fc.wait_heartbeat(timeout=0.5)
            if mh is not None and conn_fc.probably_vehicle_heartbeat(mh):
                # Got a heartbeat from remote MAVLink device, good to continue
                print("Got Heartbeat from ArduPilot (system {0} component {1})".format(mh.get_srcSystem(),
                                                                                       mh.get_srcComponent()))
                ap_system = mh.get_srcSystem()
                ap_component = mh.get_srcComponent()
                break
            if exit_event.is_set():
                print("Exiting")
                conn_fc.close()
                sys.exit(0)
    except serial.serialutil.SerialException:
        conn_fc = None
        print("Lost FC")

    # Connect the RFD, don't wait for heartbeat
    try:
        conn_rfd = mavutil.mavlink_connection(str(settings['rfd_port']), baud=settings['rfd_baud'], autoreconnect=True,
                                              source_system=ap_system, force_connected=False,
                                              source_component=ap_component)
    except serial.serialutil.SerialException:
        conn_rfd = None
        print("Lost RFD")

    # Connect the Wifi (UDP Server), don't wait for heartbeat
    conn_wifi = mavutil.mavlink_connection("udpout:{0}".format(settings['wifi_remote']),
                                           autoreconnect=True,
                                           source_system=ap_system, force_connected=False,
                                           source_component=ap_component)
    conn_wifi_client = mavutil.mavlink_connection("tcpin:{0}".format(settings['wifi_remote_client']),
                                           autoreconnect=True,
                                           source_system=ap_system, force_connected=False,
                                           source_component=ap_component)
    # Connect the Skylink (UDP Server), don't wait for heartbeat
    conn_skylink = mavutil.mavlink_connection("udpout:{0}".format(settings['skylink_remote']), autoreconnect=True,
                                              source_system=ap_system, force_connected=False,
                                              source_component=ap_component)

    while True:
        # Process messages from all links
        try:
            if conn_fc:
                try:
                    m_fc = conn_fc.recv_msg()
                except serial.serialutil.SerialException:
                    m_fc = None
                    print("Lost FC")
            if conn_streamer:
                m_streamer = conn_streamer.recv_match(type=['COMMAND_ACK', 'STATUSTEXT'], blocking=False)
            if conn_rfd:
                try:
                    m_rfd = conn_rfd.recv_msg()
                except serial.serialutil.SerialException:
                    m_rfd = None
                    # print("Lost RFD")
            if conn_wifi:
                m_wifi = conn_wifi.recv_msg()
            if conn_wifi_client:
                m_wifi_client = conn_wifi_client.recv_msg()
            if conn_skylink:
                m_skylink = conn_skylink.recv_msg()
        except (BlockingIOError, KeyboardInterrupt):
            break

        if m_fc is None and m_rfd is None and m_wifi and m_wifi_client is None and m_skylink is None and m_streamer is None:
            time.sleep(0.01)

        # remove old de-duped messages
        for buf in list(msgbuf_for_last_2_seconds):
            if time.time() - msgbuf_for_last_2_seconds[buf] > 2:
                del msgbuf_for_last_2_seconds[buf]
        for buf_client in list(msgbuf_for_last_2_seconds_client):
            if time.time() - msgbuf_for_last_2_seconds_client[buf_client] > 2:
                del msgbuf_for_last_2_seconds_client[buf_client]

        # pass messages along, filtering any heartbeats from groundpi
        try:
            if m_rfd:
                if gcs_system == 0 and m_rfd.get_type() not in ["RADIO_STATUS", "BAD_DATA"]:
                    print("Locked onto RGCS at {0}:{1}".format(m_rfd.get_srcSystem(), m_rfd.get_srcComponent()))
                    gcs_system = m_rfd.get_srcSystem()
                    gcs_component = m_rfd.get_srcComponent()
                    set_sys_comp(conn_fc, gcs_system, gcs_component)
                if m_rfd.get_type() not in ["RADIO_STATUS", "BAD_DATA"]:
                    # Don't forward radio status packet generated autmatically from the RFD
                    time_since_last_rfd_wifi = time.time()
                    if not isFromGroundPi(m_rfd) and makeUID(m_rfd) not in msgbuf_for_last_2_seconds:
                        msgbuf_for_last_2_seconds[makeUID(m_rfd)] = time.time()
                        # Send the video commands to streamer, everything else to ArduPilot
                        if m_rfd.get_type() == "COMMAND_LONG" and m_rfd.command in [mavutil.mavlink.MAV_CMD_VIDEO_START_STREAMING,
                                                                                    mavutil.mavlink.MAV_CMD_VIDEO_STOP_STREAMING]:
                            conn_streamer.write(m_rfd.get_msgbuf())
                        else:
                            conn_fc.write(m_rfd.get_msgbuf())
        except serial.serialutil.SerialException:
            conn_fc = None
            print("Lost FC")

        try:
            if m_wifi_client:
                if gcs_system_client == 0:
                    print("Client Locked onto WGCS at {0}:{1}".format(m_wifi_client.get_srcSystem(), m_wifi_client.get_srcComponent()))
                    gcs_system_client = m_wifi_client.get_srcSystem()
                    gcs_component_client = m_wifi_client.get_srcComponent()
                    set_sys_comp(conn_fc, gcs_system_client, gcs_component_client) # confirmer
                if not isFromGroundPi(m_wifi_client) and makeUID(m_wifi_client) not in msgbuf_for_last_2_seconds_client:
                    msgbuf_for_last_2_seconds_client[makeUID(m_wifi_client)] = time.time()
                    # Send the video commands to streamer, everything else to ArduPilot
                    if m_wifi_client.get_type() == "COMMAND_LONG" and m_wifi_client.command in [mavutil.mavlink.MAV_CMD_VIDEO_START_STREAMING,
                                                                                  mavutil.mavlink.MAV_CMD_VIDEO_STOP_STREAMING]:
                        conn_streamer.write(m_wifi_client.get_msgbuf())
                    else:
                        conn_fc.write(m_wifi_client.get_msgbuf())
                # time_since_last_rfd_wifi_client = time.time()
        except serial.serialutil.SerialException:
            conn_fc = None
            print("Client Lost FC")

        try:
            if m_wifi:
                if gcs_system == 0:
                    print("Locked onto WGCS at {0}:{1}".format(m_wifi.get_srcSystem(), m_wifi.get_srcComponent()))
                    gcs_system = m_wifi.get_srcSystem()
                    gcs_component = m_wifi.get_srcComponent()
                    set_sys_comp(conn_fc, gcs_system, gcs_component)
                if not isFromGroundPi(m_wifi) and makeUID(m_wifi) not in msgbuf_for_last_2_seconds:
                    msgbuf_for_last_2_seconds[makeUID(m_wifi)] = time.time()
                    # Send the video commands to streamer, everything else to ArduPilot
                    if m_wifi.get_type() == "COMMAND_LONG" and m_wifi.command in [mavutil.mavlink.MAV_CMD_VIDEO_START_STREAMING,
                                                                                  mavutil.mavlink.MAV_CMD_VIDEO_STOP_STREAMING]:
                        conn_streamer.write(m_wifi.get_msgbuf())
                    else:
                        conn_fc.write(m_wifi.get_msgbuf())
                time_since_last_rfd_wifi = time.time()
        except serial.serialutil.SerialException:
            conn_fc = None
            print("Lost FC")
        try:
            if m_skylink:
                if gcs_system == 0:
                    print("Locked onto SGCS at {0}:{1}".format(m_skylink.get_srcSystem(), m_skylink.get_srcComponent()))
                    gcs_system = m_skylink.get_srcSystem()
                    gcs_component = m_skylink.get_srcComponent()
                    set_sys_comp(conn_fc, gcs_system, gcs_component)
                    # Switch to SATCOM, if not already
                    if connection_state is not CommsState.ON_SATCOM:
                        connection_state = CommsState.ON_SATCOM
                        print("Going to Satellite")
                if not isFromGroundPi(m_skylink) and makeUID(m_skylink) not in msgbuf_for_last_2_seconds:
                    msgbuf_for_last_2_seconds[makeUID(m_skylink)] = time.time()
                    # Send the video commands to streamer, everything else to ArduPilot
                    if m_skylink.get_type() == "COMMAND_LONG" and m_skylink.command in [mavutil.mavlink.MAV_CMD_VIDEO_START_STREAMING,
                                                                                        mavutil.mavlink.MAV_CMD_VIDEO_STOP_STREAMING]:
                        conn_streamer.write(m_skylink.get_msgbuf())
                    else:
                        conn_fc.write(m_skylink.get_msgbuf())
        except serial.serialutil.SerialException:
            conn_fc = None
            print("Lost FC")

        # Check for inital connection on Wifi or RFD
        if time_since_last_rfd_wifi > 0 and connection_state == CommsState.WAITING_FOR_RFD_WIFI:
            connection_state = CommsState.ON_RFD_WIFI
            print("Initial connection on RFD_WIFI")
        # Switch to SATCOM
        elif ((time.time() - time_since_last_rfd_wifi) > settings['hb_timeout'] and
              connection_state == CommsState.ON_RFD_WIFI):
            connection_state = CommsState.ON_SATCOM
            print("Going to Satellite")
        # Switch back to RFD/Wifi
        elif (connection_state == CommsState.ON_SATCOM and
              (time.time() - time_since_last_rfd_wifi) < settings['hb_timeout']):
            connection_state = CommsState.ON_RFD_WIFI
            print("Going to RFD/Wifi")

        # need to pass along HBs from flight controller in order
        # to initialise the video streamer properly
        if m_fc and m_fc.get_type() == "HEARTBEAT":
            # print("HBS")
            conn_streamer.write(m_fc.get_msgbuf())

        #if m_fc:
        if m_streamer and m_streamer.get_type() == "HEARTBEAT":
            m_streamer = None
        for m in [m_fc, m_streamer]:
            # if m_fc.get_type() == "HEARTBEAT":
            #     print("Got HB")
            if m is None:
                continue
            if conn_rfd:
                try:
                    conn_rfd.write(m.get_msgbuf())
                except (struct.error, NotImplementedError):
                    pass
                except serial.serialutil.SerialException:
                    conn_rfd = None
                    # print("Lost RFD")
            if conn_wifi:
                try:
                    conn_wifi.write(m.get_msgbuf())
                except (struct.error, NotImplementedError):
                    pass
            if conn_wifi_client:
                try:
                    conn_wifi_client.write(m.get_msgbuf())
                except (struct.error, NotImplementedError):
                    pass    
            try:
                # Only write message to Skylink if in whitelist
                if m.get_type() in settings['skylink_messages'] and connection_state == CommsState.ON_SATCOM:
                    freq = settings['skylink_messages'][m.get_type()]
                    if freq == -1:
                        conn_skylink.write(m.get_msgbuf())
                        message_sent_time[m.get_type()] = time.time()
                        data_rate += len(m.get_msgbuf())
                    elif freq > 0:
                        if m.get_type() not in message_sent_time:
                            message_sent_time[m.get_type()] = time.time()
                        elif (time.time() - message_sent_time[m.get_type()]) > (1/freq):
                            conn_skylink.write(m.get_msgbuf())
                            message_sent_time[m.get_type()] = time.time()
                            data_rate += len(m.get_msgbuf()) + 28  # 20 bytes for IPV4 header, 8 for UDP header
                    # else:
                    #     print("Not sending {0}".format(m.get_type()))
            except (struct.error, NotImplementedError):
                pass
            # print("Got {0} from FC".format(m.get_type()))

        # report data rate, skylink status and system status once per 120 sec
        if time.time() - time_since_last_report > 120:
            # kbits/sec = ((bytes * 8)/delta_t)/1000
            data_rate_kbits = ((data_rate*8)/(time.time() - time_since_last_report))/1000

            # Get Skylink signal strength
            text_msg = ''
            if settings['skylink_ip'] != 0:
                # {"dataEnabled":true,"enabled":true,"imei":"","serialNumber":"","signal":{"bars":4,"dbm":-114}}
                try:
                    url = "http://{0}/api/v1/status/satellite".format(settings['skylink_ip'])
                    resp = requests.get(url=url, timeout=0.1)
                    data = resp.json()
                    # print(data)
                    signal = data['signal']['dbm']
                    bars = data['signal']['bars']
                except:
                    signal = "N/A"
                    bars = "N/A"

                print("Avg Skylink data rate is {0:.2f} kbit/sec, Signal {1}dBm, {2}/5 bars".format(data_rate_kbits,
                                                                                                    signal,
                                                                                                    bars))
                print("Mode: {0}".format(connection_state))
                text_msg = 'Skylink {0:.2f}kb/s, Signal {1}dBm, {2}/5 bars'.format(data_rate_kbits,
                                                                                       signal,
                                                                                       bars)
            # Get goexec signal strength
            if settings['goexec_ip'] != 0:
                try:
                    urlsignal = "http://{0}/networkStatus".format(settings['goexec_ip'])
                    urlstatus = "http://{0}/dataConnectionState".format(settings['goexec_ip'])
                    respsignal = requests.get(url=urlsignal, timeout=0.1)
                    respstatus = requests.get(url=urlstatus, timeout=0.1)
                    datasignal = respsignal.json()
                    datastatus = respstatus.json()
                    # print(data)
                    bars = datasignal['networkStatus']['signalStrength']
                    signal = datasignal['networkStatus']['signalLevel']
                    goStatus = datastatus['dataConnectionState']['status']
                except:
                    signal = "N/A"
                    bars = "N/A"
                    goStatus = -1

                if goStatus == 1:
                    print("Avg GoExec rate is {0:.2f} kb/s, Signal {1}dBm, {2}/5 bars".format(data_rate_kbits,
                                                                                                  signal,
                                                                                                  bars))
                    print("Mode: {0}".format(connection_state))
                    text_msg = 'GoExec {0:.2f}kb/s, Signal {1}dBm, {2}/5 bars'.format(data_rate_kbits,
                                                                                          signal,
                                                                                          bars)
                elif goStatus == 0:
                    text_msg = 'GoExec not active, Signal {0}dBm, {1}/5 bars. Trying to activate'.format(signal, bars)
                    print(text_msg)
                    try:
                        # Try activating GoExec if not active
                        dataPut = {'dataConnectionState': {'status': 1, 'duration': 14400}}
                        r = requests.put("http://{0}/dataConnectionState".format(settings['goexec_ip']), json=dataPut)
                        print("GoExec init response: {0}".format(r.content))
                        if r.content != b'{}':
                            resp = json.loads(r.content)
                            if resp['errorMessage'] != '':
                                text_msg = "GoExec error: {0}".format(resp['errorMessage'].split(',')[-1])
                    except:
                        text_msg = "GoExec not active. Unable to communicate with modem."
                else:
                    text_msg = "GoExec not active. Unable to communicate with modem."

                # send over non-satcom links
                if connection_state == CommsState.ON_RFD_WIFI:
                    try:
                        conn_rfd.mav.statustext_send(
                                mavutil.mavlink.MAV_SEVERITY_INFO, text_msg.encode())
                    except (struct.error, NotImplementedError, AttributeError):
                        pass
                    try:
                        conn_wifi.mav.statustext_send(
                                mavutil.mavlink.MAV_SEVERITY_INFO, text_msg.encode())
                    except (struct.error, NotImplementedError, AttributeError):
                        pass

            data_rate = 0
            time_since_last_report = time.time()

            if conn_skylink and text_msg != '':
                try:
                    if connection_state == CommsState.ON_SATCOM:
                        conn_skylink.mav.statustext_send(
                            mavutil.mavlink.MAV_SEVERITY_INFO, text_msg.encode())
                except (struct.error, NotImplementedError, AttributeError):
                    pass

            # single ping over satcom to keep the route open
            # ping(settings['skylink_remote'].split(':')[0])

            # Send general stats
            if conn_wifi:
                delta_wifi = conn_wifi.mav_count - rx_packets_wifi
            else:
                delta_wifi = 0
            if conn_rfd:
                delta_rfd = conn_rfd.mav_count - rx_packets_rfd
            else:
                delta_rfd = 0
            delta_skylink = conn_skylink.mav_count - rx_packets_skylink
            stats_str = "Air 2min: {0} Wifi, {1} RFD, {2} Satellite".format(delta_wifi,
                                                                                     delta_rfd,
                                                                                     delta_skylink)
            if conn_wifi:
                rx_packets_wifi = conn_wifi.mav_count
            else:
                rx_packets_wifi = 0
            if conn_rfd:
                rx_packets_rfd = conn_rfd.mav_count
            else:
                rx_packets_rfd = 0
            rx_packets_skylink = conn_skylink.mav_count
            print(stats_str)
            if connection_state == CommsState.ON_RFD_WIFI:
                try:
                    conn_rfd.mav.statustext_send(
                            mavutil.mavlink.MAV_SEVERITY_INFO, stats_str.encode())
                except (struct.error, NotImplementedError, AttributeError):
                    pass
                try:
                    conn_wifi.mav.statustext_send(
                            mavutil.mavlink.MAV_SEVERITY_INFO, stats_str.encode())
                except (struct.error, NotImplementedError, AttributeError):
                    pass
            elif connection_state == CommsState.ON_SATCOM:
                try:
                    conn_skylink.mav.statustext_send(
                            mavutil.mavlink.MAV_SEVERITY_INFO, stats_str.encode())
                except (struct.error, NotImplementedError, AttributeError):
                    pass

            # Try reconnecting any failed serial devices
            if conn_rfd is None:
                try:
                    conn_rfd = mavutil.mavlink_connection(str(settings['rfd_port']), baud=settings['rfd_baud'],
                                                          autoreconnect=True,
                                                          source_system=ap_system, force_connected=False,
                                                          source_component=ap_component)
                    print("Reconnected RFD")
                except serial.serialutil.SerialException:
                    conn_rfd = None
                    # print("Lost RFD")
            if conn_fc is None:
                try:
                    conn_fc = mavutil.mavlink_connection(str(settings['fc_port']), baud=settings['fc_baud'],
                                                         autoreconnect=True,
                                                         source_system=1, force_connected=False,
                                                         source_component=mavutil.mavlink.MAV_COMP_ID_PERIPHERAL)
                    set_sys_comp(conn_fc, gcs_system, gcs_component)
                except serial.serialutil.SerialException:
                    conn_fc = None
                    print("Lost FC")
                print("Reconnected FC")
        if exit_event.is_set():
            break

    # Cleanup
    print("Exiting")
    if conn_fc:
        conn_fc.close()
    if conn_rfd:
        conn_rfd.close()
    if conn_wifi:
        conn_wifi.close()
    if conn_wifi_client:
        conn_wifi_client.close()
    if conn_skylink:
        conn_skylink.close()
