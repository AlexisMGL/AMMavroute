#!/usr/bin/env python3
"""
AMRoute AirPi - Meshnet Edition
Simplified telemetry bridge:
  FC (serial) <-> AirPi <-> GCS (UDP via NordVPN Meshnet)

- No RFD / Wifi / SAT fallback logic
- Keeps per-message filtering based on 'skylink_messages' in settings-air.yml
- Sends optional video commands to a separate streamer endpoint
"""

from pymavlink import mavutil
import yaml
import time
import serial
import struct

def parse_host_port(hostport):
    """Parse 'host:port' string into (host, int(port))."""
    parts = hostport.split(':')
    if len(parts) != 2:
        raise ValueError(f"Invalid host:port string: {hostport}")
    host = parts[0]
    port = int(parts[1])
    return host, port


def connect_fc(port, baud):
    """Connect to the flight controller (serial), with retries."""
    while True:
        try:
            print(f"[FC] Connecting on {port} @ {baud}...")
            conn = mavutil.mavlink_connection(
                str(port),
                baud=baud,
                autoreconnect=True,
                source_system=1,
                source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER
            )
            print("[FC] Waiting for heartbeat...")
            mh = conn.wait_heartbeat(timeout=5)
            if mh is not None and conn.probably_vehicle_heartbeat(mh):
                print(f"[FC] Got heartbeat from ArduPilot "
                      f"(system {mh.get_srcSystem()} component {mh.get_srcComponent()})")
                return conn
            else:
                print("[FC] No heartbeat, retrying in 2s...")
        except serial.serialutil.SerialException as e:
            print(f"[FC] Serial error: {e}. Retrying in 2s...")
        except Exception as e:
            print(f"[FC] Unexpected error: {e}. Retrying in 2s...")
        time.sleep(2)


def connect_gcs(host, port):
    """Create a UDP connection to the GCS (Meshnet)."""
    url = f"udpout:{host}:{port}"
    print(f"[GCS] Connecting to {url} ...")
    conn = mavutil.mavlink_connection(
        url,
        autoreconnect=True,
        source_system=1,
        source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER
    )
    return conn


def connect_streamer(streamer_ipport):
    """Connect to the video streamer endpoint, if configured."""
    if not streamer_ipport:
        return None
    url = f"udpout:{streamer_ipport}"
    print(f"[STREAMER] Connecting to {url} ...")
    conn = mavutil.mavlink_connection(
        url,
        autoreconnect=True,
        source_system=1,
        source_component=mavutil.mavlink.MAV_COMP_ID_PERIPHERAL
    )
    return conn


def main():
    print("----- AMRoute AirPi Meshnet -----\nStarting...")

    # --- Load settings ---
    with open('settings-air.yml', 'r') as file:
        settings = yaml.safe_load(file)

    fc_port = settings['fc_port']
    fc_baud = settings['fc_baud']

    gcs_host, gcs_port = parse_host_port(settings['gcs_remote'])
    streamer_ipport = settings.get('streamer_ipport', None)

    msg_filters = settings.get('skylink_messages', {})

    print(f"[CONFIG] FC port          : {fc_port} @ {fc_baud}")
    print(f"[CONFIG] GCS (Meshnet)    : {gcs_host}:{gcs_port}")
    print(f"[CONFIG] Streamer ip:port : {streamer_ipport}")
    print(f"[CONFIG] Filtered messages: {len(msg_filters)} types\n")

    # --- Create connections ---
    conn_fc = connect_fc(fc_port, fc_baud)
    conn_gcs = connect_gcs(gcs_host, gcs_port)
    conn_streamer = connect_streamer(streamer_ipport)

    # For rate limiting and stats
    message_sent_time = {}   # type -> last_send_time
    data_rate = 0
    last_report = time.time()
    REPORT_PERIOD = 120.0    # seconds

    try:
        while True:
            # --- From FC to GCS (main direction) ---
            try:
                m_fc = conn_fc.recv_msg()
            except serial.serialutil.SerialException as e:
                print(f"[FC] Lost FC: {e}, reconnecting...")
                conn_fc = connect_fc(fc_port, fc_baud)
                m_fc = None
            except Exception as e:
                print(f"[FC] Error receiving from FC: {e}")
                m_fc = None

            if m_fc is not None:
                # Forward HEARTBEAT to streamer (for init), like the original script
                if conn_streamer and m_fc.get_type() == "HEARTBEAT":
                    try:
                        conn_streamer.write(m_fc.get_msgbuf())
                    except (struct.error, NotImplementedError) as e:
                        print(f"[STREAMER] Error sending HB: {e}")

                # Apply filter/rate limiting for FC->GCS
                mtype = m_fc.get_type()
                if mtype in msg_filters:
                    freq = msg_filters[mtype]
                    now = time.time()

                    if freq == -1:
                        # Always send
                        try:
                            conn_gcs.write(m_fc.get_msgbuf())
                            data_rate += len(m_fc.get_msgbuf()) + 28
                        except (struct.error, NotImplementedError) as e:
                            print(f"[GCS] Error sending {mtype}: {e}")

                    elif freq > 0:
                        last = message_sent_time.get(mtype, 0)
                        if (now - last) >= (1.0 / freq):
                            try:
                                conn_gcs.write(m_fc.get_msgbuf())
                                message_sent_time[mtype] = now
                                data_rate += len(m_fc.get_msgbuf()) + 28
                            except (struct.error, NotImplementedError) as e:
                                print(f"[GCS] Error sending {mtype}: {e}")
                    # freq == 0 -> do not send
                # If not in msg_filters: drop silently

            # --- From GCS to FC ---
            try:
                m_gcs = conn_gcs.recv_msg()
            except Exception as e:
                print(f"[GCS] Error receiving from GCS: {e}")
                m_gcs = None

            if m_gcs is not None:
                # Commandes vidéo : forward aussi au streamer
                if (m_gcs.get_type() == "COMMAND_LONG" and
                    m_gcs.command in [mavutil.mavlink.MAV_CMD_VIDEO_START_STREAMING,
                                      mavutil.mavlink.MAV_CMD_VIDEO_STOP_STREAMING]):
                    if conn_streamer:
                        try:
                            conn_streamer.write(m_gcs.get_msgbuf())
                        except (struct.error, NotImplementedError) as e:
                            print(f"[STREAMER] Error sending video cmd: {e}")

                # Forward tout vers le FC
                try:
                    conn_fc.write(m_gcs.get_msgbuf())
                except (struct.error, NotImplementedError, serial.serialutil.SerialException) as e:
                    print(f"[FC] Error sending from GCS: {e}")

            # --- Periodic stats ---
            now = time.time()
            if (now - last_report) > REPORT_PERIOD:
                if (now - last_report) > 0:
                    data_rate_kbits = ((data_rate * 8) / (now - last_report)) / 1000.0
                else:
                    data_rate_kbits = 0.0

                stats_str = f"AirPi Meshnet 2min avg: {data_rate_kbits:.2f} kbit/s to GCS"
                print(stats_str)

                # Optionnel : envoyer en STATUSTEXT vers le GCS via FC
                try:
                    conn_fc.mav.statustext_send(
                        mavutil.mavlink.MAV_SEVERITY_INFO,
                        stats_str.encode()
                    )
                except Exception:
                    pass

                data_rate = 0
                last_report = now

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n[MAIN] KeyboardInterrupt, exiting...")

    # Cleanup
    print("Exiting")
    try:
        if conn_fc:
            conn_fc.close()
    except Exception:
        pass
    try:
        if conn_gcs:
            conn_gcs.close()
    except Exception:
        pass
    try:
        if conn_streamer:
            conn_streamer.close()
    except Exception:
        pass


if __name__ == '__main__':
    main()
