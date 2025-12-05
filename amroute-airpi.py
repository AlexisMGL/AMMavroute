#!/usr/bin/env python3
"""
AMRoute AirPi - Meshnet Edition
Simplified telemetry bridge:
  FC (serial) <-> AirPi <-> GCS (UDP via NordVPN Meshnet)

- No filtering / no rate limiting: forwards everything
- Sends optional video commands to a separate streamer endpoint
- Sends STATUSTEXT to MissionPlanner every 30s with throughput + data usage
"""

from pymavlink import mavutil
import yaml
import time
import serial
import struct

UDP_IP_OVERHEAD = 28  # approx: IPv4(20) + UDP(8)

def parse_host_port(hostport):
    parts = hostport.split(':')
    if len(parts) != 2:
        raise ValueError(f"Invalid host:port string: {hostport}")
    host = parts[0]
    port = int(parts[1])
    return host, port


def connect_fc(port, baud):
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
                print("[FC] No heartbeat, retrying in 5s...")
        except serial.serialutil.SerialException as e:
            print(f"[FC] Serial error: {e}. Retrying in 5s...")
        except Exception as e:
            print(f"[FC] Unexpected error: {e}. Retrying in 5s...")
        time.sleep(5)


def connect_gcs(host, port):
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


def clamp_statustext(s: str, max_len: int = 50) -> str:
    # STATUSTEXT classic is short; keep it compact and safe ASCII-ish.
    if len(s) <= max_len:
        return s
    return s[:max_len]


def main():
    print("----- AMRoute AirPi Meshnet -----\nStarting...")

    with open('settings-air.yml', 'r') as file:
        settings = yaml.safe_load(file)

    fc_port = settings['fc_port']
    fc_baud = settings['fc_baud']
    gcs_host, gcs_port = parse_host_port(settings['gcs_remote'])
    streamer_ipport = settings.get('streamer_ipport', None)

    print(f"[CONFIG] FC port          : {fc_port} @ {fc_baud}")
    print(f"[CONFIG] GCS (Meshnet)    : {gcs_host}:{gcs_port}")
    print(f"[CONFIG] Streamer ip:port : {streamer_ipport}\n")

    conn_fc = connect_fc(fc_port, fc_baud)
    conn_gcs = connect_gcs(gcs_host, gcs_port)
    conn_streamer = connect_streamer(streamer_ipport)

    # ---- Stats (per 30s + totals) ----
    REPORT_PERIOD = 30.0
    last_report = time.time()

    period_out_bytes = 0   # AirPi -> GCS
    period_in_bytes  = 0   # GCS -> AirPi (approx from received msg sizes)

    total_out_bytes = 0
    total_in_bytes  = 0

    try:
        while True:
            # --- From FC to GCS ---
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
                if conn_streamer and m_fc.get_type() == "HEARTBEAT":
                    try:
                        conn_streamer.write(m_fc.get_msgbuf())
                    except (struct.error, NotImplementedError) as e:
                        print(f"[STREAMER] Error sending HB: {e}")

                try:
                    buf = m_fc.get_msgbuf()
                    conn_gcs.write(buf)
                    n = len(buf) + UDP_IP_OVERHEAD
                    period_out_bytes += n
                    total_out_bytes += n
                except (struct.error, NotImplementedError) as e:
                    print(f"[GCS] Error sending {m_fc.get_type()}: {e}")

            # --- From GCS to FC ---
            try:
                m_gcs = conn_gcs.recv_msg()
            except Exception as e:
                print(f"[GCS] Error receiving from GCS: {e}")
                m_gcs = None

            if m_gcs is not None:
                # Count inbound meshnet traffic (approx)
                try:
                    in_n = len(m_gcs.get_msgbuf()) + UDP_IP_OVERHEAD
                    period_in_bytes += in_n
                    total_in_bytes += in_n
                except Exception:
                    pass

                if (m_gcs.get_type() == "COMMAND_LONG" and
                    m_gcs.command in [mavutil.mavlink.MAV_CMD_VIDEO_START_STREAMING,
                                      mavutil.mavlink.MAV_CMD_VIDEO_STOP_STREAMING]):
                    if conn_streamer:
                        try:
                            conn_streamer.write(m_gcs.get_msgbuf())
                        except (struct.error, NotImplementedError) as e:
                            print(f"[STREAMER] Error sending video cmd: {e}")

                try:
                    conn_fc.write(m_gcs.get_msgbuf())
                except (struct.error, NotImplementedError, serial.serialutil.SerialException) as e:
                    print(f"[FC] Error sending from GCS: {e}")

            # --- Periodic STATUSTEXT to Mission Planner (every 30s) ---
            now = time.time()
            if (now - last_report) >= REPORT_PERIOD:
                dt = now - last_report if (now - last_report) > 0 else REPORT_PERIOD

                out_kbps = (period_out_bytes * 8) / dt / 1000.0
                in_kbps  = (period_in_bytes  * 8) / dt / 1000.0

                out_mb = total_out_bytes / 1_000_000.0
                in_mb  = total_in_bytes  / 1_000_000.0

                msg = f"Mesh O:{out_kbps:.0f}k I:{in_kbps:.0f}k Up:{out_mb:.1f}MB Dn:{in_mb:.1f}MB"
                msg = clamp_statustext(msg, 50)

                print(f"[STATS] {msg}")

                # Send directly to GCS so Mission Planner surely shows it
                try:
                    conn_gcs.mav.statustext_send(
                        mavutil.mavlink.MAV_SEVERITY_INFO,
                        msg.encode(errors="ignore")
                    )
                except Exception as e:
                    print(f"[GCS] Failed to send STATUSTEXT: {e}")

                period_out_bytes = 0
                period_in_bytes = 0
                last_report = now

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n[MAIN] KeyboardInterrupt, exiting...")

    print("Exiting")
    try:
        if conn_fc: conn_fc.close()
    except Exception:
        pass
    try:
        if conn_gcs: conn_gcs.close()
    except Exception:
        pass
    try:
        if conn_streamer: conn_streamer.close()
    except Exception:
        pass


if __name__ == '__main__':
    main()
