#!/usr/bin/env python3
"""
AMRoute AirPi - Meshnet Edition
Simplified telemetry bridge:
  FC (serial) <-> AirPi <-> GCS (UDP via NordVPN Meshnet)

- No filtering / no rate limiting: forwards everything
- Sends optional video commands to a separate streamer endpoint
- Sends STATUSTEXT to MissionPlanner every 30s with:
    * average total throughput over last 30s in kB/s (IN+OUT)
    * cumulative IN and OUT in MB
"""

from pymavlink import mavutil
import yaml
import time
import serial
import struct

# Approximate overhead for IP+UDP per packet (for stats only)
UDP_IP_OVERHEAD = 28  # IPv4(20) + UDP(8)


def parse_host_port(hostport: str):
    parts = hostport.split(":")
    if len(parts) != 2:
        raise ValueError(f"Invalid host:port string: {hostport}")
    host = parts[0]
    port = int(parts[1])
    return host, port


def connect_fc(port, baud):
    """
    Connect to the flight controller, wait for heartbeat,
    and return (connection, system_id, component_id).
    """
    while True:
        try:
            print(f"[FC] Connecting on {port} @ {baud}...")
            conn = mavutil.mavlink_connection(
                str(port),
                baud=baud,
                autoreconnect=True,
                source_system=1,
                source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
            )
            print("[FC] Waiting for heartbeat...")
            mh = conn.wait_heartbeat(timeout=5)
            if mh is not None and conn.probably_vehicle_heartbeat(mh):
                ap_sys = mh.get_srcSystem()
                ap_comp = mh.get_srcComponent()
                print(
                    "[FC] Got heartbeat from ArduPilot "
                    f"(system {ap_sys} component {ap_comp})"
                )
                return conn, ap_sys, ap_comp
            else:
                print("[FC] No heartbeat, retrying in 5s...")
        except serial.serialutil.SerialException as e:
            print(f"[FC] Serial error: {e}. Retrying in 5s...")
        except Exception as e:
            print(f"[FC] Unexpected error: {e}. Retrying in 5s...")
        time.sleep(5)


def connect_gcs(host, port, ap_system):
    """
    Connect to GCS via UDP. We set source_system to ap_system
    so STATUSTEXT appear as if they come from the vehicle.
    """
    url = f"udpout:{host}:{port}"
    print(f"[GCS] Connecting to {url} ...")
    conn = mavutil.mavlink_connection(
        url,
        autoreconnect=True,
        source_system=ap_system,
        source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
    )
    # Safety: ensure underlying mav object has the same sysid
    conn.mav.srcSystem = ap_system
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
        source_component=mavutil.mavlink.MAV_COMP_ID_PERIPHERAL,
    )
    return conn


def clamp_statustext(s: str, max_len: int = 50) -> str:
    """Clamp STATUSTEXT to classic MAVLink length, keep it ASCII-safe."""
    if len(s) <= max_len:
        return s
    return s[:max_len]


def main():
    print("----- AMRoute AirPi Meshnet -----\nStarting...")

    with open("settings-air.yml", "r") as file:
        settings = yaml.safe_load(file)

    fc_port = settings["fc_port"]
    fc_baud = settings["fc_baud"]
    gcs_host, gcs_port = parse_host_port(settings["gcs_remote"])
    streamer_ipport = settings.get("streamer_ipport", None)

    print(f"[CONFIG] FC port          : {fc_port} @ {fc_baud}")
    print(f"[CONFIG] GCS (Meshnet)    : {gcs_host}:{gcs_port}")
    print(f"[CONFIG] Streamer ip:port : {streamer_ipport}\n")

    # First connect to FC to get system/component IDs
    conn_fc, ap_system, ap_component = connect_fc(fc_port, fc_baud)
    conn_gcs = connect_gcs(gcs_host, gcs_port, ap_system)
    conn_streamer = connect_streamer(streamer_ipport)

    # ---- Stats (per 30s + totals) ----
    REPORT_PERIOD = 30.0
    last_report = time.time()

    # Per-period bytes (for average over last 30s)
    period_out_bytes = 0  # AirPi -> GCS
    period_in_bytes = 0   # GCS  -> AirPi

    # Totals since script start
    total_out_bytes = 0
    total_in_bytes = 0

    try:
        while True:
            # --- From FC to GCS ---
            try:
                m_fc = conn_fc.recv_msg()
            except serial.serialutil.SerialException as e:
                print(f"[FC] Lost FC: {e}, reconnecting...")
                conn_fc, ap_system, ap_component = connect_fc(fc_port, fc_baud)
                # Make sure GCS source_system matches the (possibly new) FC sysid
                conn_gcs = connect_gcs(gcs_host, gcs_port, ap_system)
                m_fc = None
            except Exception as e:
                print(f"[FC] Error receiving from FC: {e}")
                m_fc = None

            if m_fc is not None:
                # Forward HEARTBEAT also to streamer to keep it initialized
                if conn_streamer and m_fc.get_type() == "HEARTBEAT":
                    try:
                        conn_streamer.write(m_fc.get_msgbuf())
                    except (struct.error, NotImplementedError) as e:
                        print(f"[STREAMER] Error sending HB: {e}")

                # Forward everything to GCS, count bytes
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
                # Approximate inbound traffic from GCS via Meshnet
                try:
                    buf_gcs = m_gcs.get_msgbuf()
                    in_n = len(buf_gcs) + UDP_IP_OVERHEAD
                    period_in_bytes += in_n
                    total_in_bytes += in_n
                except Exception:
                    buf_gcs = m_gcs.get_msgbuf()

                # Forward video commands also to streamer
                if (
                    m_gcs.get_type() == "COMMAND_LONG"
                    and m_gcs.command
                    in [
                        mavutil.mavlink.MAV_CMD_VIDEO_START_STREAMING,
                        mavutil.mavlink.MAV_CMD_VIDEO_STOP_STREAMING,
                    ]
                ):
                    if conn_streamer:
                        try:
                            conn_streamer.write(buf_gcs)
                        except (struct.error, NotImplementedError) as e:
                            print(f"[STREAMER] Error sending video cmd: {e}")

                # Forward to FC
                try:
                    conn_fc.write(buf_gcs)
                except (
                    struct.error,
                    NotImplementedError,
                    serial.serialutil.SerialException,
                ) as e:
                    print(f"[FC] Error sending from GCS: {e}")

            # --- Periodic STATUSTEXT to Mission Planner (every 30s) ---
            now = time.time()
            if (now - last_report) >= REPORT_PERIOD:
                dt = now - last_report if (now - last_report) > 0 else REPORT_PERIOD

                # Average total throughput over last 30s in kB/s
                interval_total_bytes = period_in_bytes + period_out_bytes
                avg_kBps = (interval_total_bytes / dt) / 1024.0

                # Cumulative IN/OUT in MB (since script start)
                out_MB = total_out_bytes / (1024.0 * 1024.0)
                in_MB = total_in_bytes / (1024.0 * 1024.0)

                msg = f"NET30 {avg_kBps:.1f}kB/s IN {in_MB:.1f}MB OUT {out_MB:.1f}MB"
                msg = clamp_statustext(msg, 50)

                # Console AirPi
                print(f"[STATS] {msg}")

                # STATUSTEXT to GCS (Mission Planner), with sysid == ap_system
                try:
                    conn_gcs.mav.statustext_send(
                        mavutil.mavlink.MAV_SEVERITY_INFO,
                        msg.encode(errors="ignore"),
                    )
                except Exception as e:
                    print(f"[GCS] Failed to send STATUSTEXT: {e}")

                # Reset period counters
                period_out_bytes = 0
                period_in_bytes = 0
                last_report = now

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n[MAIN] KeyboardInterrupt, exiting...")

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


if __name__ == "__main__":
    main()
