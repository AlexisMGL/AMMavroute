#!/usr/bin/env python3
"""
AMMcamera V1.6.2 - USB Webcam (Logitech C920) + RTP H.265 streaming

Pipeline:
  v4l2src (/dev/video0) -> videoconvert -> videoscale -> x265enc -> rtph265pay -> udpsink

Control:
  - Start via MAV_CMD_VIDEO_START_STREAMING
  - Stop  via MAV_CMD_VIDEO_STOP_STREAMING
"""

from pymavlink import mavutil
import yaml
import sys
import threading
import time
import gi
import platform
import os
import signal

gi.require_version("Gst", "1.0")
from gi.repository import Gst

exit_event = threading.Event()
stop_pipeline_flag = threading.Event()


def signal_handler(signum, frame):
    exit_event.set()


def is_debian_bookworm() -> bool:
    try:
        return (
            platform.system() == "Linux"
            and os.path.exists("/etc/os-release")
            and "bookworm" in open("/etc/os-release").read()
        )
    except Exception:
        return False


def start_gstreamer_pipeline(
    udpendpoint: str,
    bitrate: int,
    height: int,
    width: int,
    framerate: int,
    resize: float,
    crop_height: int,
    crop_width: int,
    is_h264: bool,  # not used, kept for compat
    video_device: str,
    overlay_timestamp: bool = False,
) -> None:
    """
    Pipeline:
      v4l2src (raw) -> videoconvert -> videoscale -> caps -> x265enc -> queue(leaky) -> RTP H.265 -> udpsink
    """

    # ---- Crop / resize (optional) ----
    if crop_height != -1 and crop_width != -1:
        offset_left = (width - crop_width) // 2
        offset_right = (width - crop_width) - offset_left
        offset_top = (height - crop_height) // 2
        offset_bottom = (height - crop_height) - offset_top

        new_width = int(crop_width * resize)
        new_height = int(crop_height * resize)

        if crop_width > width or crop_height > height:
            raise ValueError(
                "Crop dimensions should not exceed the original video dimensions."
            )
    else:
        offset_left = offset_right = offset_top = offset_bottom = 0
        new_width = width
        new_height = height

    # ---- Source: C920 in raw mode (/dev/video0 by default) ----
    s_src = f"v4l2src device={video_device}"

    # Convert / scale / crop
    s_pre = (
        " ! videoconvert "
        "! videoscale "
        "! videorate "
        f"! video/x-raw,width={new_width},height={new_height},framerate={framerate}/1,format=I420 "
        f"! videocrop left={offset_left} right={offset_right} top={offset_top} bottom={offset_bottom} "
    )
    if overlay_timestamp:
        # Overlay the send-side wall-clock to measure end-to-end latency on the viewer.
        s_pre += (
            '! clockoverlay time-format="%H:%M:%S.%3N" shaded-background=true '
            "halignment=right valignment=bottom "
        )

    # ---- Software H.265 encoder ----
    # key-int-max = framerate -> about 1s GOP, faster recovery after packet loss
    # queue leaky, bounded to about 2s of video
    if framerate <= 0:
        keyint = 1
    else:
        keyint = framerate

    s_h265 = (
        "x265enc tune=zerolatency bitrate={0} speed-preset=ultrafast "
        "key-int-max={1} "
        "! video/x-h265,profile=main,stream-format=byte-stream "
        "! queue max-size-time=2000000000 max-size-buffers=0 max-size-bytes=0 leaky=downstream "
        "! rtph265pay config-interval=1 pt=96"
    ).format(bitrate, keyint)

    host, port = udpendpoint.split(":")
    s_sink = f"udpsink host={host} port={port} sync=false async=false"

    pipeline_str = s_src + s_pre + "! " + s_h265 + " ! " + s_sink

    print("GStreamer pipeline:")
    print(pipeline_str)

    pipeline = Gst.parse_launch(pipeline_str)
    pipeline.set_state(Gst.State.PLAYING)

    print("Server sending UDP stream to " + udpendpoint)

    bus = pipeline.get_bus()
    while not stop_pipeline_flag.is_set() and not exit_event.is_set():
        msg = bus.timed_pop_filtered(
            100 * Gst.MSECOND, Gst.MessageType.ERROR | Gst.MessageType.EOS
        )
        if msg:
            if msg.type == Gst.MessageType.ERROR:
                err, debug = msg.parse_error()
                print(f"Error: {err}, {debug}")
                break
            elif msg.type == Gst.MessageType.EOS:
                print("End of Stream")
                break

    print("Cleaning up stream")
    pipeline.set_state(Gst.State.NULL)


if __name__ == "__main__":

    print("----- AMMcamera V1.6.2 -----")
    print("Starting...")

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    settings = {}
    conn_mavlink = None
    gstreamer_thread = None

    # --- Load settings ---
    try:
        with open("settings-camera.yml", "r") as file:
            settings = yaml.safe_load(file)
    except yaml.scanner.ScannerError:
        print("Error: Bad settings-camera.yml")
        sys.exit(1)
    except FileNotFoundError:
        print("Error: settings-camera.yml not found")
        sys.exit(1)

    Gst.init(None)

    video_device = settings.get("video_device", "/dev/video0")

    # --- Connect MAVLink and wait for heartbeat ---
    try:
        conn_mavlink = mavutil.mavlink_connection(
            str(settings["mavlink_source"]),
            autoreconnect=True,
            source_system=1,
            force_connected=False,
            source_component=mavutil.mavlink.MAV_COMP_ID_PERIPHERAL,
        )
        while True:
            m = conn_mavlink.wait_heartbeat(timeout=0.5)
            if m is not None and conn_mavlink.probably_vehicle_heartbeat(m):
                fc_compid = m.get_srcComponent()
                fc_sysid = m.get_srcSystem()
                print(
                    "Got Heartbeat from Autopilot (system {0} component {1})".format(
                        fc_sysid, fc_compid
                    )
                )
                conn_mavlink.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_CAMERA,
                    mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                    0,
                    0,
                    0,
                )
                conn_mavlink.source_system = fc_sysid
                conn_mavlink.source_component = fc_compid
                conn_mavlink.mav.srcSystem = fc_sysid
                conn_mavlink.mav.srcComponent = fc_compid
                break
            if exit_event.is_set():
                print("Exiting (signal)")
                conn_mavlink.close()
                sys.exit(0)
    except Exception as e:
        print(f"Error connecting MAVLink: {e}")
        if conn_mavlink:
            conn_mavlink.close()
        sys.exit(0)

    # --- Main loop: listen for camera commands ---
    while not exit_event.is_set():
        try:
            if conn_mavlink:
                try:
                    m_mavlink = conn_mavlink.recv_match(
                        type="COMMAND_LONG", blocking=False
                    )
                except Exception:
                    m_mavlink = None
                    print("Lost FC")
                    time.sleep(0.5)
                    continue
        except (BlockingIOError, KeyboardInterrupt):
            break

        if m_mavlink:
            if m_mavlink.get_type() == "COMMAND_LONG":
                if (
                    m_mavlink.command
                    == mavutil.mavlink.MAV_CMD_VIDEO_START_STREAMING
                ):
                    if not gstreamer_thread:
                        print("Starting stream")
                        conn_mavlink.mav.statustext_send(
                            mavutil.mavlink.MAV_SEVERITY_INFO,
                            "AMMcamera: Starting Video".encode(),
                        )
                        stop_pipeline_flag.clear()
                        gstreamer_thread = threading.Thread(
                            target=start_gstreamer_pipeline,
                            args=(
                                settings["video_endpoint"],
                                settings["bitrate"],
                                settings["height"],
                                settings["width"],
                                settings["framerate"],
                                settings["resize"],
                                settings["crop_height"],
                                settings["crop_width"],
                                settings.get("is_h264", False),
                                video_device,
                                settings.get("overlay_timestamp", False),
                            ),
                        )
                        gstreamer_thread.start()
                    else:
                        print("Stream already started")
                        conn_mavlink.mav.statustext_send(
                            mavutil.mavlink.MAV_SEVERITY_INFO,
                            "AMMcamera: Video already started".encode(),
                        )
                    conn_mavlink.mav.command_ack_send(
                        mavutil.mavlink.MAV_CMD_VIDEO_START_STREAMING,
                        mavutil.mavlink.MAV_RESULT_ACCEPTED,
                    )

                elif (
                    m_mavlink.command
                    == mavutil.mavlink.MAV_CMD_VIDEO_STOP_STREAMING
                ):
                    print("Stopping stream")
                    if gstreamer_thread:
                        conn_mavlink.mav.statustext_send(
                            mavutil.mavlink.MAV_SEVERITY_INFO,
                            "AMMcamera: Stopping Video".encode(),
                        )
                        stop_pipeline_flag.set()
                        gstreamer_thread.join()
                        gstreamer_thread = None
                    else:
                        print("Stream already stopped")
                        conn_mavlink.mav.statustext_send(
                            mavutil.mavlink.MAV_SEVERITY_INFO,
                            "AMMcamera: Video already stopped".encode(),
                        )
                    conn_mavlink.mav.command_ack_send(
                        mavutil.mavlink.MAV_CMD_VIDEO_STOP_STREAMING,
                        mavutil.mavlink.MAV_RESULT_ACCEPTED,
                    )
        else:
            time.sleep(0.1)

    print("Exiting main loop")
    if conn_mavlink:
        conn_mavlink.close()
