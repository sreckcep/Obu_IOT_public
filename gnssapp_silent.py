"""
pygnssutils - gnssapp.py

*** FOR ILLUSTRATION ONLY - NOT FOR PRODUCTION USE ***

Skeleton GNSS application which continuously receives and parses NMEA, UBX or RTCM
data from a receiver until the stop Event is set or stop() method invoked. Assumes
receiver is connected via serial USB or UART1 port.

The app also implements basic methods needed by certain pygnssutils classes.

Optional keyword arguments:

- recvqueue - if defined, a tuple of (raw_data, parsed_data) from the receiver will
  be placed on this Queue. This Queue can then be consumed by an external application.
- sendqueue - if defined, any data placed on this Queue will be sent to the receiver
  (e.g. UBX commands/polls or NTRIP RTCM data). Data must be a tuple of 
  (raw_data, parsed_data).
- idonly - determines whether the app prints out the entire parsed message,
  or just the message identity.
- enableubx - suppresses NMEA receiver output and substitutes a minimum set
  of UBX messages instead (NAV-PVT, NAV-SAT, NAV-DOP, RXM-RTCM).
- showhacc - show estimate of horizonal accuracy in metres (if available).

Created on 27 Jul 2023

:author: semuadmin
:copyright: SEMU Consulting © 2023
:license: BSD 3-Clause


tweaks by Pierre Beckers 
"""
# pylint: disable=invalid-name, too-many-instance-attributes

from argparse import ArgumentDefaultsHelpFormatter, ArgumentParser
import pprint
from queue import Empty, Queue
from threading import Event, Thread
from time import sleep

from pynmeagps import NMEAMessageError, NMEAParseError
from pyrtcm import RTCMMessage, RTCMMessageError, RTCMParseError
from serial import Serial

from pyubx2 import (
    NMEA_PROTOCOL,
    RTCM3_PROTOCOL,
    UBX_PROTOCOL,
    UBXMessage,
    UBXMessageError,
    UBXParseError,
    UBXReader,
)

DISCONNECTED = 0
CONNECTED = 1

class GNSSData():
    connected : int
    lat: int
    lon: int
    alt: int
    hAcc: int
    speed : int
    sAcc : int
    heading: int
    headAcc: int

class GNSSSkeletonApp:
    """
    Skeleton GNSS application which communicates with a GNSS receiver.
    """

    def __init__(
        self, port: str, baudrate: int, timeout: float, stopevent: Event, **kwargs
    ):
        """
        Constructor.

        :param str port: serial port e.g. "/dev/ttyACM1"
        :param int baudrate: baudrate
        :param float timeout: serial timeout in seconds
        :param Event stopevent: stop event
        """

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.stopevent = stopevent
        self.recvqueue = kwargs.get("recvqueue", None)
        self.sendqueue = kwargs.get("sendqueue", None)
        self.idonly = kwargs.get("idonly", True)
        self.enableubx = kwargs.get("enableubx", False)
        self.showhacc = kwargs.get("showhacc", False)
        self.stream = None
        self.connected = DISCONNECTED
        self.gnssData : GNSSData = GNSSData()
        self.gnssData.connected = DISCONNECTED
        self.gnssData.alt = 0
        self.gnssData.headAcc = 0
        self.gnssData.heading = 0
        self.gnssData.hAcc = 127
        self.gnssData.lat = 0
        self.gnssData.lon = 0
        self.gnssData.speed = 0
        self.gnssData.sAcc = 0

    def __enter__(self):
        """
        Context manager enter routine.
        """

        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        """
        Context manager exit routine.

        Terminates app in an orderly fashion.
        """

        self.stop()

    def run(self):
        """
        Run GNSS reader/writer.
        """

        self.enable_ubx(self.enableubx)

        self.stream = Serial(self.port, self.baudrate, timeout=self.timeout)
        self.connected = CONNECTED
        self.stopevent.clear()

        read_thread = Thread(
            target=self._read_loop,
            args=(
                self.stream,
                self.stopevent,
                self.recvqueue,
                self.sendqueue,
            ),
            daemon=True,
        )
        read_thread.start()

    def stop(self):
        """
        Stop GNSS reader/writer.
        """

        self.stopevent.set()
        self.connected = DISCONNECTED
        if self.stream is not None:
            self.stream.close()

    def _read_loop(
        self, stream: Serial, stopevent: Event, recvqueue: Queue, sendqueue: Queue
    ):
        """
        THREADED
        Reads and parses incoming GNSS data from the receiver,
        and sends any queued output data to the receiver.

        :param Serial stream: serial stream
        :param Event stopevent: stop event
        :param Queue recvqueue: queue for messages from receiver
        :param Queue sendqueue: queue for messages to send to receiver
        """

        ubr = UBXReader(
            stream, protfilter=(NMEA_PROTOCOL | UBX_PROTOCOL | RTCM3_PROTOCOL)
        )
        while not stopevent.is_set():
            try:
                if stream.in_waiting:
                    raw_data, parsed_data = ubr.read()
                    
                    if parsed_data:
                        # extract current navigation solution
                        self._extract_coordinates(parsed_data)

                        if recvqueue is None:
                            
                            #print data on stdout
                            op = (
                                "GNSS>> " + parsed_data.identity
                                if self.idonly
                                else parsed_data
                            )
                            #print(op)
                        else:
                            # place data on receive queue
                            recvqueue.put((raw_data, parsed_data))

                # send any queued output data to receiver
                self._send_data(ubr.datastream, sendqueue)

            except (
                UBXMessageError,
                UBXParseError,
                NMEAMessageError,
                NMEAParseError,
                RTCMMessageError,
                RTCMParseError,
            ) as err:
                print(f"Error parsing data stream {err}")
                continue

    def _extract_coordinates(self, parsed_data: object):
        """
        Extract current navigation solution from NMEA or UBX message.

        :param object parsed_data: parsed NMEA or UBX navigation message
        """
        #print(parsed_data.__dict__)
        if hasattr(parsed_data, "lat"):
            self.lat = parsed_data.lat
            self.gnssData.lat = int(parsed_data.lat * 10_000_000)
        if hasattr(parsed_data, "lon"):
            self.lon = parsed_data.lon
            self.gnssData.lon = int(parsed_data.lon * 10_000_000)
        if hasattr(parsed_data, "alt"):
            self.alt = parsed_data.alt
            self.gnssData.alt = int(parsed_data.alt)
        if hasattr(parsed_data, "hMSL"):  # UBX hMSL is in mm. -> cm for ITS
            self.alt = parsed_data.hMSL / 10
            self.alt = parsed_data.hMSL / 10
        if hasattr(parsed_data, "sep"):
            self.sep = parsed_data.sep
        if hasattr(parsed_data, "hMSL") and hasattr(parsed_data, "height"):
            self.sep = (parsed_data.height - parsed_data.hMSL) / 10
        if hasattr(parsed_data, "gSpeed"):
            self.speed = parsed_data.gSpeed
            self.gnssData.speed = parsed_data.gSpeed
        if hasattr(parsed_data, "sAcc"):
            self.sAcc = parsed_data.sAcc
            self.gnssData.sAcc = parsed_data.sAcc
        if hasattr(parsed_data, "hAcc"):  # UBX hAcc is in mm
            unit = 1 if parsed_data.identity == "PUBX00" else 1000
            self.gnssData.horizontalAcc = parsed_data.hAcc / unit
        if hasattr(parsed_data, "heading"):
            self.heading = parsed_data.heading 
            self.gnssData.heading = int(parsed_data.heading * 10000)
        if hasattr(parsed_data, "headVeh"):
            self.heading = parsed_data.headVeh
            self.gnssData.heading = int(parsed_data.headVeh * 10000)
        if hasattr(parsed_data, "headAcc"):
            self.headAcc = parsed_data.headAcc
            self.gnssData.headAcc = parsed_data.headAcc
        if hasattr(parsed_data, "hAcc"):
            self.hAcc = parsed_data.hAcc
            self.gnssData.hAcc = parsed_data.hAcc
    def _send_data(self, stream: Serial, sendqueue: Queue):
        """
        Send any queued output data to receiver.
        Queue data is tuple of (raw_data, parsed_data).

        :param Serial stream: serial stream
        :param Queue sendqueue: queue for messages to send to receiver
        """

        if sendqueue is not None:
            try:
                while not sendqueue.empty():
                    data = sendqueue.get(False)
                    raw, parsed = data
                    source = "NTRIP" if isinstance(parsed, RTCMMessage) else "GNSS"
                    # if self.idonly:
                    #     print(f"{source}<< {parsed.identity}")
                    # else:
                    #     print(parsed)
                    stream.write(raw)
                    sendqueue.task_done()
            except Empty:
                pass

    def enable_ubx(self, enable: bool):
        """
        Enable UBX output and suppress NMEA.

        :param bool enable: enable UBX and suppress NMEA output
        """

        layers = 1
        transaction = 0
        cfg_data = []
        for port_type in ("USB", "UART1"):
            cfg_data.append((f"CFG_{port_type}OUTPROT_NMEA", not enable))
            cfg_data.append((f"CFG_{port_type}OUTPROT_UBX", enable))
            cfg_data.append((f"CFG_MSGOUT_UBX_NAV_PVT_{port_type}", enable))
            cfg_data.append((f"CFG_MSGOUT_UBX_NAV_SAT_{port_type}", enable * 4))
            cfg_data.append((f"CFG_MSGOUT_UBX_NAV_DOP_{port_type}", enable * 4))
            cfg_data.append((f"CFG_MSGOUT_UBX_RXM_RTCM_{port_type}", enable))

        msg = UBXMessage.config_set(layers, transaction, cfg_data)
        self.sendqueue.put((msg.serialize(), msg))

    def get_coordinates(self) -> tuple:
        """
        Return current receiver navigation solution.
        (method needed by certain pygnssutils classes)

        :return: tuple of (connection status, lat, lon, alt and sep)
        :rtype: tuple
        """

        return (self.connected, self.lat, self.lon, self.alt, self.sep)

    def getSpeed(self) -> float:
        return self.speed
    
    def getGNSSData(self) -> GNSSData:
        return self.gnssData
        

if __name__ == "__main__":
    arp = ArgumentParser(
        formatter_class=ArgumentDefaultsHelpFormatter,
    )
    arp.add_argument(
        "-P", "--port", required=False, help="Serial port", default="/dev/ttyACM1"
    )
    arp.add_argument(
        "-B", "--baudrate", required=False, help="Baud rate", default=38400, type=int
    )
    arp.add_argument(
        "-T", "--timeout", required=False, help="Timeout in secs", default=3, type=float
    )

    args = arp.parse_args()
    recv_queue = Queue()  # set to None to print data to stdout
    send_queue = Queue()
    stop_event = Event()

    try:
        print("Starting GNSS reader/writer...\n")
        with GNSSSkeletonApp(
            args.port,
            int(args.baudrate),
            float(args.timeout),
            stop_event,
            recvqueue=recv_queue,
            sendqueue=send_queue,
            idonly=False,
            enableubx=True,
            showhacc=True,
        ) as gna:
            gna.run()
            while True:
                sleep(1)

    except KeyboardInterrupt:
        stop_event.set()
        print("Terminated by user")