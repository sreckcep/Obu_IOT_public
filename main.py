from pygnssutils import GNSSNTRIPClient
from gnssapp_silent import GNSSSkeletonApp
from time import sleep
from queue import Queue
from threading import Event
import json
import datetime
from cam import *
import  paho.mqtt.client as mqtt
import logging

MQTT_HOST = '195.220.53.60' #'autocampus.fr'
MQTT_TOPIC = 'TestTopic/ITS/'
MQTT_USER = ''
MQTT_PASS = ''
MQTT_ID = "python-OBU"
MQTT_EXTERNAL_PORT = 10883
MQTT_INTERNAL_PORT = 1883

NTRIP_HOST = '92.245.148.38'
NTRIP_PORT = 2101
NTRIP_MOUNTPOINT = 'B612_RTK_CORR'


SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 38400
TIMEOUT = 10

ROVER_ID = 1
send_queue = Queue()  # data to receiver placed on this queue
stop_event = Event()
idonly = False
def main():
    logging.basicConfig(filename='logs/CAMS.log',
                    filemode='a',
                    format='%(asctime)s|%(msecs)d|%(name)s|%(levelname)s|%(message)s',
                    datefmt='%H:%M:%S',
                    level=logging.DEBUG)
    try:
        print(f"Starting GNSS reader/writer on {SERIAL_PORT} @ {BAUDRATE}...\n")
        with GNSSSkeletonApp(
            SERIAL_PORT,
            BAUDRATE,
            TIMEOUT,
            stopevent=stop_event,
            recvqueue=None,
            sendqueue=send_queue,
            enableubx=True,
            showhacc=False,
        ) as gna:
            gna.run()
            sleep(2)  # wait for receiver to output at least 1 navigation solution

            print(f"Starting NTRIP client on {NTRIP_HOST}:{NTRIP_PORT}...\n")
            with GNSSNTRIPClient(gna, logtofile=1, logpath='logs', verbosity=1) as gnc:
                streaming = gnc.run(
                    ipprot="IPv4",
                    server=NTRIP_HOST,
                    port=NTRIP_PORT,
                    mountpoint=NTRIP_MOUNTPOINT,
                    output=send_queue,  # send NTRIP data to receiver
                )
                while True:
                    try:
                        mqttClient = connect_mqtt()
                        break
                    
                    except KeyboardInterrupt:
                        stop_event.set()
                        print("Terminated by user")
                        exit()
                    except:
                        print("error while connectin to mqtt server. Reconnecting...")
                        sleep(1)
                while streaming and not stop_event.is_set():  # run until user presses CTRL-C
                    gnssData = gna.getGNSSData()
                    jsonCAM = getCAM(gnssData.lat, gnssData.lon, gnssData.alt, gnssData.hAcc, gnssData.speed, gnssData.sAcc, gnssData.heading, gnssData.headAcc)
                    logging.info(jsonCAM)
                    try:
                        status = mqttPublish(mqttClient, jsonCAM, MQTT_TOPIC)
                    except:
                        print("error while trying to send message. Reconnecting...")
                        sleep(1)
                        mqttClient = connect_mqtt()
                    sleep(1)
                    
                sleep(1)
    except KeyboardInterrupt:
        stop_event.set()
        mqttClient.disconnect()
        print("Terminated by user")
        


    

def mqttPublish(client: mqtt.Client, msg, topic):
    result = client.publish(topic, msg)
    status = result[0]
    if status == 0:
        print(f"Send `{msg}` to topic `{topic}`")
    else:
        print(f"Failed to send message to topic {topic}")
    return status
    

def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)
    
    client = mqtt.Client(client_id=MQTT_ID, callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    client.username_pw_set(MQTT_USER, MQTT_PASS)
    client.on_connect = on_connect
    client.connect(MQTT_HOST, MQTT_EXTERNAL_PORT)
    return client

def getCAM(lat: int, lon: int, alt: int, hAcc: int, speed: int, sAcc: int, heading: int, headAcc: int):    
    
    pdu = CAM()
    
    # pdu.itsPduHeader = ItsPduHeader()
    pdu.itsPduHeader.messageId = MessageID.cam
    pdu.itsPduHeader.protocolVersion = 1
    pdu.itsPduHeader.stationId = ROVER_ID

    # pdu.camPayload = CamPayload()
    # pdu.camPayload.camParameters = CamParameters()
    # pdu.camPayload.camParameters.basicContainer = BasicContainer()
    # pdu.camPayload.camParameters.basicContainer.referencePosition = ReferencePositionWithConfidence()
    pdu.camPayload.camParameters.basicContainer.referencePosition.alt = alt
    pdu.camPayload.camParameters.basicContainer.referencePosition.lat = lat
    pdu.camPayload.camParameters.basicContainer.referencePosition.lon = lon
    pdu.camPayload.camParameters.basicContainer.referencePosition.hAcc = hAcc
    pdu.camPayload.camParameters.basicContainer.trafficParticipantType = TrafficParticipantType.lightTruck

    # pdu.camPayload.camParameters.highFrequencyContainer = HighFrequencyContainer()
    pdu.camPayload.camParameters.highFrequencyContainer.speed.speedValue = speed
    pdu.camPayload.camParameters.highFrequencyContainer.speed.speedConfidence = sAcc
    
    pdu.camPayload.camParameters.highFrequencyContainer.heading.headingValue = heading
    pdu.camPayload.camParameters.highFrequencyContainer.heading.headingConfidence = headAcc
    

    delta = datetime.datetime.now(datetime.timezone.utc).microsecond // 1000
    pdu.camPayload.generationDeltaTime = delta
    
    dumped = json.dumps(pdu, cls=CustomEncoder)
    return dumped

class CustomEncoder(json.JSONEncoder):
    def default(self, obj):
        #if (type(obj) in [CAM, TrafficParticipantType, ReferencePositionWithConfidence, BasicContainer, HighFrequencyContainer, CamParameters, ItsPduHeader, CamPayload, MessageID]):
        try:
            return json.JSONEncoder.default(self, obj)
        except TypeError:
            return obj.__dict__

if __name__ == "__main__":
    main()
