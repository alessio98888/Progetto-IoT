# -*- coding: utf-8 -*-
"""
Created on Tue Jan 18 20:29:16 2022

@author: 39340
"""
from pypref import Preferences
import paho.mqtt.client as mqtt
import json
import requests
import websockets
import asyncio
import sys
import threading
from requests.structures import CaseInsensitiveDict
import time

effectiveMaxTemp = None
actuatorThingLedUrl = None
tempReadingThreadRunning = False
tempReadingThread = None
stopThread = False
maxTempPreferences = dict()
mutex = threading.Lock()
headers = CaseInsensitiveDict()
headers["Content-Type"] = "application/json"
dataActivate = '{"on":true}'
dataDeactivate = '{"on":false}'
writers = dict()
actuatorOn = False
TEMP_VALUE_TOPIC = "/devices/mobile/sensors/temp/value"
TEMP_WRITER_THING_DESCRIPTION_TOPIC = "/devices/mobile/tempWriter"
TEMP_READER_THING_DESCRIPTION_TOPIC = "/devices/mobile/tempReader"
ACTUATOR_THING_DESCRIPTION_TOPIC = "/devices/actuators/ac"
MAX_TRIES = 5
async def getPropertiesFromTempThing(websocketUrl):
    #global tempReadingThreadRunning
    #tempReadingThreadRunning = True
    global actuatorOn
    actuatorOn = False
    max_tries = MAX_TRIES
    timeout = False
    while True:
        if timeout:
            break
        global stopThread
        if stopThread:
            print("Thread temp killed")
            stopThread = False
            break;
        print("Connecting to websocket: " + websocketUrl)
        try:
            while True:
                if stopThread:
                    break;

                writer = writers[websocketUrl]

                async with websockets.connect(websocketUrl) as websocket:
                    print("\n")
                    tempFromThingMessage = await websocket.recv()
                    tempFromThingDict = json.loads(str(tempFromThingMessage))
                    #print(tempFromThingDict)
                    if writer:
                        tempFromThingFloat = float(tempFromThingDict["data"]["temp"])
                        # Pub temp for it to be visible by the reader thing
                        client.publish(TEMP_VALUE_TOPIC,
                                       str(round(tempFromThingFloat,2)) + " C")

                    maxTempFromThingFloat = None
                    if "maxTempPreference" in tempFromThingDict["data"]:
                        maxTempFromThingFloat = float(tempFromThingDict["data"]["maxTempPreference"])
                        maxTempPreferences[str(websocketUrl)] = maxTempFromThingFloat

                    mutex.acquire()
                    global effectiveMaxTemp
                    values = [v for v in list(maxTempPreferences.values()) if v is not None]
                    if len(values) == 0:
                        effectiveMaxTemp = None
                    else:
                        effectiveMaxTemp = int(sum(values)/len(values))
                    mutex.release()

                    if maxTempFromThingFloat is not None:
                        print("Preferred max temp from thing("+
                            websocketUrl+"): " + str(maxTempFromThingFloat))

                    print("Effective max temp: " + str(effectiveMaxTemp))
                    if writer:
                        print("<< Temperature from thing: " + str(tempFromThingFloat))

                    if writer and actuatorThingLedUrl is not None and effectiveMaxTemp is not None:
                        if effectiveMaxTemp < tempFromThingFloat and not actuatorOn:
                            print("Actuator Activated")
                            resp = requests.put(actuatorThingLedUrl,
                                            headers=headers,
                                            data=dataActivate)
                            print("Activation status code: ", resp.status_code)
                            actuatorOn = True
                        elif actuatorOn and tempFromThingFloat < effectiveMaxTemp:
                            print("Actuator Deactivated")
                            resp = requests.put(actuatorThingLedUrl,
                                        headers=headers,
                                        data=dataDeactivate)
                            actuatorOn = False
                            print("Activation status code: ", resp.status_code)

                    await asyncio.sleep(10)
        except:
            print("WEBSOCKET ERROR: " + websocketUrl)

            if writer:
                try:
                    deactivateActuator(actuatorThingLedUrl, headers, dataDeactivate)
                except:
                    print("Actuator not reachable")
                    continue

            max_tries -= 1
            if max_tries == 0:
                print(websocketUrl + " TIMEOUT")
                maxTempPreferences.pop(str(websocketUrl))
                writers.pop(websocketUrl)
                timeout = True
                break

            print(sys.exc_info()[0])
            print("Waiting 5 seconds and retrying")
            await asyncio.sleep(5)

def deactivateActuator(actuatorThingLedUrl, headers, dataDeactivate):
    if actuatorThingLedUrl is not None:
        try:
            resp = requests.put(actuatorThingLedUrl,
                        headers=headers,
                        data=dataDeactivate)
                        
            global actuatorOn
            actuatorOn = False
            print("Activation status code: ", resp.status_code)
        except:
            print("Actuator not reachable")

        print("Actuator Deactivated")

def on_connect(client, userdata, flags, rc):  # The callback for when the client connects to the broker
    print("Connected to mqtt broker with result code {0}".format(str(rc)))  # Print result of connection attempt
    client.subscribe("/devices/#")  # Subscribe to the topic of the temperature, receive any messages published on it

def on_message_temp_thing(client, userdata, msg, writer):  # The callback for when a PUBLISH message is received from the server.
    sub_message = json.loads(msg.payload)
    tempSensorThingUrl = "http://"+sub_message["IP"]+ sub_message["modelURI"]

    if writer:
        print("Writer temperature sensor registered: " + tempSensorThingUrl)
    else:
        print("Reader temperature sensor registered: " + tempSensorThingUrl)


    if(sub_message["name"] == "tempSensor"):
        #asyncio.get_event_loop().run_until_complete(
            #getTempFromThing("ws://"+ sub_message["IP"]+ sub_message["modelURI"]))
        #asyncio.run(getTempFromThing("ws://"+ sub_message["IP"]+ sub_message["modelURI"]))
        #global websocketUrl
        #websocketUrl = "ws://"+ "weathersensor" + sub_message["modelURI"]
        websocketUrl = "ws://"+ sub_message["IP"]+ sub_message["modelURI"]

        readTempThingAlreadyRunning = False

        if websocketUrl in maxTempPreferences:
            readTempThingAlreadyRunning = True
        else:
            maxTempPreferences[str(websocketUrl)] = None


        writers[websocketUrl] = writer

        #global tempReadingThreadRunning
        #tempReadingThreadRunning = False
        #global stopThread
        #stopThread = True
        #threadToKill =
        #print("Killing old thread temp")
        #time.sleep(10)
        if not readTempThingAlreadyRunning:
            print("Start temp thing thread, writer = " + str(writer))
            runThingTempReader(websocketUrl)

def on_message_actuator(client, userdata, msg):
    print("Actuator registered")
    sub_message = json.loads(msg.payload)
    global actuatorThingLedUrl
    actuatorThingLedUrl = "http://"+sub_message["IP"] + sub_message["modelURI"]+ "/properties/on"

    print("Actuator led url: " + actuatorThingLedUrl)

    deactivateActuator(actuatorThingLedUrl, headers, dataDeactivate)

def runThingTempReader(websocketUrl):
    tempReadingThread = threading.Thread(target=asyncio.run, args=
                                (getPropertiesFromTempThing(websocketUrl),))
    tempReadingThread.start()

def on_message(client, userdata, msg):
    #print(msg.topic+" "+str(msg.qos)+" "+str(msg.payload))
    if msg.topic == TEMP_WRITER_THING_DESCRIPTION_TOPIC:
        on_message_temp_thing(client, userdata, msg, True)
    elif msg.topic == TEMP_READER_THING_DESCRIPTION_TOPIC:
        on_message_temp_thing(client, userdata, msg, False)
    elif msg.topic == ACTUATOR_THING_DESCRIPTION_TOPIC:
        on_message_actuator(client, userdata, msg)
    else:
        pass

client = mqtt.Client("iotMqttClient")
client.on_connect = on_connect  # Define callback function for successful connection
client.on_message = on_message
client.connect('127.0.0.1', 1883)

def thread():
    client.loop_forever()

threading.Thread(target=thread).start()


















