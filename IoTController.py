# -*- coding: utf-8 -*-
"""
Created on Tue Jan 18 20:29:16 2022

@author: 39340
"""
from itertools import count
from pypref import Preferences
import paho.mqtt.client as mqtt
import json
import requests
import websockets
import websockets.exceptions as wsExceptions
import asyncio
import sys
import threading
from requests.structures import CaseInsensitiveDict
import time
MAX_TIMES_TEMP_WRITER_NOT_SEEN = 3
thingsConnectionTries = dict()
things = set() 
MAX_THING_CONNECTION_TRIES = 3
effectiveMaxTemp = None
actuatorThingLedUrl = None
tempReadingThreadRunning = False
mainThreadHandle = None
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
THING_DESCRIPTION_TOPIC = "/devices/mobile"
ACTUATOR_THING_DESCRIPTION_TOPIC = "/devices/actuators/ac"
MAX_TRIES = 5
async def mainThread():
    #global tempReadingThreadRunning
    counterTempWriterNotSeen = 0
    global actuatorOn
    global actuatorThingLedUrl
    global things
    actuatorOn = False
    timeout = False
    while True:
        if timeout:
            break
        while True:
            if stopThread:
                break;

            canExecAsTempWriter = False
            tempWriterPreviouslyFound = False
            try:
                for websocketUrl in things:
                    async with websockets.connect(websocketUrl) as websocket:
                        print("\n")
                        tempFromThingMessage = await websocket.recv()
                        tempFromThingDict = json.loads(str(tempFromThingMessage))

                        tempFromThingFloat = float(tempFromThingDict["data"]["temp"])
                        if tempFromThingFloat != -1 and not tempWriterPreviouslyFound:
                            canExecAsTempWriter = True
                            tempWriterPreviouslyFound = True

                        if canExecAsTempWriter:
                            # Pub temp for it to be visible by the reader thing
                            client.publish(TEMP_VALUE_TOPIC,
                                        str(round(tempFromThingFloat,2)) + " C")

                            print("<< Temperature from thing("+ websocketUrl+"): " + str(tempFromThingFloat))

                        maxTempFromThingFloat = None
                        if "maxTempPreference" in tempFromThingDict["data"]:
                            maxTempFromThingFloat = float(tempFromThingDict["data"]["maxTempPreference"])
                            maxTempPreferences[str(websocketUrl)] = maxTempFromThingFloat

                        global effectiveMaxTemp
                        values = [v for v in list(maxTempPreferences.values()) if v is not None]
                        if len(values) == 0:
                            effectiveMaxTemp = None
                        else:
                            effectiveMaxTemp = int(sum(values)/len(values))

                        if maxTempFromThingFloat is not None:
                            print("Preferred max temp from thing("+
                                websocketUrl+"): " + str(maxTempFromThingFloat))

                        print("Effective max temp: " + str(effectiveMaxTemp))

                        if canExecAsTempWriter and actuatorThingLedUrl is not None and effectiveMaxTemp is not None:
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
                        canExecAsTempWriter = False
                    thingsConnectionTries[websocketUrl] = MAX_THING_CONNECTION_TRIES 
            except requests.exceptions.RequestException:
                print("Actuator not reachable.")
            except Exception:
                print("Thing '" + websocketUrl +"' not reachable.")
                thingsConnectionTries[websocketUrl]-=1;
                print("Thing '" + websocketUrl +"': " + str(thingsConnectionTries[websocketUrl]) + " connection attempts remaining.")
                if thingsConnectionTries[websocketUrl] == 0:
                    things.remove(str(websocketUrl))
                    thingsConnectionTries.pop(websocketUrl)
                    print("Thing '" + websocketUrl +"' removed from directory. In order to re-enabling this Thing, its re-registration is required.")
                    if str(websocketUrl) in maxTempPreferences:
                        maxTempPreferences.pop(str(websocketUrl))
            await asyncio.sleep(3)
            if not tempWriterPreviouslyFound:
                counterTempWriterNotSeen+=1
            else:
                counterTempWriterNotSeen = 0;
            
            if counterTempWriterNotSeen > MAX_TIMES_TEMP_WRITER_NOT_SEEN:
                print("Temperature not available for too long.")
                deactivateActuator(actuatorThingLedUrl, headers)
                counterTempWriterNotSeen=0;

def deactivateActuator(actuatorThingLedUrl, headers):
    if actuatorThingLedUrl is not None:
        try:
            resp = requests.put(actuatorThingLedUrl,
                        headers=headers,
                        data=dataDeactivate)
                        
            global actuatorOn
            actuatorOn = False
            print("Activation status code: ", resp.status_code)
        except Exception:
            print("ATTENTION: actuator not reachable")

        print("Actuator Deactivated")

def on_connect(client, userdata, flags, rc):  # The callback for when the client connects to the broker
    print("Connected to mqtt broker with result code {0}".format(str(rc)))  # Print result of connection attempt
    client.subscribe("/devices/#")  # Subscribe to the topic of the temperature, receive any messages published on it

def on_message_temp_thing(client, userdata, msg):  # The callback for when a PUBLISH message is received from the server.
    sub_message = json.loads(msg.payload)
    websocketUrl = "ws://"+ sub_message["IP"]+ sub_message["modelURI"]
    if(sub_message["name"] == "tempSensor" and str(websocketUrl) not in things):
        things.add(str(websocketUrl)) 
        thingsConnectionTries[websocketUrl] = MAX_THING_CONNECTION_TRIES
        print("Thing registered: " + websocketUrl)


    

def on_message_actuator(client, userdata, msg):
    print("Actuator registered")
    sub_message = json.loads(msg.payload)
    global actuatorThingLedUrl
    actuatorThingLedUrl = "http://"+sub_message["IP"] + sub_message["modelURI"]+ "/properties/on"

    print("Actuator led url: " + actuatorThingLedUrl)

    deactivateActuator(actuatorThingLedUrl, headers)

def runMainThread():
    mainThreadHandle = threading.Thread(target=asyncio.run, args=
                                (mainThread(),))
    mainThreadHandle.start()

def on_message(client, userdata, msg):
    if msg.topic == THING_DESCRIPTION_TOPIC:
        on_message_temp_thing(client, userdata, msg)
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

runMainThread()

















