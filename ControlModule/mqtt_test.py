import paho.mqtt.client as mqtt
import time

broker_address = "10.1.1.9"

# def on_connect(client, userdata, flags, rc):
#     print("Connected with result code " + str(rc))

# def on_message(client, userdata, msg):
#     print(msg.topic + " " + str(msg.payload))

# client = mqtt.Client()
# client.on_connect = open_connection
# client.on_message = on_message

# client.connect("mqtt.eclipseprojects.io", 1883, 60)

# client.loop_forever(timeout=1.0, max_packets=1, retry_first_connection=False)

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))


def on_message(client, userdata, message):
    print("message received: ", str(message.payload.decode("utf-8")))
    print("message topic: ", message.topic)
    print("message qos: ", message.qos)
    print("message retain flag: ", message.retain)

client = mqtt.Client(client_id="control")
client.on_message=on_message
# client.on_connect=on_connect
client.connect(broker_address)
# client.loop_start()
client.subscribe("testTopic")
num = 0
# while True:
#     num = num + 1
#     client.publish("Debug", "This is pi: " + str(num))
#     time.sleep(1)
# client.publish("testTopic", "hello from python")
# time.sleep(4)
# client.loop_stop()
client.publish("Lock", "")
client.loop_forever()