#codigo puente de hivemq para pubsub o traductor

import paho.mqtt.client as mqtt
from google.cloud import pubsub_v1
import logging
import time


logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


PROJECT_ID = "proyect-aura"

PUBSUB_TOPIC_SCAN = "rover-telemetry-scan"
PUBSUB_TOPIC_ENV = "rover-telemetry-environment"

PUBSUB_SUBSCRIPTION_COMMANDS = "comandos-rover-sub"
PUBSUB_SUBSCRIPTION_MODE = "modo-rover-sub"

#hivemq 
HIVEMQ_HOST = "7fe8be268dc24603acb3408e30de575d.s1.eu.hivemq.cloud"
HIVEMQ_PORT = 8883
HIVEMQ_USER = "Proyect_Aura"
HIVEMQ_PASSWORD = "Test_test1"

HIVEMQ_TOPICS_FROM_ROVER = [
    ("rover/telemetry/scan", 0),
    ("rover/telemetry/environment", 0)
]
#topicos
HIVEMQ_TOPIC_COMMANDS = "rover/commands"
HIVEMQ_TOPIC_MODE = "rover/mode/set"

#topicos mapeados de hivemq a pubsub
HIVEMQ_TO_PUBSUB_MAP = {
    "rover/telemetry/scan": PUBSUB_TOPIC_SCAN,
    "rover/telemetry/environment": PUBSUB_TOPIC_ENV
}
PUBSUB_TO_HIVEMQ_MAP = {
    PUBSUB_SUBSCRIPTION_COMMANDS: HIVEMQ_TOPIC_COMMANDS,
    PUBSUB_SUBSCRIPTION_MODE: HIVEMQ_TOPIC_MODE
}
#clientes
publisher = pubsub_v1.PublisherClient()
subscriber = pubsub_v1.SubscriberClient()
mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
def on_connect(client, userdata, flags, rc, properties=None):
    """Callback para cuando nos conectamos a HiveMQ."""
    if rc == 0:
        logging.info("Conectado exitosamente a HiveMQ.")
        # topicos de rovr
        client.subscribe(HIVEMQ_TOPICS_FROM_ROVER)
        logging.info(f"Suscrito a los tópicos de HiveMQ: {HIVEMQ_TOPICS_FROM_ROVER}")
    else:
        logging.error(f"Fallo al conectar a HiveMQ, código de error: {rc}")

def on_message(client, userdata, msg):
    """Callback para cuando recibimos un mensaje del ROVER vía HiveMQ."""
    try:
        pubsub_topic_name = HIVEMQ_TO_PUBSUB_MAP.get(msg.topic)
        if pubsub_topic_name:
            payload = msg.payload
            logging.info(f"Recibido de HiveMQ (tópico: {msg.topic}) -> Reenviando a Pub/Sub (tópico: {pubsub_topic_name})")
            
            topic_path = publisher.topic_path(PROJECT_ID, pubsub_topic_name)
            future = publisher.publish(topic_path, payload)
            future.result() 
        else:
            logging.warning(f"Recibido mensaje en tópico no mapeado de HiveMQ: {msg.topic}")
    except Exception as e:
        logging.error(f"Error procesando mensaje de HiveMQ: {e}")

def pubsub_callback(message):
    """Callback para cuando recibimos un mensaje de la APLICACIÓN WEB vía Google Pub/Sub."""
    try:
        subscription_name = message.subscription.split("/")[-1]
        hivemq_topic = PUBSUB_TO_HIVEMQ_MAP.get(subscription_name)
        
        if hivemq_topic:
            payload = message.data
            logging.info(f"Recibido de Pub/Sub (suscripción: {subscription_name}) -> Reenviando a HiveMQ (tópico: {hivemq_topic})")
            
            mqtt_client.publish(hivemq_topic, payload)
            message.ack()  
        else:
            logging.warning(f"Recibido mensaje de suscripción no mapeada de Pub/Sub: {subscription_name}")
            message.nack() 
    except Exception as e:
        logging.error(f"Error procesando mensaje de Pub/Sub: {e}")
        message.nack()


def main():
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    mqtt_client.username_pw_set(HIVEMQ_USER, HIVEMQ_PASSWORD)
    mqtt_client.tls_set()
    mqtt_client.connect(HIVEMQ_HOST, HIVEMQ_PORT)
    
    mqtt_client.loop_start()

    subscription_path_commands = subscriber.subscription_path(PROJECT_ID, PUBSUB_SUBSCRIPTION_COMMANDS)
    subscription_path_mode = subscriber.subscription_path(PROJECT_ID, PUBSUB_SUBSCRIPTION_MODE)

    streaming_pull_future_commands = subscriber.subscribe(subscription_path_commands, callback=pubsub_callback)
    streaming_pull_future_mode = subscriber.subscribe(subscription_path_mode, callback=pubsub_callback)
    
    logging.info(f"Escuchando en la suscripción de Pub/Sub: {PUBSUB_SUBSCRIPTION_COMMANDS}")
    logging.info(f"Escuchando en la suscripción de Pub/Sub: {PUBSUB_SUBSCRIPTION_MODE}")

    try:
        while True:
            time.sleep(60)
    except KeyboardInterrupt:
        logging.info("Script detenido por el usuario. Cerrando conexiones...")
        streaming_pull_future_commands.cancel()
        streaming_pull_future_mode.cancel()
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        logging.info("Conexiones cerradas limpiamente.")

if __name__ == "__main__":
    main()