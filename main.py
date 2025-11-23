import asyncio
import json
import logging
import time
import websockets
import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
import cv2
import board
import busio
import socket
import serial
import adafruit_dht
import adafruit_bh1750
import smbus2

from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaStreamTrack
from picamera2 import Picamera2

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s') 
logger = logging.getLogger(__name__)

# Pines
GPIO.setwarnings(False)
MOTOR_TRASERO_IN1 = 17
MOTOR_TRASERO_IN2 = 27
MOTOR_TRASERO_ENA = 18
MOTOR_GIRO_IN3 = 23
MOTOR_GIRO_IN4 = 24
MOTOR_GIRO_ENB = 13
SERVO_PIN = 12
PIN_TRIGGER = 5
PIN_ECHO = 6
DHT22_PIN = board.D22
MQ2_PIN = 20
MQ135_PIN = 21

VELOCIDAD_MANUAL = 50
VELOCIDAD_AUTO = 32
UMBRAL_ABISMO_CM = 3000

# MQTT 
MQTT_TOPIC_COMMANDS = "rover/commands"
MQTT_TOPIC_MODE_SET = "rover/mode/set"
MQTT_TOPIC_TELEMETRY_SCAN = "rover/telemetry/scan"
MQTT_TOPIC_TELEMETRY_ENV = "rover/telemetry/environment"

HIVE_HOST = "7fe8be268dc24603acb3408e30de575d.s1.eu.hivemq.cloud"
HIVE_PORT = 8883
HIVE_USERNAME = "Proyect_Aura"
HIVE_PASSWORD = "Test_test1"
SIGNALING_SERVER_URL = "wss://streaming-server-969287216391.us-central1.run.app/ws"
APN = "internet.nuevatel.com"
TOF_I2C_ADDRESS = 0x52

class RoverState:
    def __init__(self):
        self.is_auto_mode = False
        self.latest_scan_data = []
        self.abismo_detectado = False
        self.connection_mode = None

rover_state = RoverState()

velocidad_motor = None
velocidad_giro = None
servo = None
dht_device = None
bh1750 = None
bus_for_tof = None

def read_bh1750_blocking():
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        sensor = adafruit_bh1750.BH1750(i2c)
        return round(sensor.lux, 2)
    except Exception:
        return None

def initialize_hardware():
    global velocidad_motor, velocidad_giro, servo, dht_device, bh1750, bus_for_tof
    
    GPIO.setmode(GPIO.BCM)
    # Motores y Servo
    GPIO.setup([MOTOR_TRASERO_IN1, MOTOR_TRASERO_IN2, MOTOR_TRASERO_ENA,
                MOTOR_GIRO_IN3, MOTOR_GIRO_IN4, MOTOR_GIRO_ENB,
                SERVO_PIN, PIN_TRIGGER], GPIO.OUT)
    # Sensores
    GPIO.setup([PIN_ECHO, MQ2_PIN, MQ135_PIN], GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    velocidad_motor = GPIO.PWM(MOTOR_TRASERO_ENA, 100)
    velocidad_giro = GPIO.PWM(MOTOR_GIRO_ENB, 100)
    velocidad_motor.start(0)
    velocidad_giro.start(0)
    
    servo = GPIO.PWM(SERVO_PIN, 50)
    servo.start(0)
    
    try:
        dht_device = adafruit_dht.DHT22(DHT22_PIN, use_pulseio=False)
        bus_for_tof = smbus2.SMBus(1)
        i2c_adafruit = busio.I2C(board.SCL, board.SDA)
        bh1750 = adafruit_bh1750.BH1750(i2c_adafruit)
    except Exception as e:
        logger.error(f"Fallo init sensores: {e}")

def move_forward(speed):
    GPIO.output(MOTOR_TRASERO_IN1, GPIO.HIGH); GPIO.output(MOTOR_TRASERO_IN2, GPIO.LOW)
    velocidad_motor.ChangeDutyCycle(speed)

def move_backward(speed):
    GPIO.output(MOTOR_TRASERO_IN1, GPIO.LOW); GPIO.output(MOTOR_TRASERO_IN2, GPIO.HIGH)
    velocidad_motor.ChangeDutyCycle(speed)

def stop_movement():
    GPIO.output(MOTOR_TRASERO_IN1, GPIO.LOW); GPIO.output(MOTOR_TRASERO_IN2, GPIO.LOW)
    velocidad_motor.ChangeDutyCycle(0)
    center()

def turn_left(speed):
    GPIO.output(MOTOR_GIRO_IN3, GPIO.LOW); GPIO.output(MOTOR_GIRO_IN4, GPIO.HIGH)
    velocidad_giro.ChangeDutyCycle(speed)

def turn_right(speed):
    GPIO.output(MOTOR_GIRO_IN3, GPIO.HIGH); GPIO.output(MOTOR_GIRO_IN4, GPIO.LOW)
    velocidad_giro.ChangeDutyCycle(speed)
    
def center():
    GPIO.output(MOTOR_GIRO_IN3, GPIO.LOW); GPIO.output(MOTOR_GIRO_IN4, GPIO.LOW)
    velocidad_giro.ChangeDutyCycle(0)

async def girar_derecha_especial(speed):
    # Maniobra evacion (derecha)
    turn_left(15); move_backward(55); await asyncio.sleep(1.8)
    turn_right(35); move_forward(50); await asyncio.sleep(1.5)
    turn_left(18); move_backward(55); await asyncio.sleep(1.6)
    turn_right(35); move_forward(50); await asyncio.sleep(1.5)
    turn_left(20); move_backward(55); await asyncio.sleep(1.7)
    turn_right(35); move_forward(50); await asyncio.sleep(1.5)
    turn_left(20); move_backward(55); await asyncio.sleep(1.7)
    turn_right(35); move_forward(50); await asyncio.sleep(1.5)
    turn_left(20); move_backward(55); await asyncio.sleep(1.6)
    turn_right(35); move_forward(50); await asyncio.sleep(1.5)
    stop_movement()

async def girar_izquierda_especial(speed):
    # Maniobra evacion (izquierda)
    turn_right(35); move_backward(54); await asyncio.sleep(1.5)
    turn_left(20); move_forward(50); await asyncio.sleep(1.5)
    turn_right(35); move_backward(54); await asyncio.sleep(1.5)
    turn_left(20); move_forward(50); await asyncio.sleep(1.5)
    turn_right(35); move_backward(53); await asyncio.sleep(1.5)
    turn_left(20); move_forward(50); await asyncio.sleep(1.5)
    turn_right(35); move_backward(53); await asyncio.sleep(1.5)
    turn_left(20); move_forward(50); await asyncio.sleep(1.5)
    turn_right(35); move_backward(52); await asyncio.sleep(1.5)
    turn_left(20); move_forward(50); await asyncio.sleep(1.5)
    turn_right(50); await asyncio.sleep(0.5)
    stop_movement()

async def perform_evasive_turn():
    scan = rover_state.latest_scan_data
    if not scan: return
    
    # Decidir lado basado en espacio libre
    left_dist = sum(item['distance'] for item in scan if item['angle'] > 90)
    right_dist = sum(item['distance'] for item in scan if item['angle'] < 90)
    
    if right_dist > left_dist:
        await girar_derecha_especial(VELOCIDAD_AUTO)
    else:
        await girar_izquierda_especial(VELOCIDAD_AUTO)

async def set_servo_angle(angle):
    duty = 2 + (angle / 18)
    servo.ChangeDutyCycle(duty)
    await asyncio.sleep(0.3)
    servo.ChangeDutyCycle(0)

async def read_tof_sensor():
    try:
        bus_for_tof.write_byte(TOF_I2C_ADDRESS, 0)
        await asyncio.sleep(0.05)
        data = bus_for_tof.read_i2c_block_data(TOF_I2C_ADDRESS, 0, 2)
        dist_mm = (data[0] << 8) | data[1]
        dist_cm = dist_mm / 10
        if 0 < dist_cm < 200:
            return round(dist_cm, 1)
        return None
    except Exception:
        return None

def read_ultrasonic_sensor():
    try:
        GPIO.output(PIN_TRIGGER, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(PIN_TRIGGER, GPIO.LOW)
        start_time, stop_time = time.time(), time.time()
        timeout = start_time + 0.1
        
        while GPIO.input(PIN_ECHO) == 0 and start_time < timeout:
            start_time = time.time()
        while GPIO.input(PIN_ECHO) == 1 and stop_time < timeout:
            stop_time = time.time()
            
        return ((stop_time - start_time) * 34300) / 2
    except Exception:
        return None

async def read_environment_sensors():
    env_data = {
        "temperature": None, 
        "humidity": None, 
        "light_lux": None, 
        "smoke_detected": GPIO.input(MQ2_PIN) == GPIO.LOW, 
        "bad_air_quality": GPIO.input(MQ135_PIN) == GPIO.LOW
    }
    loop = asyncio.get_running_loop()
    env_data["light_lux"] = await loop.run_in_executor(None, read_bh1750_blocking)
    
    # Reintentos DHT22
    for _ in range(3):
        try:
            await asyncio.sleep(0.5)
            t = dht_device.temperature
            h = dht_device.humidity
            if t is not None and h is not None:
                env_data["temperature"] = round(t, 1)
                env_data["humidity"] = round(h, 1)
                break
        except RuntimeError:
            continue
    return env_data

def check_internet_connection(host="8.8.8.8", port=53, timeout=3):
    try:
        socket.setdefaulttimeout(timeout)
        socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect((host, port))
        return True
    except socket.error:
        return False

def connect_via_sim800l(apn):
    logger.info("Conectando SIM800L...")
    ser = None
    try:
        ser = serial.Serial('/dev/serial0', 9600, timeout=2)
        commands = [
            ("AT+SAPBR=0,1", 1), 
            ("AT", 1), 
            ("AT+SAPBR=3,1,\"Contype\",\"GPRS\"", 1), 
            (f'AT+SAPBR=3,1,"APN","{apn}"', 1), 
            ("AT+SAPBR=1,1", 5), 
            ("AT+SAPBR=2,1", 2)
        ]
        
        for cmd, delay in commands:
            ser.write((cmd + '\r\n').encode())
            time.sleep(delay)
            response = ser.read_all().decode(errors='ignore')
            
            if cmd == "AT+SAPBR=2,1" and "+SAPBR: 1,1," not in response:
                ser.close(); return False
            elif "OK" not in response and cmd != "AT+SAPBR=2,1": # Validación simple
                ser.close(); return False
                
        ser.close()
        time.sleep(1)
        return check_internet_connection()
    except Exception:
        if ser and ser.is_open: ser.close()
        return False

def on_connect(client, userdata, flags, rc, properties=None):
    logger.info(f"Conectado MQTT: {rc}")
    client.subscribe(MQTT_TOPIC_COMMANDS)
    client.subscribe(MQTT_TOPIC_MODE_SET)

def on_message(client, userdata, msg):
    payload = msg.payload.decode()
    
    if msg.topic == MQTT_TOPIC_MODE_SET:
        rover_state.is_auto_mode = (payload == "auto")
        stop_movement()
        logger.info(f"Modo: {'AUTO' if rover_state.is_auto_mode else 'MANUAL'}")
        
    elif msg.topic == MQTT_TOPIC_COMMANDS:
        if rover_state.is_auto_mode: return
        if rover_state.abismo_detectado: return
        
        if payload == "forward": move_forward(VELOCIDAD_MANUAL)
        elif payload == "backward": move_backward(VELOCIDAD_MANUAL)
        elif payload == "left": turn_left(VELOCIDAD_MANUAL)
        elif payload == "right": turn_right(VELOCIDAD_MANUAL)
        elif payload == "center": center()
        elif payload == "stop": stop_movement()

class PiCameraTrack(MediaStreamTrack):
    kind = "video"
    def __init__(self, picam2):
        super().__init__()
        self.picam2 = picam2
    async def recv(self):
        from av import VideoFrame
        array = self.picam2.capture_array()
        frame_rgb = cv2.cvtColor(array, cv2.COLOR_RGBA2RGB)
        frame = VideoFrame.from_ndarray(frame_rgb, format="rgb24")
        return frame

async def webrtc_task(pc, websocket):
    while True:
        message_str = await websocket.recv()
        message = json.loads(message_str)
        if message["type"] == "answer":
            await pc.setRemoteDescription(RTCSessionDescription(sdp=message["sdp"], type=message["type"]))
        elif message["type"] == "ice-candidate" and message.get("candidate"):
            from aiortc.sdp import candidate_from_sdp
            candidate = candidate_from_sdp(message['candidate']['candidate'].split(":", 1)[1])
            candidate.sdpMid = message['candidate']['sdpMid']
            candidate.sdpMLineIndex = message['candidate']['sdpMLineIndex']
            await pc.addIceCandidate(candidate)

async def radar_scan_task(mqtt_client):
    while True:
        scan_results = []
        for angle in range(0, 181, 30):
            await set_servo_angle(angle)
            distance = await read_tof_sensor()
            if distance is not None:
                scan_results.append({"angle": angle, "distance": distance})
        rover_state.latest_scan_data = scan_results
        mqtt_client.publish(MQTT_TOPIC_TELEMETRY_SCAN, json.dumps(scan_results))
        await asyncio.sleep(1)

async def environment_task(mqtt_client):
    while True:
        env_data = await read_environment_sensors()
        mqtt_client.publish(MQTT_TOPIC_TELEMETRY_ENV, json.dumps(env_data))
        await asyncio.sleep(10)

async def cliff_detection_task():
    while True:
        distancia_suelo = read_ultrasonic_sensor()
        if distancia_suelo is not None and distancia_suelo > UMBRAL_ABISMO_CM:
            if not rover_state.abismo_detectado:
                logger.warning("Abismo detectado.")
                rover_state.abismo_detectado = True
                stop_movement()
                await perform_evasive_turn()
        else:
            if rover_state.abismo_detectado:
                rover_state.abismo_detectado = False
        await asyncio.sleep(0.2)

async def auto_mode_task():
    while True:
        if rover_state.is_auto_mode and not rover_state.abismo_detectado:
            scan = rover_state.latest_scan_data
            if not scan: 
                await asyncio.sleep(0.5)
                continue
                
            center_distance = next((item['distance'] for item in scan if item.get('angle') == 90), 200)
            
            if center_distance < 70:
                stop_movement()
                await asyncio.sleep(0.5)
                await perform_evasive_turn()
            else:
                move_forward(VELOCIDAD_AUTO)
        await asyncio.sleep(0.5)

async def main():
    initialize_hardware()
    
    if check_internet_connection():
        rover_state.connection_mode = 'wifi'
        logger.info("Modo: WIFI")
    elif connect_via_sim800l(APN):
        rover_state.connection_mode = 'cellular'
        logger.info("Modo: CELULAR")
    else:
        logger.error("Sin conexión a internet")
        GPIO.cleanup()
        return

    # Cliente MQTT
    mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    mqtt_client.username_pw_set(HIVE_USERNAME, HIVE_PASSWORD)
    mqtt_client.tls_set()
    mqtt_client.connect(HIVE_HOST, HIVE_PORT, 60)
    mqtt_client.loop_start()

    tasks_to_run = [radar_scan_task(mqtt_client), environment_task(mqtt_client), cliff_detection_task(), auto_mode_task()]
    pc, picam2 = None, None

    # Configuración de Video (Solo WiFi)
    if rover_state.connection_mode == 'wifi':
        try:
            picam2 = Picamera2()
            config = picam2.create_video_configuration(main={"size": (640, 480)})
            picam2.configure(config)
            picam2.start()
            time.sleep(1)
            
            pc = RTCPeerConnection()
            pc.addTrack(PiCameraTrack(picam2))
            websocket = await websockets.connect(SIGNALING_SERVER_URL)
            offer = await pc.createOffer()
            await pc.setLocalDescription(offer)
            await websocket.send(json.dumps({"type": "offer", "sdp": pc.localDescription.sdp}))
            tasks_to_run.append(webrtc_task(pc, websocket))
            logger.info("Video streaming activo")
        except Exception as e:
            logger.error(f"Error video: {e}")
            if pc: await pc.close()
            if picam2: picam2.stop()
    
    try:
        await asyncio.gather(*tasks_to_run)
    except Exception as e:
        logger.error(f"Error en loop principal: {e}")
    finally:
        if pc: await pc.close()
        if picam2: picam2.stop()
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        GPIO.cleanup()

if __name__ == "__main__":
    asyncio.run(main())
