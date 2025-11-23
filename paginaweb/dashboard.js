const MQTT_BROKER_URL = 'wss://7fe8be268dc24603acb3408e30de575d.s1.eu.hivemq.cloud:8884/mqtt';
const MQTT_USERNAME = 'Proyect_Aura';
const MQTT_PASSWORD = 'Test_test1';
const SIGNALING_SERVER_URL = 'wss://streaming-server-969287216391.us-central1.run.app/ws';

const MAX_RADAR_DISTANCE = 180;

document.addEventListener('DOMContentLoaded', () => {
    const mqttStatus = document.getElementById('mqtt-status');
    const videoStatus = document.getElementById('video-status');
    const modeToggle = document.getElementById('mode-toggle');
    const manualControlsSection = document.getElementById('manual-controls-section');
    const autoModeSection = document.getElementById('auto-mode-section');
    
    const client = mqtt.connect(MQTT_BROKER_URL, {
        username: MQTT_USERNAME,
        password: MQTT_PASSWORD,
    });

    client.on('connect', () => {
        mqttStatus.textContent = 'Conectado';
        mqttStatus.style.color = '#53bf9d';
        client.subscribe('rover/telemetry/scan', (err) => !err && console.log('Suscrito a scan'));
        client.subscribe('rover/telemetry/environment', (err) => !err && console.log('Suscrito a environment'));
    });

    client.on('error', (err) => {
        mqttStatus.textContent = `Error: ${err.message}`;
        mqttStatus.style.color = '#e94560';
    });
    
    client.on('message', (topic, message) => {
        try {
            if (topic === 'rover/telemetry/scan') {
                updateRadar(JSON.parse(message.toString()));
            }
            if (topic === 'rover/telemetry/environment') {
                updateEnvironmentUI(JSON.parse(message.toString()));
            }
        } catch (e) {
            console.error("Error al procesar mensaje MQTT:", e);
        }
    });

    modeToggle.addEventListener('change', (event) => {
        const isAutoMode = event.target.checked;
        const mode = isAutoMode ? 'auto' : 'manual';
        
        client.publish('rover/mode/set', mode);
        console.log(`Modo cambiado a: ${mode}`);
        
        manualControlsSection.style.display = isAutoMode ? 'none' : '';
        autoModeSection.style.display = isAutoMode ? '' : 'none';
    });

    const buttons = {
    'btn-forward': 'forward',
    'btn-backward': 'backward',
    'btn-left': 'left',
    'btn-right': 'right',
    'btn-center': 'center', 
    'btn-stop': 'stop'
    };
    for (const [id, command] of Object.entries(buttons)) {
        document.getElementById(id)?.addEventListener('click', () => {
            client.publish('rover/commands', command);
        });
    }
    const canvas = document.getElementById('radar-canvas');
    const ctx = canvas.getContext('2d');
    let latestScanData = []; 

    function updateRadar(scanData) { latestScanData = scanData; }

    function drawRadar() {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        const centerX = canvas.width / 2;
        const centerY = canvas.height - 20;
        const radius = canvas.height - 40;
        ctx.strokeStyle = 'rgba(91, 192, 190, 0.5)';
        ctx.beginPath();
        ctx.arc(centerX, centerY, radius, Math.PI, 0);
        ctx.stroke();
        for (let i = 1; i <= 3; i++) {
            ctx.beginPath();
            ctx.arc(centerX, centerY, radius * (i / 3), Math.PI, 0);
            ctx.stroke();
        }
        for (let i = 0; i <= 180; i += 30) {
            const angleRad = i * Math.PI / 180;
            ctx.beginPath();
            ctx.moveTo(centerX, centerY);
            ctx.lineTo(centerX - radius * Math.cos(angleRad), centerY - radius * Math.sin(angleRad));
            ctx.stroke();
        }
        ctx.beginPath();
        ctx.arc(centerX, centerY, 8, 0, 2 * Math.PI);
        ctx.fillStyle = '#e94560';
        ctx.fill();
        latestScanData.forEach(point => {
            if (point.distance && point.distance < MAX_RADAR_DISTANCE) {
                const distanceRatio = point.distance / MAX_RADAR_DISTANCE;
                const angleRad = point.angle * Math.PI / 180;
                const x = centerX - (radius * distanceRatio) * Math.cos(angleRad);
                const y = centerY - (radius * distanceRatio) * Math.sin(angleRad);
                ctx.beginPath();
                ctx.arc(x, y, 4, 0, 2 * Math.PI);
                ctx.fillStyle = '#f9a826';
                ctx.fill();
            }
        });
        requestAnimationFrame(drawRadar);
    }
    drawRadar();

    function updateEnvironmentUI(data) {
        document.getElementById('temp-data').textContent = data.temperature?.toFixed(1) ?? '--';
        document.getElementById('humidity-data').textContent = data.humidity?.toFixed(1) ?? '--';
        document.getElementById('air-quality-data').textContent = data.bad_air_quality ? 'Mala' : 'Buena';
        document.getElementById('smoke-data').textContent = data.smoke_detected ? 'Detectado' : 'Normal';
        document.getElementById('light-data').textContent = data.light_lux?.toFixed(0) ?? '--';
    }

    let pc; let ws;

    function startVideo() {
        videoStatus.textContent = 'Conectando...';
        pc = new RTCPeerConnection({ sdpSemantics: 'unified-plan' });
        pc.ontrack = (event) => {
            document.getElementById('remote-video').srcObject = event.streams[0];
            videoStatus.textContent = 'Conectado';
            videoStatus.style.color = '#53bf9d';
        };
        pc.oniceconnectionstatechange = () => {
            if (pc.iceConnectionState === 'failed' || pc.iceConnectionState === 'disconnected' || pc.iceConnectionState === 'closed') {
                videoStatus.textContent = 'Desconectado';
                videoStatus.style.color = '#e94560';
            }
        };
        ws = new WebSocket(SIGNALING_SERVER_URL);
        ws.onopen = () => { videoStatus.textContent = 'Negociando...'; };
        ws.onmessage = async (event) => {
            const message = JSON.parse(event.data);
            if (message.type === 'offer') {
                try {
                    await pc.setRemoteDescription(message);
                    const answer = await pc.createAnswer();
                    await pc.setLocalDescription(answer);
                    ws.send(JSON.stringify(pc.localDescription));
                } catch (e) { console.error('Error al manejar la oferta:', e); }
            } else if (message.type === 'ice-candidate' && message.candidate) {
                try { await pc.addIceCandidate(message.candidate); } 
                catch (e) { console.error('Error al añadir ICE candidate:', e); }
            }
        };
        ws.onclose = () => { if (pc) pc.close(); };
        ws.onerror = () => {
            videoStatus.textContent = 'Error de conexión';
            videoStatus.style.color = '#e94560';
        };
    }
    startVideo();
});