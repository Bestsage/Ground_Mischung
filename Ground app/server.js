/**
 * Rocket Telemetry Serial Server
 * Reads data from /dev/ttyACM0 and broadcasts via WebSocket
 */

const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');
const WebSocket = require('ws');
const http = require('http');
const fs = require('fs');
const path = require('path');

// Configuration
const SERIAL_PORT = process.env.SERIAL_PORT || '/dev/ttyACM0';
const BAUD_RATE = parseInt(process.env.BAUD_RATE) || 115200;
const HTTP_PORT = parseInt(process.env.HTTP_PORT) || 8080;
const WS_PORT = parseInt(process.env.WS_PORT) || 8081;

// MIME types for serving static files
const MIME_TYPES = {
    '.html': 'text/html',
    '.css': 'text/css',
    '.js': 'application/javascript',
    '.json': 'application/json',
    '.png': 'image/png',
    '.ico': 'image/x-icon'
};

// ============== HTTP Server ==============
const httpServer = http.createServer((req, res) => {
    let filePath = req.url === '/' ? '/index.html' : req.url;
    filePath = path.join(__dirname, filePath);

    const ext = path.extname(filePath);
    const contentType = MIME_TYPES[ext] || 'text/plain';

    fs.readFile(filePath, (err, content) => {
        if (err) {
            res.writeHead(404);
            res.end('File not found');
        } else {
            res.writeHead(200, { 'Content-Type': contentType });
            res.end(content);
        }
    });
});

httpServer.listen(HTTP_PORT, () => {
    console.log(`🌐 HTTP Server running at http://localhost:${HTTP_PORT}`);
});

// ============== WebSocket Server ==============
const wss = new WebSocket.Server({ port: WS_PORT });
let clients = [];

wss.on('connection', (ws) => {
    console.log('📡 New WebSocket client connected');
    clients.push(ws);

    // Send connection status
    ws.send(JSON.stringify({
        type: 'status',
        connected: serialConnected,
        monitoring: serialMonitoringEnabled
    }));

    ws.on('close', () => {
        console.log('📡 WebSocket client disconnected');
        clients = clients.filter(c => c !== ws);
    });

    ws.on('message', (message) => {
        try {
            const data = JSON.parse(message);
            if (data.type === 'reconnect') {
                reconnectSerial();
            } else if (data.type === 'serial_stop') {
                stopSerialMonitoring();
            } else if (data.type === 'serial_start') {
                startSerialMonitoring();
            } else if (data.type === 'command') {
                // Forward command to serial port
                sendCommand(data.command, data.value);
            }
        } catch (e) {
            console.error('Invalid message:', e);
        }
    });
});

// Send command to serial port
function sendCommand(command, value) {
    if (!serialPort || !serialPort.isOpen) {
        console.log('❌ Cannot send command: serial port not open');
        return;
    }

    const cmdStr = `CMD:${command}:${value}\n`;
    console.log(`📤 Sending command: ${cmdStr.trim()}`);

    serialPort.write(cmdStr, (err) => {
        if (err) {
            console.error(`❌ Error sending command: ${err.message}`);
        }
    });
}

function broadcast(data) {
    const message = JSON.stringify(data);
    clients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
            client.send(message);
        }
    });
}

console.log(`📡 WebSocket Server running on ws://localhost:${WS_PORT}`);

// ============== Serial Port ==============
let serialPort = null;
let parser = null;
let serialConnected = false;
let serialMonitoringEnabled = true;
let reconnectAttempts = 0;
let reconnectTimer = null;
const MAX_RECONNECT_ATTEMPTS = 10;
const RECONNECT_DELAY = 3000;

function broadcastStatus(error = null) {
    const payload = {
        type: 'status',
        connected: serialConnected,
        monitoring: serialMonitoringEnabled
    };

    if (error) {
        payload.error = error;
    }

    broadcast(payload);
}

function clearReconnectTimer() {
    if (reconnectTimer) {
        clearTimeout(reconnectTimer);
        reconnectTimer = null;
    }
}

function connectSerial() {
    if (!serialMonitoringEnabled) {
        console.log('⏸️ Serial monitoring is paused, skipping connection attempt');
        return;
    }

    if (serialPort && serialPort.isOpen) {
        console.log('ℹ️ Serial port already open');
        return;
    }

    console.log(`🔌 Attempting to connect to ${SERIAL_PORT} at ${BAUD_RATE} baud...`);

    serialPort = new SerialPort({
        path: SERIAL_PORT,
        baudRate: BAUD_RATE,
        autoOpen: false
    });

    // Use readline parser to get complete lines
    parser = serialPort.pipe(new ReadlineParser({ delimiter: '\n' }));

    serialPort.open((err) => {
        if (err) {
            console.error(`❌ Error opening serial port: ${err.message}`);
            serialConnected = false;
            broadcastStatus(err.message);
            scheduleReconnect();
            return;
        }

        console.log(`✅ Serial port ${SERIAL_PORT} opened successfully`);
        serialConnected = true;
        reconnectAttempts = 0;
        clearReconnectTimer();
        broadcastStatus();
    });

    // Handle incoming data
    parser.on('data', (line) => {
        line = line.trim();

        // Check for telemetry data: DATA:{"rssi":...}
        if (line.startsWith('DATA:')) {
            try {
                const jsonStr = line.substring(5); // Remove 'DATA:' prefix
                const telemetry = JSON.parse(jsonStr);
                console.log(`📦 Telemetry: RSSI=${telemetry.rssi}dBm LQ=${telemetry.lq}% Alt=${telemetry.alt}m`);
                broadcast({
                    type: 'telemetry',
                    data: telemetry,
                    timestamp: Date.now()
                });
            } catch (e) {
                console.error(`❌ JSON parse error: ${e.message}`);
                console.log(`📝 Raw line: ${line}`);
            }
        }
        // Check for config status: CFG:{"tlm":...}
        else if (line.startsWith('CFG:')) {
            try {
                const jsonStr = line.substring(4); // Remove 'CFG:' prefix
                const config = JSON.parse(jsonStr);
                console.log(`⚙️ Config: TLM=${config.tlm} Rate=${config.rate} PWR=${config.pwr}%`);
                broadcast({
                    type: 'config',
                    data: config,
                    timestamp: Date.now()
                });
            } catch (e) {
                console.error(`❌ Config JSON parse error: ${e.message}`);
            }
        }
        // Debug/log other data
        else if (line.length > 0) {
            console.log(`📝 Debug: ${line}`);
            broadcast({
                type: 'debug',
                message: line,
                timestamp: Date.now()
            });
        }
    });

    // Handle errors
    serialPort.on('error', (err) => {
        console.error(`❌ Serial port error: ${err.message}`);
        serialConnected = false;
        broadcastStatus(err.message);
    });

    // Handle close
    serialPort.on('close', () => {
        console.log('🔌 Serial port closed');
        serialConnected = false;
        broadcastStatus();
        scheduleReconnect();
    });
}

function scheduleReconnect() {
    if (!serialMonitoringEnabled) {
        return;
    }

    if (reconnectTimer) {
        return;
    }

    if (reconnectAttempts < MAX_RECONNECT_ATTEMPTS) {
        reconnectAttempts++;
        console.log(`🔄 Reconnecting in ${RECONNECT_DELAY / 1000}s (attempt ${reconnectAttempts}/${MAX_RECONNECT_ATTEMPTS})...`);
        reconnectTimer = setTimeout(() => {
            reconnectTimer = null;
            connectSerial();
        }, RECONNECT_DELAY);
    } else {
        console.log('❌ Max reconnect attempts reached. Use manual reconnect.');
    }
}

function reconnectSerial() {
    console.log('🔄 Manual reconnect requested');
    serialMonitoringEnabled = true;
    reconnectAttempts = 0;
    clearReconnectTimer();
    broadcastStatus();

    if (serialPort && serialPort.isOpen) {
        serialPort.close(() => {
            connectSerial();
        });
    } else {
        connectSerial();
    }
}

function stopSerialMonitoring() {
    if (!serialMonitoringEnabled) {
        console.log('⏸️ Serial monitoring already paused');
        return;
    }

    console.log('⏹️ Stopping serial monitoring');
    serialMonitoringEnabled = false;
    reconnectAttempts = 0;
    clearReconnectTimer();

    if (parser) {
        parser.removeAllListeners('data');
        parser = null;
    }

    if (serialPort) {
        serialPort.removeAllListeners('error');
        serialPort.removeAllListeners('close');

        if (serialPort.isOpen) {
            serialPort.close(() => {
                serialConnected = false;
                broadcastStatus();
            });
        } else {
            serialConnected = false;
            broadcastStatus();
        }
    } else {
        serialConnected = false;
        broadcastStatus();
    }
}

function startSerialMonitoring() {
    if (serialMonitoringEnabled) {
        console.log('▶️ Serial monitoring already running');
        reconnectAttempts = 0;
        connectSerial();
        return;
    }

    console.log('▶️ Starting serial monitoring');
    serialMonitoringEnabled = true;
    reconnectAttempts = 0;
    clearReconnectTimer();
    broadcastStatus();
    connectSerial();
}

// List available serial ports
async function listPorts() {
    try {
        const ports = await SerialPort.list();
        console.log('\n📋 Available serial ports:');
        ports.forEach(port => {
            console.log(`   - ${port.path} (${port.manufacturer || 'Unknown manufacturer'})`);
        });
        console.log('');
    } catch (err) {
        console.error('Error listing ports:', err);
    }
}

// Start
async function start() {
    console.log('\n🚀 Rocket Telemetry Server Starting...\n');
    await listPorts();
    connectSerial();
}

start();

// Graceful shutdown
process.on('SIGINT', () => {
    console.log('\n👋 Shutting down...');

    clearReconnectTimer();

    if (serialPort && serialPort.isOpen) {
        serialPort.close();
    }

    wss.close();
    httpServer.close();

    process.exit(0);
});
