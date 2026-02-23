/**
 * Rocket Telemetry Dashboard
 * Multi-page dashboard with GPS map, 3D visualization, charts, and logs
 */

class RocketTelemetry {
    constructor() {
        this.data = {
            temperature: null,
            pressure: null,
            altitude: null,
            velocity: null,
            longitude: null,
            latitude: null,
            voltage: null,
            satellites: null,
            gyroX: null,
            gyroY: null,
            gyroZ: null,
            accelX: null,
            accelY: null,
            accelZ: null,
            signalStrength: null,
            downlinkRssi: null,   // Downlink RSSI from ELRS
            linkQuality: null,    // New: ELRS link quality (0-100%)
            packetCount: null     // New: CRSF packet count
        };

        // Config from hardware
        this.config = null;

        this.previousData = { ...this.data };
        this.simulationInterval = null;

        // WebSocket connection
        this.ws = null;
        this.wsReconnectInterval = null;
        this.isConnected = false;
        this.serialMonitoring = true;

        // 3D Scenes (mini and full)
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.rocket = null;
        this.flame = null;

        // Full 3D scene
        this.sceneFull = null;
        this.cameraFull = null;
        this.rendererFull = null;
        this.rocketFull = null;
        this.flameFull = null;

        // Maps (mini and full)
        this.miniMap = null;
        this.fullMap = null;
        this.rocketMarker = null;
        this.rocketMarkerFull = null;
        this.trackLine = null;
        this.trackLineFull = null;
        this.trackCoordinates = [];
        this.launchPosition = null;

        // Charts
        this.charts = {};
        this.chartData = {
            timestamps: [],
            altitude: [],
            velocity: [],
            temperature: [],
            pressure: [],
            accelX: [],
            accelY: [],
            accelZ: [],
            gyroX: [],
            gyroY: [],
            gyroZ: []
        };
        this.maxDataPoints = 60;

        // Attitude estimation
        this.attitude = { roll: 0, pitch: 0, yaw: 0 };
        this.lastUpdateTime = Date.now();

        // Data recording for export
        this.isRecording = false;
        this.recordedData = [];
        this.recordingStartTime = null;

        // Logs
        this.logEntries = [];
        this.maxLogEntries = 500;

        // Current page
        this.currentPage = 'dashboard';

        this.init();
    }

    init() {
        // DOM Elements
        this.elements = {
            packetInput: document.getElementById('packetInput'),
            parseBtn: document.getElementById('parseBtn'),
            simulateBtn: document.getElementById('simulateBtn'),
            connectBtn: document.getElementById('connectBtn'),
            baudRate: document.getElementById('baudRate'),
            serialStatus: document.getElementById('serialStatus'),
            statusIndicator: document.getElementById('statusIndicator'),
            statusText: document.getElementById('statusText'),
            rawPacket: document.getElementById('rawPacket'),
            lastUpdate: document.getElementById('lastUpdate'),

            // Data displays
            temperature: document.getElementById('temperature'),
            pressure: document.getElementById('pressure'),
            altitude: document.getElementById('altitude'),
            velocity: document.getElementById('velocity'),
            longitude: document.getElementById('longitude'),
            latitude: document.getElementById('latitude'),
            voltage: document.getElementById('voltage'),
            satellites: document.getElementById('satellites'),
            gyroX: document.getElementById('gyroX'),
            gyroY: document.getElementById('gyroY'),
            gyroZ: document.getElementById('gyroZ'),
            accelX: document.getElementById('accelX'),
            accelY: document.getElementById('accelY'),
            accelZ: document.getElementById('accelZ'),
            signalStrength: document.getElementById('signalStrength'),
            linkQuality: document.getElementById('linkQuality'),
            packetCount: document.getElementById('packetCount'),

            // Visual elements
            voltageBar: document.getElementById('voltageBar'),
            signalBars: document.getElementById('signalBars'),

            // Export elements
            recordBtn: document.getElementById('recordBtn'),
            exportCSVBtn: document.getElementById('exportCSVBtn'),
            exportJSONBtn: document.getElementById('exportJSONBtn'),
            clearDataBtn: document.getElementById('clearDataBtn'),
            recordCount: document.getElementById('recordCount'),
            recordStatus: document.getElementById('recordStatus'),

            // Page title
            pageTitle: document.getElementById('pageTitle'),

            // Logs
            logsContent: document.getElementById('logsContent'),
            clearLogsBtn: document.getElementById('clearLogsBtn'),
            autoScrollLogs: document.getElementById('autoScrollLogs'),
            lastPacket: document.getElementById('lastPacket'),

            // Map controls
            centerMapBtn: document.getElementById('centerMapBtn'),
            clearTrackBtn: document.getElementById('clearTrackBtn'),
            followRocket: document.getElementById('followRocket'),

            // Sidebar status
            sidebarStatus: document.getElementById('sidebarStatus'),
            connectionText: document.getElementById('connectionText')
        };

        // Event listeners for buttons
        if (this.elements.parseBtn) {
            this.elements.parseBtn.addEventListener('click', () => this.parseInput());
        }
        if (this.elements.simulateBtn) {
            this.elements.simulateBtn.addEventListener('click', () => this.toggleSimulation());
        }
        if (this.elements.connectBtn) {
            this.elements.connectBtn.addEventListener('click', () => this.toggleConnection());
        }
        if (this.elements.packetInput) {
            this.elements.packetInput.addEventListener('keypress', (e) => {
                if (e.key === 'Enter') this.parseInput();
            });
        }

        // Export event listeners
        if (this.elements.recordBtn) {
            this.elements.recordBtn.addEventListener('click', () => this.toggleRecording());
        }
        if (this.elements.exportCSVBtn) {
            this.elements.exportCSVBtn.addEventListener('click', () => this.exportCSV());
        }
        if (this.elements.exportJSONBtn) {
            this.elements.exportJSONBtn.addEventListener('click', () => this.exportJSON());
        }
        if (this.elements.clearDataBtn) {
            this.elements.clearDataBtn.addEventListener('click', () => this.clearRecordedData());
        }

        // Logs event listeners
        if (this.elements.clearLogsBtn) {
            this.elements.clearLogsBtn.addEventListener('click', () => this.clearLogs());
        }

        // Map event listeners
        if (this.elements.centerMapBtn) {
            this.elements.centerMapBtn.addEventListener('click', () => this.centerMap());
        }
        if (this.elements.clearTrackBtn) {
            this.elements.clearTrackBtn.addEventListener('click', () => this.clearTrack());
        }

        // ELRS Configuration event listeners
        this.initElrsControls();

        // Navigation event listeners
        this.initNavigation();

        // Initialize 3D scene (mini)
        this.init3DScene();

        // Initialize maps
        this.initMaps();

        // Initialize charts
        this.initCharts();

        // Auto-connect to WebSocket server
        this.connectWebSocket();

        // Start animation loop
        this.animate();

        // Add initial log
        this.addLog('System initialized', 'info');
    }

    /**
     * Initialize ELRS configuration controls
     */
    initElrsControls() {
        // ARM state
        this.isArmed = false;

        // Apply config button
        const applyBtn = document.getElementById('applyElrsBtn');
        if (applyBtn) {
            applyBtn.addEventListener('click', () => {
                const tlm = document.getElementById('tlmRatio')?.value;
                const rate = document.getElementById('packetRate')?.value;
                const pwr = document.getElementById('txPower')?.value;

                if (tlm) this.sendCommand('TLM', tlm);
                if (rate) this.sendCommand('RATE', rate);
                if (pwr) this.sendCommand('PWR', pwr);

                this.addLog('ELRS config applied', 'success');
            });
        }

        // ARM button
        const armBtn = document.getElementById('armBtn');
        if (armBtn) {
            armBtn.addEventListener('click', () => {
                this.isArmed = !this.isArmed;
                this.sendCommand('ARM', this.isArmed ? 1 : 0);

                armBtn.textContent = this.isArmed ? '🔓 ARMED' : '🔒 SAFE';
                armBtn.classList.toggle('armed', this.isArmed);

                this.addLog(this.isArmed ? '⚠️ ARMED!' : '✓ Disarmed',
                    this.isArmed ? 'error' : 'success');
            });
        }
    }

    /**
     * Initialize page navigation
     */
    initNavigation() {
        const navItems = document.querySelectorAll('.nav-item');
        const pageTitles = {
            'dashboard': 'Dashboard',
            'map': 'GPS Map',
            'charts': 'Charts',
            '3d': '3D View',
            'logs': 'Telemetry Logs'
        };

        navItems.forEach(item => {
            item.addEventListener('click', () => {
                const page = item.dataset.page;

                // Update active nav item
                navItems.forEach(nav => nav.classList.remove('active'));
                item.classList.add('active');

                // Update page title
                if (this.elements.pageTitle) {
                    this.elements.pageTitle.textContent = pageTitles[page] || 'Dashboard';
                }

                // Show corresponding page
                document.querySelectorAll('.page').forEach(p => p.classList.remove('active'));
                document.getElementById(`page-${page}`)?.classList.add('active');

                this.currentPage = page;

                // Initialize page-specific elements
                this.onPageChange(page);
            });
        });
    }

    /**
     * Handle page change
     */
    onPageChange(page) {
        switch (page) {
            case 'map':
                // Invalidate map size after transition
                setTimeout(() => {
                    if (this.fullMap) {
                        this.fullMap.invalidateSize();
                        if (this.data.latitude && this.data.longitude) {
                            this.centerMap();
                        }
                    }
                }, 100);
                break;

            case '3d':
                // Initialize full 3D scene if not already done
                if (!this.sceneFull) {
                    this.init3DSceneFull();
                } else {
                    this.onWindowResizeFull();
                }
                break;

            case 'charts':
                // Resize charts
                Object.values(this.charts).forEach(chart => chart.resize());
                break;
        }
    }

    /**
     * Initialize mini map on dashboard
     */
    initMaps() {
        // Mini map on dashboard
        const miniMapContainer = document.getElementById('miniMap');
        if (miniMapContainer && typeof L !== 'undefined') {
            this.miniMap = L.map('miniMap', {
                zoomControl: false,
                attributionControl: false
            }).setView([48.8566, 2.3522], 13);

            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                maxZoom: 19
            }).addTo(this.miniMap);

            // Create rocket icon
            const rocketIcon = L.divIcon({
                className: 'rocket-marker',
                html: '🚀',
                iconSize: [30, 30],
                iconAnchor: [15, 15]
            });

            this.rocketMarker = L.marker([48.8566, 2.3522], { icon: rocketIcon }).addTo(this.miniMap);

            // Track line
            this.trackLine = L.polyline([], {
                color: '#00d4ff',
                weight: 3,
                opacity: 0.8
            }).addTo(this.miniMap);
        }

        // Full map on GPS page
        const fullMapContainer = document.getElementById('fullMap');
        if (fullMapContainer && typeof L !== 'undefined') {
            this.fullMap = L.map('fullMap', {
                zoomControl: true
            }).setView([48.8566, 2.3522], 15);

            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                maxZoom: 19,
                attribution: '© OpenStreetMap'
            }).addTo(this.fullMap);

            // Create rocket icon
            const rocketIconFull = L.divIcon({
                className: 'rocket-marker',
                html: '🚀',
                iconSize: [30, 30],
                iconAnchor: [15, 15]
            });

            this.rocketMarkerFull = L.marker([48.8566, 2.3522], { icon: rocketIconFull }).addTo(this.fullMap);

            // Track line
            this.trackLineFull = L.polyline([], {
                color: '#00d4ff',
                weight: 3,
                opacity: 0.8
            }).addTo(this.fullMap);

            // Launch position marker
            this.launchMarker = null;
        }
    }

    /**
     * Update maps with new GPS position
     */
    updateMaps() {
        if (this.data.latitude === null || this.data.longitude === null) return;
        if (this.data.latitude === 0 && this.data.longitude === 0) return;

        const lat = this.data.latitude;
        const lon = this.data.longitude;

        // Store launch position
        if (!this.launchPosition) {
            this.launchPosition = [lat, lon];

            // Add launch marker on full map
            if (this.fullMap) {
                const launchIcon = L.divIcon({
                    className: 'launch-marker',
                    html: '📍',
                    iconSize: [24, 24],
                    iconAnchor: [12, 24]
                });
                this.launchMarker = L.marker(this.launchPosition, { icon: launchIcon }).addTo(this.fullMap);
                this.launchMarker.bindPopup('Launch Site');
            }
        }

        // Update track coordinates
        this.trackCoordinates.push([lat, lon]);

        // Limit track points
        if (this.trackCoordinates.length > 1000) {
            this.trackCoordinates.shift();
        }

        // Update mini map
        if (this.miniMap && this.rocketMarker) {
            this.rocketMarker.setLatLng([lat, lon]);
            this.trackLine.setLatLngs(this.trackCoordinates);
            this.miniMap.setView([lat, lon], this.miniMap.getZoom());
        }

        // Update full map
        if (this.fullMap && this.rocketMarkerFull) {
            this.rocketMarkerFull.setLatLng([lat, lon]);
            this.trackLineFull.setLatLngs(this.trackCoordinates);

            // Follow rocket if enabled
            if (this.elements.followRocket?.checked) {
                this.fullMap.setView([lat, lon], this.fullMap.getZoom());
            }
        }

        // Update map info panel
        document.getElementById('mapLatitude')?.textContent &&
            (document.getElementById('mapLatitude').textContent = lat.toFixed(6));
        document.getElementById('mapLongitude')?.textContent &&
            (document.getElementById('mapLongitude').textContent = lon.toFixed(6));
        document.getElementById('mapAltitude')?.textContent &&
            (document.getElementById('mapAltitude').textContent = (this.data.altitude?.toFixed(1) || '--') + ' m');
        document.getElementById('mapSatellites')?.textContent &&
            (document.getElementById('mapSatellites').textContent = this.data.satellites || '--');

        // Calculate distance from launch
        if (this.launchPosition) {
            const distance = this.calculateDistance(
                this.launchPosition[0], this.launchPosition[1],
                lat, lon
            );
            document.getElementById('mapDistance')?.textContent &&
                (document.getElementById('mapDistance').textContent = distance.toFixed(0) + ' m');
        }
    }

    /**
     * Calculate distance between two coordinates (Haversine formula)
     */
    calculateDistance(lat1, lon1, lat2, lon2) {
        const R = 6371000; // Earth's radius in meters
        const φ1 = lat1 * Math.PI / 180;
        const φ2 = lat2 * Math.PI / 180;
        const Δφ = (lat2 - lat1) * Math.PI / 180;
        const Δλ = (lon2 - lon1) * Math.PI / 180;

        const a = Math.sin(Δφ / 2) * Math.sin(Δφ / 2) +
            Math.cos(φ1) * Math.cos(φ2) *
            Math.sin(Δλ / 2) * Math.sin(Δλ / 2);
        const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

        return R * c;
    }

    /**
     * Center map on rocket
     */
    centerMap() {
        if (this.data.latitude && this.data.longitude) {
            if (this.fullMap) {
                this.fullMap.setView([this.data.latitude, this.data.longitude], 16);
            }
            if (this.miniMap) {
                this.miniMap.setView([this.data.latitude, this.data.longitude], 15);
            }
        }
    }

    /**
     * Clear track history
     */
    clearTrack() {
        this.trackCoordinates = [];
        this.launchPosition = null;

        if (this.trackLine) {
            this.trackLine.setLatLngs([]);
        }
        if (this.trackLineFull) {
            this.trackLineFull.setLatLngs([]);
        }
        if (this.launchMarker && this.fullMap) {
            this.fullMap.removeLayer(this.launchMarker);
            this.launchMarker = null;
        }

        this.addLog('Track history cleared', 'warning');
    }

    /**
     * Initialize Three.js 3D scene (mini view)
     */
    init3DScene() {
        const container = document.getElementById('rocket3DContainer');
        if (!container) return;

        const width = container.clientWidth || 300;
        const height = 200;

        // Scene
        this.scene = new THREE.Scene();

        // Camera
        this.camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 1000);
        this.camera.position.set(5, 3, 5);
        this.camera.lookAt(0, 0, 0);

        // Renderer
        this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
        this.renderer.setSize(width, height);
        this.renderer.setClearColor(0x000000, 0);
        container.appendChild(this.renderer.domElement);

        // Lights
        const ambientLight = new THREE.AmbientLight(0x404040, 0.5);
        this.scene.add(ambientLight);

        const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
        directionalLight.position.set(5, 10, 5);
        this.scene.add(directionalLight);

        const pointLight = new THREE.PointLight(0x00d4ff, 0.5);
        pointLight.position.set(-5, 5, -5);
        this.scene.add(pointLight);

        // Create rocket
        const rocketData = this.createRocketModel();
        this.rocket = rocketData.rocket;
        this.flame = rocketData.flame;
        this.scene.add(this.rocket);

        // Add grid
        const gridHelper = new THREE.GridHelper(10, 10, 0x2a3548, 0x1a2235);
        this.scene.add(gridHelper);

        // Add axes helper
        const axesHelper = new THREE.AxesHelper(3);
        this.scene.add(axesHelper);

        // Handle resize
        window.addEventListener('resize', () => this.onWindowResize());
    }

    /**
     * Initialize full 3D scene
     */
    init3DSceneFull() {
        const container = document.getElementById('rocket3DContainerFull');
        if (!container) return;

        const width = container.clientWidth || 800;
        const height = container.clientHeight || 600;

        // Scene
        this.sceneFull = new THREE.Scene();

        // Camera
        this.cameraFull = new THREE.PerspectiveCamera(45, width / height, 0.1, 1000);
        this.cameraFull.position.set(8, 5, 8);
        this.cameraFull.lookAt(0, 0, 0);

        // Renderer
        this.rendererFull = new THREE.WebGLRenderer({ antialias: true, alpha: true });
        this.rendererFull.setSize(width, height);
        this.rendererFull.setClearColor(0x131a2a, 1);
        container.appendChild(this.rendererFull.domElement);

        // Lights
        const ambientLight = new THREE.AmbientLight(0x404040, 0.5);
        this.sceneFull.add(ambientLight);

        const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
        directionalLight.position.set(5, 10, 5);
        this.sceneFull.add(directionalLight);

        const pointLight = new THREE.PointLight(0x00d4ff, 0.5);
        pointLight.position.set(-5, 5, -5);
        this.sceneFull.add(pointLight);

        // Create rocket
        const rocketData = this.createRocketModel();
        this.rocketFull = rocketData.rocket;
        this.flameFull = rocketData.flame;
        this.sceneFull.add(this.rocketFull);

        // Add grid
        const gridHelper = new THREE.GridHelper(20, 20, 0x2a3548, 0x1a2235);
        this.sceneFull.add(gridHelper);

        // Add axes helper
        const axesHelper = new THREE.AxesHelper(5);
        this.sceneFull.add(axesHelper);

        // Handle resize
        window.addEventListener('resize', () => this.onWindowResizeFull());
    }

    /**
     * Create 3D rocket model
     */
    createRocketModel() {
        const rocket = new THREE.Group();

        // Rocket body (cylinder)
        const bodyGeometry = new THREE.CylinderGeometry(0.3, 0.3, 2.5, 32);
        const bodyMaterial = new THREE.MeshPhongMaterial({
            color: 0xcccccc,
            shininess: 100
        });
        const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
        body.position.y = 0;
        rocket.add(body);

        // Nose cone
        const noseGeometry = new THREE.ConeGeometry(0.3, 0.8, 32);
        const noseMaterial = new THREE.MeshPhongMaterial({
            color: 0xff4444,
            shininess: 100
        });
        const nose = new THREE.Mesh(noseGeometry, noseMaterial);
        nose.position.y = 1.65;
        rocket.add(nose);

        // Fins (4 fins)
        const finShape = new THREE.Shape();
        finShape.moveTo(0, 0);
        finShape.lineTo(0.5, 0);
        finShape.lineTo(0.1, 0.8);
        finShape.lineTo(0, 0.8);
        finShape.lineTo(0, 0);

        const finExtrudeSettings = { depth: 0.02, bevelEnabled: false };
        const finGeometry = new THREE.ExtrudeGeometry(finShape, finExtrudeSettings);
        const finMaterial = new THREE.MeshPhongMaterial({
            color: 0x00d4ff,
            shininess: 80
        });

        for (let i = 0; i < 4; i++) {
            const fin = new THREE.Mesh(finGeometry, finMaterial);
            fin.rotation.x = Math.PI / 2;
            fin.rotation.z = (Math.PI / 2) * i;
            fin.position.y = -1.25;
            fin.position.x = Math.cos((Math.PI / 2) * i) * 0.3;
            fin.position.z = Math.sin((Math.PI / 2) * i) * 0.3;
            rocket.add(fin);
        }

        // Engine nozzle
        const nozzleGeometry = new THREE.CylinderGeometry(0.15, 0.25, 0.3, 32);
        const nozzleMaterial = new THREE.MeshPhongMaterial({
            color: 0x333333,
            shininess: 50
        });
        const nozzle = new THREE.Mesh(nozzleGeometry, nozzleMaterial);
        nozzle.position.y = -1.4;
        rocket.add(nozzle);

        // Flame effect
        const flameGeometry = new THREE.ConeGeometry(0.2, 0.8, 32);
        const flameMaterial = new THREE.MeshBasicMaterial({
            color: 0xff6600,
            transparent: true,
            opacity: 0.8
        });
        const flame = new THREE.Mesh(flameGeometry, flameMaterial);
        flame.position.y = -1.9;
        flame.rotation.x = Math.PI;
        flame.visible = false;
        rocket.add(flame);

        return { rocket, flame };
    }

    /**
     * Initialize Chart.js charts
     */
    initCharts() {
        // Common dataset options - no points, thin lines
        const datasetDefaults = {
            pointRadius: 0,
            pointHoverRadius: 3,
            borderWidth: 1.5,
            tension: 0.3
        };

        const chartOptions = {
            responsive: true,
            maintainAspectRatio: false,
            animation: { duration: 0 },
            scales: {
                x: {
                    display: true,
                    grid: { color: '#2a3548' },
                    ticks: { color: '#8892a6', maxTicksLimit: 6 }
                },
                y: {
                    display: true,
                    grid: { color: '#2a3548' },
                    ticks: { color: '#8892a6' }
                }
            },
            plugins: {
                legend: {
                    labels: { color: '#8892a6', boxWidth: 12 }
                }
            }
        };

        // Altitude & Velocity Chart
        const altitudeCanvas = document.getElementById('altitudeChart');
        if (altitudeCanvas) {
            this.charts.altitude = new Chart(altitudeCanvas, {
                type: 'line',
                data: {
                    labels: [],
                    datasets: [
                        {
                            label: 'Altitude (m)',
                            data: [],
                            borderColor: '#00d4ff',
                            backgroundColor: 'rgba(0, 212, 255, 0.1)',
                            fill: true,
                            ...datasetDefaults
                        },
                        {
                            label: 'Velocity (m/s)',
                            data: [],
                            borderColor: '#00ff88',
                            backgroundColor: 'rgba(0, 255, 136, 0.1)',
                            fill: true,
                            yAxisID: 'y1',
                            ...datasetDefaults
                        }
                    ]
                },
                options: {
                    ...chartOptions,
                    scales: {
                        ...chartOptions.scales,
                        y1: {
                            type: 'linear',
                            display: true,
                            position: 'right',
                            grid: { drawOnChartArea: false },
                            ticks: { color: '#00ff88' }
                        }
                    }
                }
            });
        }

        // Temperature & Pressure Chart
        const envCanvas = document.getElementById('environmentChart');
        if (envCanvas) {
            this.charts.environment = new Chart(envCanvas, {
                type: 'line',
                data: {
                    labels: [],
                    datasets: [
                        {
                            label: 'Temperature (°C)',
                            data: [],
                            borderColor: '#ff9f43',
                            backgroundColor: 'rgba(255, 159, 67, 0.1)',
                            fill: true,
                            ...datasetDefaults
                        },
                        {
                            label: 'Pressure (kPa)',
                            data: [],
                            borderColor: '#a855f7',
                            backgroundColor: 'rgba(168, 85, 247, 0.1)',
                            fill: true,
                            yAxisID: 'y1',
                            ...datasetDefaults
                        }
                    ]
                },
                options: {
                    ...chartOptions,
                    scales: {
                        ...chartOptions.scales,
                        y1: {
                            type: 'linear',
                            display: true,
                            position: 'right',
                            grid: { drawOnChartArea: false },
                            ticks: { color: '#a855f7' }
                        }
                    }
                }
            });
        }

        // Acceleration Chart
        const accelCanvas = document.getElementById('accelChart');
        if (accelCanvas) {
            this.charts.accel = new Chart(accelCanvas, {
                type: 'line',
                data: {
                    labels: [],
                    datasets: [
                        {
                            label: 'X',
                            data: [],
                            borderColor: '#ff4757',
                            ...datasetDefaults
                        },
                        {
                            label: 'Y',
                            data: [],
                            borderColor: '#00ff88',
                            ...datasetDefaults
                        },
                        {
                            label: 'Z',
                            data: [],
                            borderColor: '#00d4ff',
                            ...datasetDefaults
                        }
                    ]
                },
                options: chartOptions
            });
        }

        // Attitude Chart (was Gyroscope)
        const gyroCanvas = document.getElementById('gyroChart');
        if (gyroCanvas) {
            this.charts.gyro = new Chart(gyroCanvas, {
                type: 'line',
                data: {
                    labels: [],
                    datasets: [
                        {
                            label: 'Pitch',
                            data: [],
                            borderColor: '#ff4757',
                            ...datasetDefaults
                        },
                        {
                            label: 'Roll',
                            data: [],
                            borderColor: '#00ff88',
                            ...datasetDefaults
                        },
                        {
                            label: 'Yaw',
                            data: [],
                            borderColor: '#00d4ff',
                            ...datasetDefaults
                        }
                    ]
                },
                options: chartOptions
            });
        }
    }

    /**
     * Update charts with new data
     */
    updateCharts() {
        const timestamp = new Date().toLocaleTimeString();

        // Add new data points
        this.chartData.timestamps.push(timestamp);
        this.chartData.altitude.push(this.data.altitude ?? 0);
        this.chartData.velocity.push(this.data.velocity ?? 0);
        this.chartData.temperature.push(this.data.temperature ?? 0);
        this.chartData.pressure.push(this.data.pressure ?? 0);
        this.chartData.accelX.push(this.data.accelX ?? 0);
        this.chartData.accelY.push(this.data.accelY ?? 0);
        this.chartData.accelZ.push(this.data.accelZ ?? 0);
        this.chartData.gyroX.push(this.data.gyroX ?? 0);
        this.chartData.gyroY.push(this.data.gyroY ?? 0);
        this.chartData.gyroZ.push(this.data.gyroZ ?? 0);

        // Limit data points
        if (this.chartData.timestamps.length > this.maxDataPoints) {
            this.chartData.timestamps.shift();
            this.chartData.altitude.shift();
            this.chartData.velocity.shift();
            this.chartData.temperature.shift();
            this.chartData.pressure.shift();
            this.chartData.accelX.shift();
            this.chartData.accelY.shift();
            this.chartData.accelZ.shift();
            this.chartData.gyroX.shift();
            this.chartData.gyroY.shift();
            this.chartData.gyroZ.shift();
        }

        // Update altitude chart
        if (this.charts.altitude) {
            this.charts.altitude.data.labels = this.chartData.timestamps;
            this.charts.altitude.data.datasets[0].data = this.chartData.altitude;
            this.charts.altitude.data.datasets[1].data = this.chartData.velocity;
            this.charts.altitude.update('none');
        }

        // Update environment chart
        if (this.charts.environment) {
            this.charts.environment.data.labels = this.chartData.timestamps;
            this.charts.environment.data.datasets[0].data = this.chartData.temperature;
            this.charts.environment.data.datasets[1].data = this.chartData.pressure;
            this.charts.environment.update('none');
        }

        // Update acceleration chart
        if (this.charts.accel) {
            this.charts.accel.data.labels = this.chartData.timestamps;
            this.charts.accel.data.datasets[0].data = this.chartData.accelX;
            this.charts.accel.data.datasets[1].data = this.chartData.accelY;
            this.charts.accel.data.datasets[2].data = this.chartData.accelZ;
            this.charts.accel.update('none');
        }

        // Update gyroscope chart
        if (this.charts.gyro) {
            this.charts.gyro.data.labels = this.chartData.timestamps;
            this.charts.gyro.data.datasets[0].data = this.chartData.gyroX;
            this.charts.gyro.data.datasets[1].data = this.chartData.gyroY;
            this.charts.gyro.data.datasets[2].data = this.chartData.gyroZ;
            this.charts.gyro.update('none');
        }
    }

    /**
 * Update 3D rocket attitude from attitude data (angles in degrees)
 * The hardware sends absolute angles (pitch, roll, yaw), not angular velocities
 */
    updateRocketAttitude() {
        if (this.data.gyroX === null) return;

        // Target angles from hardware (already in degrees)
        const targetPitch = (this.data.gyroX ?? 0) * Math.PI / 180;  // p = pitch
        const targetRoll = (this.data.gyroY ?? 0) * Math.PI / 180;   // r = roll  
        const targetYaw = (this.data.gyroZ ?? 0) * Math.PI / 180;    // y = yaw

        // Smooth interpolation factor (0.1 = smooth, 1.0 = instant)
        const smoothing = 0.15;

        // Lerp current attitude towards target for smooth movement
        this.attitude.pitch += (targetPitch - this.attitude.pitch) * smoothing;
        this.attitude.roll += (targetRoll - this.attitude.roll) * smoothing;
        this.attitude.yaw += (targetYaw - this.attitude.yaw) * smoothing;

        // Apply to mini rocket
        if (this.rocket) {
            this.rocket.rotation.x = this.attitude.pitch;
            this.rocket.rotation.z = this.attitude.roll;
            this.rocket.rotation.y = this.attitude.yaw;
        }

        // Apply to full rocket
        if (this.rocketFull) {
            this.rocketFull.rotation.x = this.attitude.pitch;
            this.rocketFull.rotation.z = this.attitude.roll;
            this.rocketFull.rotation.y = this.attitude.yaw;
        }

        // Update attitude displays (show actual received values, not smoothed)
        const rollDeg = (this.data.gyroY ?? 0).toFixed(1) + '°';
        const pitchDeg = (this.data.gyroX ?? 0).toFixed(1) + '°';
        const yawDeg = (this.data.gyroZ ?? 0).toFixed(1) + '°';

        // Mini view
        const rollEl = document.getElementById('rollValue');
        const pitchEl = document.getElementById('pitchValue');
        const yawEl = document.getElementById('yawValue');
        if (rollEl) rollEl.textContent = rollDeg;
        if (pitchEl) pitchEl.textContent = pitchDeg;
        if (yawEl) yawEl.textContent = yawDeg;

        // Full view
        const rollFullEl = document.getElementById('rollValueFull');
        const pitchFullEl = document.getElementById('pitchValueFull');
        const yawFullEl = document.getElementById('yawValueFull');
        if (rollFullEl) rollFullEl.textContent = rollDeg;
        if (pitchFullEl) pitchFullEl.textContent = pitchDeg;
        if (yawFullEl) yawFullEl.textContent = yawDeg;

        // Accel values in full view
        const accelXFullEl = document.getElementById('accelXFull');
        const accelYFullEl = document.getElementById('accelYFull');
        const accelZFullEl = document.getElementById('accelZFull');
        if (accelXFullEl) accelXFullEl.textContent = (this.data.accelX?.toFixed(2) ?? '--') + ' m/s²';
        if (accelYFullEl) accelYFullEl.textContent = (this.data.accelY?.toFixed(2) ?? '--') + ' m/s²';
        if (accelZFullEl) accelZFullEl.textContent = (this.data.accelZ?.toFixed(2) ?? '--') + ' m/s²';
    }

    /**
     * Animation loop
     */
    animate() {
        requestAnimationFrame(() => this.animate());

        // Flame animation
        if (this.flame && this.flame.visible) {
            this.flame.scale.y = 0.8 + Math.random() * 0.4;
            this.flame.scale.x = 0.8 + Math.random() * 0.2;
        }
        if (this.flameFull && this.flameFull.visible) {
            this.flameFull.scale.y = 0.8 + Math.random() * 0.4;
            this.flameFull.scale.x = 0.8 + Math.random() * 0.2;
        }

        // Render mini scene
        if (this.renderer && this.scene && this.camera) {
            this.renderer.render(this.scene, this.camera);
        }

        // Render full scene if visible
        if (this.currentPage === '3d' && this.rendererFull && this.sceneFull && this.cameraFull) {
            this.rendererFull.render(this.sceneFull, this.cameraFull);
        }
    }

    /**
     * Handle window resize (mini)
     */
    onWindowResize() {
        const container = document.getElementById('rocket3DContainer');
        if (!container || !this.camera || !this.renderer) return;

        const width = container.clientWidth;
        const height = 200;

        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(width, height);
    }

    /**
     * Handle window resize (full)
     */
    onWindowResizeFull() {
        const container = document.getElementById('rocket3DContainerFull');
        if (!container || !this.cameraFull || !this.rendererFull) return;

        const width = container.clientWidth;
        const height = container.clientHeight;

        this.cameraFull.aspect = width / height;
        this.cameraFull.updateProjectionMatrix();
        this.rendererFull.setSize(width, height);
    }

    /**
     * Connect to WebSocket server
     */
    connectWebSocket() {
        const wsUrl = 'ws://localhost:8081';
        console.log(`Connecting to WebSocket server at ${wsUrl}...`);
        this.addLog(`Connecting to ${wsUrl}...`, 'info');

        try {
            this.ws = new WebSocket(wsUrl);

            this.ws.onopen = () => {
                console.log('WebSocket connected');
                this.addLog('WebSocket connected', 'success');
                this.updateConnectionUI(true);

                if (this.wsReconnectInterval) {
                    clearInterval(this.wsReconnectInterval);
                    this.wsReconnectInterval = null;
                }
            };

            this.ws.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    this.handleWebSocketMessage(data);
                } catch (e) {
                    console.error('Failed to parse WebSocket message:', e);
                }
            };

            this.ws.onclose = () => {
                console.log('WebSocket disconnected');
                this.isConnected = false;
                this.updateConnectionUI(false);
                this.addLog('WebSocket disconnected', 'warning');
                this.scheduleReconnect();
            };

            this.ws.onerror = (error) => {
                console.error('WebSocket error:', error);
                this.addLog('WebSocket error', 'error');
            };

        } catch (error) {
            console.error('Failed to connect WebSocket:', error);
            this.scheduleReconnect();
        }
    }

    /**
     * Schedule WebSocket reconnection
     */
    scheduleReconnect() {
        if (!this.wsReconnectInterval) {
            this.wsReconnectInterval = setInterval(() => {
                if (!this.ws || this.ws.readyState === WebSocket.CLOSED) {
                    console.log('Attempting to reconnect WebSocket...');
                    this.connectWebSocket();
                }
            }, 3000);
        }
    }

    /**
     * Handle incoming WebSocket messages
     */
    handleWebSocketMessage(data) {
        switch (data.type) {
            case 'status':
                if (typeof data.monitoring === 'boolean') {
                    this.serialMonitoring = data.monitoring;
                }
                this.isConnected = data.connected;
                this.updateConnectionUI(data.connected);
                if (data.connected) {
                    this.addLog('Serial port connected', 'success');
                } else if (this.serialMonitoring) {
                    this.addLog('Serial port disconnected', 'warning');
                } else {
                    this.addLog('Serial monitoring paused', 'warning');
                }
                if (data.error) {
                    this.addLog(`Error: ${data.error}`, 'error');
                }
                break;

            case 'telemetry':
                try {
                    // New JSON format: data.data contains telemetry object
                    this.applyTelemetry(data.data);
                    this.updateDisplay();
                    this.setConnected(true);

                    // Update logs with summary
                    if (this.elements.lastPacket) {
                        const t = data.data;
                        this.elements.lastPacket.textContent =
                            `RSSI:${t.rssi}dBm LQ:${t.lq}% Alt:${t.alt}m`;
                    }
                    if (this.elements.lastUpdate) {
                        this.elements.lastUpdate.textContent = new Date().toLocaleTimeString();
                    }

                    // Add to logs page
                    this.addLog(`DATA: RSSI=${data.data.rssi} LQ=${data.data.lq}% Alt=${data.data.alt}m`, 'data');

                    // Record data if recording
                    this.recordDataPoint(JSON.stringify(data.data));
                } catch (error) {
                    console.warn('Telemetry parse error:', error.message);
                }
                break;

            case 'config':
                // Handle config status from hardware
                try {
                    this.applyConfig(data.data);
                    this.addLog(`Config: TLM=${data.data.tlm} Rate=${data.data.rate} PWR=${data.data.pwr}%`, 'info');
                } catch (error) {
                    console.warn('Config parse error:', error.message);
                }
                break;

            case 'debug':
                // Debug messages from hardware
                this.addLog(data.message, 'warning');
                break;
        }
    }

    /**
     * Apply telemetry data from JSON object
     * Maps hardware JSON fields to internal data structure
     */
    applyTelemetry(t) {
        this.previousData = { ...this.data };

        // Map JSON fields to internal data structure
        // t = {"rssi":-40,"drssi":-50,"lq":98,"p":1.2,"r":-0.3,"y":45.0,"vario":0.5,"batt":3700,"sats":8,"alt":100,"pkts":1234,"lat":48.8,"lon":2.35,"temp":22.5,"press":1013.2}
        if (t.rssi !== undefined) this.data.signalStrength = t.rssi;
        if (t.drssi !== undefined) this.data.downlinkRssi = t.drssi;
        if (t.lq !== undefined) this.data.linkQuality = t.lq;
        if (t.p !== undefined) this.data.gyroX = t.p;      // Pitch
        if (t.r !== undefined) this.data.gyroY = t.r;      // Roll
        if (t.y !== undefined) this.data.gyroZ = t.y;      // Yaw
        if (t.vario !== undefined) this.data.velocity = t.vario;
        if (t.batt !== undefined) this.data.voltage = t.batt;  // mV
        if (t.sats !== undefined) this.data.satellites = t.sats;
        if (t.alt !== undefined) this.data.altitude = t.alt;
        if (t.pkts !== undefined) this.data.packetCount = t.pkts;
        if (t.lat !== undefined) this.data.latitude = t.lat;
        if (t.lon !== undefined) this.data.longitude = t.lon;
        if (t.temp !== undefined) this.data.temperature = t.temp;
        if (t.press !== undefined) this.data.pressure = t.press;

        return this.data;
    }

    /**
     * Apply config status from hardware
     */
    applyConfig(cfg) {
        // cfg = {"tlm":7,"rate":1,"pwr":100,"theme":0,"scr":0,"act":0}
        this.config = {
            tlmRatio: cfg.tlm,
            packetRate: cfg.rate,
            txPower: cfg.pwr,
            theme: cfg.theme,
            screen: cfg.scr,
            isActive: cfg.act === 1
        };

        // Update UI if config panel exists
        this.updateConfigUI();
    }

    /**
     * Update config UI elements if they exist
     */
    updateConfigUI() {
        if (!this.config) return;

        const tlmEl = document.getElementById('tlmRatio');
        const rateEl = document.getElementById('packetRate');
        const pwrEl = document.getElementById('txPower');

        if (tlmEl) tlmEl.value = this.config.tlmRatio;
        if (rateEl) rateEl.value = this.config.packetRate;
        if (pwrEl) pwrEl.value = this.config.txPower;
    }

    /**
     * Send command to hardware via WebSocket
     */
    sendCommand(command, value) {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify({
                type: 'command',
                command: command,
                value: value
            }));
            this.addLog(`Sent: CMD:${command}:${value}`, 'info');
        } else {
            this.addLog('Cannot send command: not connected', 'error');
        }
    }

    /**
     * Update connection UI
     */
    updateConnectionUI(connected) {
        const statusEl = this.elements.sidebarStatus;
        const textEl = this.elements.connectionText;
        const connectBtn = this.elements.connectBtn;

        if (statusEl) {
            statusEl.classList.toggle('connected', connected);
        }
        if (textEl) {
            textEl.textContent = connected ? 'Connected' : 'Disconnected';
        }
        if (connectBtn) {
            const wsReady = this.ws && this.ws.readyState === WebSocket.OPEN;
            connectBtn.textContent = wsReady
                ? (this.serialMonitoring ? '⏹ Stop Serial' : '▶️ Start Serial')
                : '🔌 Connect';
            connectBtn.classList.toggle('connected', connected);
        }
    }

    /**
     * Toggle connection
     */
    toggleConnection() {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            if (this.serialMonitoring) {
                this.ws.send(JSON.stringify({ type: 'serial_stop' }));
                this.addLog('Stopping serial monitoring...', 'info');
            } else {
                this.ws.send(JSON.stringify({ type: 'serial_start' }));
                this.addLog('Starting serial monitoring...', 'info');
            }
        } else {
            this.connectWebSocket();
        }
    }

    /**
     * Parse telemetry packet
     */
    parsePacket(packet) {
        if (!packet.startsWith('@')) {
            throw new Error('Invalid packet: must start with @');
        }

        const data = packet.substring(1);

        const patterns = {
            temperature: /T(-?\d+\.?\d*)/,
            pressure: /P(-?\d+\.?\d*)/,
            altitude: /A(-?\d+\.?\d*)/,
            velocity: /v(-?\d+\.?\d*)/,
            longitude: /l(-?\d+\.?\d*)/,
            latitude: /L(-?\d+\.?\d*)/,
            voltage: /V(-?\d+)/,
            satellites: /S(\d+)/,
            gyroX: /x(-?\d+\.?\d*)/,
            gyroY: /y(-?\d+\.?\d*)/,
            gyroZ: /z(-?\d+\.?\d*)/,
            accelX: /X(-?\d+\.?\d*)/,
            accelY: /Y(-?\d+\.?\d*)/,
            accelZ: /Z(-?\d+\.?\d*)/,
            signalStrength: /s(-?\d+)/
        };

        this.previousData = { ...this.data };

        for (const [key, pattern] of Object.entries(patterns)) {
            const match = data.match(pattern);
            if (match) {
                this.data[key] = parseFloat(match[1]);
            }
        }

        return this.data;
    }

    /**
     * Parse input from text field
     */
    parseInput() {
        const packet = this.elements.packetInput?.value?.trim();

        if (!packet) {
            this.addLog('No packet to parse', 'warning');
            return;
        }

        try {
            this.parsePacket(packet);
            this.updateDisplay();
            this.setConnected(true);
            this.addLog(packet, 'data');
            this.recordDataPoint(packet);
        } catch (error) {
            this.addLog(`Parse error: ${error.message}`, 'error');
        }
    }

    /**
     * Update all display elements
     */
    updateDisplay() {
        // Main telemetry values
        this.updateValue('temperature', this.data.temperature?.toFixed(2) ?? '--');
        this.updateValue('pressure', this.data.pressure?.toFixed(2) ?? '--');
        this.updateValue('altitude', this.data.altitude?.toFixed(2) ?? '--');
        this.updateValue('velocity', this.data.velocity?.toFixed(2) ?? '--');

        // GPS
        this.updateValue('latitude', this.data.latitude?.toFixed(6) ?? '--');
        this.updateValue('longitude', this.data.longitude?.toFixed(6) ?? '--');
        this.updateValue('satellites', this.data.satellites ?? '--');

        // RF Link (new fields from hardware)
        this.updateValue('voltage', this.data.voltage ?? '--');
        this.updateValue('signalStrength', this.data.signalStrength ?? '--');
        this.updateValue('linkQuality', this.data.linkQuality ?? '--');
        this.updateValue('packetCount', this.data.packetCount ?? '--');

        // IMU (now uses pitch/roll/yaw from ELRS)
        this.updateValue('gyroX', this.data.gyroX?.toFixed(2) ?? '--');
        this.updateValue('gyroY', this.data.gyroY?.toFixed(2) ?? '--');
        this.updateValue('gyroZ', this.data.gyroZ?.toFixed(2) ?? '--');
        this.updateValue('accelX', this.data.accelX?.toFixed(2) ?? '--');
        this.updateValue('accelY', this.data.accelY?.toFixed(2) ?? '--');
        this.updateValue('accelZ', this.data.accelZ?.toFixed(2) ?? '--');

        // Update visuals
        this.updateVoltageBar();
        this.updateSignalBars();
        this.updateLinkQualityBar();
        this.updateRocketAttitude();
        this.updateCharts();
        this.updateMaps();
    }

    /**
     * Update single value element
     */
    updateValue(elementId, value) {
        const element = this.elements[elementId];
        if (!element) return;

        const oldValue = element.textContent;
        element.textContent = value;

        if (oldValue !== value && value !== '--') {
            element.classList.add('value-changed');
            setTimeout(() => element.classList.remove('value-changed'), 500);
        }
    }

    /**
     * Update voltage bar (voltage in mV, typical LiPo range 3000-4200mV)
     */
    updateVoltageBar() {
        const bar = this.elements.voltageBar;
        if (!bar || this.data.voltage === null) return;

        // Convert mV to percentage (3000mV = 0%, 4200mV = 100%)
        const minVoltage = 3000;
        const maxVoltage = 4200;
        const percentage = Math.min(100, Math.max(0,
            ((this.data.voltage - minVoltage) / (maxVoltage - minVoltage)) * 100
        ));
        bar.style.width = percentage + '%';

        // Color coding based on voltage level
        if (percentage < 20) {
            bar.classList.add('critical');
            bar.classList.remove('warning');
        } else if (percentage < 40) {
            bar.classList.add('warning');
            bar.classList.remove('critical');
        } else {
            bar.classList.remove('warning', 'critical');
        }
    }

    /**
     * Update signal strength bars
     */
    updateSignalBars() {
        const container = this.elements.signalBars;
        if (!container || this.data.signalStrength === null) return;

        const signal = this.data.signalStrength;
        const bars = container.querySelectorAll('.bar');

        // Map signal strength to bars (typically -90 to -30 dBm)
        const normalized = Math.min(1, Math.max(0, (signal + 90) / 60));
        const activeBars = Math.ceil(normalized * bars.length);

        bars.forEach((bar, i) => {
            bar.classList.toggle('active', i < activeBars);
        });
    }

    /**
     * Update link quality bar (0-100%)
     */
    updateLinkQualityBar() {
        const bar = document.getElementById('lqBar');
        if (!bar || this.data.linkQuality === null) return;

        const percentage = Math.min(100, Math.max(0, this.data.linkQuality));
        bar.style.width = percentage + '%';

        // Color coding based on link quality
        if (percentage < 30) {
            bar.classList.add('critical');
            bar.classList.remove('warning', 'good');
        } else if (percentage < 60) {
            bar.classList.add('warning');
            bar.classList.remove('critical', 'good');
        } else {
            bar.classList.add('good');
            bar.classList.remove('warning', 'critical');
        }
    }

    /**
     * Set connected state
     */
    setConnected(connected) {
        this.isConnected = connected;
    }

    /**
     * Add log entry
     */
    addLog(message, type = 'info') {
        const logsContent = this.elements.logsContent;
        if (!logsContent) return;

        const entry = document.createElement('div');
        entry.className = `log-entry ${type}`;

        const timestamp = new Date().toLocaleTimeString();
        entry.innerHTML = `<span class="log-time">[${timestamp}]</span> ${message}`;

        logsContent.appendChild(entry);

        // Limit entries
        while (logsContent.children.length > this.maxLogEntries) {
            logsContent.removeChild(logsContent.firstChild);
        }

        // Auto scroll
        if (this.elements.autoScrollLogs?.checked) {
            logsContent.scrollTop = logsContent.scrollHeight;
        }
    }

    /**
     * Clear logs
     */
    clearLogs() {
        if (this.elements.logsContent) {
            this.elements.logsContent.innerHTML = '<div class="log-entry info">Logs cleared</div>';
        }
    }

    /**
     * Toggle recording
     */
    toggleRecording() {
        this.isRecording = !this.isRecording;

        if (this.isRecording) {
            this.recordingStartTime = new Date();
            this.recordedData = [];
            this.elements.recordBtn?.classList.add('recording');
            if (this.elements.recordBtn) {
                this.elements.recordBtn.textContent = '⏹️ Stop';
            }
            this.addLog('Recording started', 'success');
        } else {
            this.elements.recordBtn?.classList.remove('recording');
            if (this.elements.recordBtn) {
                this.elements.recordBtn.textContent = '🔴 Record';
            }
            this.addLog(`Recording stopped: ${this.recordedData.length} packets`, 'info');
        }

        this.updateRecordCount();
    }

    /**
     * Record data point
     */
    recordDataPoint(packet) {
        if (!this.isRecording) return;

        const elapsed = this.recordingStartTime ? Date.now() - this.recordingStartTime.getTime() : 0;

        this.recordedData.push({
            timestamp: new Date().toISOString(),
            elapsedMs: elapsed,
            rawPacket: packet,
            ...this.data
        });

        this.updateRecordCount();
    }

    /**
     * Update record count display
     */
    updateRecordCount() {
        if (this.elements.recordCount) {
            this.elements.recordCount.textContent = this.recordedData.length;
        }
    }

    /**
     * Export to CSV
     */
    exportCSV() {
        if (this.recordedData.length === 0) {
            alert('No data to export! Start recording first.');
            return;
        }

        const headers = [
            'timestamp', 'elapsedMs', 'rawPacket',
            'temperature', 'pressure', 'altitude', 'velocity',
            'latitude', 'longitude', 'voltage', 'satellites',
            'gyroX', 'gyroY', 'gyroZ',
            'accelX', 'accelY', 'accelZ',
            'signalStrength'
        ];

        const csvRows = [headers.join(',')];

        for (const row of this.recordedData) {
            const values = headers.map(h => {
                const val = row[h];
                if (val === null || val === undefined) return '';
                if (typeof val === 'string' && val.includes(',')) {
                    return `"${val}"`;
                }
                return val;
            });
            csvRows.push(values.join(','));
        }

        const csvContent = csvRows.join('\n');
        this.downloadFile(csvContent, 'telemetry_data.csv', 'text/csv');
        this.addLog(`Exported ${this.recordedData.length} packets to CSV`, 'success');
    }

    /**
     * Export to JSON
     */
    exportJSON() {
        if (this.recordedData.length === 0) {
            alert('No data to export! Start recording first.');
            return;
        }

        const exportData = {
            metadata: {
                exportTime: new Date().toISOString(),
                recordingStart: this.recordingStartTime?.toISOString(),
                totalPackets: this.recordedData.length
            },
            data: this.recordedData
        };

        const jsonContent = JSON.stringify(exportData, null, 2);
        this.downloadFile(jsonContent, 'telemetry_data.json', 'application/json');
        this.addLog(`Exported ${this.recordedData.length} packets to JSON`, 'success');
    }

    /**
     * Download file
     */
    downloadFile(content, filename, mimeType) {
        const timestamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
        const finalFilename = filename.replace('.', `_${timestamp}.`);

        const blob = new Blob([content], { type: mimeType });
        const url = URL.createObjectURL(blob);

        const a = document.createElement('a');
        a.href = url;
        a.download = finalFilename;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
    }

    /**
     * Clear recorded data
     */
    clearRecordedData() {
        if (this.recordedData.length === 0) return;

        if (confirm(`Clear ${this.recordedData.length} recorded packets?`)) {
            this.recordedData = [];
            this.recordingStartTime = null;
            this.updateRecordCount();
            this.addLog('Recorded data cleared', 'warning');
        }
    }

    /**
     * Toggle simulation
     */
    toggleSimulation() {
        if (this.simulationInterval) {
            clearInterval(this.simulationInterval);
            this.simulationInterval = null;
            if (this.elements.simulateBtn) {
                this.elements.simulateBtn.textContent = 'Simulate';
            }
            if (this.flame) this.flame.visible = false;
            if (this.flameFull) this.flameFull.visible = false;
            this.addLog('Simulation stopped', 'info');
        } else {
            this.simulationInterval = setInterval(() => this.generateSimulatedData(), 100);
            if (this.elements.simulateBtn) {
                this.elements.simulateBtn.textContent = 'Stop';
            }
            if (this.flame) this.flame.visible = true;
            if (this.flameFull) this.flameFull.visible = true;
            this.addLog('Simulation started', 'success');
        }
    }

    /**
     * Generate simulated data
     */
    generateSimulatedData() {
        const time = Date.now() / 1000;

        const temp = 20 + Math.sin(time * 0.1) * 5 + Math.random() * 2;
        const pressure = 101.3 - Math.sin(time * 0.05) * 10;
        const altitude = Math.max(0, 1000 * Math.sin(time * 0.02) + 500 + Math.random() * 10);
        const velocity = Math.cos(time * 0.02) * 100 + Math.random() * 5;
        const longitude = 2.3522 + Math.sin(time * 0.001) * 0.001;
        const latitude = 48.8566 + Math.cos(time * 0.001) * 0.001;
        const voltage = 85 + Math.sin(time * 0.05) * 10;
        const satellites = Math.floor(8 + Math.random() * 6);
        const gyroX = Math.sin(time * 0.5) * 2;
        const gyroY = Math.cos(time * 0.3) * 2;
        const gyroZ = Math.sin(time * 0.7) * 1;
        const accelX = Math.sin(time * 0.4) * 2;
        const accelY = -9.81 + Math.cos(time * 0.3) * 0.5;
        const accelZ = Math.sin(time * 0.6) * 1;
        const signal = -50 + Math.sin(time * 0.2) * 20;

        const packet = `@T${temp.toFixed(2)}P${pressure.toFixed(2)}A${altitude.toFixed(2)}v${velocity.toFixed(2)}l${longitude.toFixed(6)}L${latitude.toFixed(6)}V${Math.round(voltage)}S${satellites}x${gyroX.toFixed(2)}y${gyroY.toFixed(2)}z${gyroZ.toFixed(2)}X${accelX.toFixed(2)}Y${accelY.toFixed(2)}Z${accelZ.toFixed(2)}s${Math.round(signal)}`;

        if (this.elements.packetInput) {
            this.elements.packetInput.value = packet;
        }

        this.parsePacket(packet);
        this.updateDisplay();
        this.setConnected(true);
        this.recordDataPoint(packet);
    }
}

// Initialize
document.addEventListener('DOMContentLoaded', () => {
    window.telemetry = new RocketTelemetry();
});
