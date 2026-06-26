// Konfigurasi
const WS_URL = `ws://${window.location.hostname}/ws`;
const MAX_POINTS = 60;

// State
let tempData = [];
let timeLabels = [];

// Chart.js
const ctx = document.getElementById('tempChart').getContext('2d');
const chart = new Chart(ctx, {
    type: 'line',
    data: {
        labels: timeLabels,
        datasets: [{
            label: 'Temperature (°C)',
            data: tempData,
            borderColor: '#fbbf24',
            backgroundColor: 'rgba(251, 191, 36, 0.1)',
            fill: true,
            tension: 0.3,
            pointRadius: 0,
            borderWidth: 2
        }, {
            label: 'Setpoint (°C)',
            data: [],
            borderColor: '#ef4444',
            borderDash: [5, 5],
            pointRadius: 0,
            borderWidth: 2,
            fill: false
        }]
    },
    options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: { duration: 200 },
        plugins: {
            legend: { display: false }
        },
        scales: {
            x: {
                display: true,
                ticks: { color: '#9ca3af', maxTicksLimit: 10, font: { size: 10 } },
                grid: { color: 'rgba(75, 85, 99, 0.3)' }
            },
            y: {
                min: 0,
                max: 120,
                ticks: { color: '#9ca3af', font: { size: 10 } },
                grid: { color: 'rgba(75, 85, 99, 0.3)' }
            }
        }
    }
});

// WebSocket
let ws = null;
let reconnectTimer = null;

function connectWs() {
    if (ws && ws.readyState === WebSocket.OPEN) return;

    ws = new WebSocket(WS_URL);
    const status = document.getElementById('connectionStatus');

    ws.onopen = () => {
        status.textContent = 'Connected';
        status.className = 'px-3 py-1 rounded text-sm connected';
        if (reconnectTimer) {
            clearInterval(reconnectTimer);
            reconnectTimer = null;
        }
    };

    ws.onclose = () => {
        status.textContent = 'Disconnected';
        status.className = 'px-3 py-1 rounded text-sm disconnected';
        if (!reconnectTimer) {
            reconnectTimer = setInterval(connectWs, 3000);
        }
    };

    ws.onerror = (err) => {
        console.error('WS Error:', err);
        ws.close();
    };

    ws.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            updateDashboard(data);
        } catch (e) {
            console.error('Parse error:', e);
        }
    };
}

function sendWs(msg) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify(msg));
    }
}

// Dashboard Update
function updateDashboard(d) {
    // Stat cards
    document.getElementById('tempValue').innerHTML = d.temp.toFixed(2) + '<span class="text-lg">°C</span>';
    document.getElementById('tempDetail').textContent = `SP: ${d.setpoint.toFixed(1)}°C | Error: ${d.error.toFixed(1)}°C`;

    document.getElementById('blowerValue').textContent = d.blower + '%';
    document.getElementById('blowerDetail').textContent = `FIS: ${d.fisOut.toFixed(1)}% | u: ${d.u_fopid.toFixed(3)}`;

    document.getElementById('rpmValue').textContent = d.rpm.toFixed(1);
    const rpmStatuses = ['Startup', 'Normal', 'Low Warn', 'High Warn', 'Low Error', 'High Error'];
    const rpmStatusColors = ['text-gray-400', 'text-green-400', 'text-yellow-400', 'text-yellow-400', 'text-red-400', 'text-red-400'];
    document.getElementById('rpmDetail').innerHTML = `Status: <span class="${rpmStatusColors[d.rpmStatus] || ''}">${rpmStatuses[d.rpmStatus] || '?'}</span>`;

    document.getElementById('powerValue').innerHTML = d.power.toFixed(0) + '<span class="text-lg">W</span>';
    document.getElementById('powerDetail').textContent = `${d.voltage.toFixed(1)}V / ${d.current.toFixed(2)}A | PF: ${d.pf.toFixed(2)}`;

    // Mode badge
    const badge = document.getElementById('modeBadge');
    badge.textContent = d.mode;
    badge.className = 'px-3 py-1 rounded text-sm ' + (d.mode === 'FUZZY' ? 'fuzzy bg-green-700' : d.mode === 'MANUAL' ? 'manual bg-blue-700' : 'none bg-gray-600');

    // Mode button
    const modeBtn = document.getElementById('modeBtn');
    if (d.mode === 'FUZZY') {
        modeBtn.textContent = 'Running Fuzzy Mode...';
        modeBtn.className = 'w-full bg-yellow-600 text-white py-3 rounded font-semibold cursor-default';
        modeBtn.disabled = true;
    } else {
        modeBtn.textContent = 'Start Fuzzy Mode';
        modeBtn.className = 'w-full bg-green-600 hover:bg-green-700 text-white py-3 rounded font-semibold';
        modeBtn.disabled = false;
    }

    // PID Info
    document.getElementById('pidValues').textContent = `u: ${d.u_fopid.toFixed(3)} | Int: ${d.integral.toFixed(2)} | D: ${d.derivative.toFixed(3)}`;
    document.getElementById('fisValues').textContent = `FIS Output: ${d.fisOut.toFixed(1)}%`;

    // Chart update
    const now = new Date();
    const timeStr = now.getHours().toString().padStart(2, '0') + ':' +
                    now.getMinutes().toString().padStart(2, '0') + ':' +
                    now.getSeconds().toString().padStart(2, '0');

    tempData.push(d.temp);
    timeLabels.push(timeStr);

    if (tempData.length > MAX_POINTS) {
        tempData.shift();
        timeLabels.shift();
    }

    chart.data.labels = timeLabels;
    chart.data.datasets[0].data = tempData;

    // Setpoint line
    const spData = new Array(tempData.length).fill(d.setpoint);
    chart.data.datasets[1].data = spData;

    chart.update('none');
}

// Actions
function setSetpoint() {
    const val = parseFloat(document.getElementById('spInput').value);
    if (!isNaN(val)) {
        sendWs({ setpoint: val });
    }
}

function setServo() {
    const val = parseInt(document.getElementById('servoInput').value);
    if (!isNaN(val) && val >= 0 && val <= 180) {
        sendWs({ servo: val });
    }
}

function toggleMode() {
    sendWs({ mode: 'FUZZY' });
}

function stopSystem() {
    if (confirm('EMERGENCY STOP - Matikan semua output?')) {
        sendWs({ stop: true });
    }
}

function refreshLogs() {
    fetch('/api/logs')
        .then(r => r.json())
        .then(files => {
            const container = document.getElementById('logList');
            if (files.length === 0) {
                container.innerHTML = '<span class="text-gray-500">No log files found.</span>';
                return;
            }
            container.innerHTML = files.map(f =>
                `<a href="/api/download?file=${encodeURIComponent(f)}" target="_blank" class="block hover:text-amber-400">${f}</a>`
            ).join('');
        })
        .catch(() => {
            document.getElementById('logList').innerHTML = '<span class="text-red-400">Failed to load logs.</span>';
        });
}

// Init
connectWs();
refreshLogs();
setInterval(refreshLogs, 10000);
