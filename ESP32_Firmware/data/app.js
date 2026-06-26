const HOST = window.location.hostname;
const WS_URL = `ws://${HOST}/ws`;
const MAX_POINTS = 60;

let tempData = [];
let timeLabels = [];
let isFirstData = true;

const ctx = document.getElementById('tempChart').getContext('2d');
const chart = new Chart(ctx, {
    type: 'line',
    data: {
        labels: timeLabels,
        datasets: [{
            label: 'Temperature (°C)',
            data: tempData,
            borderColor: '#fbbf24',
            backgroundColor: 'rgba(251, 191, 36, 0.08)',
            fill: false,
            tension: 0.2,
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
        maintainAspectRatio: true,
        animation: false,
        plugins: {
            legend: { display: false }
        },
        scales: {
            x: {
                ticks: { color: '#9ca3af', maxTicksLimit: 10, font: { size: 10 } },
                grid: { color: 'rgba(75, 85, 99, 0.2)' }
            },
            y: {
                min: 0,
                max: 120,
                ticks: { color: '#9ca3af', font: { size: 10 } },
                grid: { color: 'rgba(75, 85, 99, 0.2)' }
            }
        }
    }
});

let ws = null;
let reconnectTimer = null;

function connectWs() {
    if (ws && ws.readyState === WebSocket.OPEN) return;
    ws = new WebSocket(WS_URL);
    const el = document.getElementById('connectionStatus');

    ws.onopen = () => {
        el.textContent = 'Connected';
        el.className = 'px-3 py-1 rounded text-sm connected';
        if (reconnectTimer) { clearInterval(reconnectTimer); reconnectTimer = null; }
    };

    ws.onclose = () => {
        el.textContent = 'Disconnected';
        el.className = 'px-3 py-1 rounded text-sm disconnected';
        if (!reconnectTimer) reconnectTimer = setInterval(connectWs, 3000);
    };

    ws.onerror = () => { ws.close(); };
    ws.onmessage = (ev) => {
        try { render(JSON.parse(ev.data)); }
        catch (e) { console.error('Parse:', e); }
    };
}

function send(o) {
    if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(o));
}

function render(d) {
    // cards
    document.getElementById('tempValue').innerHTML = d.temp.toFixed(1) + '<span class="text-lg">°C</span>';
    document.getElementById('tempDetail').textContent = `SP: ${d.setpoint.toFixed(1)}°C | Error: ${d.error.toFixed(1)}°C`;

    document.getElementById('blowerValue').textContent = d.blower + '%';
    document.getElementById('blowerDetail').textContent = `FIS: ${d.fisOut.toFixed(1)}% | u: ${d.u_fopid.toFixed(3)}`;

    document.getElementById('rpmValue').textContent = d.rpm.toFixed(1);
    const st = ['Startup','Normal','LowWarn','HiWarn','LowErr','HiErr'];
    const sc = ['text-gray-400','text-green-400','text-yellow-400','text-yellow-400','text-red-400','text-red-400'];
    document.getElementById('rpmDetail').innerHTML = `Status: <span class="${sc[d.rpmStatus]||''}">${st[d.rpmStatus]||'?'}</span>`;

    document.getElementById('powerValue').innerHTML = d.power.toFixed(0) + '<span class="text-lg">W</span>';
    document.getElementById('powerDetail').textContent = `${d.voltage.toFixed(1)}V / ${d.current.toFixed(2)}A`;

    const badge = document.getElementById('modeBadge');
    badge.textContent = d.mode;
    badge.className = 'px-3 py-1 rounded text-sm ' + (
        d.mode === 'FUZZY' ? 'bg-green-700' :
        d.mode === 'MANUAL' ? 'bg-blue-700' : 'bg-gray-600');

    const btn = document.getElementById('modeBtn');
    if (d.mode === 'FUZZY') {
        btn.textContent = 'Fuzzy Active';
        btn.className = 'w-full bg-yellow-600 text-white py-3 rounded font-semibold cursor-default';
        btn.disabled = true;
    } else {
        btn.textContent = 'Start Fuzzy Mode';
        btn.className = 'w-full bg-green-600 hover:bg-green-700 text-white py-3 rounded font-semibold';
        btn.disabled = false;
    }

    document.getElementById('pidValues').textContent =
        `u: ${d.u_fopid.toFixed(3)}  I: ${d.integral.toFixed(2)}  D: ${d.derivative.toFixed(3)}`;
    document.getElementById('fisValues').textContent = `FIS: ${d.fisOut.toFixed(1)}%`;

    // chart
    const now = new Date();
    const ts = String(now.getHours()).padStart(2,'0') + ':' +
               String(now.getMinutes()).padStart(2,'0') + ':' +
               String(now.getSeconds()).padStart(2,'0');

    if (isFirstData) {
        for (let i = 0; i < MAX_POINTS; i++) {
            tempData.push(d.temp);
            timeLabels.push('');
        }
        isFirstData = false;
    }

    tempData.push(d.temp);
    timeLabels.push(ts);
    if (tempData.length > MAX_POINTS) { tempData.shift(); timeLabels.shift(); }

    chart.data.labels = [...timeLabels];
    chart.data.datasets[0].data = [...tempData];
    chart.data.datasets[1].data = new Array(tempData.length).fill(d.setpoint);
    chart.update('none');
}

function setSetpoint() {
    const v = parseFloat(document.getElementById('spInput').value);
    if (!isNaN(v)) send({ setpoint: v });
}
function setServo() {
    const v = parseInt(document.getElementById('servoInput').value);
    if (!isNaN(v) && v >= 0 && v <= 180) send({ servo: v });
}
function toggleMode() { send({ mode: 'FUZZY' }); }
function stopSystem() { if (confirm('EMERGENCY STOP?')) send({ stop: true }); }

function refreshLogs() {
    fetch('/api/logs')
        .then(r => r.json())
        .then(files => {
            const el = document.getElementById('logList');
            if (!files.length) { el.innerHTML = '<span class="text-gray-500">No logs</span>'; return; }
            el.innerHTML = files.map(f =>
                `<a href="/api/download?file=${encodeURIComponent(f)}" target="_blank" class="block hover:text-amber-400 text-xs">${f}</a>`
            ).join('');
        })
        .catch(() => { document.getElementById('logList').innerHTML = '<span class="text-red-400">Failed</span>'; });
}

connectWs();
refreshLogs();
setInterval(refreshLogs, 10000);
