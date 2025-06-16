let mapLayout;
let socket;
let currentRouteData = null;

document.addEventListener('DOMContentLoaded', function () {
    initializeMap();
    initWebSocket();
    setDefaultDateTime();
});

function setDefaultDateTime() {
    const now = new Date();
    const yesterday = new Date(now.getTime() - 24 * 60 * 60 * 1000);

    const todayStr = now.toISOString().split('T')[0];
    const yesterdayStr = yesterday.toISOString().split('T')[0];
    const currentTimeStr = now.toTimeString().split(' ')[0];
    const yesterdayTimeStr = yesterday.toTimeString().split(' ')[0];

    document.getElementById('start-date').value = yesterdayStr;
    document.getElementById('start-time').value = yesterdayTimeStr;
    document.getElementById('end-date').value = todayStr;
    document.getElementById('end-time').value = currentTimeStr;
}

function initWebSocket() {
    socket = new WebSocket('ws://localhost:8765');

    socket.addEventListener('open', function (event) {
        document.getElementById('status').innerText = 'Connected to database - Ready to query routes';
        console.log('WebSocket connection established');
    });

    socket.addEventListener('message', function (event) {
        const message = JSON.parse(event.data);

        if (message.type === 'route_data') {
            handleRouteData(message);
        }
    });

    socket.addEventListener('close', function (event) {
        document.getElementById('status').innerText = 'Database connection lost. Reconnecting...';
        console.log('WebSocket connection closed. Attempting to reconnect...');
        setTimeout(initWebSocket, 2000);
    });

    socket.addEventListener('error', function (event) {
        document.getElementById('status').innerText = 'Database connection error';
        console.error('WebSocket error:', event);
    });
}

function initializeMap() {
    fetch('map_config.json')
        .then(response => response.json())
        .then(config => {
            mapLayout = config.layout;

            const mapData = [
                {
                    type: 'scattermapbox',
                    lon: config.streets.lon,
                    lat: config.streets.lat,
                    mode: 'lines',
                    line: { width: 2, color: 'gray' },
                    hoverinfo: 'none',
                    name: 'Streets'
                },

                {
                    type: 'scattermapbox',
                    lon: [],
                    lat: [],
                    mode: 'lines+markers',
                    line: { width: 4, color: 'green' },
                    marker: { size: 6, color: 'red' },
                    name: 'Historical Route',
                    hovertemplate: 'Lat: %{lat}<br>Lon: %{lon}<br>Time: %{text}<extra></extra>',
                    text: []
                }
            ];

            Plotly.newPlot('mapPlot', mapData, mapLayout);
        })
        .catch(error => {
            console.error('Error loading map configuration:', error);
            document.getElementById('status').innerText = 'Error loading map configuration';
        });
}

function queryRoute() {
    const startDate = document.getElementById('start-date').value;
    const startTime = document.getElementById('start-time').value;
    const endDate = document.getElementById('end-date').value;
    const endTime = document.getElementById('end-time').value;

    if (!startDate || !startTime || !endDate || !endTime) {
        alert('Please select correct date and time values');
        return;
    }

    if (new Date(startDate + ' ' + startTime) >= new Date(endDate + ' ' + endTime)) {
        alert('Start time must be before end time');
        return;
    }

    const queryButton = document.getElementById('query-route');
    queryButton.disabled = true;
    queryButton.innerText = 'Querying...';

    const request = {
        type: 'get_route',
        start_date: startDate,
        start_time: startTime,
        end_date: endDate,
        end_time: endTime
    };

    socket.send(JSON.stringify(request));
}

function handleRouteData(message) {
    const queryButton = document.getElementById('query-route');
    queryButton.disabled = false;
    queryButton.innerText = 'Get Route';

    const routeData = message.data;
    const routeInfo = document.getElementById('route-info');

    if (routeData.length === 0) {
        routeInfo.innerHTML = '<span style="color: orange;">No data found in the selected time period.</span>';
        return;
    }

    currentRouteData = routeData;

    const routeLons = routeData.map(point => point.lon);
    const routeLats = routeData.map(point => point.lat);
    const timestamps = routeData.map(point => new Date(point.timestamp).toLocaleString('hu-HU'));

    Plotly.restyle('mapPlot', {
        'lon': [routeLons],
        'lat': [routeLats],
        'text': [timestamps]
    }, [1]);

    const startTime = new Date(routeData[0].timestamp).toLocaleString('hu-HU');
    const endTime = new Date(routeData[routeData.length - 1].timestamp).toLocaleString('hu-HU');

    routeInfo.innerHTML = `
        <strong style="color: green;">Route loaded successfully:</strong><br>
        Number of points: ${routeData.length}<br>
        Start: ${startTime}<br>
        End: ${endTime}
    `;

    if (routeData.length > 0) {
        const centerLat = routeLats.reduce((a, b) => a + b, 0) / routeLats.length;
        const centerLon = routeLons.reduce((a, b) => a + b, 0) / routeLons.length;

        Plotly.relayout('mapPlot', {
            'mapbox.center': {
                lat: centerLat,
                lon: centerLon
            },
            'mapbox.zoom': 13
        });
    }
}

function clearRoute() {
    Plotly.restyle('mapPlot', {
        'lon': [[]],
        'lat': [[]],
        'text': [[]]
    }, [1]);

    document.getElementById('route-info').innerHTML = '';
    currentRouteData = null;

    Plotly.relayout('mapPlot', {
        'mapbox.center': {
            lat: mapLayout.mapbox.center.lat,
            lon: mapLayout.mapbox.center.lon
        },
        'mapbox.zoom': 13
    });
}
