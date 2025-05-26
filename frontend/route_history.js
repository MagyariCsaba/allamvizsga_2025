// Global variables
let mapLayout;
let socket;
let currentRouteData = null;

// Initialize map on page load
document.addEventListener('DOMContentLoaded', function () {
    initializeMap();
    initWebSocket();
    setDefaultDateTime();
});

// Set default date and time values
function setDefaultDateTime() {
    const now = new Date();
    const yesterday = new Date(now.getTime() - 24 * 60 * 60 * 1000);

    // Format dates for input fields
    const todayStr = now.toISOString().split('T')[0];
    const yesterdayStr = yesterday.toISOString().split('T')[0];
    const currentTimeStr = now.toTimeString().split(' ')[0];
    const yesterdayTimeStr = yesterday.toTimeString().split(' ')[0];

    document.getElementById('start-date').value = yesterdayStr;
    document.getElementById('start-time').value = yesterdayTimeStr;
    document.getElementById('end-date').value = todayStr;
    document.getElementById('end-time').value = currentTimeStr;
}

// Initialize WebSocket connection
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

// Function to initialize the map
function initializeMap() {
    fetch('map_config.json')
        .then(response => response.json())
        .then(config => {
            mapLayout = config.layout;

            const mapData = [
                // Streets layer
                {
                    type: 'scattermapbox',
                    lon: config.streets.lon,
                    lat: config.streets.lat,
                    mode: 'lines',
                    line: { width: 2, color: 'gray' },
                    hoverinfo: 'none',
                    name: 'Streets'
                },
                // Historical route trace
                {
                    type: 'scattermapbox',
                    lon: [],
                    lat: [],
                    mode: 'lines+markers',
                    line: { width: 4, color: 'green' },
                    marker: { size: 6, color: 'green' },
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

// Function to query route data
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

    // Disable button during request
    const queryButton = document.getElementById('query-route');
    queryButton.disabled = true;
    queryButton.innerText = 'Querying...';

    // Send request via WebSocket
    const request = {
        type: 'get_route',
        start_date: startDate,
        start_time: startTime,
        end_date: endDate,
        end_time: endTime
    };

    socket.send(JSON.stringify(request));
}

// Handle route data response
function handleRouteData(message) {
    // Re-enable button
    const queryButton = document.getElementById('query-route');
    queryButton.disabled = false;
    queryButton.innerText = 'Get Route';

    const routeData = message.data;
    const routeInfo = document.getElementById('route-info');

    if (routeData.length === 0) {
        routeInfo.innerHTML = '<span style="color: orange;">No data found in the selected time period.</span>';
        return;
    }

    // Store route data
    currentRouteData = routeData;

    // Extract coordinates and timestamps for plotting
    const routeLons = routeData.map(point => point.lon);
    const routeLats = routeData.map(point => point.lat);
    const timestamps = routeData.map(point => new Date(point.timestamp).toLocaleString('hu-HU'));

    // Update the route trace on the map
    Plotly.restyle('mapPlot', {
        'lon': [routeLons],
        'lat': [routeLats],
        'text': [timestamps]
    }, [1]);

    // Update route info
    const startTime = new Date(routeData[0].timestamp).toLocaleString('hu-HU');
    const endTime = new Date(routeData[routeData.length - 1].timestamp).toLocaleString('hu-HU');

    routeInfo.innerHTML = `
        <strong style="color: green;">Route loaded successfully:</strong><br>
        Number of points: ${routeData.length}<br>
        Start: ${startTime}<br>
        End: ${endTime}
    `;

    // Center map on route
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

// Function to clear route
function clearRoute() {
    // Clear route trace from map
    Plotly.restyle('mapPlot', {
        'lon': [[]],
        'lat': [[]],
        'text': [[]]
    }, [1]);

    // Clear route info
    document.getElementById('route-info').innerHTML = '';
    currentRouteData = null;

    // Reset map view
    Plotly.relayout('mapPlot', {
        'mapbox.center': {
            lat: mapLayout.mapbox.center.lat,
            lon: mapLayout.mapbox.center.lon
        },
        'mapbox.zoom': 13
    });
}
