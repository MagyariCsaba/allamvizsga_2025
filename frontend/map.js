let mapLayout;
let socket;
let isFirstUpdate = true;

// Initialize map on page load
document.addEventListener('DOMContentLoaded', function() {
    initializeMap();
    initWebSocket();
});

// Initialize WebSocket connection
function initWebSocket() {
    socket = new WebSocket('ws://localhost:8765');

    socket.addEventListener('open', function(event) {
        document.getElementById('status').innerText = 'WebSocket connected - Live tracking active';
        console.log('WebSocket connection established');
    });

    socket.addEventListener('message', function(event) {
        const message = JSON.parse(event.data);

        if (message.type === 'coordinates_update') {
            updateBicyclePosition(message.data);
            document.getElementById('status').innerText =
                'Last update: ' + new Date().toLocaleTimeString() +
                ' - Position: ' + message.data.rear_lat.toFixed(6) + ', ' + message.data.rear_lon.toFixed(6);
        }
    });

    socket.addEventListener('close', function(event) {
        document.getElementById('status').innerText = 'WebSocket disconnected. Reconnecting...';
        console.log('WebSocket connection closed. Attempting to reconnect...');
        setTimeout(initWebSocket, 2000);
    });

    socket.addEventListener('error', function(event) {
        document.getElementById('status').innerText = 'WebSocket error';
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
                    line: {width: 2, color: 'gray'},
                    hoverinfo: 'none',
                    name: 'Streets'
                },
                // Bicycle frame (live position)
                {
                    type: 'scattermapbox',
                    lon: [],
                    lat: [],
                    mode: 'lines+markers',
                    marker: {size: 10, color: 'blue'},
                    line: {width: 4, color: 'blue'},
                    name: 'Bicycle Frame'
                },
                // Front wheel (live position)
                {
                    type: 'scattermapbox',
                    lon: [],
                    lat: [],
                    mode: 'lines+markers',
                    marker: {size: 8, color: 'red'},
                    line: {width: 3, color: 'red'},
                    name: 'Front Wheel'
                }
                ];

            Plotly.newPlot('mapPlot', mapData, mapLayout);
        })
        .catch(error => {
            console.error('Error loading map configuration:', error);
            document.getElementById('status').innerText = 'Error loading map configuration';
        });
}

// Function to update bicycle position (live tracking)
function updateBicyclePosition(coords) {
    Plotly.restyle('mapPlot', {
        'lon': [[coords.rear_lon, coords.front_lon]],
        'lat': [[coords.rear_lat, coords.front_lat]]
    }, [1]);

    Plotly.restyle('mapPlot', {
        'lon': [[coords.front_lon, coords.wheel_lon]],
        'lat': [[coords.front_lat, coords.wheel_lat]]
    }, [2]);

    centerMapOnBicycle(coords.rear_lat, coords.rear_lon);
}

function centerMapOnBicycle(lat, lon) {
    const updatedLayout = {
        'mapbox.center': {
            lat: lat,
            lon: lon
        }
    };

    if (isFirstUpdate) {
        updatedLayout['mapbox.zoom'] = 15;
        isFirstUpdate = false;
    }

    Plotly.relayout('mapPlot', updatedLayout);
}