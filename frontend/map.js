// Global variable to store map layout
let mapLayout;
let socket;


// Initialize map on page load
document.addEventListener('DOMContentLoaded', function() {
    initializeMap();
    initWebSocket();
});

// Initialize WebSocket connection
function initWebSocket() {
    // Create WebSocket connection
    socket = new WebSocket('ws://localhost:8765');

    // Connection opened handler
    socket.addEventListener('open', function(event) {
        document.getElementById('status').innerText = 'WebSocket connected';
        console.log('WebSocket connection established');
    });

    // Listen for messages
    socket.addEventListener('message', function(event) {
        const coords = JSON.parse(event.data);
        updateBicyclePosition(coords);

        document.getElementById('status').innerText =
            'Last update: ' + new Date().toLocaleTimeString() +
            ' - Position: ' + coords.rear_lat.toFixed(6) + ', ' + coords.rear_lon.toFixed(6);
    });

    // Connection closed handler
    socket.addEventListener('close', function(event) {
        document.getElementById('status').innerText = 'WebSocket disconnected. Reconnecting...';
        console.log('WebSocket connection closed. Attempting to reconnect...');
        // Try to reconnect after 2 seconds
        setTimeout(initWebSocket, 2000);
    });

    // Error handler
    socket.addEventListener('error', function(event) {
        document.getElementById('status').innerText = 'WebSocket error';
        console.error('WebSocket error:', event);
    });
}

// Function to initialize the map
function initializeMap() {
    // Fetch map configuration from the server
    fetch('map_config.json')
        .then(response => response.json())
        .then(config => {
            // Store map layout for later use
            mapLayout = config.layout;

            // Create the initial map
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
                // Bicycle frame (initially empty)
                {
                    type: 'scattermapbox',
                    lon: [],
                    lat: [],
                    mode: 'lines+markers',
                    marker: {size: 10, color: 'blue'},
                    line: {width: 4, color: 'blue'},
                    name: 'Bicycle Frame'
                },
                // Front wheel (initially empty)
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

            // Create the map

            Plotly.newPlot('mapPlot', mapData, mapLayout);

            /// For backward compatibility - keep polling until WebSocket connection is established
            checkAndLoadInitialPosition();

        })
        .catch(error => {
            console.error('Error loading map configuration:', error);
            document.getElementById('status').innerText = 'Error loading map configuration';
        });
}

// Function to check for initial position data
function checkAndLoadInitialPosition() {
    // Only use this as a fallback if WebSocket isn't connected yet
    if (socket && socket.readyState === WebSocket.OPEN) {
        return;  // WebSocket is connected, no need for HTTP fallback
    }

    // Add random query parameter to prevent caching
    fetch('coordinates.json?_=' + new Date().getTime())
        .then(response => response.json())
        .then(coords => {
            updateBicyclePosition(coords);

            document.getElementById('status').innerText =
                'Last update (HTTP): ' + new Date().toLocaleTimeString() +
                ' - Position: ' + coords.rear_lat.toFixed(6) + ', ' + coords.rear_lon.toFixed(6);
        })
        .catch(error => {
            console.error('Error fetching coordinates:', error);
        });
}

// Function to update bicycle position
function updateBicyclePosition(coords) {
    Plotly.restyle('mapPlot', {
        'lon': [[coords.rear_lon, coords.front_lon]],
        'lat': [[coords.rear_lat, coords.front_lat]]
    }, [1]);

    Plotly.restyle('mapPlot', {
        'lon': [[coords.front_lon, coords.wheel_lon]],
        'lat': [[coords.front_lat, coords.wheel_lat]]
    }, [2]);
}