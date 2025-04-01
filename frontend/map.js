// Global variable to store map layout
let mapLayout;

// Initialize map on page load
document.addEventListener('DOMContentLoaded', function() {
    initializeMap();
});

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

            // Start regular updates

            setInterval(updateCoordinates, 1000);
        })
        .catch(error => {
            console.error('Error loading map configuration:', error);
            document.getElementById('status').innerText = 'Error loading map configuration';
        });
}

// Function to update coordinates
function updateCoordinates() {
    // Add random query parameter to prevent caching
    fetch('coordinates.json?_=' + new Date().getTime())
        .then(response => response.json())
        .then(coords => {

            Plotly.restyle('mapPlot', {
                'lon': [[coords.rear_lon, coords.front_lon]],
                'lat': [[coords.rear_lat, coords.front_lat]]
            }, [1]);

            Plotly.restyle('mapPlot', {
                'lon': [[coords.front_lon, coords.wheel_lon]],
                'lat': [[coords.front_lat, coords.wheel_lat]]
            }, [2]);

            document.getElementById('status').innerText =
                'Last update: ' + new Date().toLocaleTimeString() +
                ' - Position: ' + coords.rear_lat.toFixed(6) + ', ' + coords.rear_lon.toFixed(6);
        })
        .catch(error => {
            console.error('Error fetching coordinates:', error);
        });
}