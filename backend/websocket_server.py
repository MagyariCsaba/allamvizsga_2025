import asyncio
import json
import websockets
import os
import threading
from pathlib import Path


class WebSocketServer:
    def __init__(self, port=8765):
        self.port = port
        self.connected_clients = set()
        self.frontend_dir = Path(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))).joinpath("frontend")
        self.json_file = self.frontend_dir.joinpath("coordinates.json")
        self.running = False
        self.server = None
        self.server_thread = None

    async def handler(self, websocket):
        # Register client
        self.connected_clients.add(websocket)
        try:
            # Keep connection open
            async for message in websocket:
                # We don't expect any messages from clients in this app
                pass
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            # Unregister client
            self.connected_clients.remove(websocket)

    async def broadcast_coordinates(self, coordinates):
        if not self.connected_clients:
            return

        # Convert to JSON
        message = json.dumps(coordinates)

        # Send to all connected clients
        await asyncio.gather(
            *[client.send(message) for client in self.connected_clients]
        )

    async def start_server(self):
        self.running = True
        async with websockets.serve(self.handler, "localhost", self.port):
            while self.running:
                await asyncio.sleep(1)

    def start(self):
        """Start the WebSocket server in a separate thread"""
        self.server_thread = threading.Thread(target=self._run_server)
        self.server_thread.daemon = True
        self.server_thread.start()
        print(f"WebSocket server started at ws://localhost:{self.port}")

    def _run_server(self):
        """Run the asyncio event loop in the thread"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        # Create the server coroutine
        server_coroutine = self.start_server()

        # Run the server until stopped
        loop.run_until_complete(server_coroutine)
        loop.close()

    def update_coordinates(self, coordinates):
        """Update coordinates and broadcast to all clients"""
        # Save coordinates to file (for backward compatibility)
        with open(self.json_file, 'w') as f:
            json.dump(coordinates, f)
            f.flush()
            os.fsync(f.fileno())

        # Create a new event loop for broadcasting
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        # Broadcast the coordinates
        try:
            loop.run_until_complete(self.broadcast_coordinates(coordinates))
        finally:
            loop.close()

    def stop(self):
        """Stop the WebSocket server"""
        self.running = False
        if self.server_thread:
            self.server_thread.join(timeout=2)
            print("WebSocket server stopped")