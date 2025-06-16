import asyncio
import json
import websockets
import os
import threading
from pathlib import Path
from backend.route_handler import RouteHandler


class WebSocketServer:
    def __init__(self, port=8765):
        self.port = port
        self.connected_clients = set()
        self.frontend_dir = Path(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))).joinpath("frontend")
        self.running = False
        self.server = None
        self.server_thread = None
        self.route_handler = RouteHandler()

    async def handler(self, websocket):
        self.connected_clients.add(websocket)
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    await self.handle_client_message(websocket, data)
                except json.JSONDecodeError:
                    await websocket.send(json.dumps({
                        'type': 'error',
                        'message': 'Invalid JSON format'
                    }))
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.connected_clients.remove(websocket)

    async def handle_client_message(self, websocket, data):
        message_type = data.get('type')

        if message_type == 'get_route':
            start_date = data.get('start_date')
            start_time = data.get('start_time')
            end_date = data.get('end_date')
            end_time = data.get('end_time')

            route_data = self.route_handler.get_route_for_period(
                start_date, start_time, end_date, end_time
            )

            response = {
                'type': 'route_data',
                'data': route_data,
                'start_date': start_date,
                'start_time': start_time,
                'end_date': end_date,
                'end_time': end_time
            }

            await websocket.send(json.dumps(response))

    async def broadcast_coordinates(self, coordinates):
        if not self.connected_clients:
            return

        message = json.dumps({
            'type': 'coordinates_update',
            'data': coordinates
        })

        await asyncio.gather(
            *[client.send(message) for client in self.connected_clients],
            return_exceptions=True
        )

    async def start_server(self):
        self.running = True
        async with websockets.serve(self.handler, "localhost", self.port):
            while self.running:
                await asyncio.sleep(1)

    def start(self):
        self.server_thread = threading.Thread(target=self._run_server)
        self.server_thread.daemon = True
        self.server_thread.start()
        print(f"WebSocket server started at ws://localhost:{self.port}")

    def _run_server(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        server_coroutine = self.start_server()

        loop.run_until_complete(server_coroutine)
        loop.close()

    def update_coordinates(self, coordinates):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        try:
            loop.run_until_complete(self.broadcast_coordinates(coordinates))
        finally:
            loop.close()

    def stop(self):
        self.running = False
        if self.server_thread:
            self.server_thread.join(timeout=2)
            print("WebSocket server stopped")