import asyncio
import websockets

async def async_input(prompt: str = ""):
    return await asyncio.get_event_loop().run_in_executor(None, input, prompt)

async def send_commands(websocket):
    while True:
        try:
            command = await async_input("Enter command: ")
            await websocket.send(command)
            print(f"Sent: {command}")
        except Exception as e:
            print(f"Error sending command: {e}")
            break

async def receive_messages(websocket):
    try:
        async for message in websocket:
            print(f"Received from server: {message}")
    except websockets.exceptions.ConnectionClosedError:
        print("Connection closed by server.")
    except asyncio.CancelledError:
        print("Receive task cancelled.")
    except Exception as e:
        print(f"Unexpected receive error: {e}")

async def main():
    uri = "ws://10.24.0.173"
    try:
        async with websockets.connect(uri) as websocket:
            print(f"Connected to {uri}")
            # Run sender and receiver concurrently
            await asyncio.gather(
                send_commands(websocket),
                receive_messages(websocket)
            )
    except Exception as e:
        print(f"Could not connect to server: {e}")

asyncio.run(main())
