import asyncio
import websockets
import numpy as np
import cv2

async def receive_image(websocket, path):
    try:
        image_data = await websocket.recv()
        
        # Convert received bytes to a numpy array
        np_arr = np.frombuffer(image_data, np.uint8)
        
        # Decode the image (assuming it is encoded as JPEG)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if image is not None:
            # Display the image in a window
            cv2.imshow("Received Image", image)
            cv2.waitKey(1)  # Needed to refresh the window
        else:
            print("Failed to decode image.")

    except Exception as e:
        print(f"Error receiving image: {e}")

async def main():
    server = await websockets.serve(receive_image, "0.0.0.0", 8765)
    await server.wait_closed()

asyncio.run(main())
