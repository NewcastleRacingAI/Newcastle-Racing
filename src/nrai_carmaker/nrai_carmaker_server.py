import numpy as np
import cv2
import asyncio


async def show_image(queue: asyncio.Queue[np.ndarray]):
    while True:
        image = await queue.get()
        cv2.imshow("RGB" if image.dtype == np.uint8 else "Depth", image)
        cv2.waitKey(1)
        queue.task_done()


def handle_input(queue: asyncio.Queue[np.ndarray]):
    async def handle(reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        INT_SIZE = 8
        HEADER_SIZE = 3 * INT_SIZE  # 3 values, each 8 bytes (64 bits)
        while True:
            try:
                data = await reader.read(HEADER_SIZE)
            except ConnectionResetError:
                return

            width, height, data_length = [
                int.from_bytes(data[i : i + INT_SIZE], byteorder="little", signed=False)
                for i in range(0, HEADER_SIZE, INT_SIZE)
            ]
            size = int(data_length / (width * height))

            received = 0
            chunks = []

            while received < data_length:
                try:
                    chunk = await reader.read(data_length - received)
                except ConnectionResetError:
                    return
                received += len(chunk)
                chunks.append(chunk)
            image_bytes = b"".join(chunks)
            print("Done")

            if size == 3: # cv2 expects BRG, not RGB
                image = np.frombuffer(image_bytes, np.uint8).reshape(height, width, size)[:, :, ::-1]
            elif size == 2: # We normalize by 2^16 and invert the values. white=close, black=Far
                image = 1 - np.frombuffer(image_bytes, np.uint16).reshape(height, width, 1) / 65535
            await queue.put(image)

    return handle


async def main():
    HOST, PORT = "0.0.0.0", 8000
    queue = asyncio.Queue()

    server = await asyncio.start_server(handle_input(queue), HOST, PORT)
    addrs = ", ".join(str(sock.getsockname()) for sock in server.sockets)
    print(f"Serving on {addrs}")

    async with server:
        try:
            await asyncio.gather(server.serve_forever(), show_image(queue))
        except asyncio.CancelledError:
            print("Closing server...")


if __name__ == "__main__":
    asyncio.run(main())
