import depthai as dai
import cv2
import multiprocessing
import numpy as np
import signal
import time

def list_and_stream_devices(stop_event, feed_queue):
    available_devices = dai.Device.getAllAvailableDevices()
    if available_devices:
        print("Available devices:")
        processes = []
        for device_info in available_devices:
            print(f"Device name: {device_info.getMxId()}")
            process = multiprocessing.Process(target=start_camera_stream, args=(device_info, stop_event, feed_queue))
            processes.append(process)
            process.start()
            time.sleep(1)  # Adding a delay to prevent resource contention
        for process in processes:
            process.join()
    else:
        print("No devices found.")

def start_camera_stream(device_info, stop_event, feed_queue):
    while not stop_event.is_set():
        try:
            print(f"Starting stream for device {device_info.getMxId()}")
            pipeline = dai.Pipeline()

            cam_rgb = pipeline.create(dai.node.ColorCamera)
            cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)  # Updated to use CAM_A
            cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            cam_rgb.setFps(5)  # Set low FPS to reduce bandwidth usage
            cam_rgb.setInterleaved(False)
            cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

            video_encoder = pipeline.create(dai.node.VideoEncoder)
            video_encoder.setDefaultProfilePreset(5, dai.VideoEncoderProperties.Profile.MJPEG)
            cam_rgb.video.link(video_encoder.input)

            xout = pipeline.create(dai.node.XLinkOut)
            xout.setStreamName("video")
            video_encoder.bitstream.link(xout.input)

            with dai.Device(pipeline, usb2Mode=True) as device:  # Using usb2Mode for compatibility
                q_video = device.getOutputQueue(name="video", maxSize=30, blocking=True)
                while not stop_event.is_set():
                    video_packet = q_video.get()
                    frame = cv2.imdecode(video_packet.getData(), cv2.IMREAD_COLOR)
                    feed_queue.put((device_info.getMxId(), frame))
                    if stop_event.is_set():
                        break
        except Exception as e:
            print(f"Error with device {device_info.getMxId()}: {e}")
            print("Attempting to reconnect...")
            time.sleep(5)  # Wait before retrying

def display_feeds(stop_event, feed_queue):
    cv2.namedWindow("Camera Feeds", cv2.WINDOW_NORMAL)
    feeds = {}
    while not stop_event.is_set():
        while not feed_queue.empty():
            device_id, frame = feed_queue.get()
            feeds[device_id] = frame
        
        if feeds:
            # Determine the grid size
            num_feeds = len(feeds)
            grid_size = int(np.ceil(np.sqrt(num_feeds)))
            h, w, _ = list(feeds.values())[0].shape
            canvas = np.zeros((h * grid_size, w * grid_size, 3), dtype=np.uint8)
            
            for idx, (device_id, frame) in enumerate(feeds.items()):
                y, x = divmod(idx, grid_size)
                canvas[y*h:(y+1)*h, x*w:(x+1)*w] = frame

            cv2.imshow("Camera Feeds", canvas)
        
        if cv2.waitKey(1) == ord('q'):
            stop_event.set()
            break
    
    cv2.destroyAllWindows()

def signal_handler(sig, frame, stop_event):
    print("Ctrl+C received. Stopping processes...")
    stop_event.set()

if __name__ == "__main__":
    stop_event = multiprocessing.Event()
    feed_queue = multiprocessing.Queue()
    signal.signal(signal.SIGINT, lambda sig, frame: signal_handler(sig, frame, stop_event))
    
    display_process = multiprocessing.Process(target=display_feeds, args=(stop_event, feed_queue))
    display_process.start()
    
    list_and_stream_devices(stop_event, feed_queue)
    
    display_process.join()
