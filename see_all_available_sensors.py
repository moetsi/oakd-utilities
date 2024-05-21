import depthai as dai
import cv2
import multiprocessing
import numpy as np
import signal
import time
import os

def list_and_stream_devices(stop_event, feed_queue, processes):
    available_devices = dai.Device.getAllAvailableDevices()
    if available_devices:
        print("Available devices:")
        for device_info in available_devices:
            print(f"Device name: {device_info.getMxId()}, {device_info.name}")
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
                ip_address = device_info.name
                q_video = device.getOutputQueue(name="video", maxSize=30, blocking=True)
                while not stop_event.is_set():
                    video_packet = q_video.get()
                    frame = cv2.imdecode(video_packet.getData(), cv2.IMREAD_COLOR)
                    
                    # Add IP address text with black outline and white text
                    font_scale = 3
                    thickness = 3
                    text_size = cv2.getTextSize(ip_address, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)[0]
                    text_x = frame.shape[1] - text_size[0] - 10  # 10 pixels padding from the right edge
                    text_y = frame.shape[0] - 20  # 20 pixels padding from the bottom edge
                    
                    # Draw black outline
                    outline_thickness = thickness + 4
                    cv2.putText(frame, ip_address, (text_x, text_y), 
                                cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), outline_thickness, cv2.LINE_AA)
                    
                    # Draw white text
                    cv2.putText(frame, ip_address, (text_x, text_y), 
                                cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)
                    
                    feed_queue.put((device_info.getMxId(), frame))
                    if stop_event.is_set():
                        break
        except Exception as e:
            if stop_event.is_set():
                break
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

def signal_handler(sig, frame, stop_event, processes):
    print("Ctrl+C received. Stopping processes...")
    stop_event.set()
    for process in processes:
        if process.is_alive():
            process.terminate()
            process.join()
    os._exit(0)

if __name__ == "__main__":
    stop_event = multiprocessing.Event()
    feed_queue = multiprocessing.Queue()
    
    processes = []
    display_process = multiprocessing.Process(target=display_feeds, args=(stop_event, feed_queue))
    processes.append(display_process)
    display_process.start()
    
    signal.signal(signal.SIGINT, lambda sig, frame: signal_handler(sig, frame, stop_event, processes))
    
    try:
        list_and_stream_devices(stop_event, feed_queue, processes)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received in main. Setting stop event...")
        stop_event.set()
    
    display_process.join()
    while not feed_queue.empty():
        feed_queue.get()  # Ensure queue is emptied
