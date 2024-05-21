import depthai as dai
import cv2
import multiprocessing
import signal
import time

def list_and_stream_devices(stop_event):
    available_devices = dai.Device.getAllAvailableDevices()
    if available_devices:
        print("Available devices:")
        processes = []
        for device_info in available_devices:
            print(f"Device name: {device_info.getMxId()}")
            process = multiprocessing.Process(target=start_camera_stream, args=(device_info, stop_event))
            processes.append(process)
            process.start()
            time.sleep(1)  # Adding a delay to prevent resource contention
        for process in processes:
            process.join()
    else:
        print("No devices found.")

def start_camera_stream(device_info, stop_event):
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
                window_name = f"Video Feed {device_info.getMxId()}"
                cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

                while not stop_event.is_set():
                    video_packet = q_video.get()
                    frame = cv2.imdecode(video_packet.getData(), cv2.IMREAD_COLOR)
                    cv2.imshow(window_name, frame)

                    if cv2.waitKey(1) == ord('q'):
                        stop_event.set()
                        break

                cv2.destroyAllWindows()
        except Exception as e:
            print(f"Error with device {device_info.getMxId()}: {e}")
            print("Attempting to reconnect...")
            time.sleep(5)  # Wait before retrying

def signal_handler(sig, frame, stop_event):
    print("Ctrl+C received. Stopping processes...")
    stop_event.set()

if __name__ == "__main__":
    stop_event = multiprocessing.Event()
    signal.signal(signal.SIGINT, lambda sig, frame: signal_handler(sig, frame, stop_event))
    list_and_stream_devices(stop_event)
