class CameraNotConfigured(RuntimeError):
    pass


def open_camera():
    """Open Camera V2 with Picamera2 on a compatible Raspberry Pi OS installation."""
    try:
        from picamera2 import Picamera2
    except ImportError as exc:
        raise CameraNotConfigured("Picamera2 is not installed on this Raspberry Pi OS.") from exc
    camera = Picamera2()
    camera.configure(camera.create_video_configuration(main={"size": (640, 480), "format": "RGB888"}))
    camera.start()
    return camera
