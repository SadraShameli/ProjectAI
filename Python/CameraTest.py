import time
from picamera2 import Picamera2

picam2 = Picamera2()
camera_config = picam2.create_still_configuration(
    main={"size": (2448, 3264)}
)
picam2.configure(camera_config)
picam2.start()
time.sleep(2)

