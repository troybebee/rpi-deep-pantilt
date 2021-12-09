import logging
from multiprocessing import Value, Process, Manager, Event

import pantilthat as pth
import signal
import sys
import time
import RPi.GPIO as GPIO

from rpi_deep_pantilt.detect.camera import run_pantilt_detect
from rpi_deep_pantilt.control.pid import PIDController

logging.basicConfig()
LOGLEVEL = logging.getLogger().getEffectiveLevel()

# Fox1 setup
GPIO.setmode(GPIO.BCM)
FOX1 = 21
GPIO.setup(FOX1, GPIO.OUT, initial=GPIO.LOW)


RESOLUTION = (320, 320)

PAN_SERVO_MIN = -90
PAN_SERVO_MAX = 90

TILT_SERVO_MIN = -45
TILT_SERVO_MAX = 45 

CENTER = (
    RESOLUTION[0] // 2,
    RESOLUTION[1] // 2
)

# function to handle keyboard interrupt


def signal_handler(sig, frame):
    # print a status message
    print("[INFO] You pressed `ctrl + c`! Exiting...")

    # disable the servos
    pth.servo_enable(1, False)
    pth.servo_enable(2, False)

    # disable fire control
    GPIO.output(FOX1, GPIO.LOW)
    GPIO.cleanup(FOX1)

    # exit
    sys.exit()

# Fox1 Fire Control 
def set_fox1(fox1):
    GPIO.output(FOX1, GPIO.LOW)
    while True:
       fox1.wait()
       GPIO.output(FOX1, GPIO.HIGH)
       time.sleep(1)
       GPIO.output(FOX1, GPIO.LOW)
       fox1.clear() 

def set_reset(reset, run):
    while True:
       run.set()
       reset.wait()
       print("Reset event ...")
       run.clear()
       time.sleep(1)
       print("Resume ...")
       reset.clear()
       run.set()

def in_range(val, start, end):
    # determine the input vale is in the supplied range
    return (val >= start and val <= end)


def set_servos(pan, tilt, reset, run):
    # signal trap to handle keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)

    while True:
        if reset.is_set():
           pth.pan(0)
           pth.tilt(0)
           
        run.wait()
        pan_angle = -1 * pan.value
        tilt_angle = tilt.value

        # if the pan angle is within the range, pan
        if in_range(pan_angle, PAN_SERVO_MIN, PAN_SERVO_MAX):
            pth.pan(pan_angle)
        else:
            logging.info(f'pan_angle not in range {pan_angle}')
            reset.set() 

        if in_range(tilt_angle, TILT_SERVO_MIN, TILT_SERVO_MAX):
            pth.tilt(tilt_angle)
        else:
            logging.info(f'tilt_angle not in range {tilt_angle}')
            reset.set()

def pid_process(output, p, i, d, box_coord, origin_coord, action, reset, run):
    # signal trap to handle keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)

    # create a PID and initialize it
    p = PIDController(p.value, i.value, d.value)
    p.reset()

    # loop indefinitely
    while True:
       if reset.is_set():
           logging.info(f'Reset PID for {action} ....')
           p.reset()
       run.wait()
       error = origin_coord - box_coord.value
       output.value = p.update(error)
       # logging.info(f'{action} error {error} angle: {output.value}')
 
def pantilt_process_manager(
    model_cls,
    labels=('person',),
    rotation=0
):

    pth.servo_enable(1, True)
    pth.servo_enable(2, True)
    with Manager() as manager:
        # set initial bounding box (x, y)-coordinates to center of frame
        center_x = manager.Value('i', 0)
        center_y = manager.Value('i', 0)

        center_x.value = RESOLUTION[0] // 2
        center_y.value = RESOLUTION[1] // 2

        # Reset signals
        reset = Event()
        run = Event() 

        # Fire/Fox1 signal
        fox1 = Event()

        # pan and tilt angles updated by independent PID processes
        pan = manager.Value('i', 0)
        tilt = manager.Value('i', 0)

        # PID gains for panning

        pan_p = manager.Value('f', 0.05)
        # 0 time integral gain until inferencing is faster than ~50ms
        pan_i = manager.Value('f', 0.1)
        pan_d = manager.Value('f', 0)

        # PID gains for tilting
        tilt_p = manager.Value('f', 0.15)
        # 0 time integral gain until inferencing is faster than ~50ms
        tilt_i = manager.Value('f', 0.2)
        tilt_d = manager.Value('f', 0)

        detect_processr = Process(target=run_pantilt_detect,
                                  args=(reset, run, fox1, center_x, center_y, labels, model_cls, rotation))

        pan_process = Process(target=pid_process,
                              args=(pan, pan_p, pan_i, pan_d, center_x, CENTER[0], 'pan', reset, run))

        tilt_process = Process(target=pid_process,
                               args=(tilt, tilt_p, tilt_i, tilt_d, center_y, CENTER[1], 'tilt', reset, run))

        servo_process = Process(target=set_servos, args=(pan, tilt, reset, run))

        fox1_process = Process(target=set_fox1, args=(fox1,))

        reset_process = Process(target=set_reset, args=(reset, run))

        detect_processr.start()
        pan_process.start()
        tilt_process.start()
        servo_process.start()
        fox1_process.start()
        reset_process.start()

        detect_processr.join()
        pan_process.join()
        tilt_process.join()
        servo_process.join()
        fox1_process.join()
        reset_process.join()


if __name__ == '__main__':
    pantilt_process_manager()
