import RPi.GPIO as GPIO
import time

FIRE = 21

print("Setting up...")
GPIO.setmode(GPIO.BCM)
GPIO.setup(FIRE, GPIO.OUT)
GPIO.output(FIRE, GPIO.LOW)

print("Fire.")
GPIO.output(FIRE, GPIO.HIGH)
time.sleep(3)
print("Stop.")
GPIO.output(FIRE, GPIO.LOW)

GPIO.cleanup(FIRE)

print("Exit")
exit



