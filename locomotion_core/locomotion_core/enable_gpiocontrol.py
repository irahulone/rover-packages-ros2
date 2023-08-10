import Jetson.GPIO as GPIO
import sys
 
# Handles time
import time 
 
def main(arg):
    led_pin = 7
    GPIO.setmode(GPIO.BOARD) 
    GPIO.setup(led_pin, GPIO.OUT, initial=GPIO.LOW) 
    arg = bool(arg)
    if arg == True:
        print(arg)
        GPIO.output(led_pin, GPIO.LOW)
    else:
        GPIO.output(led_pin, GPIO.HIGH)


if __name__ == "__main__":
  main(sys.argv[1])