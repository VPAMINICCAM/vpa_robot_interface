import RPi.GPIO as GPIO
import time

# Set the GPIO mode to BCM (Broadcom chip pin numbering)
GPIO.setmode(GPIO.BCM)

# Set GPIO23 as an output pin
GPIO.setup(23, GPIO.OUT)

def main():
    try:
        # Set GPIO23 to high
        GPIO.output(23, GPIO.HIGH)
        print("GPIO23 is set to HIGH")
        
        # Keep the pin high for 5 seconds
        time.sleep(5)

    except KeyboardInterrupt:
        print("Script interrupted")

    finally:
        # Clean up GPIO settings (resets all GPIO pins)
        GPIO.cleanup()
        print("GPIO cleaned up")

if __name__ == '__main__':
    main()
