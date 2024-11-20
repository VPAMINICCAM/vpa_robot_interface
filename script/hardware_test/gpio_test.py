import RPi.GPIO as GPIO
import time

# Pin Definitions
GPIO_PIN = 23  # GPIO 23 (BCM numbering)

def main():
    try:
        # GPIO Setup
        GPIO.setmode(GPIO.BCM)  # Use BCM numbering
        GPIO.setup(GPIO_PIN, GPIO.OUT)  # Set GPIO23 as output
        
        # Set GPIO High
        print(f"Setting GPIO {GPIO_PIN} HIGH for 10 seconds...")
        GPIO.output(GPIO_PIN, GPIO.HIGH)
        
        # Wait for 10 seconds
        time.sleep(10)
        
        # Set GPIO Low
        print(f"Setting GPIO {GPIO_PIN} LOW.")
        GPIO.output(GPIO_PIN, GPIO.LOW)

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        # Clean up GPIO
        GPIO.cleanup()
        print("GPIO cleanup complete.")

if __name__ == "__main__":
    main()
