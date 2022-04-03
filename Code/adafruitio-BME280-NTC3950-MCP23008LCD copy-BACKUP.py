"""
Required libraries in lib folder:
- adafruit_bus_device
- adafruit_esp32spi
- adafruit_io
- adafruit_mcp230xx
- adafruit_register
- adafruit_thermistor
- adafruit_requests
- adafruit_bme280
- adafuit_charactor_lcd
---------------------------------------------------------
Should be pulling ambient Temp + Humidity values from BME280, as well as
Temp values from the Thermistor which will live inside the warm hide.

supervisor.reset() did not seem to be keeping system online during deployment. Changed to microcontroller.reset(). Hopefullly that will resolve the issue.

"""


import time
from microcontroller import cpu
import board
import busio
from digitalio import DigitalInOut
from adafruit_esp32spi import adafruit_esp32spi
from adafruit_esp32spi import adafruit_esp32spi_wifimanager
import adafruit_esp32spi.adafruit_esp32spi_socket as socket
import adafruit_minimqtt.adafruit_minimqtt as MQTT
from adafruit_io.adafruit_io import IO_MQTT
import adafruit_io
from adafruit_bme280 import basic as adafruit_bme280
import microcontroller
import adafruit_thermistor
import supervisor
import adafruit_character_lcd.character_lcd_i2c as character_lcd


i2c = busio.I2C(scl=board.GP1, sda=board.GP0)  # uses I2C0 (LCD)

# Modify this if you have a different sized Character LCD
lcd_columns = 20
lcd_rows = 4

# initialize the lcd class
lcd = character_lcd.Character_LCD_I2C(i2c, lcd_columns, lcd_rows)

### WiFi ###

# Get wifi details and more from a secrets.py file
try:
    from secrets import secrets
except ImportError:
    print("WiFi secrets are kept in secrets.py, please add them there!")
    raise

# Raspberry Pi RP2040
esp32_cs = DigitalInOut(board.GP13)
esp32_ready = DigitalInOut(board.GP14)
esp32_reset = DigitalInOut(board.GP15)

spi = busio.SPI(board.GP10, board.GP11, board.GP12)
esp = adafruit_esp32spi.ESP_SPIcontrol(spi, esp32_cs, esp32_ready, esp32_reset)

wifi = adafruit_esp32spi_wifimanager.ESPSPI_WiFiManager(esp, secrets)

# Configure the RP2040 Pico LED Pin as an output
led_pin = DigitalInOut(board.LED)
led_pin.switch_to_output()

# Define callback functions which will be called when certain events happen.
# pylint: disable=unused-argument
def connected(client):
    # Connected function will be called when the client is connected to Adafruit IO.
    print("Connected to Adafruit IO! ")


def subscribe(client, userdata, topic, granted_qos):
    # This method is called when the client subscribes to a new feed.
    print("Subscribed to {0} with QOS level {1}".format(topic, granted_qos))


# pylint: disable=unused-argument
def disconnected(client):
    # Disconnected function will be called when the client disconnects.
    print("Disconnected from Adafruit IO!")


def on_led_msg(client, topic, message):
    # Method called whenever user/feeds/led has a new value
    print("New message on topic {0}: {1} ".format(topic, message))
    if message == "ON":
        led_pin.value = True
    elif message == "OFF":
        led_pin.value = False
    else:
        print("Unexpected message on LED feed.")


# Connect to WiFi
print("Resetting ESP32, wait 15 seconds...")
lcd.backlight = True
lcd.clear()
lcd.blink = True
lcd.message = "Initializing..."
wifi.reset()
time.sleep(15)
print("Connecting to WiFi...")
lcd.message = "\nWiFi..."
wifi.connect()
print("Connected!")

# Initialize MQTT interface with the esp interface
MQTT.set_socket(socket, esp)

# Initialize a new MQTT Client object
mqtt_client = MQTT.MQTT(
    broker="io.adafruit.com",
    username=secrets["aio_username"],
    password=secrets["aio_key"],
)

# Initialize an Adafruit IO MQTT Client
io = IO_MQTT(mqtt_client)

# Connect the callback methods defined above to Adafruit IO
io.on_connect = connected
io.on_disconnect = disconnected
io.on_subscribe = subscribe

# Set up a callback for the led feed
io.add_feed_callback("led", on_led_msg)

# Connect to Adafruit IO
print("Connecting to Adafruit IO...")
lcd.message = "\n\nIO..."
try:
    io.connect()
except:
    microcontroller.reset()

# Subscribe to all messages on the led feed
io.subscribe("led")

# configure i2c
bmei2c = busio.I2C(scl=board.GP3, sda=board.GP2)  # uses I2C1 (BME280)


# initialise mcp9808 using the default address:
bme280 = bme280 = adafruit_bme280.Adafruit_BME280_I2C(bmei2c, 0x76)

# Set up NTC3950
thermistor = adafruit_thermistor.Thermistor(board.GP26, 10000.0, 10000.0, 25.0, 3950.0, high_side=False)



prv_refresh_time = 0.0
while True:
    # Poll for incoming messages
    try:
        io.loop()
    except:   #(ValueError, RuntimeError) as e:
        print("Failed to get data, resetting Pico and ESP32\n")
        microcontroller.reset()

    # Send a new temperature reading to IO every 60 seconds

    lcd.blink = False

    if (time.monotonic() - prv_refresh_time) > 60:
        led_pin.value = True
        # read tempurate from bme280
        temp = bme280.temperature * 9 / 5 + 32
        # truncate to two decimal points
        temp = str(temp)[:5]

        # read temperature from thermistor and define variable for LCD to read from

        def ntc_temp():
            NTCtempC = thermistor.temperature
            NTCtempF = NTCtempC * 9/5.0 + 32
            NTCtempF = str(round(NTCtempF, 2))
            NTCtempF = str(NTCtempF)
            return NTCtempF

        #ThermistorTempF = ntc_temp()

        def bme_temp():
            bmeTempC = bme280.temperature
            bmeTempF = bmeTempC * 9/5.0 + 32
            bmeTempF = str(round(bmeTempF, 2))
            bmeTempF = str(bmeTempF)
            return bmeTempF

        #AmbientTempF = bme_temp()

        def bme_humidity():
            humidity = bme280.relative_humidity
            humidity = str(round(humidity, 2))
            humidity = str(humidity)
            return humidity


        ## Print data to LCD
        lcd.backlight = True
        lcd.clear
        lcd.message = "     Mr. Snake\n" + "Ambient Temp:" + bme_temp() + "F\n" + "WarmHide Temp:" + ntc_temp() + "F\n" + "Humidity:" + bme_humidity() + "%"


        print("warm hide temp is: %s degrees F" % ntc_temp())
        print("Ambient Temp is: %s degrees F" % bme_temp())

        # publish it to io
        print("Publishing %s to ambient temperature feed..." % bme_temp())
        io.publish("mr-snake-ambient-temp", bme_temp())

        print("publishing %s to warm hide temp feed..." % ntc_temp())
        io.publish("mr-snake-warmhide-temp", ntc_temp())

        print("publishing %s to humidity feed..." % bme_humidity())
        io.publish("mr-snake-humidity", bme_humidity())

        print("Published!")
        led_pin.value = False
        prv_refresh_time = time.monotonic()