"""
Required libraries in lib folder:
- adafruit_bus_device
- adafruit_mcp230xx
- adafruit_bme280
- adafuit_charactor_lcd
---------------------------------------------------------
Pulls Ambient Temp + Humidity from BME280 and displays values on LCD display.
Refresh rate is set to 30 seconds.
Intigrated LED on Pico blinks during refresh cycle.

"""


import time
from microcontroller import cpu
import board
import busio
from digitalio import DigitalInOut
from adafruit_bme280 import basic as adafruit_bme280
import microcontroller
import supervisor
import adafruit_character_lcd.character_lcd_i2c as character_lcd


i2c = busio.I2C(scl=board.GP1, sda=board.GP0)  # uses I2C0 (LCD)

# Modify this if you have a different sized Character LCD:
lcd_columns = 20
lcd_rows = 4

# initialize the lcd class:
lcd = character_lcd.Character_LCD_I2C(i2c, lcd_columns, lcd_rows)


# configure i2c (BME280):
bmei2c = busio.I2C(scl=board.GP3, sda=board.GP2)  # uses I2C1 (BME280)


# initialise BME280 using the default address:
bme280 = bme280 = adafruit_bme280.Adafruit_BME280_I2C(bmei2c, 0x76)


# Configure the RP2040 Pico LED Pin as an output
led_pin = DigitalInOut(board.LED)
led_pin.switch_to_output()



prv_refresh_time = 0.0
while True:


    lcd.blink = False

    if (time.monotonic() - prv_refresh_time) > 30:
        led_pin.value = True
        # read tempurature from bme280
        temp = bme280.temperature * 9 / 5 + 32
        # truncate to two decimal points
        temp = str(temp)[:5]

  

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
        lcd.message = "     Grow House\n" + "Ambient Temp:" + bme_temp() + "F\n" + "Humidity:" + bme_humidity() + "%"


        print("Ambient Temp is: %s degrees F" % bme_temp())
        print("Humidity is: %s %%" % bme_humidity())

        
        led_pin.value = False
        prv_refresh_time = time.monotonic()