from machine import Pin, UART, PWM, I2C, ADC
import umqtt_robust2 as mqtt
from neopixel import NeoPixel
from time import sleep, ticks_ms
#import _thread
from mpu6050 import MPU6050
from gps_bare_minimum import GPS_Minimum
import math

#mqtt.sync_with_adafruitIO()
#mqtt.webprint(data)
#mqtt.besked(adafruit_data)

n = 12
np = NeoPixel(Pin(13, Pin.OUT),n)


i2c = I2C(scl=Pin(22),sda=Pin(21))
imu = MPU6050(i2c)

#########################################################################
# CONFIGURATION
gps_port = 2                               # ESP32 UART port, Educaboard ESP32 default UART port
gps_speed = 9600                           # UART speed, defauls u-blox speed
#########################################################################
# OBJECTS
uart = UART(gps_port, gps_speed)           # UART object creation
gps = GPS_Minimum(uart)                    # GPS object creation
#########################################################################
bat_adc = ADC(Pin(35))        # The battery status ADC object
bat_adc.atten(ADC.ATTN_11DB)           # Full range: 3,3 V
bat_scaling = 4.2 / 4095



def read_battery_voltage():
    adc_val = bat_adc.read()
    voltage = bat_scaling * adc_val
    return voltage

read_battery_voltage()
print(bat_adc.read())
print(read_battery_voltage())

def battery_percentage():
    percentage = (read_battery_voltage() / 3.77) * 100
    return percentage

battery_percentage()
print(battery_percentage())


def get_adafruit_gps():
    speed = lat = lon = None # Opretter variabler med None som værdi
    if gps.receive_nmea_data():
        # hvis der er kommet end bruggbar værdi på alle der skal anvendes
        if gps.get_speed() != -999 and gps.get_latitude() != -999.0 and gps.get_longitude() != -999.0 and gps.get_validity() == "A":
            # gemmer returværdier fra metodekald i variabler
            speed =str(gps.get_speed()) 
            lat = str(gps.get_latitude()) 
            lon = str(gps.get_longitude())
            # returnerer data med  adafruit gps format
            return speed + "," + lat + "," + lon + "," + "0.0"
        else: # hvis ikke både hastighed, latitude og longtitude er korrekte 
            print(f"GPS data to adafruit not valid:\nspeed: {speed}\nlatitude: {lat}\nlongtitude: {lon}")
            return False
    else:
        return False


tackling_indikator = 0

def set_color(tackling_indikator):
    np[tackling_indikator] = (10,0,10)
    np.write()

    
    
status = True #hører til IMU

def clear_neo():
    for i in range(n):
        np[i] = (0,0,0)
        np.write()
        
clear_neo()

buzzer = PWM(Pin(26), freq=420, duty=0)

tid = 0

while True:
    try:
        #adafruit batteri kode her: <3
#         read_battery_voltage()
#         battery_percentage()
#         perc = battery_percentage()
#         mqtt.web_print(perc, 'tec9na/feeds/ESP32feed')
#         sleep(3)
        
        
        
        #IMU koden kommer her:#############################################################
        ###### printer hele dictionary som returneres fra get_values metoden, er dog udkommenteret lige nu
        #print(imu.get_values()) 
        sleep(0.01)
        vals = imu.get_values()
        value_acceleration_x = vals["acceleration x"]
        print(f"ACCELERATION ER: {value_acceleration_x}")
        #print(vals["acceleration x"])
            
        #ligge ned kode
        if vals["acceleration x"] < -1000 and status == False:
            print("spiller er: TACKLET")
            status = True
            if tackling_indikator < 10 :
                
                set_color(tackling_indikator)
                tackling_indikator = tackling_indikator+1
        
        print("")
        #stå op kode:
        if vals["acceleration x"] > 5000 and status == True:
            status = False
            print("spiller er: OPREJST")
        print("")
        
        #Her kommer GPS koden:##############################################################
        
        gps_data = get_adafruit_gps()
        if gps_data: # hvis der er korrekt data så send til adafruit
            print(f'\ngps_data er: {gps_data}')
            mqtt.web_print(gps_data, 'tec9na/feeds/mapfeed/csv')
            sleep(3)
        
        mqtt.web_print(battery_percentage())
        sleep(3)
        
        #Her sender vi og modtager data mellem ESP og Adafruit##############################

        if len(mqtt.besked) != 0: # Her nulstilles indkommende beskeder
            mqtt.besked = ""            
        mqtt.sync_with_adafruitIO() # igangsæt at sende og modtage data med Adafruit IO
        
        
        
        
        # lav liste med hastighedsmålinger
        # tilføj nuværende hastighed til listen
        # lav tæller variabel (15x loop med 2 sek delay = 30 sekunder)
            # når tælleren er gået så tjek liste med hastighedsmålinger
             # hvis den højeste hastighed i listen er under 5 km/t så aktiver buzzer
        
        print("")
        #selvvalgt krav#####################################################################
        if gps.get_speed() < 1: #hvis man er inaktiv
            print("SPILLER ER: INAKTIV")
            tid = tid + 2   #bruges til at tælle 2 op hver gang loopet kører OG man er inaktiv. Tæller med 2 da vi sleeper på 2 i loopet og vi vil gerne have at den siger en buzzer lyd efter 30 sekunder.
            if tid >= 30: #hvis der er gået 30 sekunder så spiller buzzeren en lyd
                buzzer.duty(512) #buzzeren tænder
        
        if gps.get_speed() > 1: #hvis man er aktiv
            print("SPILLER ER: AKTIV")
            tid = 0 #timeren nulstilles
            buzzer.duty(0) #buzzeren stopper med at spille lyd
            
                
                
        
        
    except KeyboardInterrupt:
        print("Ctrl+C pressed - exiting program.")
        sys.exit()
