
import time
from ADCDevice import *
from gpiozero import AngularServo

adc = ADCDevice()  # Oggetto ADC
ServoGPIO= 4
ServoGPIO2 = 5
SERVO_DELAY_SEC = 0.001 
myCorrection=0.0
maxPW=(2.5+myCorrection)/1000
minPW=(0.5-myCorrection)/1000
servo =  AngularServo(ServoGPIO,initial_angle=0,min_angle=0, max_angle=180,min_pulse_width=minPW,max_pulse_width=maxPW)
servo2 = AngularServo(ServoGPIO,initial_angle=0,min_angle=0, max_angle=90,min_pulse_width=minPW,max_pulse_width=maxPW)


def setup():
    """ci dice quale adc abbiamo collegato (palesemente inutile)"""
    global adc
    if adc.detectI2C(0x48):  # Rileva il PCF8591
        adc = PCF8591()
    elif adc.detectI2C(0x4b):  # Rileva l'ADS7830
        adc = ADS7830()
    else:
        print("Nessun indirizzo I2C valido trovato!")
        exit(-1)

def mapImputToAngle(imput_value, input_min, input_max, angle_min, angle_max):
    """mappa l'angolo rispetto al valore che prende in generale"""
    angle = ((imput_value - input_min)*(angle_max-angle_min))/(input_max-input_min) + angle_min
    return angle

def MapToAngleServoFlex(input):
    """ci da l'angolo a cui deve girare il servo per la flessione"""
    if (input>=135):
        angle = mapImputToAngle(input,135,255,0,180)
    elif(input <=120):
        input2=120-input
        angle = mapImputToAngle(input2,0,120,0,180)
    else:
        angle=0
    return angle
    
def MapToAngleServoRot(input):
    """ci da l'angolo a cui deve girare il servo per la rotazione"""
    if (input>=135):
        angle = mapImputToAngle(input,135,255,0,90)
    elif(input <=120):
        input2=120-input
        angle = mapImputToAngle(input2,0,120,0,90)
    else:
        angle=0
    return angle
  

def loop():
    
    while True:
        
        servo_value =adc.analogRead(1)  #valore del servo dato lungo le  y
        servo_angle =MapToAngleServoFlex(servo_value)
        print(f"angolo = {servo_angle}")
        servo.angle = servo_angle   # il servo stra facile da girare

        servo_value2 =adc.analogRead(0)  #valore del servo dato lungo le  y
        servo_angle2 =MapToAngleServoRot(servo_value2)
        print(f"angolo2 = {servo_angle2}")
        servo2.angle = servo_angle2   # il servo stra facile da girare
        
        time.sleep(0.1) 


def destroy():
    adc.close()

if __name__ == '__main__':
    print('Program is starting...')
    setup()
    try:
        loop()
    except KeyboardInterrupt:
        destroy()
        print("Ending program")


