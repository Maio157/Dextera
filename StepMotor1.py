



from gpiozero import OutputDevice
import time
from ADCDevice import *

motorPins = (18, 23, 24, 25)  # Pin collegati al motore passo-passo
motors = list(map(lambda pin: OutputDevice(pin), motorPins))
CCWStep = (0x01, 0x02, 0x04, 0x08)  # Ordine per rotazione antioraria
CWStep = (0x08, 0x04, 0x02, 0x01)   # Ordine per rotazione oraria

adc = ADCDevice()  # Oggetto ADC

def setup():
    global adc
    if adc.detectI2C(0x48):  # Rileva il PCF8591
        adc = PCF8591()
    elif adc.detectI2C(0x4b):  # Rileva l'ADS7830
        adc = ADS7830()
    else:
        print("Nessun indirizzo I2C valido trovato!")
        exit(-1)

def moveOnePeriod(direction, ms):
    
    for j in range(0,4,1):
        for i in range(0,4,1):
            if direction == 1:
                motors[i].on() if (CCWStep[j] == 1 << i) else motors[i].off()
            else:
                motors[i].on() if CWStep[j] == 1 << i else motors[i].off()
        
        if(ms < 3):
            ms = 3
        time.sleep(ms / 1000.0)
def moveSteps(direction, ms, steps):
    for i in range(steps):
        moveOnePeriod(direction, ms)

def motorStop():
    for i in range(0,4,1):
        motors[i].off()

def mapImputToAngle(imput_value, input_min, input_max, angle_min, angle_max):
    """mappa l'angolo rispetto al valore che prende"""
    angle = ((imput_value - input_min)*(angle_max-angle_min))/(input_max-input_min) + angle_min
    return angle

def mapToAngle2(input_value):
    """ci da l'angolo quello giusto"""
    if (input_value >=135):
        angle = mapImputToAngle(input_value, 135, 255, 0, 90)
    elif(input_value <=120):
        input_value2 = 120 - input_value
        angle = mapImputToAngle(input_value2, 0, 120, 0, 90)
    else:
        angle = 0 
    return angle

def RuotiamoCazzo(angolo, direction, ms)
    """lo fa ruotare di un certo angolo a una certa direzione"""
    passi_per_giro = 2048 / 360;
    steps = int(angolo*passi_per_giro)

    moveSteps(direction, ms, steps)


def loop():
    ms = 3
    last_angle = -1
    while True:
        input_value = adc.analogRead(0)
        print(f"value_X: {input_value}")
        best_angle = mapToAngle2(input_value)
        if best_angle != last_angle:
            if best_angle > last_angle: 
            #se l'angolo aumenta, si va in senso orario
            #  con rotazione pari alla differenza tra i due angoli
                angolo = best_angle - last_angle
                RuotiamoCazzo(angolo,1,ms)
            else:
                angolo = last_angle - best_angle
                RuotiamoCazzo(angolo,0,ms)
            
        else:
            RuotiamoCazzo(0,0,ms)

        last_angle = best_angle

        time.sleep(2) #facciamo ogni due secondi sennò esplode
    



def destroy():
    adc.close()

if __name__ == '__main__':
    print('Program is starting...')
    setup()
    try:
        loop()
    except KeyboardInterrupt:
        destroy()
        motorStop()
        print("Ending program")


