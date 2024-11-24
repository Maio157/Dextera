



from gpiozero import OutputDevice
import time
from ADCDevice import *
from gpiozero import AngularServo
motorPins = (18, 23, 24, 25)  # Pin collegati al motore passo-passo
motors = list(map(lambda pin: OutputDevice(pin), motorPins))
CCWStep = (0x01, 0x02, 0x04, 0x08)  # Ordine per rotazione antioraria
CWStep = (0x08, 0x04, 0x02, 0x01)   # Ordine per rotazione oraria

adc = ADCDevice()  # Oggetto ADC
ServoGPIO= 14
SERVO_DELAY_SEC = 0.001 
myCorrection=0.0
maxPW=(2.5+myCorrection)/1000
minPW=(0.5-myCorrection)/1000
servo =  AngularServo(ServoGPIO,initial_angle=0,min_angle=0, max_angle=180,min_pulse_width=minPW,max_pulse_width=maxPW)



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

def moveOnePeriod(direction, ms):
    """definisce come si muove lo step in senso orario e antiorario"""
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
    """fa ruotare lo step"""
    for i in range(steps):
        moveOnePeriod(direction, ms)

def motorStop():
    """fa fermare lo step"""
    for i in range(0,4,1):
        motors[i].off()

def mapImputToAngle(imput_value, input_min, input_max, angle_min, angle_max):
    """mappa l'angolo rispetto al valore che prende in generale"""
    angle = ((imput_value - input_min)*(angle_max-angle_min))/(input_max-input_min) + angle_min
    return angle

def MapToAngleStep(input_value):
    """ci da l'angolo a cui deve girare lo step"""
    if (input_value >=135):
        angle = mapImputToAngle(input_value, 135, 255, 0, 90)
    elif(input_value <=120):
        input_value2 = 120 - input_value
        angle = mapImputToAngle(input_value2, 0, 120, 0, 90)
    else:
        angle = 0 
    return angle

def MapToAngleServo(input):
    """ci da l'angolo a cui deve girare il servo"""
    if (input>=135):
        angle = mapImputToAngle(input,135,255,0,180)
    elif(input <=120):
        input2=120-input
        angle = mapImputToAngle(input2,0,120,0,180)
    else:
        angle=0
    return angle


def Ruotiamo(angolo, direction, ms):
    """fa ruotare lo step di un certo angolo in una certa direzione"""
    passi_per_grado = 2048 / 360;
    steps = int(angolo*passi_per_grado)

    moveSteps(direction, ms, steps)
  

def loop():
    ms = 3
    last_step_angle = -1
    while True:
        
        servo_value =adc.analogRead(1)  #valore del servo dato lungo le  y
        servo_angle =MapToAngleServo(servo_value)
        servo.angle = servo_angle   # il servo stra facile da girare


        step_value = adc.analogRead(0)  #valore dello step dato lungo le x
        step_angle = MapToAngleStep(step_value)
        
        if step_angle != last_step_angle:  ##tutti gli if per lo step motor
            if step_angle > last_step_angle: 
            #se l'angolo aumenta, si va in senso orario
            #  con rotazione pari alla differenza tra i due angoli
                angolo = step_angle - last_step_angle
                Ruotiamo(angolo,1,ms)
            else:
                angolo = last_step_angle - step_angle
                Ruotiamo(angolo,0,ms)
            
        else:
            Ruotiamo(0,0,ms)

        last_step_angle = step_angle

        time.sleep(1) #facciamo ogni secondo sennò esplode
    



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


