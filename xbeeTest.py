import serial

with serial.Serial('COM8', 9600, timeout=3) as Serial:
    while True:
        l = Serial.readline().decode('utf-8')
        print(l)
    
