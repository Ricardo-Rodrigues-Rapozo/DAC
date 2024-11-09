import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import functions

# Parâmetros da senoide
A = 1        # Amplitude
f = 100        # Frequência (Hz)
Fs = 3000     # Frequência de amostragem (Hz)
phi = 0      # Fase (radianos)
BUFFER_SIZE = 200
FS = 4000
valor_inteiro = 1
data = []


vect = np.zeros(BUFFER_SIZE, dtype=np.uint16)
# Gera a senoide
y,t = functions.senoide(A, f, Fs, phi,BUFFER_SIZE)
 # Normalizando o valor de y para o intervalo [0, 65535]
y_normalized = (y - y.min()) / (y.max() - y.min())  # Normalizando para [0, 1]
y_uint16 = (y_normalized * 65535).astype(np.uint16)  # Escalando para [0, 65535] e convertendo para uint16
    
plt.plot(t,y_uint16)
plt.show()
#Configura a porta serial  
ser = serial.Serial('COM3', 115200, timeout=None)
ser.flush() ## limpa dados pendentes na porta serial 
time.sleep(1)  # Espera a conexão ser estabelecida

while True:     
    ser.flush() ## limpa dados pendentes na porta serial 
    functions.send_sync_signal(ser,y_uint16,BUFFER_SIZE)
