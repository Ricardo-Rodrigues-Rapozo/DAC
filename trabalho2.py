import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import functions

# Parâmetros da senoide
A = 1        # Amplitude
f = 800        # Frequência (Hz)
Fs = 40000    # Frequência de amostragem (Hz)
phi = 0      # Fase (radianos)
BUFFER_SIZE = 100
#FS = 10000
valor_inteiro = 1
data = []
n = Fs/f ## n amostras por ciclo 
print(n)

vect = np.zeros(BUFFER_SIZE, dtype=np.uint16)
# Gera a senoide
y,t = functions.senoide(A, f, Fs, phi,BUFFER_SIZE)
 # Normalizando o valor de y para o intervalo [0, 65535]
    
plt.plot(t,y)
plt.show()
#Configura a porta serial  
ser = serial.Serial('COM3', 115200, timeout=None)
ser.flush() ## limpa dados pendentes na porta serial 
time.sleep(1)  # Espera a conexão ser estabelecida

while True:     
    functions.send_sync_signal(ser,y,BUFFER_SIZE)
