import numpy as np 
import serial
import struct
import time

def senoide(A,f,Fs,phi,num_samples):
    """
    Essa função implementa uma senoide 

    args: 
    A(int): Magnitude da senoide
    f(float) : Frequencia da senoide 
    Fs(float) : frquencia de amostragem
    phi(float): angulo de atraso (rad)
    num_samples(int): numero de amostras do vetor 
    """
    t = np.arange(0, num_samples) / Fs
    y = A * np.sin(2 * np.pi * f * t + phi) + 1.75  # Ajusta para valores não negativos
    return y, t


#Função para enviar sinal contínuo para o microcontrolador
def send_sync_signal(ser,buffer,Size_buffer):
    # Supondo que y_uint16 seja o seu vetor de valores
    for value in buffer:
 
        ser.write(np.float32(value*100).tobytes())
    print("fim")
    #time.sleep(0.1)  # Espera 1 segundo antes de enviar o próximo sinal

