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
    # Gera a senoide
    y = A * np.sin(2 * np.pi * f * t + phi)
     # Ajusta a senoide para não ter valores negativos
    #y = y - y.min()
    return y,t


#Função para enviar sinal contínuo para o microcontrolador
def send_sync_signal(ser,buffer,Size_buffer):
    sync_byte = bytes([1])  # Envia o byte 0xAA
    # Supondo que y_uint16 seja o seu vetor de valores
    for value in buffer:
    # Converte `value` em um valor `uint16_t` em formato de 2 bytes (little-endian)
        byte_data = struct.pack('<H', value)  # '<H' significa 'unsigned short' (16 bits)
    # Agora você pode enviar `byte_data` pela UART ou outro canal de comunicação
    #for i in range(Size_buffer):
        ser.flush()
        ser.write(byte_data)
    print("fim")
    time.sleep(0.1)  # Espera 1 segundo antes de enviar o próximo sinal
