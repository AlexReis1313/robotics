import socket
import ast
import time
from pygame.time import Clock

def define_constants():
    HEADER = 64 
    PORT = 5050 
    FORMAT = 'utf-8'
    DISCONNECT_MESSAGE = '!DISCONNECT'
    SERVER = '194.210.158.147'
    ADDR = (SERVER, PORT)

    return HEADER, FORMAT, DISCONNECT_MESSAGE, ADDR

def connect_to_server(ADDR, HEADER, FORMAT, DISCONNECT_MESSAGE,FPS):
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect(ADDR)
    start = time.time()
    clock = Clock()
    communicating = True
    cont = 0
    while communicating:
        data = current_state(time.time() -start)
        information_exchange(data, FORMAT, HEADER, client,cont)
        cont +=1
        clock.tick(FPS)
        
        if data[0]==4:
            information_exchange(DISCONNECT_MESSAGE, FORMAT, HEADER, client,cont)
            communicating = False

def information_exchange(data, FORMAT, HEADER, client,cont):
    #Send information

    data = str(data)
    data_s = data.encode(FORMAT)
    data_s_length = len(data_s)
    send_length = str(data_s_length).encode(FORMAT)
    send_length += b' '*(HEADER - len(send_length)) #add padding to equal to header length
    client.send(send_length)
    client.send(data_s)

    #Receive Information
    if cont <1:
        data_r_length = client.recv(HEADER).decode(FORMAT) #Receives size of message from client
        if data_r_length: #See if is a valid message
            data_r_length = int(data_r_length)
            data_r = client.recv(data_r_length).decode(FORMAT)
            print(f'[CALIBRATION MATRIX] {ast.literal_eval(data_r)}') #transforms string to whatever without parsing problems

def current_state(time): #placeholder using time

    if time < 5:
        return (-1, [0,0,0,0,0], None, [0,0,0])
    elif 5<= time < 9:
        return (2, [1,4,0,0,0], True, [0,0,39])
    else: 
        return (4, [1,4,0,0,0], True, [0,0,39])

def main():
    FPS = 1
    HEADER, FORMAT, DISCONNECT_MESSAGE, ADDR = define_constants()
    connect_to_server(ADDR, HEADER, FORMAT, DISCONNECT_MESSAGE,FPS)

if __name__ == '__main__':
    main()