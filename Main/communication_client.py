import socket
import ast

def define_constants():
    HEADER = 64 
    PORT = 5050 
    FORMAT = 'utf-8'
    DISCONNECT_MESSAGE = '!DISCONNECT'
    SERVER = '194.210.226.247'
    ADDR = (SERVER, PORT)

    return HEADER, FORMAT, DISCONNECT_MESSAGE, ADDR

def connect_to_server(ADDR, HEADER, FORMAT, DISCONNECT_MESSAGE, data):
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect(ADDR)
    send(data, FORMAT, HEADER, client)
    input() #To send next message, click space
    send(DISCONNECT_MESSAGE, FORMAT, HEADER, client)

def send(data, FORMAT, HEADER, client):
    #Send information
    data_s = data.encode(FORMAT)
    data_s_length = len(data_s)
    send_length = str(data_s_length).encode(FORMAT)
    send_length += b' '*(HEADER - len(send_length)) #add padding to equal to header length
    client.send(send_length)
    client.send(data_s)

    #Receive Information
    data_r_length = client.recv(HEADER).decode(FORMAT) #Receives size of message from client
    if data_r_length: #See if is a valid message
        data_r_length = int(data_r_length)
        data_r = client.recv(data_r_length).decode(FORMAT)
        print(f'WE RECEIVED {ast.literal_eval(data_r)}') #transforms string to tuple without parsing problems

def main():
    HEADER, FORMAT, DISCONNECT_MESSAGE, ADDR = define_constants()
    connect_to_server(ADDR, HEADER, FORMAT, DISCONNECT_MESSAGE, 'TESTE')

if __name__ == '__main__':
    main()