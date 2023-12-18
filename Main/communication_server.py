import socket

def define_constants():
    HEADER = 64 #Header before message that tells length of the message that comes next. Wont be needed for project
    PORT = 5050 
    SERVER = socket.gethostbyname(socket.gethostname()) #gets ip automatically, instead of previous line
    ADDR = (SERVER, PORT)
    FORMAT = 'utf-8'
    DISCONNECT_MESSAGE = '!DISCONNECT'

    return HEADER, FORMAT, DISCONNECT_MESSAGE, ADDR

def start_server(ADDR, HEADER, FORMAT, DISCONNECT_MESSAGE, current_state):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #tells server what type of ip address to expect and stream type of communication
    server.bind(ADDR)
    server.listen() #waiting for connection
    print(f'[LISTENING] Server is listening on {ADDR[0]}')
    while True:
        conn, addr = server.accept() #Waits for new connection to server
        handle_client(conn,addr, HEADER, FORMAT, DISCONNECT_MESSAGE, current_state)

        if KeyboardInterrupt:
            break
    
def handle_client(conn, addr, HEADER, FORMAT, DISCONNECT_MESSAGE, current_state): #Runs for each client
    print(f'[NEW CONNECTION] {addr} connected.')
    connected = True
    
    while connected:

        #Receive information
        data_r_length = conn.recv(HEADER).decode(FORMAT) #Receives size of message from client
        if data_r_length: #See if is a valid message
            data_r_length = int(data_r_length)
            data_r = conn.recv(data_r_length).decode(FORMAT)
            if data_r == DISCONNECT_MESSAGE:
                connected = False   
            print(f'[{addr}] {data_r}')

        #Send information
            current_state = str(current_state)
            data_s = current_state.encode(FORMAT)
            data_s_length = len(data_s)
            send_length = str(data_s_length).encode(FORMAT)
            send_length += b' '*(HEADER - len(send_length)) #add padding to equal to header length
            conn.send(send_length)
            conn.send(data_s)
    
    conn.close()

def get_current_state():
    return (-1, [0,0,0,0,0], None, [0,0,0])

def main():
    HEADER, FORMAT, DISCONNECT_MESSAGE, ADDR = define_constants()
    current_state = get_current_state()
    start_server(ADDR, HEADER, FORMAT, DISCONNECT_MESSAGE, current_state)

if __name__ == '__main__':
    main()