import socket
import ast

def define_constants():
    HEADER = 64 #Header before message that tells length of the message that comes next. Wont be needed for project
    PORT = 5050 
    #SERVER = socket.gethostbyname(socket.gethostname()) #gets ip automatically, instead of previous line
    SERVER = '194.210.158.147'
    ADDR = (SERVER, PORT)
    FORMAT = 'utf-8'
    DISCONNECT_MESSAGE = '!DISCONNECT'

    return HEADER, FORMAT, DISCONNECT_MESSAGE, ADDR

def start_server(ADDR, HEADER, FORMAT, DISCONNECT_MESSAGE):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #tells server what type of ip address to expect and stream type of communication
    server.bind(ADDR)
    server.listen() #waiting for connection
    print(f'[LISTENING] Server is listening on {ADDR[0]}')

    conn, addr = server.accept() #Waits for connection to server
    handle_client(conn,addr, HEADER, FORMAT, DISCONNECT_MESSAGE)

def handle_client(conn, addr, HEADER, FORMAT, DISCONNECT_MESSAGE, info_computer_share): #Runs for each client
    print(f'[NEW CONNECTION] {addr} connected.')
    connected = True
    cont = 0
    while connected:

        #Receive information
        data_r_length = conn.recv(HEADER).decode(FORMAT) #Receives size of message from client
        if data_r_length: #See if is a valid message
            data_r_length = int(data_r_length)
            data_r = conn.recv(data_r_length).decode(FORMAT)
            if data_r == DISCONNECT_MESSAGE:
                print('[DISCONNECTING] GOODBYE!')
                connected = False
                info_computer_share['state']=4
            else:   
                data = ast.literal_eval(data_r)
                print(f'[NEW MESSAGE] {data}')
                info_computer_share['state']=data[0]
                info_computer_share['last_bisturi_pos']=data[1]
                info_computer_share['cutting_plan']=data[-1]


               

                

        if cont ==0: #Only send it on first iteration
        #Send information - calibration matrix
            cal_matrix = str(get_calibration_matrix())
            data_s = cal_matrix.encode(FORMAT)
            data_s_length = len(data_s)
            send_length = str(data_s_length).encode(FORMAT)
            send_length += b' '*(HEADER - len(send_length)) #add padding to equal to header length
            conn.send(send_length)
            conn.send(data_s)
            cont += 1
    
    conn.close()

def get_calibration_matrix(): #Needs implemementing
    return [[1,2,3],[4,5,6],[7,8,9]]

def main():
    info_computer_share = {'state': -1, 'last_bisturi_pos': [0,0,0,0,0],  'cutting_plan':[0,0] }

    HEADER, FORMAT, DISCONNECT_MESSAGE, ADDR = define_constants()
    start_server(ADDR, HEADER, FORMAT, DISCONNECT_MESSAGE, info_computer_share)

if __name__ == '__main__':
    main()