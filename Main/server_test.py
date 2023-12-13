import socket

#HOST='192.168.15.1'
HOST='194.210.177.59'
PORT= 50000


with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    with conn:
        print(f"Connected by {addr}")
        while True:
            data = conn.recv(1024)
            if not data:
                break
            print(data.decode())
            conn.sendall(data) 
            message = input("Enter a message: ")
            conn.send(message.encode('utf-8'))
    
