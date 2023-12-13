import socket
HOST='194.210.178.3'

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect((HOST, 50000))

while True:
    message = input("Enter a message: ")
    client.send(message.encode('utf-8'))
    