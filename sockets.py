# Server (Computer A)

import socket

# Create a socket object
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Define the server address and port
server_address = ('your_server_ip', 12345)  # Change 'your_server_ip' to the actual IP address. Common port numbers vary within the range of 1024 to 49151. Any numbers in this range can be used for the port.

# Bind the socket to the server address
server_socket.bind(server_address)

# Listen for incoming connections
server_socket.listen(1)

print("Waiting for a connection...")

# Accept a connection
client_socket, client_address = server_socket.accept()
print(f"Connection from {client_address}")

# Receive data from the client
data = client_socket.recv(1024)
print(f"Received: {data.decode('utf-8')}")

# Close the connection
client_socket.close()
server_socket.close()

###########################################################################################################
###########################################################################################################

# Client (Computer B)

import socket

# Create a socket object
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Define the server address and port
server_address = ('server_ip_of_computer_a', 12345)  # Change 'server_ip_of_computer_a' to the actual IP address

# Connect to the server
client_socket.connect(server_address)

# Send data to the server
message = "Hello, Computer A!"
client_socket.sendall(message.encode('utf-8'))

# Close the connection
client_socket.close()


#### RUN THE SERVER SCRIPT ON COMPUTER A AND THEN RUN THE CLIENT SCRIPT ON COMPUTER B 