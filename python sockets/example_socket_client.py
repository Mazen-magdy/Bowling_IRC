import socket

def socket_client():
    # Create socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Connect to the server
    sock.connect(('192.168.71.1', 5555))

    # Prepare the message from the user
    while True:
        message = str(input("Enter the message you want to send to the server: "))
        if message == "close":
            # Send the message to initialize the connection
            sock.send(str(message).encode("utf-8"))
            
            # Close the socket
            print("Connction is terminated!")
            sock.close()
            break
        else:
            # Send the message to initialize the connection
            sock.send(str(message).encode("utf-8"))

            # Recieve a message from the server
            response = sock.recv(1024).decode("utf-8")
            print(f"Server response: {response}")

    


socket_client()