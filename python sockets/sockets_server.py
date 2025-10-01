import socket


def socket_server_setup():
    # create a socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # bind the socket
    server_socket.bind(('192.168.175.119', 5555))  # we might change the ip and the port
                                    # the client does not need bind because it sends not recieving

    # start listening for the connection
    server_socket.listen(5)

    # accepting the connection
    # knowing the address of the client after connection with it
    client_socket, client_address = server_socket.accept()
    print(f"Connection started with {client_address}")

    while True:
        # receiving the response from the client
        response = client_socket.recv(1024).decode("utf-8")
        print(f"Message received: {response}")

          

        # check if the user wants to close the connection 
        if response == "close":
            print("Connection is terminated by the client!")
            # close the connection
            client_socket.close()
            server_socket.close()
            break
        else:
            # we get the (message) from the openCV code after analyzing the camera frames
            message = str(input("Enter the response right or left or forward or backward (r,l,f,b), B or C(b,c), or \"close\" : "))
            # message = vision_output()
            # message = 122
            # send the massage
            #client_socket.send(str(message).encode("utf-8"))        # Example of a message --> "Welcome to the server !".encode("utf-8").lower()
            client_socket.send(message)
          

def vision_output():
    pass


socket_server_setup()