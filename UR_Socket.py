import socket

PORT = 30002
HOST = "172.28.60.3"


def client():
    s = socket.socket()
    s.connect((HOST, PORT))

    command = 'set_digital_out(2, True)\n'
    s.send(command)
    received_data = s.recv(1024)

    s.close()


if __name__ == '__main__':
    client()