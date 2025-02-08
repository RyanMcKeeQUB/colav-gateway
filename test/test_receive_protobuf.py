import socket
from proto_gen.missionRequest_pb2 import MissionRequest

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 9999))

while True:
    try:
        sock.settimeout(2)
        data, addr = sock.recvfrom(1024)

        print (f"Received message from endpoint: {addr}")

        print (data)
        mission_request = MissionRequest()
        mission_request.ParseFromString(data)

        print (mission_request.tag)

    except TimeoutError:
        print ('No response, Continueing to listen......')