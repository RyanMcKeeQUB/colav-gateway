import controllerFeedback_pb2
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 7300))

while True:
    try:
        sock.settimeout(2)
        data = sock.recv(1024)

        if data != None:
            controller_feedback = controllerFeedback_pb2.MissionRequest()
            controller_feedback.ParseFromString(data)
            break

    except Exception as e:
        print("Socket timeout, Continueing to listen.....")


print(controller_feedback.mission_tag)
