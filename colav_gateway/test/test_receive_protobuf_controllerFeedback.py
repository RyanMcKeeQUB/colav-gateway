from proto_gen.controllerFeedback_pb2 import ControllerFeedback
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 7300))

while True:
    try:
        sock.settimeout(2)
        data, addr = sock.recvfrom(1024)
        controller_feedback = ControllerFeedback()
        controller_feedback.ParseFromString(data)
        break

    except Exception as e:
        print("Socket timeout, Continueing to listen.....")


print(controller_feedback.mission_tag)
