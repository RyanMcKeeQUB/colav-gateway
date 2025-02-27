import socket

@staticmethod
def setup_udp_socket(address, timeout):
    """Sets up a udp socket"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(address)
    sock.settimeout(timeout)
    return sock