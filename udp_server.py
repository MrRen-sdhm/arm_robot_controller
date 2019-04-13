import logging
import socket
import time

log = logging.getLogger('udp_server')

def udp_server(host='0.0.0.0', port=1123):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    log.info("Listening on udp %s:%s" % (host, port))
#     s.bind((host, port))
    s.connect(("192.168.123.3", 1124))
    while True:
        s.send(b'ping')
        print('sent')
        time.sleep(1)
        yield b''
        # (data, addr) = s.recvfrom(128*1024)
        # yield data


FORMAT_CONS = '%(asctime)s %(name)-12s %(levelname)8s\t%(message)s'
logging.basicConfig(level=logging.DEBUG, format=FORMAT_CONS)

for data in udp_server():
    print(data.decode('utf8'), end='')