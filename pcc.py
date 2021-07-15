# GBA -> MCU -> PC -> Remote Server
# Remote Server -> PC -> MCU -> GBA
#
# Adapter GB PC bridge/connector
#
# Requirements:
#    Python 3
#    pySerial (pip install pyserial)
#
#
# Usage: python pcc.py (SERIAL PORT) (BAUD RATE, usually 9600)
#
import socket
import serial
import select
import sys
from enum import Enum

CHUNK_LENGTH = 254

class CommandId(Enum):
    DISCONNECT = 0
    TCP_CONNECT = 1
    SEND = 2
    RECEIVE = 3


class Packet:
    def __init__(self, length: int):
        self.len = length
        self.data = bytearray()
        self.id = 0


class Socket:
    def __init__(self):
        self.socket = None

    def connect(self, ip: str, port: int, udp: bool = False) -> bool:
        if udp:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        else:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        try:
            self.socket.connect((ip, port))
        except TimeoutError:
            print("Connect to {}:{} fail".format(ip, port))
            return False
        except socket.timeout:
            print("Connect to {}:{} fail".format(ip, port))
            return False

        self.socket.setblocking(False)
        self.socket.settimeout(5)
        print("Connect to {}:{} ok".format(ip, port))
        return True

    def disconnect(self) -> None:
        self.socket.shutdown(socket.SHUT_RDWR)

    def close(self) -> None:
        self.socket.close()

    def send(self, data: bytes) -> None:
        print("Send to socket {}".format(data))

        tot = 0
        data_len = len(data)
        while tot < data_len:
            bt = self.socket.send(data[tot:])
            if bt == 0:
                raise RuntimeError("socket disconnected")
            tot += bt

    def read(self) -> bytes:
        data_len = CHUNK_LENGTH

        r_list = [self.socket]
        w_list = [self.socket]
        x_list = [self.socket]
        (r_list, w_list, x_list) = select.select(r_list, w_list, x_list)

        if len(x_list) > 0 and x_list[0] == self.socket:
            raise RuntimeError("socket disconnected")
        if (len(r_list) > 0 and r_list[0] != self.socket) or len(r_list) < 1:
            print("Socket recv NULL")
            return b""
        if len(w_list) < 1 or w_list[0] != self.socket:
            raise RuntimeError("socket write fail")

        data = self.socket.recv(data_len)

        if data == b"":
            raise RuntimeError("socket read fail")
        print("Socket recv {}".format(data))
        return data


class Serial:
    def __init__(self, port: str, baud: int):
        self.serial = serial.Serial(port, baud, bytesize=8, parity='N', stopbits=1)
        self.synced = False
        self.curr = None

    def is_open(self) -> bool:
        return self.serial.is_open

    def read(self) -> bool:
        x = self.serial.read(2)
        if x == b'':
            return False

        if x[0] != 0xCD:
            return False

        self.curr = Packet(x[1])
        self.curr.data = b""

        while self.curr.data == b"":
            x = self.serial.read(self.curr.len + 2)

            if x[0] != 0xCD:
                self.curr = None
                return False

            self.curr.id = x[1]
            self.curr.data = x[2:]

        print("Serial read: id {} data {}".format(self.curr.id, str(self.curr.data)))
        return True

    def write(self, data: bytes, dyn: bool) -> None:
        data_len = 0

        if data != None:
            data_len = min(len(data), CHUNK_LENGTH)

        sd_length = 1 + data_len

        if dyn:
            sd_length += 1

        sd = bytearray(sd_length)
        sd[0] = 0xCE

        if dyn:
            sd[1] = data_len

            if data_len > 0:
                sd[2:data_len+2] = data
        elif data_len > 0:
            sd[1:data_len+1] = data

        print("Serial write {}".format(str(sd)))
        self.serial.write(sd)

    def write_err(self) -> None:
        print("Serial write error")
        self.serial.write(b'\xCF')


IP = "127.0.0.1"


class Connector:
    MAX_CONNECTIONS = 16

    def __init__(self, port: str, baud: int):
        self.serial = Serial(port, baud)
        self.socket = []

        for x in range(self.MAX_CONNECTIONS):
            self.socket.append(Socket())

    def start(self):
        if self.serial.read():
            command_id = self.serial.curr.id
            data = self.serial.curr.data

            # data[0] = Connection ID
            # data[1..4] = IP
            # data[5..6] = Port
            if command_id == CommandId.TCP_CONNECT.value:
                cid = data[0]
                port = int.from_bytes(data[5:7], byteorder="big")

                if self.socket[cid].connect(IP, port):
                    self.serial.write(b"\x01", False)
                else:
                    self.serial.write(b"\x00", False)

            elif command_id == CommandId.SEND.value:
                cid = data[0]
                data_len = data[1]
                write_data = data[2:data_len+2]

                if b"USER" in write_data:
                    write_data = write_data[:-2]
                    write_data += b"@gbj.dion.ne.jp\r\n"

                try:
                    self.socket[cid].send(write_data)
                except RuntimeError:
                    self.serial.write_err()
                    return

                self.serial.write(b"\x01", False)

            elif command_id == CommandId.RECEIVE.value:
                cid = data[0]

                try:
                    read_data = self.socket[cid].read()
                except RuntimeError:
                    self.serial.write_err()
                    return

                self.serial.write(read_data, True)

            elif command_id == CommandId.DISCONNECT.value:
                cid = data[0]
                self.socket[cid].disconnect()
                self.socket[cid].close()

            else:
                print("Unknown command " + str(command_id))
                self.serial.write_err()


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: %s [port] [baud rate]" % sys.argv[0])
    else:
        connector = Connector(sys.argv[1], int(sys.argv[2]))

        while True:
            connector.start()
