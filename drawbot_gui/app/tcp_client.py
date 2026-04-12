import socket


class TCPClient:
    def __init__(self):
        self.sock = None
        self.buffer = ""

    def connect(self, ip, port):
        self.disconnect()

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect((ip, port))
            sock.settimeout(0.1)

            self.sock = sock
            return True

        except Exception as e:
            print("❌ Erreur connexion :", e)
            self.sock = None
            return False

    def send(self, message):
        if not self.sock:
            return

        try:
            self.sock.sendall((message + "\n").encode())
        except Exception as e:
            print("❌ Erreur envoi :", e)

    def receive_lines(self):
        if not self.sock:
            return []

        try:
            data = self.sock.recv(1024)
            if not data:
                return []

            self.buffer += data.decode(errors="ignore")

            lines = []
            while "\n" in self.buffer:
                line, self.buffer = self.buffer.split("\n", 1)
                lines.append(line.strip())

            return lines

        except socket.timeout:
            return []
        except Exception as e:
            print("❌ Erreur réception :", e)
            return []

    def disconnect(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None