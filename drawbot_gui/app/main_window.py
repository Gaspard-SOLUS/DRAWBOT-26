from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QTextEdit, QGridLayout,
    QDoubleSpinBox, QGroupBox
)
from PySide6.QtCore import QTimer

from app.tcp_client import TCPClient
from app.config import ROBOT_IP, ROBOT_PORT
from app.protocol import parse_message, build_pid_command


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Drawbot GUI")
        self.resize(900, 600)

        self.client = TCPClient()

        self.telemetry_labels = {}

        central = QWidget()
        self.setCentralWidget(central)

        main_layout = QHBoxLayout()
        central.setLayout(main_layout)

        left_layout = QVBoxLayout()
        right_layout = QVBoxLayout()

        main_layout.addLayout(left_layout, 1)
        main_layout.addLayout(right_layout, 1)

        # =========================
        # Connexion
        # =========================
        self.status_label = QLabel("❌ Non connecté")
        self.ip_label = QLabel(f"Robot : {ROBOT_IP}:{ROBOT_PORT}")

        self.connect_btn = QPushButton("Connecter")
        self.disconnect_btn = QPushButton("Déconnecter")

        left_layout.addWidget(self.status_label)
        left_layout.addWidget(self.ip_label)
        left_layout.addWidget(self.connect_btn)
        left_layout.addWidget(self.disconnect_btn)

        # =========================
        # Commandes robot
        # =========================
        control_box = QGroupBox("Commandes robot")
        control_layout = QVBoxLayout()
        control_box.setLayout(control_layout)

        self.forward_btn = QPushButton("Avancer")
        self.backward_btn = QPushButton("Reculer")
        self.left_btn = QPushButton("Gauche")
        self.right_btn = QPushButton("Droite")
        self.stop_btn = QPushButton("STOP")

        control_layout.addWidget(self.forward_btn)
        control_layout.addWidget(self.backward_btn)
        control_layout.addWidget(self.left_btn)
        control_layout.addWidget(self.right_btn)
        control_layout.addWidget(self.stop_btn)

        left_layout.addWidget(control_box)

        # =========================
        # PID
        # =========================
        pid_box = QGroupBox("Réglage PID")
        pid_layout = QGridLayout()
        pid_box.setLayout(pid_layout)

        self.kp_spin = QDoubleSpinBox()
        self.ki_spin = QDoubleSpinBox()
        self.kd_spin = QDoubleSpinBox()

        for spin in (self.kp_spin, self.ki_spin, self.kd_spin):
            spin.setDecimals(3)
            spin.setRange(0.0, 1000.0)
            spin.setSingleStep(0.1)

        self.kp_spin.setValue(1.0)
        self.ki_spin.setValue(0.0)
        self.kd_spin.setValue(0.0)

        self.send_pid_btn = QPushButton("Envoyer PID")

        pid_layout.addWidget(QLabel("Kp"), 0, 0)
        pid_layout.addWidget(self.kp_spin, 0, 1)
        pid_layout.addWidget(QLabel("Ki"), 1, 0)
        pid_layout.addWidget(self.ki_spin, 1, 1)
        pid_layout.addWidget(QLabel("Kd"), 2, 0)
        pid_layout.addWidget(self.kd_spin, 2, 1)
        pid_layout.addWidget(self.send_pid_btn, 3, 0, 1, 2)

        left_layout.addWidget(pid_box)

        # =========================
        # Télémétrie
        # =========================
        telemetry_box = QGroupBox("Télémétrie")
        telemetry_layout = QGridLayout()
        telemetry_box.setLayout(telemetry_layout)

        telemetry_keys = [
            "targetL", "targetR",
            "speedL", "speedR",
            "errL", "errR",
            "outL", "outR"
        ]

        for i, key in enumerate(telemetry_keys):
            name_label = QLabel(key)
            value_label = QLabel("--")
            telemetry_layout.addWidget(name_label, i, 0)
            telemetry_layout.addWidget(value_label, i, 1)
            self.telemetry_labels[key] = value_label

        right_layout.addWidget(telemetry_box)

        # =========================
        # Logs
        # =========================
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        right_layout.addWidget(self.log_box)

        # =========================
        # Connexions boutons
        # =========================
        self.connect_btn.clicked.connect(self.connect_robot)
        self.disconnect_btn.clicked.connect(self.disconnect_robot)

        self.forward_btn.clicked.connect(lambda: self.send_command("FORWARD"))
        self.backward_btn.clicked.connect(lambda: self.send_command("BACKWARD"))
        self.left_btn.clicked.connect(lambda: self.send_command("LEFT"))
        self.right_btn.clicked.connect(lambda: self.send_command("RIGHT"))
        self.stop_btn.clicked.connect(lambda: self.send_command("STOP"))

        self.send_pid_btn.clicked.connect(self.send_pid)

        # =========================
        # Timer lecture
        # =========================
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_data)
        self.timer.start(100)

    def log(self, text: str):
        self.log_box.append(text)

    def connect_robot(self):
        self.log(f"Tentative de connexion à {ROBOT_IP}:{ROBOT_PORT}")
        ok = self.client.connect(ROBOT_IP, ROBOT_PORT)

        if ok:
            self.status_label.setText("✅ Connecté")
            self.log(f"Connecté au robot {ROBOT_IP}:{ROBOT_PORT}")
            self.client.send("PING")
            self.log("➡️ PING")
        else:
            self.status_label.setText("❌ Connexion échouée")
            self.log(f"Connexion échouée vers {ROBOT_IP}:{ROBOT_PORT}")

    def disconnect_robot(self):
        self.client.disconnect()
        self.status_label.setText("❌ Non connecté")
        self.log("Déconnecté")

    def send_command(self, msg: str):
        self.client.send(msg)
        self.log(f"➡️ {msg}")

    def send_pid(self):
        cmd = build_pid_command(
            self.kp_spin.value(),
            self.ki_spin.value(),
            self.kd_spin.value()
        )
        self.client.send(cmd)
        self.log(f"➡️ {cmd}")

    def read_data(self):
        lines = self.client.receive_lines()

        for line in lines:
            msg_type, data = parse_message(line)

            if msg_type == "ACK":
                self.log(f"✅ ACK {data['command']}")

            elif msg_type == "TEL":
                for key, value in data.items():
                    if key in self.telemetry_labels:
                        self.telemetry_labels[key].setText(f"{value:.2f}")

            else:
                self.log(f"⬅️ {line}")