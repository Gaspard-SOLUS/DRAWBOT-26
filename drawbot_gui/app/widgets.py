from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QLabel,
    QPushButton, QLineEdit, QGroupBox
)


# ======================
# Bloc connexion WiFi
# ======================
class ConnectionWidget(QGroupBox):
    def __init__(self):
        super().__init__("Connexion WiFi")

        layout = QVBoxLayout()

        self.ip_input = QLineEdit()
        self.ip_input.setPlaceholderText("IP du robot (ex: 192.168.1.50)")

        self.port_input = QLineEdit()
        self.port_input.setPlaceholderText("Port (ex: 1234)")

        self.connect_btn = QPushButton("Connecter")
        self.disconnect_btn = QPushButton("Déconnecter")

        layout.addWidget(QLabel("Adresse IP"))
        layout.addWidget(self.ip_input)

        layout.addWidget(QLabel("Port"))
        layout.addWidget(self.port_input)

        layout.addWidget(self.connect_btn)
        layout.addWidget(self.disconnect_btn)

        self.setLayout(layout)


# ======================
# Bloc contrôle robot
# ======================
class ControlWidget(QGroupBox):
    def __init__(self):
        super().__init__("Contrôle Robot")

        layout = QVBoxLayout()

        self.forward_btn = QPushButton("Avancer")
        self.backward_btn = QPushButton("Reculer")
        self.left_btn = QPushButton("Gauche")
        self.right_btn = QPushButton("Droite")
        self.stop_btn = QPushButton("STOP")

        layout.addWidget(self.forward_btn)
        layout.addWidget(self.backward_btn)
        layout.addWidget(self.left_btn)
        layout.addWidget(self.right_btn)
        layout.addWidget(self.stop_btn)

        self.setLayout(layout)