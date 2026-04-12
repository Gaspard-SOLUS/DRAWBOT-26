def parse_message(raw: str):
    raw = raw.strip()

    if not raw:
        return ("EMPTY", {})

    if raw.startswith("ACK;"):
        return ("ACK", {"command": raw[4:]})

    if raw.startswith("TEL;"):
        data = {}
        payload = raw[4:]
        parts = payload.split(";")

        for part in parts:
            if "=" in part:
                key, value = part.split("=", 1)
                key = key.strip()
                value = value.strip()

                try:
                    data[key] = float(value)
                except ValueError:
                    data[key] = value

        return ("TEL", data)

    return ("RAW", {"text": raw})


def build_pid_command(kp: float, ki: float, kd: float) -> str:
    return f"SET_PID;kp={kp:.3f};ki={ki:.3f};kd={kd:.3f}"