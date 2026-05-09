"""Envoi de missions au robot via le port série selon le protocole C++.

Format attendu par le firmware (8 champs séparés par `;`, terminé par `F`):
  ID;TYPE_INT;STATUS_INT;OPTION_INT;DIRECTION_INT;TARGET_X;TARGET_Y;TARGET_THETA

Exemples utilisateurs (une ligne):
  send_mission('GO 0.5 0.0', serial_port='/dev/ttyUSB0')
  send_mission('TURN 90', serial_port='/dev/ttyUSB0')  # degrés tolérés
  send_mission('WAIT', serial_port='/dev/ttyUSB0')
  send_mission('RESUME', serial_port='/dev/ttyUSB0')

Si vous fournissez déjà la chaîne complète de 8 champs, elle sera envoyée telle quelle (ajoute 'F' si absent).
"""

from typing import Optional
import time
import math

try:
	import serial  # type: ignore
	_HAS_PYSERIAL = True
except Exception:
	_HAS_PYSERIAL = False


class MissionError(ValueError):
	pass


DEFAULT_SERIAL_PORT = "COM12"
DEFAULT_BAUD = 115200


def _gen_id() -> int:
	return int(time.time() * 1000) & 0x7FFFFFFF


def _format_mission_message(id_: int, type_int: int, status_int: int = 0, options: int = 0, direction: int = 0,
							tx: float = 0.0, ty: float = 0.0, ttheta: float = 0.0) -> str:
	# Use '.' as decimal separator, keep floats compact
	return f"{id_};{type_int};{status_int};{options};{direction};{tx:.6g};{ty:.6g};{ttheta:.6g}F"


def _is_raw_mission_string(s: str) -> bool:
	parts = s.strip().split(';')
	return len(parts) == 8


def _open_serial(serial_port: str = DEFAULT_SERIAL_PORT, baud: int = DEFAULT_BAUD):
	if not _HAS_PYSERIAL:
		raise MissionError("pyserial non installé. Installer avec 'pip install pyserial'")
	return serial.Serial(serial_port, baud, timeout=0.2)


def send_mission(command: str, backend: str = "serial", serial_port: Optional[str] = DEFAULT_SERIAL_PORT, baud: int = DEFAULT_BAUD) -> None:
	"""Envoie une mission vers le robot.

	- `command`: forme libre — soit la chaîne complète de 8 champs séparés par `;`, soit une commande courte:
		 'GO x y'  -> mission GO vers coordonnée (x,y)
		 'TURN a'  -> mission TURN vers angle `a` (deg ou rad; >2π => deg converti)
		 'WAIT'    -> mission WAIT (pause, attend RESUME)
		 'RESUME'  -> mission RESUME
	- `backend`: 'serial' ou 'console'
	- `serial_port`: requis si `backend=='serial'`
	"""
	if not command or not isinstance(command, str):
		raise MissionError("Commande invalide")

	cmd = command.strip()

	# If already raw mission string (8 fields), send as-is (ensure terminating F)
	if _is_raw_mission_string(cmd):
		payload = cmd
		if not payload.endswith('F'):
			payload = payload + 'F'
	else:
		parts = cmd.split()
		head = parts[0].upper()
		if head == 'RESUME':
			payload = _format_mission_message(_gen_id(), 3)
		elif head == 'WAIT':
			payload = _format_mission_message(_gen_id(), 2)
		elif head == 'GO':
			if len(parts) < 3:
				raise MissionError("GO requiert deux arguments: GO <x> <y>")
			try:
				x = float(parts[1])
				y = float(parts[2])
			except Exception:
				raise MissionError("Coordonnées GO invalides")
			payload = _format_mission_message(_gen_id(), 0, tx=x, ty=y)
		elif head == 'TURN':
			if len(parts) < 2:
				raise MissionError("TURN requiert un angle: TURN <angle>")
			try:
				angle = float(parts[1])
			except Exception:
				raise MissionError("Angle TURN invalide")
			# If angle is large (>2pi), assume degrees
			if abs(angle) > 2 * math.pi:
				angle = math.radians(angle)
			payload = _format_mission_message(_gen_id(), 1, ttheta=angle)
		elif head == 'STOP':
			payload = _format_mission_message(_gen_id(), 4)
		elif head == 'RESET' or head == 'R':
			payload = 'R'  # reset command single-letter; firmware detects this
		elif head == 'DFU' or head == 'D':
			payload = 'D'  # request DFU mode
		elif head == 'STOP':
			payload = 'L'
		else:
			raise MissionError(f"Commande inconnue ou format non supporté: {head}")

	if backend == 'console':
		print('[send_mission] ->', payload)
		return

	if backend == 'serial':
		if not serial_port:
			raise MissionError("serial_port requis pour backend 'serial'")
		try:
			with _open_serial(serial_port, baud) as ser:
				# Ensure payload ends with 'F' when sending mission packets
				if payload not in ('R', 'D') and not payload.endswith('F'):
					payload = payload + 'F'
				print(payload)
				ser.write(payload.encode('ascii'))
				ser.flush()
		except Exception as e:
			raise MissionError(f"Erreur en envoyant la mission sur le port série: {e}")
	else:
		raise MissionError(f"Backend inconnu: {backend}")


def print_serial_logs(serial_port: str = DEFAULT_SERIAL_PORT, baud: int = DEFAULT_BAUD, duration_s: Optional[float] = None) -> None:
	"""Affiche en continu les logs renvoyés par le moniteur série."""
	start_time = time.time()
	try:
		with _open_serial(serial_port, baud) as ser:
			print(f"[serial] écoute des logs sur {serial_port} à {baud} bauds")
			while True:
				if duration_s is not None and time.time() - start_time >= duration_s:
					break

				raw_line = ser.readline()
				if not raw_line:
					continue

				line = raw_line.decode("utf-8", errors="replace").strip()
				if line:
					print(f"[robot] {line}")
	except KeyboardInterrupt:
		print("\n[serial] arrêt de l'écoute")
	except Exception as e:
		raise MissionError(f"Erreur pendant la lecture du port série: {e}")

def emergencyStop(serial_port: str = DEFAULT_SERIAL_PORT, baud: int = DEFAULT_BAUD):
	with _open_serial(serial_port, baud) as ser:
		payload = "LXF"
		ser.write(payload.encode('ascii'))
		ser.flush()

def resume():
	send_mission('RESUME', backend='serial')

def wait():
	send_mission('WAIT', backend='serial')

def go(x: float, y: float):
	send_mission(f'GO {x} {y}', backend='serial')

def turn(angle: float):
	send_mission(f'TURN {angle}', backend='serial')

def reset():
    send_mission('RESET', backend='serial')

def stop():
    send_mission('RESET', backend='serial')

def stop():
    send_mission('STOP', backend='serial')


def go_and_back(distance: float):
	go(distance, 0.0)
	turn(180)
	go(0.0, 0.0)
	turn(0)

def oneTurn():
	turn(90)
	turn(180)
	turn(-90)
	turn(0)


if __name__ == '__main__':

	reset()
	turn(90)
	turn(0)
	
	# stop()
	#go(1.5, 0.0)

	# go(1.0, 0.0)
	# turn(90)
	# go(1.0, 0.25)
	# turn(180)
	# go(0.0, 0.25)
	# turn(270)
	# go(0.0, 0.0)
	# turn(0)
	
	# time.sleep(2)
	# emergencyStop()
	# time.sleep(5)
	# go(1.0, 0.0)
	# turn(90)
	
	# for i in range(5):
	# 	oneTurn()
	#go(1.5, 0.0)

	# for i in range(0, 3):
	# 	go(1.5, 0.0)
	# 	turn(180)
	# 	go(0.0, 0.0)
	# 	turn(0)

	#turn(180)
	# go(-0.5, 0.0)
	# turn(0)
	# go(-0.2, 0.0)
	# # for i in range(0, 10):
	# # 	oneTurn()

	print_serial_logs()
	# time.sleep(10)
	# stop()