from serial import Serial
import threading

class ArduinoInterface:

	def __init__(self, port: str = "/dev/ttyUSB1", baud: int = 9600) -> None:
		self._ser  = None
		self._connected = False
		self._lock = threading.Lock()
		try:
			self._ser = Serial(port, baud, timeout=2)
			self._connected = True
		except Exception as e:
			print(f'Arduino not available, running without lights\n{e}\n')

	@property
	def connected(self):
		return self._connected

	def send(self, cmd:str) -> str:
		if self._ser is not None:
			with self._lock:
				self._ser.write(f"{cmd}\n".encode())
				return self._ser.readline().decode().strip()
		else:
			return "NO_DEVICE"

	def close(self) -> None:
		if self._ser is not None:
			self._ser.close()
