# local imports
from .device import BaseDevice
from .type import DeviceType

# global imports
import smbus2
import time


# Constants for the LCD
LCD_CHR = 1 # Mode - Sending data
LCD_CMD = 0 # Mode - Sending command
LINE_1 = 0x80 # LCD RAM address for the 1st line
LINE_2 = 0xC0 # LCD RAM address for the 2nd line
ENABLE = 0b00000100 # Enable bit

class LcdDisplay(BaseDevice):
    def __init__(self):
        super().__init__(node_name="car_lcd", device_type=DeviceType.DISPLAY)
        
        # We can make the address a parameter too
        self.declare_parameter('address', 0x27)
        self.addr = self.get_parameter('address').value
        self.bus = None

    def _initialize(self) -> bool:
        """
        Wakes up the I2C bus and runs the LCD init sequence.
        """
        try:
            self.bus = smbus2.SMBus(1) # RPi usually uses Bus 1
            
            # Initialization sequence for 16x2 LCD
            self._byte_write(0x33, LCD_CMD) # 110011 Initialise
            self._byte_write(0x32, LCD_CMD) # 110010 Initialise
            self._byte_write(0x06, LCD_CMD) # 000110 Cursor move direction
            self._byte_write(0x0C, LCD_CMD) # 001100 Display On,Cursor Off, Blink Off
            self._byte_write(0x28, LCD_CMD) # 101000 Data length, number of lines, font size
            self._byte_write(0x01, LCD_CMD) # 000001 Clear display
            time.sleep(0.005)
            
            return True
        except Exception as e:
            self.get_logger().error(f"LCD Init Failed: {e}")
            return False

    def _loop(self) -> None:
        """
        For now, let's just display the car's uptime or status.
        """
        if self.is_active:
            # This is where you'd push dynamic data
            # In the future, this will come from a Subscriber!
            self._display_text("AISE 4020", LINE_1)
            self._display_text("SYSTEM READY", LINE_2)

    def _shutdown(self) -> None:
        """
        Clear the screen on exit so it doesn't stay 'stuck'.
        """
        if self.bus:
            self.get_logger().info("Clearing LCD screen...")
            self._byte_write(0x01, LCD_CMD)
            self.bus.close()

    # --- Hardware Specific Low-Level Methods ---

    def _byte_write(self, bits, mode):
        """ Sends a byte to the hardware via I2C """
        # LCDs use 4-bit mode via the backpack, so we send two nibbles
        # High bits
        bits_high = mode | (bits & 0xF0) | 0x08 # 0x08 is for backlight
        # Low bits
        bits_low = mode | ((bits << 4) & 0xF0) | 0x08
        
        self.bus.write_byte(self.addr, bits_high)
        self._toggle_enable(bits_high)
        self.bus.write_byte(self.addr, bits_low)
        self._toggle_enable(bits_low)

    def _toggle_enable(self, bits):
        """ Pulse the enable pin on the LCD """
        time.sleep(0.0005)
        self.bus.write_byte(self.addr, (bits | ENABLE))
        time.sleep(0.0005)
        self.bus.write_byte(self.addr, (bits & ~ENABLE))
        time.sleep(0.0005)

    def _display_text(self, message, line):
        """ Formats the string to 16 chars and sends it to a specific line """
        message = message.ljust(16, " ")
        self._byte_write(line, LCD_CMD)
        for i in range(16):
            self._byte_write(ord(message[i]), LCD_CHR)