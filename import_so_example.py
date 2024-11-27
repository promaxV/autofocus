import platform
import ctypes
import serial.tools.list_ports

print("Platform:", platform.architecture())
try:
    autofocus_so = ctypes.CDLL('./libautofocus.so')
except OSError as e:
    print(f"Error loading SO: {e}")
    exit(1)

some_string = b'TestString'
some_int = 123

autofocus_so.testDllExport(some_string, some_int)

ports = serial.tools.list_ports.comports()
print("Available serial ports:")
for port in ports:
    print(port.device)

port = input("Enter the serial port to use: ")
camera_index = int(input("Enter the camera index: "))

autofocus_so.startAutoFocus.argtypes = [ctypes.c_char_p, ctypes.c_int]
autofocus_so.startAutoFocus.restype = ctypes.c_void_p

autofocus_so.startAutoFocus(port.encode('utf-8'), camera_index)