import platform
import ctypes
import serial.tools.list_ports

print("Platform:", platform.architecture())
try:
    autofocus_dll = ctypes.WinDLL('./build/Release/autofocus.dll')
except OSError as e:
    print(f"Error loading DLL: {e}")
    exit(1)

some_string = b'TestString'
some_int = 123

autofocus_dll.testDllExport(some_string, some_int)

ports = serial.tools.list_ports.comports()
print("Available serial ports:")
for port in ports:
    print(port.device)

port = input("Enter the serial port to use: ")
camera_index = int(input("Enter the camera index: "))

autofocus_dll.startAutoFocus.argtypes = [ctypes.c_char_p, ctypes.c_int]
autofocus_dll.startAutoFocus.restype = ctypes.c_void_p

autofocus_dll.startAutoFocus(port.encode('utf-8'), camera_index)