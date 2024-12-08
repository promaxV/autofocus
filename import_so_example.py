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

autofocus_so.moveMotorSteps.argtypes = [ctypes.c_char_p, ctypes.c_ubyte, ctypes.c_int]
autofocus_so.moveMotorToPosition.argtypes = [ctypes.c_char_p, ctypes.c_ubyte, ctypes.c_int]
autofocus_so.setMotorDefaultValues.argtypes = [ctypes.c_char_p, ctypes.c_ubyte, ctypes.c_uint, ctypes.c_ulong, ctypes.c_ulong, ctypes.c_ulong]

# Пример управления мотором
port = input("Enter the serial port to use: ").encode('utf-8')
motor_id = 1
steps = 1000
unit_steps = 1
min_speed = 380000
max_speed = 1140000
accel = 10000

autofocus_so.moveMotorSteps(port, motor_id, steps)
autofocus_so.setMotorDefaultValues(port, motor_id, unit_steps, min_speed, max_speed, accel)