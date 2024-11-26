import platform
import ctypes

print("Platform:", platform.architecture())

sdk_handle = ctypes.WinDLL('./build/Release/autofocus.dll')
print(sdk_handle)

null_ptr = ctypes.POINTER(ctypes.c_int)()

some_string = b'TestString'
some_int = 123

sdk_handle.testDllExport(some_string, some_int)