import platform
import ctypes

print("Platform:", platform.architecture())

# Загрузка DLL
try:
    autofocus_dll = ctypes.CDLL('./build/Release/autofocus.dll')
except OSError as e:
    print(f"Error loading DLL: {e}")
    exit(1)

# Получение списка доступных функций
def list_functions(dll):
    functions = []
    for name in dir(dll):
        if not name.startswith('_'):
            try:
                func = getattr(dll, name)
                if callable(func):
                    functions.append(name)
            except AttributeError:
                continue
    return functions

# Вывод списка функций
functions = list_functions(autofocus_dll)
print("Available functions in DLL:")
for func in functions:
    print(func)
print()

# Проверка наличия функции startAutoFocus
if 'startAutoFocus' in functions:
    # Определение аргументов и возвращаемого типа функции
    autofocus_dll.startAutoFocus.argtypes = [ctypes.c_char_p, ctypes.c_int]
    autofocus_dll.startAutoFocus.restype = None

    # Вызов функции
    port = b'COM3'  # Замените на ваш порт
    camera_index = 0  # Замените на ваш индекс камеры

    try:
        autofocus_dll.startAutoFocus(port, camera_index)
        print("Function 'startAutoFocus' called successfully.")
    except Exception as e:
        print(f"Error calling function 'startAutoFocus': {e}")
else:
    print("Error: Function 'startAutoFocus' not found in DLL.")

# Проверка наличия функции testDllExport
if 'testDllExport' in functions:
    # Определение аргументов и возвращаемого типа функции
    autofocus_dll.testDllExport.argtypes = [ctypes.c_char_p, ctypes.c_int]
    autofocus_dll.testDllExport.restype = None

    # Вызов функции
    some_string = b'TestString'
    some_int = 123

    try:
        autofocus_dll.testDllExport(some_string, some_int)
        print("Function 'testDllExport' called successfully.")
    except Exception as e:
        print(f"Error calling function 'testDllExport': {e}")
else:
    print("Error: Function 'testDllExport' not found in DLL.")
