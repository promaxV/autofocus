import platform
import ctypes

print("Platform:", platform.architecture())

# Загрузка .so файла
try:
    autofocus_so = ctypes.CDLL('./build/Release/libautofocus.so')
except OSError as e:
    print(f"Error loading .so file: {e}")
    exit(1)

# Получение списка доступных функций
def list_functions(so):
    functions = []
    for name in dir(so):
        if not name.startswith('_'):
            try:
                func = getattr(so, name)
                if callable(func):
                    functions.append(name)
            except AttributeError:
                continue
    return functions

# Вывод списка функций
functions = list_functions(autofocus_so)
print("Available functions in .so file:")
for func in functions:
    print(func)
print()

# Проверка наличия функции startAutoFocus
if 'startAutoFocus' in functions:
    # Определение аргументов и возвращаемого типа функции
    autofocus_so.startAutoFocus.argtypes = [ctypes.c_char_p, ctypes.c_int]
    autofocus_so.startAutoFocus.restype = None

    # Вызов функции
    port = b'COM3'  # Замените на ваш порт
    camera_index = 0  # Замените на ваш индекс камеры

    try:
        autofocus_so.startAutoFocus(port, camera_index)
        print("Function 'startAutoFocus' called successfully.")
    except Exception as e:
        print(f"Error calling function 'startAutoFocus': {e}")
else:
    print("Error: Function 'startAutoFocus' not found in .so file.")

# Проверка наличия функции testDllExport
if 'testDllExport' in functions:
    # Определение аргументов и возвращаемого типа функции
    autofocus_so.testDllExport.argtypes = [ctypes.c_char_p, ctypes.c_int]
    autofocus_so.testDllExport.restype = None

    # Вызов функции
    some_string = b'TestString'
    some_int = 123

    try:
        autofocus_so.testDllExport(some_string, some_int)
        print("Function 'testDllExport' called successfully.")
    except Exception as e:
        print(f"Error calling function 'testDllExport': {e}")
else:
    print("Error: Function 'testDllExport' not found in .so file.")
