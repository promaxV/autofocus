# Документация модуля AutoFocus

## Экспортируемые функции

### `startAutoFocus`

**Описание:**
Запускает процесс автоматической фокусировки.

**Аргументы:**
- `const char* portName`: Имя порта для подключения.
- `int cameraIndex`: Индекс камеры.

### `autoFocusWindow`

**Описание:**
Открывает окно с изображением с камеры. По нажатию клавиши ПРОБЕЛ запускается автоматическая фокусировка. По нажатию клавиши ESC происходит выход из программы.

**Аргументы:**
- `const char* portName`: Имя порта для подключения.
- `int cameraIndex`: Индекс камеры.

### `moveMotorSteps`

**Описание:**
Перемещает мотор на заданное количество шагов.

**Аргументы:**
- `const char* portName`: Имя порта для подключения.
- `unsigned char motorId`: Идентификатор мотора.
- `int steps`: Количество шагов.

### `setMotorDefaultValues`

**Описание:**
Устанавливает стандартные значения параметров для мотора.

**Аргументы:**
- `const char* portName`: Имя порта для подключения.
- `unsigned char motorId`: Идентификатор мотора.
- `unsigned int unitSteps`: Количество шагов за единицу хода.
- `unsigned long minSpeed`: Минимальная скорость.
- `unsigned long maxSpeed`: Максимальная скорость.
- `unsigned long accel`: Ускорение.

## Пример импорта функций из DLL или SO файла

### Импорт из DLL (Windows)

```c
#include <windows.h>
#include <stdio.h>

typedef void (*StartAutoFocus)(const char*, int);
typedef void (*MoveMotorSteps)(const char*, unsigned char, int);
typedef void (*SetMotorDefaultValues)(const char*, unsigned char, unsigned int, unsigned long, unsigned long, unsigned long);

int main() {
    HINSTANCE hInstLibrary = LoadLibrary("autofocus.dll");
    if (hInstLibrary) {
        StartAutoFocus startAutoFocus = (StartAutoFocus)GetProcAddress(hInstLibrary, "startAutoFocus");
        MoveMotorSteps moveMotorSteps = (MoveMotorSteps)GetProcAddress(hInstLibrary, "moveMotorSteps");
        SetMotorDefaultValues setMotorDefaultValues = (SetMotorDefaultValues)GetProcAddress(hInstLibrary, "setMotorDefaultValues");

        if (startAutoFocus && moveMotorSteps && setMotorDefaultValues) {
            startAutoFocus("COM3", 0);
            moveMotorSteps("COM3", 1, 100);
            setMotorDefaultValues("COM3", 1, 10, 100, 1000, 10);
        }

        FreeLibrary(hInstLibrary);
    } else {
        printf("Не удалось загрузить библиотеку.\n");
    }

    return 0;
}
```

### Импорт из SO файла (Linux)

```c
#include <dlfcn.h>
#include <stdio.h>

typedef void (*StartAutoFocus)(const char*, int);
typedef void (*MoveMotorSteps)(const char*, unsigned char, int);
typedef void (*SetMotorDefaultValues)(const char*, unsigned char, unsigned int, unsigned long, unsigned long, unsigned long);

int main() {
    void* handle = dlopen("libautofocus.so", RTLD_LAZY);
    if (handle) {
        StartAutoFocus startAutoFocus = (StartAutoFocus)dlsym(handle, "startAutoFocus");
        MoveMotorSteps moveMotorSteps = (MoveMotorSteps)dlsym(handle, "moveMotorSteps");
        SetMotorDefaultValues setMotorDefaultValues = (SetMotorDefaultValues)dlsym(handle, "setMotorDefaultValues");

        if (startAutoFocus && moveMotorSteps && setMotorDefaultValues) {
            startAutoFocus("COM3", 0);
            moveMotorSteps("COM3", 1, 100);
            setMotorDefaultValues("COM3", 1, 10, 100, 1000, 10);
        }

        dlclose(handle);
    } else {
        printf("Не удалось загрузить библиотеку: %s\n", dlerror());
    }

    return 0;
}
```

## Пример импорта и использования в Python

### Импорт из DLL (Windows)

```python
import ctypes

# Загрузка библиотеки
autofocus = ctypes.WinDLL('autofocus.dll')

# Определение типов аргументов и возвращаемых значений функций
autofocus.startAutoFocus.argtypes = [ctypes.c_char_p, ctypes.c_int]
autofocus.moveMotorSteps.argtypes = [ctypes.c_char_p, ctypes.c_ubyte, ctypes.c_int]
autofocus.setMotorDefaultValues.argtypes = [ctypes.c_char_p, ctypes.c_ubyte, ctypes.c_uint, ctypes.c_ulong, ctypes.c_ulong, ctypes.c_ulong]

# Вызов функций
autofocus.startAutoFocus(b"COM3", 0)
autofocus.moveMotorSteps(b"COM3", 1, 100)
autofocus.setMotorDefaultValues(b"COM3", 1, 10, 100, 1000, 10)
```

### Импорт из SO файла (Linux)

```python
import ctypes

# Загрузка библиотеки
autofocus = ctypes.CDLL('./libautofocus.so')

# Определение типов аргументов и возвращаемых значений функций
autofocus.startAutoFocus.argtypes = [ctypes.c_char_p, ctypes.c_int]
autofocus.moveMotorSteps.argtypes = [ctypes.c_char_p, ctypes.c_ubyte, ctypes.c_int]
autofocus.setMotorDefaultValues.argtypes = [ctypes.c_char_p, ctypes.c_ubyte, ctypes.c_uint, ctypes.c_ulong, ctypes.c_ulong, ctypes.c_ulong]

# Вызов функций
autofocus.startAutoFocus(b"COM3", 0)
autofocus.moveMotorSteps(b"COM3", 1, 100)
autofocus.setMotorDefaultValues(b"COM3", 1, 10, 100, 1000, 10)
```