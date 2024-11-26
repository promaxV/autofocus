#include <iostream>
#include <windows.h>

typedef void (*StartAutoFocusFunc)(const char*, int);
typedef void (*TestDllExportFunc)(const char*, int);

int main() {
    HINSTANCE hinstLib = LoadLibrary(TEXT("autofocus.dll"));
    if (hinstLib == NULL) {
        std::cerr << "Error: Could not load the DLL." << std::endl;
        return 1;
    }

    StartAutoFocusFunc startAutoFocus = (StartAutoFocusFunc)GetProcAddress(hinstLib, "startAutoFocus");
    TestDllExportFunc testDllExport = (TestDllExportFunc)GetProcAddress(hinstLib, "testDllExport");

    if (!startAutoFocus || !testDllExport) {
        std::cerr << "Error: Could not locate the functions." << std::endl;
        FreeLibrary(hinstLib);
        return 1;
    }

    std::string port;
    std::cout << "Enter the serial port name: ";
    std::cin >> port;

    int camIndex;
    std::cout << "Enter the camera index: ";
    std::cin >> camIndex;

    testDllExport(port.c_str(), camIndex);
    startAutoFocus(port.c_str(), camIndex);

    FreeLibrary(hinstLib);
    return 0;
}
