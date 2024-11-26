#include <iostream>
#include <dlfcn.h>

typedef void (*StartAutoFocusFunc)(const char*, int);
typedef void (*TestDllExportFunc)(const char*, int);

int main() {
    void* handle = dlopen("./libautofocus.so", RTLD_LAZY);
    if (!handle) {
        std::cerr << "Error: Could not load the shared library." << std::endl;
        return 1;
    }

    StartAutoFocusFunc startAutoFocus = (StartAutoFocusFunc)dlsym(handle, "startAutoFocus");
    TestDllExportFunc testDllExport = (TestDllExportFunc)dlsym(handle, "testDllExport");

    if (!startAutoFocus || !testDllExport) {
        std::cerr << "Error: Could not locate the functions." << std::endl;
        dlclose(handle);
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

    dlclose(handle);
    return 0;
}
