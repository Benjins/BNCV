@echo  off

cl /Od /Zi /Bt /bigobj /DBNS_DEBUG test_main.cpp /Fetest_main.exe


:: /I external/imgui/examples/libs/glfw/include

:: kernel32.lib user32.lib Gdi32.lib Shell32.lib legacy_stdio_definitions.lib legacy_stdio_wide_specifiers.lib Opengl32.lib 
 