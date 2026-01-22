# Solution A: Fix ArmClang C++ Standard Library Headers (e.g., <array> not found)

This patch solves errors like:
- `'array' file not found` (armclang)

Your project includes C++ standard headers (e.g., `<array>`, `<cstdint>`, `<cstring>`). If your build command does not add ArmClang's C++ include directories, ArmClang cannot find these headers.

## 1) Locate ARMCLANG root

Typical locations:
- **Keil MDK v5**: `C:\Keil_v5\ARM\ARMCLANG`
- **Keil MDK v6**: `C:\Keil_v6\ARM\ARMCLANG`

You should see:
- `...\ARMCLANG\bin\armclang.exe`
- `...\ARMCLANG\include\`
- `...\ARMCLANG\include\c++\v1\`

## 2) Add include paths to the compiler

You must add BOTH include directories:

- `...\ARMCLANG\include`
- `...\ARMCLANG\include\c++\v1`

### Option A: Keil uVision (Arm Compiler 6)

1. Open **Options for Target**
2. Go to **C/C++ (AC6)**
3. In **Include Paths**, add:
   - `C:\Keil_v5\ARM\ARMCLANG\include`
   - `C:\Keil_v5\ARM\ARMCLANG\include\c++\v1`

If you prefer using **Misc Controls**, add:

- `--include_directory="C:\\Keil_v5\\ARM\\ARMCLANG\\include"`
- `--include_directory="C:\\Keil_v5\\ARM\\ARMCLANG\\include\\c++\\v1"`

### Option B: VSCode + armclang (tasks / Makefile / custom build)

Add these flags to **every C++ compile** command:

- `-I"<ARMCLANG_ROOT>/include"`
- `-I"<ARMCLANG_ROOT>/include/c++/v1"`

Example:

```
armclang --target=arm-arm-none-eabi -mcpu=cortex-m3 -mthumb \
  -I"C:/Keil_v5/ARM/ARMCLANG/include" \
  -I"C:/Keil_v5/ARM/ARMCLANG/include/c++/v1" \
  -I"<your_project_includes>" \
  -c your_file.cpp -o your_file.o
```

Important notes:
- If you are using `--nostdinc++` or `-nostdinc++`, **remove it**.
- If you have a separate `CFLAGS` and `CXXFLAGS`, make sure these go into **CXXFLAGS**.

## 3) IntelliSense (optional)

If VSCode shows squiggles even after build is fixed, update `.vscode/c_cpp_properties.json` with the same include paths.

A template is provided in this patch: `vscode/c_cpp_properties.json`.

## 4) Validate

After applying, these should compile without the header error:
- `InitArg.h` (uses `<array>`)
- `BTCPP.h` (uses `<array>`)

