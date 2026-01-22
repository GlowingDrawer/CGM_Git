Route A (Defaults-only) patch

What this patch does:
1) Provides a defaults-only main.cpp:
   - Electrochemical parameters are compile-time defaults at the top of main.cpp.
   - You edit defaults, then re-flash firmware.
   - Runtime commands supported: HELP/SHOW/START/STOP/PAUSE/RESUME.
   - JSON output includes Mode and Mark (DPV sampling flags).

2) Updates DACManager.h/.cpp:
   - Split constant output into two cached values:
       - cachedScanConstantVal: used by scan channel in IT mode.
       - cachedBiasConstantVal: always used by bias channel.
     This allows IT mode to hold scan channel at one potential while keeping bias at another.

How to use:
- Replace your project's main.cpp with routeA_defaults_patch/main.cpp
- Replace DACManager.h/.cpp with the patched versions from this folder.
- Rebuild and re-flash.

Edit points:
- main.cpp -> section "DEFAULT electrochemical parameters".
