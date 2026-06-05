# BLK360 runtime library (not included in this repo)

`libBLK360.so` is **proprietary Leica software** and is intentionally **not**
committed to this public repository (it is git-ignored).

To build the `blk360_scanner` package, obtain `libBLK360.so` from Leica's BLK360
SDK and place it here:

```
src/blk360_scanner/third_party/lib/libBLK360.so
```

`CMakeLists.txt` links the node against it and copies it next to the installed
executable (resolved via `$ORIGIN` at runtime).

Without this file, **only** `blk360_scanner` fails to build — the rest of the
stack (`blk360_bringup`, `blk360_stop_scan` incl. the mock scanner, exploration,
SLAM, Nav2) builds and runs in simulation just fine:

```bash
colcon build --packages-skip blk360_scanner
```
