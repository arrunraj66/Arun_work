# AUV middleware backup manifest

- Source archive: `auv_middleware_sandbox_backup_20260924_0504.tar.gz`
- Snapshot date: 2026-09-24 05:04
- Raw branch: `backup/2026-09-24-0504-raw`
- Verified branch: `backup/2026-09-24-0504-verified`

## Excluded generated files

The archive's `.cache/clangd/` index and Python `__pycache__/` bytecode are not
source code and were intentionally excluded from Git. They can be recreated by
the editor or Python interpreter.

## Audit corrections on the verified branch

1. Added `lidar_monitor_main.cpp` to `apps/CMakeLists.txt`.
2. Added `scan_publisher_recorder_main.cpp` to `apps/CMakeLists.txt` inside the
   `SICK_SCAN_XD_FOUND` block. The 2D service script depends on this target.
3. Normalized shell and Python scripts as executable files.
4. Ignored generated Protobuf Python files, SQLite runtime files, logs, and
   Python bytecode.
5. Added Ubuntu 22.04 continuous integration for configure, build, tests, shell
   syntax, and Python syntax.

## Verification scope

The CI job validates all targets and tests that do not need the physical SICK
SDK or sensor. Real sensor applications remain conditional on
`SICK_SCAN_XD_FOUND` and must be validated on the sensor workstation.

The provided SQLite `shim.c` links the Ubuntu system SQLite library. Therefore,
despite the directory name `third_party/sqlite3`, this snapshot still requires
`libsqlite3-dev` during a clean build.
