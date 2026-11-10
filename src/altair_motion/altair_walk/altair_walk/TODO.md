# TODO

- Belum ada buat nyambungin dari walk.py ke op3_framework module. Buat motion_module baru yg subscibe array data nilai joint dari walk.py, sehingga perlu juga publisher array yg isinya nilai sendi kaki buat motion_module nya.
- Belum ada fall detection, walk.py butuh data dari subscribe "/is_falling". Threshold dari gyro dan acceleration kayaknya cukup
- Belum diuji di robot asli
- Bezier curve hanya quadratic/ orde dua, artinya hanya C0 atau hanya kontinyus di posisi. Mungkin dapat diganti ke cubic bezier curve biar C1 atau kontinyus di posisi dan velocity.
- trunk_yaw: IK now accepts a trunk yaw and walk.py passes the average foot yaw; offline sweep tools/ik_yaw_sweep.py passes up to 180 deg. Still needs a sim straight-walk regression check.
- Turn controller: altair_turn package (turn_supervisor + cmd_mux) with heading_hold /target_yaw topic, wrap-safe error and correction slew. Turn envelope (max_angular vs stability) not yet measured in sim.
- Fall detection still missing (no /is_falling publisher); required before any hardware turning.
