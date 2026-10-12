# 12 × XM430 migration — Stage 1

This folder is an isolated continuation of the current 8 × XM430-W350-R + 4 × AX-12A implementation in `../arduino_uart2_quad_standard_gait/`. The target hardware is 12 × XM430-W350-R. The current implementation remains the reference baseline.

Stage 1 copies the working firmware and adds documentation only. The gait and inverse kinematics are intentionally unchanged. The four Z-axis motors initially remain fixed-position holding joints; they are still configured and commanded as AX-12A in this Stage 1 sketch. The folder name describes the migration target, not the hardware supported by this sketch today.

The motor IDs, directions, offsets, neutral Z positions, and mechanical limits of the four replacement XM430 motors are unknown and require physical calibration on the real robot. The existing AX-12A IDs and holding angles are baseline values, not replacement XM430 settings.

Stage 2 will use verified calibration to configure the four replacement Z-axis motors. Until then, do not use this sketch as a 12 × XM430 configuration. The original `../arduino_uart2_quad_standard_gait/` folder is the unchanged reference for current robot behavior.
