When launching VTOL in fixed wing mode, make sure these parameters are configured in QGC:
- CBRK_VTOLARMING: 159753
- FW_W_EN: ENABLED
- RWTO_TKOFF: ENABLED
- NAV_FORCE_VT: 0

OR if you want to permanently set these (you probably should), add these 4 lines to your 4004_gz_standard vtol file located at:
~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4004_gz_standard_vtol

param set-default CBRK_VTOLARMING 159753
param set-default FW_W_EN 1
param set-default RWTO_TKOFF 1
param set-default NAV_FORCE_VT 0