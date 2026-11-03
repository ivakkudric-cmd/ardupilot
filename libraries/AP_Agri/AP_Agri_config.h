#pragma once
#include <AP_HAL/AP_HAL_Boards.h>

// Waf 4.5.7 compiles common library objects without vehicle macros. Keep this
// config board-based; AP_Agri is linked by ArduCopter/wscript only. Other vehicle
// builds from this Copter fork must explicitly set AP_AGRI_ENABLED=0.
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL ||                                                          \
    (CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && defined(APJ_BOARD_ID) && APJ_BOARD_ID == 1063)
#define AP_AGRI_SUPPORTED 1
#else
#define AP_AGRI_SUPPORTED 0
#endif
#ifndef AP_AGRI_ENABLED
#define AP_AGRI_ENABLED AP_AGRI_SUPPORTED
#endif
#if AP_AGRI_ENABLED && !AP_AGRI_SUPPORTED
#error AP_AGRI_ENABLED is restricted to SITL and CubeOrangePlus
#endif
