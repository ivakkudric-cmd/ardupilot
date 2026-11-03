#include "Copter.h"
#if AP_AGRI_ENABLED
void Copter::agri_update() { g2.agri.update(); }
#endif
