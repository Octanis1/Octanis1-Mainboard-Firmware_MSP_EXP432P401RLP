#include "state_machine.h"

eps_status_t eps_status;
module_status_t module_status[N_MODULES]; //stores the answers to be sent to an eventual i2c request
