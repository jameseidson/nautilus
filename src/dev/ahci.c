#include <nautilus/nautilus.h>
#include <dev/ahci.h>

#ifndef NAUT_CONFIG_DEBUG_AHCI
#undef DEBUG_PRINT
#define DEBUG_PRINT(fmt, args...) 
#endif

#define ERROR(fmt, args...) ERROR_PRINT("ahci: " fmt, ##args)
#define DEBUG(fmt, args...) DEBUG_PRINT("ahci: " fmt, ##args)
#define INFO(fmt, args...) INFO_PRINT("ahci: " fmt, ##args)