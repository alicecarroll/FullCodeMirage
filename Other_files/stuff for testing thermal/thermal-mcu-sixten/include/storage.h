#ifndef STORAGE_H
#define STORAGE_H

// Loads configuration parameters from NVS; initializes defaults if keys don't exist
void loadConfiguration();

// Saves current configuration parameters from global memory to NVS
void saveConfiguration();

#endif // STORAGE_H
