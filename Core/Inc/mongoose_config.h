#pragma once

// See https://mongoose.ws/documentation/#build-options
#define MG_ARCH MG_ARCH_NEWLIB //Specifica arhitectura utilizata, în acest caz newlib, care este potrivită pentru medii embedded
#define MG_TLS MG_TLS_BUILTIN

#define MG_ENABLE_TCPIP 1  //Activează suportul pentru conexiuni TCP/IP necesar pentru MQTT
#define MG_ENABLE_CUSTOM_MILLIS 1
#define MG_ENABLE_CUSTOM_RANDOM 1
#define MG_ENABLE_PACKED_FS 1 
#define MG_ENABLE_DRIVER_STM32F 1  //Activează driverele pentru STM32.
