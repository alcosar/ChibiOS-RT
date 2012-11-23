# List of the ChibiOS/RT Cortex-M4 LM4F port files.
PORTSRC = $(CHIBIOS)/os/ports/GCC/ARMCMx/crt0.c \
          ${CHIBIOS}/os/ports/GCC/ARMCMx/LM4F/vectors.c \
          ${CHIBIOS}/os/ports/GCC/ARMCMx/chcore.c \
          ${CHIBIOS}/os/ports/GCC/ARMCMx/chcore_v7m.c \
          ${CHIBIOS}/os/ports/common/ARMCMx/nvic.c

PORTASM =

PORTINC = ${CHIBIOS}/os/ports/common/ARMCMx/CMSIS/include \
          ${CHIBIOS}/os/ports/common/ARMCMx \
          ${CHIBIOS}/os/ports/GCC/ARMCMx \
          ${CHIBIOS}/os/ports/GCC/ARMCMx/LM4F

PORTLD  = ${CHIBIOS}/os/ports/GCC/ARMCMx/LM4F/ld
