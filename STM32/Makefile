################################################################################
# İDA Otopilot (STM32F405RGT6) Makefile
# 
# Bu Makefile, GCC ARM Toolchain kullanarak STM32F405RGT6 için yazılmış FreeRTOS 
# tabanlı otopilot projesini derlemek üzere hazırlanmıştır.
# Donanımsal FPU ve ART hızlandırma derleme bayrakları aktif edilmiştir.
################################################################################

# Hedef Uygulama Adı
TARGET = aydede_otopilot

# Derleme Optimizasyonu ve Hata Ayıklama
DEBUG = 1
OPT = -O2

# Kaynak Klasör Yolları
BUILD_DIR = build

# C Derleyici Kaynakları
C_SOURCES =  \
Core/Src/main.c \
Core/Src/control.c \
Core/Src/sensors.c \
Core/Src/safety.c \
Core/Src/stm32f4xx_it.c \
Core/Src/system_stm32f4xx.c

# HAL ve FreeRTOS kaynakları (C8 düzeltmesi: yorumdan çıkarıldı)
C_SOURCES += $(wildcard Drivers/STM32F4xx_HAL_Driver/Src/*.c)
C_SOURCES += $(wildcard Middlewares/Third_Party/FreeRTOS/Source/*.c)
C_SOURCES += Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
C_SOURCES += Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c

# ASM Kaynakları (Startup dosyası)
ASM_SOURCES =  \
startup_stm32f405xx.s

################################################################################
# Derleyici Araçları (Toolchain)
################################################################################
PREFIX = arm-none-eabi-
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
################################################################################
# MCU & Donanımsal FPU Bayrakları
################################################################################
# FPU ve ART hızlandırma için F4 serisi donanımsal kayar nokta birimini (M4F) kullanır.
CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# C Makroları (Defines)
C_DEFS =  \
-DSTM32F405xx \
-DUSE_HAL_DRIVER

# C Include Klasörleri
C_INCLUDES =  \
-ICore/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
-IDrivers/CMSIS/Include \
-IMiddlewares/Third_Party/FreeRTOS/Source/include \
-IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F

# Derleyici Bayrakları (Compiler Flags)
ASFLAGS = $(MCU) $(AS_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -std=c99

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

# Bağımlılık Bilgisi Üretme
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

# Bağlayıcı Bayrakları (Linker Flags)
# Not: Linker script dosyası (STM32F405RGTx_FLASH.ld) dizinde yer almalıdır.
LDSCRIPT = STM32F405RGTx_FLASH.ld
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--gc-sections

# printf float desteğini etkinleştir (Telemetri çıktısı için kritik)
LDFLAGS += -u _printf_float
LDFLAGS += -lm  # Matematik kütüphanesi (C9 düzeltmesi: fabsf, sqrtf, cos, sqrt)

################################################################################
# Derleme Hedefleri (Build Rules)
################################################################################
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

# Obje Listesi Oluştur
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

clean:
	rm -rf $(BUILD_DIR)
  
-include $(wildcard $(BUILD_DIR)/*.d)

.PHONY: all clean
