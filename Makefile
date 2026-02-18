CMSIS_CORE_INCLUDE=cmsis/cmsis_core/CMSIS/Core/Include
CMSIS_WB_INCLUDE=cmsis/cmsis_f1/Include
APP_INCLUDE=inc

INC=-I${CMSIS_CORE_INCLUDE} -I${CMSIS_WB_INCLUDE} -I${APP_INCLUDE}
BUILD_DIR=out

STARTUP_FILE=cmsis/cmsis_f1/Source/Templates/gcc/startup_stm32f103xb.s
# LINKER_SCRIPT=cmsis/cmsis_f1/Source/Templates/gcc/linker/STM32F103XB_FLASH.ldmsis/cmsis_f1/Source/Templates/gcc/linker/STM32F103XB_FLASH.ld
LINKER_SCRIPT=link.ld
SYSTEM_STM32F1XX_FILE=system.c

CFLAGS=-ggdb -mcpu=cortex-m3 -O0

${BUILD_DIR}:
	mkdir -p ${BUILD_DIR}

${BUILD_DIR}/startup.o: ${STARTUP_FILE}
	arm-none-eabi-gcc -mcpu=cortex-m3 ${STARTUP_FILE} -c -o ${BUILD_DIR}/startup.o

${BUILD_DIR}/system.o: ${SYSTEM_STM32F1XX_FILE}
	arm-none-eabi-gcc ${INC} ${CFLAGS} ${SYSTEM_STM32F1XX_FILE} -c -o ${BUILD_DIR}/system.o

${BUILD_DIR}/main.o: main.c
	arm-none-eabi-gcc ${INC} ${CFLAGS} main.c -c -o ${BUILD_DIR}/main.o

${BUILD_DIR}/usb.o: c/usb.c
	arm-none-eabi-gcc ${INC} ${CFLAGS} c/usb.c -c -o ${BUILD_DIR}/usb.o

${BUILD_DIR}/usart.o: c/usart.c
	arm-none-eabi-gcc ${INC} ${CFLAGS} c/usart.c -c -o ${BUILD_DIR}/usart.o

${BUILD_DIR}/led.o: c/led.c
	arm-none-eabi-gcc ${INC} ${CFLAGS} c/led.c -c -o ${BUILD_DIR}/led.o

${BUILD_DIR}/systick.o: c/systick.c
	arm-none-eabi-gcc ${INC} ${CFLAGS} c/systick.c -c -o ${BUILD_DIR}/systick.o

${BUILD_DIR}/system_clock.o: c/system_clock.c
	arm-none-eabi-gcc ${INC} ${CFLAGS} c/system_clock.c -c -o ${BUILD_DIR}/system_clock.o

OBJECTS=${BUILD_DIR}/system.o ${BUILD_DIR}/startup.o ${BUILD_DIR}/main.o ${BUILD_DIR}/usb.o ${BUILD_DIR}/usart.o ${BUILD_DIR}/led.o ${BUILD_DIR}/systick.o ${BUILD_DIR}/system_clock.o

${BUILD_DIR}/firmware.elf: ${OBJECTS}
	arm-none-eabi-gcc -T ${LINKER_SCRIPT} -mcpu=cortex-m3 -ffreestanding -nostdlib ${OBJECTS} -o ${BUILD_DIR}/firmware.elf

${BUILD_DIR}/firmware.bin: ${BUILD_DIR}/firmware.elf
	arm-none-eabi-objcopy -O binary ${BUILD_DIR}/firmware.elf ${BUILD_DIR}/firmware.bin

.PHONY: build flash clean assembly

build: out ${BUILD_DIR}/firmware.bin

assembly: firmware.s

flash:
	st-flash --reset write ${BUILD_DIR}/firmware.bin 0x8000000

clean:
	rm -rf ${BUILD_DIR}
