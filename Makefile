CMSIS_CORE_INCLUDE=cmsis/cmsis_core/CMSIS/Core/Include
CMSIS_WB_INCLUDE=cmsis/cmsis_f1/Include
APP_INCLUDE=inc

INC=-I${CMSIS_CORE_INCLUDE} -I${CMSIS_WB_INCLUDE} -I${APP_INCLUDE}
BUILD_DIR=out

${BUILD_DIR}:
	mkdir -p ${BUILD_DIR}

${BUILD_DIR}/main.o: main.c
	arm-none-eabi-gcc ${INC} -mcpu=cortex-m3 main.c -c -o ${BUILD_DIR}/main.o

${BUILD_DIR}/usb.o: c/usb.c
	arm-none-eabi-gcc ${INC} -mcpu=cortex-m3 c/usb.c -c -o ${BUILD_DIR}/usb.o

OBJECTS=${BUILD_DIR}/main.o ${BUILD_DIR}/usb.o

${BUILD_DIR}/firmware.elf: ${OBJECTS}
	arm-none-eabi-gcc -T link.ld -nostdlib ${OBJECTS} -o ${BUILD_DIR}/firmware.elf

${BUILD_DIR}/firmware.bin: ${BUILD_DIR}/firmware.elf
	arm-none-eabi-objcopy -O binary ${BUILD_DIR}/firmware.elf ${BUILD_DIR}/firmware.bin

firmware.s: main.c
	arm-none-eabi-gcc -mcpu=cortex-m3 main.c -S -o firmware.s

.PHONY: build flash clean assembly

build: out ${BUILD_DIR}/firmware.bin

assembly: firmware.s

flash:
	st-flash --reset write firmware.bin 0x8000000

clean:
	rm -rf ${BUILD_DIR}
