IMAGE ?= stm32-build

ifeq ($(OS),Windows_NT)
    USER_FLAG :=
    CUR_DIR := $(CURDIR)
else
    USER_FLAG := --user $(shell id -u):$(shell id -g)
    CUR_DIR := $(PWD)
endif

build:
	docker run --rm -v "$(CUR_DIR):/workspace" -w /workspace $(USER_FLAG) $(IMAGE) make -C firmware all

check-deps:
	python3 scripts/check_deps.py

test:
	docker run --rm -v "$(CURDIR):/workspace" -w /workspace stm32-build gcc -Wall -Wextra -Og \
		-I firmware/Platform/Inc -I firmware/Devices/Inc -I firmware/App/Inc \
		tests/test_devices.c \
		firmware/Devices/Src/mpu6050.c \
		firmware/Devices/Src/bmp180.c \
		firmware/Devices/Src/w25q64.c \
		firmware/Devices/Src/servo.c \
		firmware/Devices/Src/ov7670.c \
		-lm -o tests/runner
	docker run --rm -v "$(CURDIR):/workspace" -w /workspace stm32-build ./tests/runner