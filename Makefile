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