.PHONY: \
	all debug release clean dump-debug dump-release flash openocd gdb \
	no-dbg-print flow-panic tof-panic accel-gyro-panic \
	estimator-panic stabilizer-panic crtp-rx-panic \
	exti1-panic cload cload-debug \
	cload-flow-panic cload-tof-panic cload-accel-gyro-panic \
	cload-estimator-panic cload-stabilizer-panic cload-crtp-rx-panic \
	cload-exti1-panic \

CARGO_BUILD := cargo +segstk-rust build
CARGO_FLAGS := -Zbuild-std=core,alloc

OBJCOPY := arm-none-eabi-objcopy
OBJCOPY_FLAGS := -O binary --pad-to 0 --remove-section=.bss

DEBUG_ELF := target/thumbv7em-none-eabihf/debug/flight-control
RELEASE_ELF := target/thumbv7em-none-eabihf/release/flight-control

CLOAD := python3 scripts/cfloader.py -w radio://0/80/2M/E7E7E7E7E7 flash

all: release

debug:
	$(CARGO_BUILD) $(CARGO_FLAGS)
	$(OBJCOPY) $(DEBUG_ELF) $(OBJCOPY_FLAGS) debug.bin

release:
	$(CARGO_BUILD) $(CARGO_FLAGS) --release
	$(OBJCOPY) $(RELEASE_ELF) $(OBJCOPY_FLAGS) release.bin

no-dbg-print:
	$(CARGO_BUILD) $(CARGO_FLAGS) --release --features "no_dbg_print"
	$(OBJCOPY) $(RELEASE_ELF) $(OBJCOPY_FLAGS) no-dbg-print.bin

flow-panic:
	$(CARGO_BUILD) $(CARGO_FLAGS) --release --features "flow_panic"
	$(OBJCOPY) $(RELEASE_ELF) $(OBJCOPY_FLAGS) flow-panic.bin

tof-panic:
	$(CARGO_BUILD) $(CARGO_FLAGS) --release --features "tof_panic"
	$(OBJCOPY) $(RELEASE_ELF) $(OBJCOPY_FLAGS) tof-panic.bin

accel-gyro-panic:
	$(CARGO_BUILD) $(CARGO_FLAGS) --release --features "accel_gyro_panic"
	$(OBJCOPY) $(RELEASE_ELF) $(OBJCOPY_FLAGS) accel-gyro-panic.bin

estimator-panic:
	$(CARGO_BUILD) $(CARGO_FLAGS) --release --features "estimator_panic"
	$(OBJCOPY) $(RELEASE_ELF) $(OBJCOPY_FLAGS) estimator-panic.bin

stabilizer-panic:
	$(CARGO_BUILD) $(CARGO_FLAGS) --release --features "stabilizer_panic"
	$(OBJCOPY) $(RELEASE_ELF) $(OBJCOPY_FLAGS) stabilizer-panic.bin

crtp-rx-panic:
	$(CARGO_BUILD) $(CARGO_FLAGS) --release --features "crtp_rx_panic"
	$(OBJCOPY) $(RELEASE_ELF) $(OBJCOPY_FLAGS) crtp-rx-panic.bin

exti1-panic:
	$(CARGO_BUILD) $(CARGO_FLAGS) --release --features "exti1_panic"
	$(OBJCOPY) $(RELEASE_ELF) $(OBJCOPY_FLAGS) exti1-panic.bin

dump-debug:
	arm-none-eabi-objdump -d $(DEBUG_ELF) > dump-debug.asm

dump-release:
	arm-none-eabi-objdump -d $(RELEASE_ELF) > dump-release.asm

cload-debug:
	$(CLOAD) debug.bin stm32-fw

cload:
	$(CLOAD) release.bin stm32-fw

cload-flow-panic:
	$(CLOAD) flow-panic.bin stm32-fw

cload-tof-panic:
	$(CLOAD) tof-panic.bin stm32-fw

cload-accel-gyro-panic:
	$(CLOAD) accel-gyro-panic.bin stm32-fw

cload-estimator-panic:
	$(CLOAD) estimator-panic.bin stm32-fw

cload-stabilizer-panic:
	$(CLOAD) stabilizer-panic.bin stm32-fw

cload-crtp-rx-panic:
	$(CLOAD) crtp-rx-panic.bin stm32-fw

cload-exti1-panic:
	$(CLOAD) exti1-panic.bin stm32-fw

clean:
	cargo clean
	rm -f *.elf *.bin dump-*.asm

flash:
	openocd -d2 -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c init -c targets -c "reset halt" \
                 -c "flash write_image erase release.bin 0x8004000 bin" \
                 -c "verify_image release.bin 0x8004000 bin" -c "reset run" -c shutdown

openocd:
	openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg

gdb:
	arm-none-eabi-gdb -q $(RELEASE_ELF)
