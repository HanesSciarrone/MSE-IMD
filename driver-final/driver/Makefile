ifneq ($(KERNELRELEASE),)
obj-m := mpu9250_driver.o
else
KDIR := $(HOME)/ISO_II/kernel_linux_raspberrry_pi/linux
all:
	$(MAKE) -C $(KDIR) M=$$PWD
endif
