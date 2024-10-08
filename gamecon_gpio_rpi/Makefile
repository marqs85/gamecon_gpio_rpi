obj-m := gamecon_gpio_rpi.o
KVERSION := `uname -r`
CFLAGS_gamecon_gpio_rpi.o := -O3 -Wno-implicit-fallthrough

all:
	$(MAKE) -C /lib/modules/$(KVERSION)/build M=$(PWD) modules

clean:
	$(MAKE) -C /lib/modules/$(KVERSION)/build M=$(PWD) clean
