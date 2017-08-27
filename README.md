gamecon_gpio_rpi
==============
gamecon_gpio_rpi is a Linux kernel module which allows interfacing various retro gamepads with Raspberry Pi's GPIO. See [wikipage](https://github.com/RetroPie/RetroPie-Setup/wiki/GPIO-Modules#gamecon_gpio_rpi) for further details.

Building dkms package on RPi
---------------------------------------------------
1. Install kernel headers
2. Copy gamecon_gpio_rpi-\<VERSION> folder to /usr/src/
3. dkms add gamecon_gpio_rpi/\<VERSION>
4. dkms build gamecon_gpio_rpi/\<VERSION>
5. dkms mkdeb gamecon_gpio_rpi/\<VERSION> --source-only
