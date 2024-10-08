/*
 * NES, SNES, N64, PSX, Gamecube gamepad driver for Raspberry Pi
 *
 *  Copyright (c) 2012	Markus Hiienkari
 *
 *  Based on the gamecon driver by Vojtech Pavlik
 *  Nes Fourscore support added by Christian Isaksson
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/of_device.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/version.h>
#include <asm/io.h>

MODULE_AUTHOR("Markus Hiienkari");
MODULE_DESCRIPTION("NES, SNES, N64, PSX, GC gamepad driver");
MODULE_LICENSE("GPL");

#define GC_MAX_DEVICES		6

#define BCM2708_PERI_BASE gc_bcm2708_peri_base

#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */

#define GPIO_SET *(gpio+7)
#define GPIO_CLR *(gpio+10)

#define GPIO_STATUS (*(gpio+13))

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,15,0)
#define HAVE_TIMER_SETUP
#endif

static volatile unsigned *gpio;

/* BCM board peripherals address base */
static u32 gc_bcm2708_peri_base;

/**
 * gc_bcm_peri_base_probe - Find the peripherals address base for
 * the running Raspberry Pi model. It needs a kernel with runtime Device-Tree
 * overlay support.
 *
 * Based on the userland 'bcm_host' library code from
 * https://github.com/raspberrypi/userland/blob/2549c149d8aa7f18ff201a1c0429cb26f9e2535a/host_applications/linux/libs/bcm_host/bcm_host.c#L150
 *
 * Reference: https://www.raspberrypi.org/documentation/hardware/raspberrypi/peripheral_addresses.md
 *
 * If any error occurs reading the device tree nodes/properties, then return 0.
 */
static u32 __init gc_bcm_peri_base_probe(void) {

	char *path = "/soc";
	struct device_node *dt_node;
	u32 base_address = 1;

	dt_node = of_find_node_by_path(path);
	if (!dt_node) {
		pr_err("failed to find device-tree node: %s\n", path);
		return 0;
	}

	if (of_property_read_u32_index(dt_node, "ranges", 1, &base_address)) {
		pr_err("failed to read range index 1\n");
		return 0;
	}

	if (base_address == 0) {
		if (of_property_read_u32_index(dt_node, "ranges", 2, &base_address)) {
			pr_err("failed to read range index 2\n");
			return 0;
		}
	}

	return base_address == 1 ? 0x02000000 : base_address;
}

void delayMicrosecondsHard (unsigned int howLong)
{
    ktime_t start_time, end_time;
    start_time = end_time = ktime_get();
    while (ktime_us_delta(end_time, start_time) < howLong)
         end_time = ktime_get();
}

struct gc_config {
	int args[GC_MAX_DEVICES];
	unsigned int nargs;
};

static struct gc_config gc_cfg __initdata;

module_param_array_named(map, gc_cfg.args, int, &(gc_cfg.nargs), 0);
MODULE_PARM_DESC(map, "Describes the set of pad connections (<GPIO0>,<GPIO1>,<GPIO4>,<GPIO7>,<GPIO2>,<GPIO3>)");

enum gc_type {
	GC_NONE = 0,
	GC_SNES,
	GC_NES,
	GC_GCUBE,
	GC_NESFOURSCORE,
	GC_MULTI2,
	GC_N64,
	GC_PSX,
	GC_DDR,
	GC_SNESMOUSE,
	GC_MAX
};

#define GC_REFRESH_TIME	HZ/100

struct gc_pad {
	struct input_dev *dev;
	enum gc_type type;
	char phys[32];

	struct input_dev *dev2;
	char phys2[32];
	unsigned char player_mode;
};

struct gc_nin_gpio {
	unsigned pad_id;
	unsigned cmd_setinputs;
	unsigned cmd_setoutputs;
	unsigned valid_bits;
	unsigned request;
	unsigned request_len;
	unsigned response_len;
	unsigned response_bufsize;
};

struct gc {
	struct gc_pad pads[GC_MAX_DEVICES];
	struct timer_list timer;
	int pad_count[GC_MAX];
	int used;
	struct mutex mutex;
};

struct gc_subdev {
	unsigned int idx;
};

static struct gc *gc_base;

/* GPIO pins 0, 1, 4, 7, 2, 3 */
enum pad_gpios {
	PAD1_GPIO = 0,
	PAD2_GPIO = 1,
	PAD3_GPIO = 4,
	PAD4_GPIO = 7,
	PAD5_GPIO = 2,
	PAD6_GPIO = 3
};

/* GPIO pins 10, 11 */
enum common_gpios {
	NES_CLOCK_GPIO = 10,
	NES_LATCH_GPIO = 11,
	PSX_COMMAND_GPIO = 14,
	PSX_SELECT_GPIO = 15,
	PSX_CLOCK_GPIO = 18
};

static const int gc_gpio_ids[] = { PAD1_GPIO, PAD2_GPIO, PAD3_GPIO, PAD4_GPIO, PAD5_GPIO, PAD6_GPIO };
static const unsigned long gc_status_bit[] = { 	(1<<PAD1_GPIO),
												(1<<PAD2_GPIO),
												(1<<PAD3_GPIO),
												(1<<PAD4_GPIO),
												(1<<PAD5_GPIO),
												(1<<PAD6_GPIO) };

static const char *gc_names[] = {
	NULL, "SNES pad", "NES pad", "Gamecube controller", "NES pad (Four Score)",
	"Multisystem 2-button joystick", "N64 controller", "PSX controller",
	"PSX DDR controller", "SNES mouse"
};

/*
 * N64 support.
 */

static const unsigned char gc_n64_bytes[] = { 0, 1, 13, 15, 14, 12, 10, 11, 2, 3 };
static const short gc_n64_btn[] = {
	BTN_A, BTN_B, BTN_C, BTN_X, BTN_Y, BTN_Z,
	BTN_TL, BTN_TR, BTN_TRIGGER, BTN_START
};

#define GC_N64_REQUEST_LENGTH   8               /* transmit request sequence is 8 bits long (without stop bit) */
#define GC_N64_REQUEST          0x80U		 	/* the request data command */

#define GC_N64_LENGTH			33				/* N64 response length, including stop bit */

/* buffer for samples read from pad */
#define GC_N64_BUFSIZE		100*GC_N64_LENGTH

struct gc_nin_gpio n64_prop = { GC_N64,
								0,
								0,
								0,
								GC_N64_REQUEST,
								GC_N64_REQUEST_LENGTH,
								GC_N64_LENGTH,
								GC_N64_BUFSIZE };

/* Send encoded command */
static inline void gc_n64_send_command(struct gc_nin_gpio *ningpio)
{
	int i;
	
	/* set correct GPIOs to outputs */
	*gpio &= ~ningpio->cmd_setinputs;
	*gpio |= ningpio->cmd_setoutputs;
    
/* if we keep this ratio of 1:3 and 3:1, then widening out our period 
   by scaling the delays seems to help when one of our delays waits an extra 1-2us 
 
   if our small delay ends up being close to as long as our long delay, it's a problem
   if we try to delay 1us, adding an extra 2us really confuses the protocol because the bit becomes 3us:3us
 
   if we widen it out to 2us:6us, adding an extra 2us is still 4us:6us and the controller accepts this
 */
#define N64_DELAY_SCALE 3 /* tested up to 10 (official N64 controller) */
	
	/* transmit a data request to pads */
	for (i = 0; i < ningpio->request_len; i++) {
		if ((unsigned)((ningpio->request >> i) & 1) == 0) {
			GPIO_CLR = ningpio->valid_bits;
			delayMicrosecondsHard(3*N64_DELAY_SCALE);
			GPIO_SET = ningpio->valid_bits;
			delayMicrosecondsHard(1*N64_DELAY_SCALE);
		} else {
			GPIO_CLR = ningpio->valid_bits;
			delayMicrosecondsHard(1*N64_DELAY_SCALE);
			GPIO_SET = ningpio->valid_bits;
			delayMicrosecondsHard(3*N64_DELAY_SCALE);
		}
	}
	
	/* send stop bit (let pull-up handle the last 2us)*/
	GPIO_CLR = ningpio->valid_bits;
	delayMicrosecondsHard(1);
	GPIO_SET = ningpio->valid_bits;
	
	/* set the GPIOs back to inputs */
	*gpio &= ~ningpio->cmd_setinputs;
}

/*
 * gc_n64_read_packet() reads N64 or Gamecube packet.
 * Each pad uses one bit per byte. So all pads connected to this port
 * are read in parallel.
 */

static void gc_n64_read_packet(struct gc *gc, struct gc_nin_gpio *ningpio, unsigned long *data)
{
	int i,j,k;
	unsigned prev, mindiff=1000, maxdiff=0;
	unsigned long flags;
	static unsigned long samplebuf[6500]; // =max(GC_N64_BUFSIZE, GC_GCUBE_BUFSIZE)
	
	/* disable interrupts */
	local_irq_save(flags);

	gc_n64_send_command(ningpio);
	
	/* start sampling data */
	for (i = 0; i < ningpio->response_bufsize; i++)
		samplebuf[i] = GPIO_STATUS & ningpio->valid_bits;
	
	/* enable interrupts when done */
	local_irq_restore(flags);
	
	memset(data, 0x00, ningpio->response_len);

	/* extract correct bit sequence (for each pad) from sampled data */
	for (k = 0; k < GC_MAX_DEVICES; k++) {
		if (gc->pads[k].type != ningpio->pad_id)
			continue;

		/* locate first falling edge */
		for (i = 0; i < ningpio->response_bufsize; i++) {
			if ((samplebuf[i] & gc_status_bit[k]) == 0)
				break;
		}
		
		prev = i;
		j = 0;
		
		while (j < ningpio->response_len-1 && i < ningpio->response_bufsize-1) {
			i++;
			/* detect consecutive falling edges */
			if ((samplebuf[i-1] & gc_status_bit[k]) != 0 && (samplebuf[i] & gc_status_bit[k]) == 0) {
				/* update min&max diffs */
				if (i-prev > maxdiff)
					maxdiff = i - prev;
				if (i-prev < mindiff)
					mindiff = i - prev;

				/* data is taken between 2 falling edges */
				data[j] |= samplebuf[prev+((i-prev)/2)] & gc_status_bit[k];
				j++;
				prev = i;
			}
		}
		
		/* ignore the real stop-bit as it seems to be 0 at times. Invalidate
		 * the read manually instead, if either of the following is true:
		 * 		1. Less than response_len-1 bits read detected from samplebuf
		 * 		2. Variation in falling edge intervals is too high */
		if ((j == ningpio->response_len-1) && (maxdiff < 2*mindiff))
			data[ningpio->response_len-1] |= gc_status_bit[k];
	}
}

static void gc_n64_process_packet(struct gc *gc)
{
	unsigned long data[GC_N64_LENGTH];
	struct input_dev *dev;
	int i, j;
	unsigned long s;
	signed char x, y;

	gc_n64_read_packet(gc, &n64_prop, data);

	for (i = 0; i < GC_MAX_DEVICES; i++) {

		if (gc->pads[i].type != GC_N64)
			continue;

		dev = gc->pads[i].dev;
		s = gc_status_bit[i];

        /* ensure that the response is validthere are 2 types of invalid response:
			1)  one where the reset or reserved bit is set or the stop bit isn't set
				Reset is '1' when L+R+Start are all pressed.  When Reset is '1', L and R are both reported as '1' but Start is reported as '0'.
				Reserved/Unknown, always '0' for normal gamepads
			2)	if we accidentally sent a status command [0x00] (due to bad timing sending the command),
				then the controller will reply with 050000, 050002, 050003 or 050001                         
				bit 5 and 7 would be on, all others would be off, and 30 and 31 would be don't-care (may be on or off)
		*/
//#define PRUNE_N64_STATUS_RESPONSE
        if ((s & ~(data[8] | data[9] | ~data[32])
#ifdef PRUNE_N64_STATUS_RESPONSE
               & ~(data[5] & data[7] &
                   ~(data[0] | data[1] | data[2] | data[3] | data[4] | data[6] | data[8] | data[9] | data[10] | data[11] | data[12] | data[13] | data[14] | data[15] | data[16] | data[17] | data[18] | data[18]
                             | data[19] | data[20] | data[21] | data[22] | data[23] | data[24] | data[25] | data[26] | data[27] | data[28] | data[29])
                    )
#endif
             )
            )
        {
			x = y = 0;

			for (j = 0; j < 8; j++) {
				if (data[23 - j] & s)
					x |= 1 << j;
				if (data[31 - j] & s)
					y |= 1 << j;
			}

			input_report_abs(dev, ABS_X,  x);
			input_report_abs(dev, ABS_Y, -y);

			input_report_abs(dev, ABS_HAT0X,
					 !(s & data[6]) - !(s & data[7]));
			input_report_abs(dev, ABS_HAT0Y,
					 !(s & data[4]) - !(s & data[5]));

			for (j = 0; j < 10; j++)
				input_report_key(dev, gc_n64_btn[j],
						 s & data[gc_n64_bytes[j]]);

			input_sync(dev);
		}
	}
}

#if 0
static int gc_n64_play_effect(struct input_dev *dev, void *data,
				  struct ff_effect *effect)
{
	return 0;
}

static int __init gc_n64_init_ff(struct input_dev *dev, int i)
{
	struct gc_subdev *sdev;
	int err;

	sdev = kmalloc(sizeof(*sdev), GFP_KERNEL);
	if (!sdev)
		return -ENOMEM;

	sdev->idx = i;

	input_set_capability(dev, EV_FF, FF_RUMBLE);

	err = input_ff_create_memless(dev, sdev, gc_n64_play_effect);
	if (err) {
		kfree(sdev);
		return err;
	}

	return 0;
}
#endif


/*
 * Gamecube support.
 */

static const unsigned char gc_gcube_bytes[] = { 7, 6, 5, 4, 11, 9, 10, 3 };
static const short gc_gcube_btn[] = {
	BTN_A, BTN_B, BTN_X, BTN_Y, BTN_Z,
	BTN_TL, BTN_TR, BTN_START
};

#define GC_GCUBE_REQUEST_LENGTH   24            /* transmit request sequence is 24 bits long (without stop bit) */
#define GC_GCUBE_REQUEST          0x40c002U 	/* the request data command */

#define GC_GCUBE_LENGTH			  65			/* Gamecube response length, including stop bit */

/* buffer for samples read from pad */
#define GC_GCUBE_BUFSIZE		100*GC_GCUBE_LENGTH

struct gc_nin_gpio gcube_prop = { GC_GCUBE,
								0,
								0,
								0,
								GC_GCUBE_REQUEST,
								GC_GCUBE_REQUEST_LENGTH,
								GC_GCUBE_LENGTH,
								GC_GCUBE_BUFSIZE };

static void gc_gcube_process_packet(struct gc *gc)
{
	unsigned long data[GC_GCUBE_LENGTH];
	struct input_dev *dev;
	int i, j;
	unsigned long s;
	unsigned char x, y, x2, y2, y3, y4;

	gc_n64_read_packet(gc, &gcube_prop, data);

	for (i = 0; i < GC_MAX_DEVICES; i++) {

		if (gc->pads[i].type != GC_GCUBE)
			continue;

		dev = gc->pads[i].dev;
		s = gc_status_bit[i];

		/* ensure that the response is valid */
		if (s & ~(data[0] | data[1] | ~data[8] | ~data[64])) {

			x = y = x2 = y2 = y3 = y4 = 0;

			for (j = 0; j < 8; j++) {
				if (data[23 - j] & s)
					x |= 1 << j;
				if (data[31 - j] & s)
					y |= 1 << j;
				if (data[39 - j] & s)
					x2 |= 1 << j;
				if (data[47 - j] & s)
					y2 |= 1 << j;
				if (data[55 - j] & s)
					y3 |= 1 << j;
				if (data[63 - j] & s)
					y4 |= 1 << j;					
			}

			input_report_abs(dev, ABS_X, x);
			input_report_abs(dev, ABS_Y, 0xff - y);
			input_report_abs(dev, ABS_RX, x2);
			input_report_abs(dev, ABS_RY, 0xff - y2);
			input_report_abs(dev, ABS_GAS, y3);
			input_report_abs(dev, ABS_BRAKE, y4);			
			
			input_report_abs(dev, ABS_HAT0X,
					 !(s & data[15]) - !(s & data[14]));
			input_report_abs(dev, ABS_HAT0Y,
					 !(s & data[12]) - !(s & data[13]));

			for (j = 0; j < 8; j++)
				input_report_key(dev, gc_gcube_btn[j],
						 s & data[gc_gcube_bytes[j]]);

			input_sync(dev);
		}
	}
}


/*
 * NES/SNES support.
 */

#define GC_NES_DELAY		6	/* Delay between bits - 6us */
#define GC_NES_LENGTH		8	/* The NES pads use 8 bits of data */
#define GC_SNES_LENGTH		12	/* The SNES true length is 16, but the
					   last 4 bits are unused */
#define GC_SNESMOUSE_LENGTH	32	/* The SNES mouse uses 32 bits, the first
					   16 bits are equivalent to a gamepad */
#define GC_NESFOURSCORE_LENGTH	24 /* The NES Four Score adapter uses 24
					   bits of data */

#define GC_NES_CLOCK	(1<<NES_CLOCK_GPIO)
#define GC_NES_LATCH	(1<<NES_LATCH_GPIO)

static const unsigned char gc_nes_bytes[] = { 0, 1, 2, 3 };
static const unsigned char gc_snes_bytes[] = { 8, 0, 2, 3, 9, 1, 10, 11 };
static const short gc_snes_btn[] = {
	BTN_A, BTN_B, BTN_SELECT, BTN_START, BTN_X, BTN_Y, BTN_TL, BTN_TR
};

/*
 * gc_nes_read_packet() reads a NES/SNES packet.
 * Each pad uses one bit per byte. So all pads connected to
 * this port are read in parallel.
 */

static void gc_nes_read_packet(struct gc *gc, int length, unsigned long *data)
{
	int i;

	GPIO_SET = GC_NES_CLOCK | GC_NES_LATCH;
	udelay(GC_NES_DELAY * 2);
	GPIO_CLR = GC_NES_LATCH;

	for (i = 0; i < length; i++) {
		udelay(GC_NES_DELAY);
		GPIO_CLR = GC_NES_CLOCK;
		data[i] = ~GPIO_STATUS;
		udelay(GC_NES_DELAY);
		GPIO_SET = GC_NES_CLOCK;
	}
}

static void gc_nes_process_packet(struct gc *gc)
{
	unsigned long data[GC_SNESMOUSE_LENGTH];
	struct gc_pad *pad;
	struct input_dev *dev, *dev2;
	int i, j, len;
	unsigned long s;
	unsigned char fs_connected;
	char x_rel, y_rel;

	len = gc->pad_count[GC_SNESMOUSE] ? GC_SNESMOUSE_LENGTH :
			(gc->pad_count[GC_NESFOURSCORE] ? GC_NESFOURSCORE_LENGTH :
			(gc->pad_count[GC_SNES] ? GC_SNES_LENGTH : GC_NES_LENGTH));

	gc_nes_read_packet(gc, len, data);

	for (i = 0; i < GC_MAX_DEVICES; i++) {

		pad = &gc->pads[i];
		dev = pad->dev;
		s = gc_status_bit[i];

		switch (pad->type) {

		case GC_NES:

			input_report_abs(dev, ABS_X, !(s & data[6]) - !(s & data[7]));
			input_report_abs(dev, ABS_Y, !(s & data[4]) - !(s & data[5]));

			for (j = 0; j < 4; j++)
				input_report_key(dev, gc_snes_btn[j],
						 s & data[gc_nes_bytes[j]]);
			input_sync(dev);
			break;

		case GC_SNES:

			input_report_abs(dev, ABS_X, !(s & data[6]) - !(s & data[7]));
			input_report_abs(dev, ABS_Y, !(s & data[4]) - !(s & data[5]));

			for (j = 0; j < 8; j++)
				input_report_key(dev, gc_snes_btn[j],
						 s & data[gc_snes_bytes[j]]);
			input_sync(dev);
			break;

		case GC_SNESMOUSE:
			/*
			 * The 4 unused bits from SNES controllers appear
			 * to be ID bits so use them to make sure we are
			 * dealing with a mouse.
			 * gamepad is connected. This is important since
			 * my SNES gamepad sends 1's for bits 16-31, which
			 * cause the mouse pointer to quickly move to the
			 * upper left corner of the screen.
			 */
			if (!(s & data[12]) && !(s & data[13]) &&
				!(s & data[14]) && (s & data[15])) {
				input_report_key(dev, BTN_LEFT, s & data[9]);
				input_report_key(dev, BTN_RIGHT, s & data[8]);

				x_rel = y_rel = 0;
				for (j = 0; j < 7; j++) {
					x_rel <<= 1;
					if (data[25 + j] & s)
						x_rel |= 1;

					y_rel <<= 1;
					if (data[17 + j] & s)
						y_rel |= 1;
				}

				if (x_rel) {
					if (data[24] & s)
						x_rel = -x_rel;
					input_report_rel(dev, REL_X, x_rel);
				}

				if (y_rel) {
					if (data[16] & s)
						y_rel = -y_rel;
					input_report_rel(dev, REL_Y, y_rel);
				}

				input_sync(dev);
			}
			break;
				
		case GC_NESFOURSCORE:
			/*
			 * The NES Four Score uses a 24 bit protocol in 4-player mode
			 * 1st 8 bits: controller 1    /    controller 2 (as normal)
			 * 2nd 8 bits: controller 3    /    controller 4 (new ports)
			 * 3rd 8 bits: 0,0,0,1,0,0,0,0 / 0,0,1,0,0,0,0,0 (ID codes)
			 *
			 * The last 8 bits are used to determine if a Four Score
			 * adapter is connected and if the switch is positioned in
			 * 4 player mode.
			 */
			
			dev2 = pad->dev2;
				
			/* Report first byte (first NES pad). */
			input_report_abs(dev, ABS_X, !(s & data[6]) - !(s & data[7]));
			input_report_abs(dev, ABS_Y, !(s & data[4]) - !(s & data[5]));
			
			for (j = 0; j < 4; j++)
				input_report_key(dev, gc_snes_btn[j], s & data[gc_nes_bytes[j]]);
			input_sync(dev);
			
			/* Determine if a NES Four Score ID code is available in the 3rd byte. */
			fs_connected = ( !(s & data[16]) &&	!(s & data[17]) && !(s & data[18]) &&
							  (s & data[19]) &&	!(s & data[20]) && !(s & data[21]) &&
							 !(s & data[22]) &&	!(s & data[23]) ) ||
								( !(s & data[16]) && !(s & data[17]) &&  (s & data[18]) &&
								  !(s & data[19]) && !(s & data[20]) && !(s & data[21]) &&
								  !(s & data[22]) && !(s & data[23]) );
			
			/* Check if the NES Four Score is connected and the toggle switch is set to 4-player mdoe. */
			if(fs_connected) {
				if(pad->player_mode == 2)
					pad->player_mode = 4;
				
				/* Report second byte (second NES pad). */
				input_report_abs(dev2, ABS_X, !(s & data[14]) - !(s & data[15]));
				input_report_abs(dev2, ABS_Y, !(s & data[12]) - !(s & data[13]));
				
				for (j = 0; j < 4; j++) {
					input_report_key(dev2, gc_snes_btn[j], s & data[gc_nes_bytes[j] + 8]);
				}
				input_sync(dev2);
				
			} else if(pad->player_mode == 4) {
				/* Either the toggle switch on the NES Four Score is set to 2-player mode or it is not connected.  */
				pad->player_mode = 2;
				
				/* Clear second NES pad. */
				input_report_abs(dev2, ABS_X, 0);
				input_report_abs(dev2, ABS_Y, 0);
				
				for (j = 0; j < 4; j++) {
					input_report_key(dev2, gc_snes_btn[j], 0);
				}
				input_sync(dev2);
			}
			break;
			
		default:
			break;
		}
	}
}

/*
 * PSX support
 *
 * See documentation at:
 *	http://www.geocities.co.jp/Playtown/2004/psx/ps_eng.txt	
 *	http://www.gamesx.com/controldata/psxcont/psxcont.htm
 *
 */

#define GC_PSX_DELAY	3		/* clock phase length in us. Valid clkfreq is 100kHz...500kHz. 2*udelay(3) results to ~250kHz on RPi1. */
#define GC_PSX_DELAY2	25		/* delay between bytes. */
#define GC_PSX_LENGTH	8		/* talk to the controller in bits */
#define GC_PSX_BYTES	6		/* the maximum number of bytes to read off the controller */

#define GC_PSX_MOUSE	1		/* Mouse */
#define GC_PSX_NEGCON	2		/* NegCon */
#define GC_PSX_NORMAL	4		/* Digital / Analog or Rumble in Digital mode  */
#define GC_PSX_ANALOG	5		/* Analog in Analog mode / Rumble in Green mode */
#define GC_PSX_RUMBLE	7		/* Rumble in Red mode */

#define GC_PSX_CLOCK	(1<<PSX_CLOCK_GPIO)
#define GC_PSX_COMMAND	(1<<PSX_COMMAND_GPIO)
#define GC_PSX_SELECT	(1<<PSX_SELECT_GPIO)

#define GC_PSX_ID(x)	((x) >> 4)	/* High nibble is device type */
#define GC_PSX_LEN(x)	(((x) & 0xf) << 1)	/* Low nibble is length in bytes/2 */

static const short gc_psx_abs[] = {
	ABS_HAT0X, ABS_HAT0Y, ABS_RX, ABS_RY, ABS_X, ABS_Y
};
static const short gc_psx_btn[] = {
	BTN_TL2, BTN_TR2, BTN_TL, BTN_TR, BTN_X, BTN_A, BTN_B, BTN_Y,
	BTN_SELECT, BTN_THUMBL, BTN_THUMBR, BTN_START
};
static const short gc_psx_ddr_btn[] = { BTN_0, BTN_1, BTN_2, BTN_3 };

/*
 * gc_psx_command() writes 8bit command and reads 8bit data from
 * the psx pad.
 */

static void gc_psx_command(struct gc *gc, int b, unsigned char *data)
{
	int i, j;
	unsigned long read;

	memset(data, 0, GC_MAX_DEVICES);

	for (i = 0; i < GC_PSX_LENGTH; i++, b >>= 1) {
		
		GPIO_CLR = GC_PSX_CLOCK;

		if (b & 1)
			GPIO_SET = GC_PSX_COMMAND;
		else
			GPIO_CLR = GC_PSX_COMMAND;

		udelay(GC_PSX_DELAY);
		GPIO_SET = GC_PSX_CLOCK;

		read = GPIO_STATUS;

		for (j = 0; j < GC_MAX_DEVICES; j++) {
			struct gc_pad *pad = &gc->pads[j];

			if (pad->type == GC_PSX || pad->type == GC_DDR)
				data[j] |= (read & gc_status_bit[j]) ? (1 << i) : 0;
		}

		udelay(GC_PSX_DELAY);
	}
	
	udelay(GC_PSX_DELAY2);
}

/*
 * gc_psx_read_packet() reads a whole psx packet and returns
 * device identifier code.
 */

static void gc_psx_read_packet(struct gc *gc,
				   unsigned char data[GC_MAX_DEVICES][GC_PSX_BYTES],
				   unsigned char id[GC_MAX_DEVICES])
{
	int i, j, max_len = 0;
	unsigned long flags;
	unsigned char data2[GC_MAX_DEVICES];

	local_irq_save(flags);

	/* Select pad */
	GPIO_SET = GC_PSX_CLOCK | GC_PSX_SELECT;
	
	/* Deselect, begin command */
	GPIO_CLR = GC_PSX_SELECT;
	udelay(GC_PSX_DELAY2);

	gc_psx_command(gc, 0x01, data2);	/* Access pad */
	gc_psx_command(gc, 0x42, id);		/* Get device ids */
	gc_psx_command(gc, 0, data2);		/* Dump status */

	/* Find the longest pad */
	for (i = 0; i < GC_MAX_DEVICES; i++) {
		struct gc_pad *pad = &gc->pads[i];

		if ((pad->type == GC_PSX || pad->type == GC_DDR) &&
			GC_PSX_LEN(id[i]) > max_len &&
			GC_PSX_LEN(id[i]) <= GC_PSX_BYTES) {
			max_len = GC_PSX_LEN(id[i]);
		}
	}

	/* Read in all the data */
	for (i = 0; i < max_len; i++) {
		gc_psx_command(gc, 0, data2);
		for (j = 0; j < GC_MAX_DEVICES; j++)
			data[j][i] = data2[j];
	}

	local_irq_restore(flags);

	GPIO_SET = GC_PSX_CLOCK | GC_PSX_SELECT;

	/* Set id's to the real value */
	for (i = 0; i < GC_MAX_DEVICES; i++)
		id[i] = GC_PSX_ID(id[i]);
}

static void gc_psx_report_one(struct gc_pad *pad, unsigned char psx_type,
				  unsigned char *data)
{
	struct input_dev *dev = pad->dev;
	int i;

	switch (psx_type) {

	case GC_PSX_RUMBLE:

		input_report_key(dev, BTN_THUMBL, ~data[0] & 0x02);
		input_report_key(dev, BTN_THUMBR, ~data[0] & 0x04);

	case GC_PSX_NEGCON:
	case GC_PSX_ANALOG:

		if (pad->type == GC_DDR) {
			for (i = 0; i < 4; i++)
				input_report_key(dev, gc_psx_ddr_btn[i],
						 ~data[0] & (0x10 << i));
		} else {
			for (i = 0; i < 4; i++)
				input_report_abs(dev, gc_psx_abs[i + 2],
						 data[i + 2]);

			input_report_abs(dev, ABS_HAT0X,
				!(data[0] & 0x20) - !(data[0] & 0x80));
			input_report_abs(dev, ABS_HAT0Y,
				!(data[0] & 0x40) - !(data[0] & 0x10));
		}

		for (i = 0; i < 8; i++)
			input_report_key(dev, gc_psx_btn[i], ~data[1] & (1 << i));

		input_report_key(dev, BTN_START,  ~data[0] & 0x08);
		input_report_key(dev, BTN_SELECT, ~data[0] & 0x01);

		input_sync(dev);

		break;

	case GC_PSX_NORMAL:

		if (pad->type == GC_DDR) {
			for (i = 0; i < 4; i++)
				input_report_key(dev, gc_psx_ddr_btn[i],
						 ~data[0] & (0x10 << i));
		} else {
			input_report_abs(dev, ABS_HAT0X,
				!(data[0] & 0x20) - !(data[0] & 0x80));
			input_report_abs(dev, ABS_HAT0Y,
				!(data[0] & 0x40) - !(data[0] & 0x10));

			/*
			 * For some reason if the extra axes are left unset
			 * they drift.
			 */
			for (i = 0; i < 4; i++)
				input_report_abs(dev, gc_psx_abs[i + 2], 128);
			 /* This needs to be debugged properly,
			 * maybe fuzz processing needs to be done
			 * in input_sync()
			 *				 --vojtech
			 */
		}

		for (i = 0; i < 8; i++)
			input_report_key(dev, gc_psx_btn[i], ~data[1] & (1 << i));

		input_report_key(dev, BTN_START,  ~data[0] & 0x08);
		input_report_key(dev, BTN_SELECT, ~data[0] & 0x01);

		input_sync(dev);

		break;

	default: /* not a pad, ignore */
		break;
	}
}

static void gc_psx_process_packet(struct gc *gc)
{
	unsigned char data[GC_MAX_DEVICES][GC_PSX_BYTES];
	unsigned char id[GC_MAX_DEVICES];
	struct gc_pad *pad;
	int i;

	gc_psx_read_packet(gc, data, id);

	for (i = 0; i < GC_MAX_DEVICES; i++) {
		pad = &gc->pads[i];
		if (pad->type == GC_PSX || pad->type == GC_DDR)
			gc_psx_report_one(pad, id[i], data[i]);
	}
}

/*
 * gc_timer() initiates reads of console pads data.
 */

#ifdef HAVE_TIMER_SETUP
static void gc_timer(struct timer_list *t)
{
	struct gc *gc = from_timer(gc, t, timer);
#else
static void gc_timer(unsigned long private)
{
	struct gc *gc = (void *) private;
#endif

/*
 * N64 & Gamecube pads
 */

	if (gc->pad_count[GC_N64])
		gc_n64_process_packet(gc);
		
	if (gc->pad_count[GC_GCUBE])
		gc_gcube_process_packet(gc);
		
/*
 * NES and SNES pads or mouse
 */

	if (gc->pad_count[GC_NES] ||
		gc->pad_count[GC_SNES] ||
		gc->pad_count[GC_SNESMOUSE] ||
		gc->pad_count[GC_NESFOURSCORE]) {
		gc_nes_process_packet(gc);
	}
	
/*
 * PSX controllers
 */

	if (gc->pad_count[GC_PSX] || gc->pad_count[GC_DDR])
		gc_psx_process_packet(gc);

	mod_timer(&gc->timer, jiffies + GC_REFRESH_TIME);
}

static int gc_open(struct input_dev *dev)
{
	struct gc *gc = input_get_drvdata(dev);
	int err;

	err = mutex_lock_interruptible(&gc->mutex);
	if (err)
		return err;

	if (!gc->used++)
		mod_timer(&gc->timer, jiffies + GC_REFRESH_TIME);

	mutex_unlock(&gc->mutex);
	return 0;
}

static void gc_close(struct input_dev *dev)
{
	struct gc *gc = input_get_drvdata(dev);

	mutex_lock(&gc->mutex);
	if (!--gc->used) {
		del_timer_sync(&gc->timer);
	}
	mutex_unlock(&gc->mutex);
}

static int __init gc_setup_pad(struct gc *gc, int idx, int pad_type)
{
	struct gc_pad *pad = &gc->pads[idx];
	struct input_dev *input_dev, *input_dev2;
	int i;
	int err;

	if (pad_type < 1 || pad_type >= GC_MAX) {
		pr_err("Pad type %d unknown\n", pad_type);
		return -EINVAL;
	}

	pad->dev = input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("Not enough memory for input device\n");
		return -ENOMEM;
	}

	pad->type = pad_type;

	if(pad_type == GC_NESFOURSCORE)
		snprintf(pad->phys, sizeof(pad->phys),
			"input%d_1", idx);
	else
		snprintf(pad->phys, sizeof(pad->phys),
			"input%d", idx);

	input_dev->name = gc_names[pad_type];
	input_dev->phys = pad->phys;
	input_dev->id.bustype = BUS_PARPORT;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = pad_type;
	input_dev->id.version = 0x0100;

	input_set_drvdata(input_dev, gc);

	input_dev->open = gc_open;
	input_dev->close = gc_close;

	if (pad_type != GC_SNESMOUSE) {
		input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

		for (i = 0; i < 2; i++)
			input_set_abs_params(input_dev, ABS_X + i, -1, 1, 0, 0);
	} else
		input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REL);

	gc->pad_count[pad_type]++;

	switch (pad_type) {

	case GC_N64:
		for (i = 0; i < 10; i++)
			__set_bit(gc_n64_btn[i], input_dev->keybit);

		for (i = 0; i < 2; i++) {
			input_set_abs_params(input_dev, ABS_X + i, -127, 126, 0, 2);
			input_set_abs_params(input_dev, ABS_HAT0X + i, -1, 1, 0, 0);
		}

		/*err = gc_n64_init_ff(input_dev, idx);
		if (err) {
			pr_warning("Failed to initiate rumble for N64 device %d\n", idx);
			goto err_free_dev;
		}*/
		
		/* create bitvectors read/write operations */
		n64_prop.cmd_setinputs |= (7<<(gc_gpio_ids[idx]*3));
		n64_prop.cmd_setoutputs |= (1<<(gc_gpio_ids[idx]*3));
		n64_prop.valid_bits |= gc_status_bit[idx];

		break;
		
	case GC_GCUBE:
		for (i = 0; i < 8; i++)
			__set_bit(gc_gcube_btn[i], input_dev->keybit);

		for (i = 0; i < 2; i++) {
			input_set_abs_params(input_dev, ABS_X + i, 0, 255, 0, 2);
			input_set_abs_params(input_dev, ABS_RX + i, 0, 255, 0, 2);
			input_set_abs_params(input_dev, ABS_GAS + i, 0, 255, 0, 2);
			input_set_abs_params(input_dev, ABS_HAT0X + i, -1, 1, 0, 0);
		}

		/* create bitvectors read/write operations */
		gcube_prop.cmd_setinputs |= (7<<(gc_gpio_ids[idx]*3));
		gcube_prop.cmd_setoutputs |= (1<<(gc_gpio_ids[idx]*3));
		gcube_prop.valid_bits |= gc_status_bit[idx];

		break;
		
	case GC_SNESMOUSE:
		__set_bit(BTN_LEFT, input_dev->keybit);
		__set_bit(BTN_RIGHT, input_dev->keybit);
		__set_bit(REL_X, input_dev->relbit);
		__set_bit(REL_Y, input_dev->relbit);
		break;

	case GC_SNES:
		for (i = 4; i < 8; i++)
			__set_bit(gc_snes_btn[i], input_dev->keybit);
	case GC_NES:
		for (i = 0; i < 4; i++)
			__set_bit(gc_snes_btn[i], input_dev->keybit);
		break;

	case GC_NESFOURSCORE:
		/* Create the extra input_dev generated by the NES Four Score adapter */
		pad->dev2 = input_dev2 = input_allocate_device();
		if (!input_dev2) {
			pr_err("Not enough memory for input device 2\n");
			return -ENOMEM;
		}
		snprintf(pad->phys2, sizeof(pad->phys2), "input%d_2", idx);
		
		input_dev2->name = gc_names[pad_type];
		input_dev2->phys = pad->phys2;
		input_dev2->id.bustype = BUS_PARPORT;
		input_dev2->id.vendor = 0x0001;
		input_dev2->id.product = pad_type;
		input_dev2->id.version = 0x0100;
		
		input_set_drvdata(input_dev2, gc);
		
		input_dev2->open = gc_open;
		input_dev2->close = gc_close;
		
		input_dev2->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
		
		for (i = 0; i < 2; i++)
			input_set_abs_params(input_dev2, ABS_X + i, -1, 1, 0, 0);
		
		pad->player_mode = 2;
		
		gc->pad_count[pad_type]++;
		
		for (i = 0; i < 4; i++) {
			__set_bit(gc_snes_btn[i], input_dev->keybit);
			__set_bit(gc_snes_btn[i], input_dev2->keybit);
		}
		break;

	case GC_PSX:
		for (i = 0; i < 2; i++)
			input_set_abs_params(input_dev,
						 gc_psx_abs[i], -1, 1, 0, 0);
		for (i = 0; i < 4; i++)
			input_set_abs_params(input_dev,
						 gc_psx_abs[i+2], 0, 255, 0, 28);
		for (i = 0; i < 12; i++)
			__set_bit(gc_psx_btn[i], input_dev->keybit);

		break;

	case GC_DDR:
		for (i = 0; i < 4; i++)
			__set_bit(gc_psx_ddr_btn[i], input_dev->keybit);
		for (i = 0; i < 12; i++)
			__set_bit(gc_psx_btn[i], input_dev->keybit);

		break;
	}

	err = input_register_device(pad->dev);
	if (err)
		goto err_free_dev;
		
	if(pad_type == GC_NESFOURSCORE) {
		err = input_register_device(pad->dev2);
		if(err)
			goto err_free_dev2;
	}

	/* set data pin to input */
	*(gpio+(gc_gpio_ids[idx]/10)) &= ~(7<<((gc_gpio_ids[idx]%10)*3));
	
	/* enable pull-up on GPIO4 or higher */
	if (gc_gpio_ids[idx] >= 4) {
		*(gpio+37) = 0x02;
		udelay(10);
		*(gpio+38) = (1 << gc_gpio_ids[idx]);
		udelay(10);
		*(gpio+37) = 0x00;
		*(gpio+38) = 0x00;
	}
		
	pr_info("GPIO%d configured for %s data pin\n", gc_gpio_ids[idx], gc_names[pad_type]);

	return 0;

err_free_dev2:
	input_free_device(pad->dev2);
	pad->dev2 = NULL;
err_free_dev:
	input_free_device(pad->dev);
	pad->dev = NULL;
	return err;
}

static struct gc __init *gc_probe(int *pads, int n_pads)
{
	struct gc *gc;
	int i;
	int count = 0;
	int err;

	gc = kzalloc(sizeof(struct gc), GFP_KERNEL);
	if (!gc) {
		pr_err("Not enough memory\n");
		err = -ENOMEM;
		goto err_out;
	}

	mutex_init(&gc->mutex);
	#ifdef HAVE_TIMER_SETUP
	timer_setup(&gc->timer, gc_timer, 0);
	#else
	setup_timer(&gc->timer, gc_timer, (long) gc);
	#endif

	for (i = 0; i < n_pads && i < GC_MAX_DEVICES; i++) {
		if (!pads[i])
			continue;

		err = gc_setup_pad(gc, i, pads[i]);
		if (err)
			goto err_unreg_devs;

		count++;
	}

	if (count == 0) {
		pr_err("No valid devices specified\n");
		err = -EINVAL;
		goto err_free_gc;
	}

	/* setup common pins for each pad type */
	if (gc->pad_count[GC_NES] ||
		gc->pad_count[GC_SNES] ||
		gc->pad_count[GC_SNESMOUSE] ||
		gc->pad_count[GC_NESFOURSCORE]) {

		/* set clk & latch pins to OUTPUT */
		*(gpio+(NES_CLOCK_GPIO/10)) &= ~(7<<((NES_CLOCK_GPIO%10)*3));
		*(gpio+(NES_CLOCK_GPIO/10)) |= (1<<((NES_CLOCK_GPIO%10)*3));
		*(gpio+(NES_LATCH_GPIO/10)) &= ~(7<<((NES_LATCH_GPIO%10)*3));
		*(gpio+(NES_LATCH_GPIO/10)) |= (1<<((NES_LATCH_GPIO%10)*3));
	}
	if (gc->pad_count[GC_PSX] ||
		gc->pad_count[GC_DDR]) {

		/* set clk, cmd & sel pins to OUTPUT */
		*(gpio+(PSX_CLOCK_GPIO/10)) &= ~(7<<((PSX_CLOCK_GPIO%10)*3));
		*(gpio+(PSX_CLOCK_GPIO/10)) |= (1<<((PSX_CLOCK_GPIO%10)*3));
		*(gpio+(PSX_COMMAND_GPIO/10)) &= ~(7<<((PSX_COMMAND_GPIO%10)*3));
		*(gpio+(PSX_COMMAND_GPIO/10)) |= (1<<((PSX_COMMAND_GPIO%10)*3));
		*(gpio+(PSX_SELECT_GPIO/10)) &= ~(7<<((PSX_SELECT_GPIO%10)*3));
		*(gpio+(PSX_SELECT_GPIO/10)) |= (1<<((PSX_SELECT_GPIO%10)*3));
	}

	return gc;

 err_unreg_devs:
	while (--i >= 0) {
		if (gc->pads[i].dev)
			input_unregister_device(gc->pads[i].dev);
		if (gc->pads[i].dev2)
			input_unregister_device(gc->pads[i].dev2);
	}
 err_free_gc:
	kfree(gc);
 err_out:
	return ERR_PTR(err);
}

static void gc_remove(struct gc *gc)
{
	int i;

	for (i = 0; i < GC_MAX_DEVICES; i++) {
		if (gc->pads[i].dev)
			input_unregister_device(gc->pads[i].dev);
		if (gc->pads[i].dev2)
			input_unregister_device(gc->pads[i].dev2);
	}
	kfree(gc);
}

static int __init gc_init(void)
{
	/* Get the BCM2708 peripheral address */
	gc_bcm2708_peri_base = gc_bcm_peri_base_probe();
	if (!gc_bcm2708_peri_base) {
		pr_err("failed to find peripherals address base via device-tree\n");
		return -ENODEV;
	}

	pr_info("peripherals address base at 0x%08x\n", gc_bcm2708_peri_base);

	/* Set up gpio pointer for direct register access */
	if ((gpio = ioremap(GPIO_BASE, 0xB0)) == NULL) {
		pr_err("io remap failed\n");
		return -EBUSY;
	}

	if (gc_cfg.nargs < 1) {
		pr_err("at least one device must be specified\n");
		return -EINVAL;
	} else {
		gc_base = gc_probe(gc_cfg.args, gc_cfg.nargs);
		if (IS_ERR(gc_base))
			return -ENODEV;
	}

	return 0;
}

static void __exit gc_exit(void)
{
	if (gc_base)
		gc_remove(gc_base);
			
	iounmap(gpio);
}

module_init(gc_init);
module_exit(gc_exit);
