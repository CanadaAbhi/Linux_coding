#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ioport.h>

#define MY_BASEPORT 0x0063
#define NUMBER_PORTS 1

MODULE_LICENSE("GPL");

static int mod_init(void)
{
	if (!request_region(MY_BASEPORT, NUMBER_PORTS, "myport")) {
		pr_info("request region failed for myport\n");
	} else {
		pr_info("request region success for myport\n");
		outb(0xa, MY_BASEPORT);
	}
	return 0;
}

static void mod_exit(void)
{
	unsigned int a;
	a = inb(MY_BASEPORT);
	pr_info("Value at %02x is %02x\n", MY_BASEPORT, a);
	release_region(MY_BASEPORT, NUMBER_PORTS);
}

module_init(mod_init);
module_exit(mod_exit);

