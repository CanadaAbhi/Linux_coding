#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>

MODULE_LICENSE("GPL");

#define GPIO_BASE               0x3f200000      // GPIO controller 
#define GPIO_SIZE               0xb4

#define GPFSEL0_OFFSET		0x00

uint32_t *mem;

void set_gpio_output(unsigned int gpio)
{
	int register_index = gpio/10;
	int bit_location   = (gpio%10)*3;
	uint32_t mask = 7 << bit_location;
	uint32_t old_value;
	uint32_t new_value;

	pr_info("register index:%d\t bit location:%d\n", register_index, bit_location);
	old_value = ioread32(mem + GPFSEL0_OFFSET + register_index);
	pr_info("old value:0x%02x\n", old_value);
	
	new_value =  (old_value & ~mask) | ((0b001 << bit_location) &mask);
	pr_info("new value:0x%02x\n", new_value);
	iowrite32(new_value, mem + GPFSEL0_OFFSET + register_index);

}

static int led_init(void)
{
    pr_info("%s: In init\n", __func__);
    mem = (uint32_t *)ioremap(GPIO_BASE, GPIO_SIZE);
    pr_info("ioremap returned:%px\n", mem);
    set_gpio_output(14);
    iounmap(mem);
    return 0;
}

static void led_exit(void)
{
    pr_info("%s: In exit\n", __func__);
}

module_init(led_init);
module_exit(led_exit);
