#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/delay.h>

#define PCI_INDEX_PORT	0xcf8
#define PCI_DATA_PORT	0xcfc


unsigned short read_pci_config_short(unsigned char bus, unsigned char slot, unsigned char func,
					unsigned char offset)
{
	outl(0x80000000 | (bus<<16) | (slot<<11) | (func<<8) | offset, 0xcf8);
	return inw(0xcfc + (offset&2));
}

unsigned int read_pci_config(unsigned char bus, unsigned char slot, unsigned char func, unsigned char offset)
{
        unsigned int v;
        outl(0x80000000 | (bus<<16) | (slot<<11) | (func<<8) | offset, 0xcf8);
        v = inl(0xcfc);
        return v;
}

void write_pci_config(unsigned char bus, unsigned char slot, unsigned char func, unsigned char offset,
                                    unsigned int val)
{
        outl(0x80000000 | (bus<<16) | (slot<<11) | (func<<8) | offset, 0xcf8);
        outl(val, 0xcfc);
}


int __init  hello_init(void)
{	
	u64 address;
	//02:01.0 Ethernet controller
	unsigned int bar_register = read_pci_config(2, 1, 0, 0x10);
	pr_info("bar register:%02x\n", bar_register);

	if (bar_register & 0x01) {
		pr_info("I/O Mapped I/O\n");
		address = (bar_register&0xfffffffc);
	}
	else {
		pr_info("Memory Mapped I/O\n");
		if (((bar_register >> 1) & 0x03) == 0x00) {
			pr_info("32-BIT Address space\n");
			address = (bar_register & 0xFFFFFFF0U);
		}
		else {
			unsigned long int bar_register1 = read_pci_config(2, 1, 0, 0x14);
			unsigned int size, size_bar_register1, size_bar_register2;
			
			pr_info("64-bit address space\n");
			pr_info("bar register2:%02lx\n", bar_register1);
			address = (bar_register & 0xFFFFFFF0U) +
				((bar_register1 & 0xFFFFFFFFU) << 32) ;
			

			//write all ones
			write_pci_config(2, 1, 0, 0x10, 0xffffffff);
			write_pci_config(2, 1, 0, 0x14, 0xffffffff);

			//read it
			size_bar_register1 = read_pci_config(2, 1, 0, 0x10);
			size_bar_register2 = read_pci_config(2, 1, 0, 0x14);
		
			//restore the original value
			write_pci_config(2, 1, 0, 0x10, bar_register);
			write_pci_config(2, 1, 0, 0x14, bar_register1);


			pr_info("size_bar_register1:%02x\n", size_bar_register1);
			pr_info("size_bar_register2:%02x\n", size_bar_register2);

			//Masking
			pr_info("size_bar_register1:%02x\n", ~size_bar_register1);
			pr_info("size_bar_register2:%02x\n", ~size_bar_register2);

			//clear the lowest 4 bits (prefetchable, address size, type)
			size_bar_register1 &= ~15; 
			pr_info("size_bar_register1:%02x\n", ~size_bar_register1);
			size = ~size_bar_register1 + 1;
			pr_info("size:%02x\n", size);
		}
	}
	pr_info("BAR Address Register:%02llx\n", address);
	return 0;
}

void  __exit hello_exit(void)
{

}

MODULE_LICENSE("GPL");
module_init(hello_init);
module_exit(hello_exit);
