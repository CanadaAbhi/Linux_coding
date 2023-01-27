#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/kernel.h>

#define DRIVER_NAME "bmp280"
#define DRIVER_CLASS "bmp280Class"

static struct i2c_adapter * tempsensor_i2c_adapter = NULL;
static struct i2c_client * tempsensor_i2c_client = NULL;

/* Meta Information */
MODULE_AUTHOR("AANM");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("temperature sensor driver");
MODULE_SUPPORTED_DEVICE("NONE");

/* Defines for device identification */ 
#define I2C_BUS_AVAILABLE	1		                /* The I2C Bus available on the raspberry */
#define SLAVE_DEVICE_NAME	"TEMP_SENSOR_BMP280"	        /* Device and Driver Name */
#define TEMP_SENSOR_BMP280_SLAVE_ADDRESS	0x76		/* BMP280 I2C address */

static const struct i2c_device_id bmp_id[] = {
		{ SLAVE_DEVICE_NAME, 0 }, 
		{ }
};

static struct i2c_driver bmp_driver = {
	.driver = {
		.name = SLAVE_DEVICE_NAME,
		.owner = THIS_MODULE
	}
};

static struct i2c_board_info bmp_i2c_board_info = {
	I2C_BOARD_INFO(SLAVE_DEVICE_NAME, TEMP_SENSOR_BMP280_SLAVE_ADDRESS)
};

/* Variables for Device and Deviceclass*/
static dev_t myDeviceNr;
static struct class *myClass;
static struct cdev myDevice;

/* Variables for temperature calculation */
s32 digital_T1, digital_T2, digital_T3;

/**
 * @brief Read current temperature from BMP280 sensor
 *
 * @return temperature in degree
 */
s32 temperature_conversion(void) {
	int r1, r2;
	s32 temp_actual;
	s32 smbus_d1, smbus_d2, smbus_d3;

	/* Read Temperature */
	smbus_d1 = i2c_smbus_read_byte_data(bmp280_i2c_client, 0xFA);
	smbus_d2 = i2c_smbus_read_byte_data(bmp280_i2c_client, 0xFB);
	smbus_d3 = i2c_smbus_read_byte_data(bmp280_i2c_client, 0xFC);
	temp_actual = ((smbus_d1<<16) | (smbus_d2<<8) | smbus_d3) >> 4;

	/* Calculate temperature in degree */
	r1 = ((((temp_actual >> 3) - (digital_T1 << 1))) * (digital_T2)) >> 11;

	r2 = (((((temp_actual >> 4) - (digital_T1)) * ((temp_actual >> 4) - (digital_T1))) >> 12) * (digital_T3)) >> 14;
	return ((r1 + r2) *5 +128) >> 8;
}

/**
 * @brief Get data out of buffer
 */
static ssize_t driver_read(struct file *File, char *user_buffer, size_t count, loff_t *offs) {
	int to_copy, byte_count, final_read_value;
	char out_string[20];
	int temperature;

	/* Get amount of bytes to copy */
	to_copy = min(sizeof(out_string), count);

	/* Get temperature */
	temperature = temperature_conversion();
	snprintf(out_string, sizeof(out_string), "%d.%d\n", temperature/100, temperature%100);

	/* Copy Data to user */
	byte_count = copy_to_user(user_buffer, out_string, to_copy);

	/* Calculate delta */
	delta = to_copy - byte_count;

	return final_read_value;
}

/**
 * @brief This function is called, when the device file is opened
 */
static int driver_open(struct inode *deviceFile, struct file *instance) {
	printk("Custom driver Open called  \n");
	return 0;
}

/**
 * @brief This function is called, when the device file is close
 */
static int driver_close(struct inode *deviceFile, struct file *instance) {
	printk("Custom driver close called\n");
	return 0;
}

/* Map the file operations */
static struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = driver_open,
	.release = driver_close,
	.read = driver_read,
};


/**
 * @brief function, which is called after loading module to kernel, do initialization there
 */
static int __init ModuleInit(void) {
	int ret = -1;
	u8 id;
	printk("Module Init called\n");

	/* Allocate Device Nr */
	if ( alloc_chrdev_region(&myDeviceNr, 0, 1, DRIVER_NAME) < 0) {
		printk("Can't allocate device number!\n");
	}
	printk("MyDeviceDriver - Device Nr %d was registered\n", myDeviceNr);

	/* Create Device Class */
	if ((myClass = class_create(THIS_MODULE, DRIVER_CLASS)) == NULL) {
		printk("Device Class can not be created!\n");
		goto ClassError;
	}

	/* Create Device file */
	if (device_create(myClass, NULL, myDeviceNr, NULL, DRIVER_NAME) == NULL) {
		printk("Can not create device file!\n");
		goto FileError;
	}

	/* Initialize Device file */
	cdev_init(&myDevice, &fops);

	/* register device to kernel */
	if (cdev_add(&myDevice, myDeviceNr, 1) == -1) {
		printk("Registering of device to linux kernel failed!\n");
		goto KernelError;
	}

	tempsensor_i2c_adapter = i2c_get_adapter(I2C_BUS_AVAILABLE);

	if(tempsensor_i2c_adapter != NULL) {
		tempsensor_i2c_client = i2c_new_device(tempsensor_i2c_adapter, &bmp_i2c_board_info);
		if(tempsensor_i2c_client != NULL) {
			if(i2c_add_driver(&bmp_driver) != -1) {
				ret = 0;
			}
			else
				printk("Can't add driver...\n");
		}
		i2c_put_adapter(tempsensor_i2c_adapter);
	}
	printk("Temp Sensor Driver added!\n");

	/* Read Chip ID */
	id = i2c_smbus_read_byte_data(tempsensor_i2c_client, 0xD0);
	printk("ID: 0x%x\n", id);

	/* Read Calibration Values */
	digital_T1 = i2c_smbus_read_word_data(tempsensor_i2c_client, 0x88);
	digital_T2 = i2c_smbus_read_word_data(tempsensor_i2c_client, 0x8a);
	digital_T3 = i2c_smbus_read_word_data(tempsensor_i2c_client, 0x8c);

	if(digital_T2 > 32767)
		digital_T2 -= 65536;

	if(digital_T3 > 32767)
		digital_T3 -= 65536;

	/* Initialice the sensor */
	i2c_smbus_write_byte_data(tempsensor_i2c_client, 0xf5, 5<<5);
	i2c_smbus_write_byte_data(tempsensor_i2c_client, 0xf4, ((5<<5) | (5<<2) | (3<<0)));
	return ret;

KernelError:
	device_destroy(myClass, myDeviceNr);
FileError:
	class_destroy(myClass);
ClassError:
	unregister_chrdev(myDeviceNr, DRIVER_NAME);
	return (-1);
}

/**
 * @brief function, which is called when removing module from kernel
 * free alocated resources
 */
static void __exit ModuleExit(void) {
	printk("MyDeviceDriver - Goodbye, Kernel!\n");
	i2c_unregister_device(tempsensor_i2c_client);
	i2c_del_driver(&bmp_driver);
	cdev_del(&myDevice);
    device_destroy(myClass, myDeviceNr);
    class_destroy(myClass);
    unregister_chrdev_region(myDeviceNr, 1);
}

module_init(ModuleInit);
module_exit(ModuleExit);
