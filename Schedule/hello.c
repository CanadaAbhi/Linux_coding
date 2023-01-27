#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/list.h>


int base_minor = 0;
char *device_name = "mychardev";
int count = 1;
dev_t devicenumber;

static struct class *class = NULL;
static struct device *device = NULL;
static struct cdev *mycdev = NULL;

struct my_task_queue
{
	struct task_struct *task;
	struct list_head list_head;
};

bool write_available = false;

LIST_HEAD(head);


MODULE_LICENSE("GPL");

static int device_open(struct inode *inode, struct file *file)
{
	pr_info("%s\n", __func__);
	return 0;
}

static int device_release(struct inode *inode, struct file *file)
{
	pr_info("%s\n", __func__);
        return 0;
}

static ssize_t device_read(struct file *file, char __user *user_buffer,
                      size_t count, loff_t *offset)
{
	pr_info("%s\n", __func__);

	if (!write_available)
	{
		struct my_task_queue mytask;
		mytask.task = current;
		list_add_tail(&mytask.list_head, &head);
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();
	}
	write_available = false;
	pr_info("%s\n", __func__);
        return 0;
}

static ssize_t device_write(struct file *file, const char __user *user_buffer,
                       size_t count, loff_t *offset)
{
	pr_info("%s\n", __func__);
	write_available = true;
	if(!list_empty(&head))
	{
		struct my_task_queue *sleeping_task;
		sleeping_task = list_entry(head.next, struct my_task_queue, list_head);
		list_del(head.next);
		wake_up_process(sleeping_task->task);

	}
	pr_info("%s\n", __func__);
        return count;
}



struct file_operations device_fops = {
	.read = device_read,
	.write = device_write,
	.open = device_open,
	.release = device_release
};


static int test_hello_init(void)
{
	class = class_create(THIS_MODULE, "myclass");

	if (!alloc_chrdev_region(&devicenumber, base_minor, count, device_name)) {
		printk("Device number registered\n");
		printk("Major number received:%d\n", MAJOR(devicenumber));

		device = device_create(class, NULL, devicenumber, NULL, "mydevice");
		mycdev = cdev_alloc();
		if (mycdev) {
			mycdev->ops = &device_fops;
			mycdev->owner = THIS_MODULE;
			cdev_add(mycdev, devicenumber, count);
		}

	}
	else
		printk("Device number registration Failed\n");

	return 0;
}

static void test_hello_exit(void)
{
	device_destroy(class, devicenumber);
        class_destroy(class);
	cdev_del(mycdev);
	unregister_chrdev_region(devicenumber, count);
}

module_init(test_hello_init);
module_exit(test_hello_exit);
