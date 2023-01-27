#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>   
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
 
 
char  buffer[] = "Hello Linux" ;
#define MAX_SIZE sizeof(buffer)
int buffer_index;


MODULE_LICENSE("GPL");
 
 
static struct proc_dir_entry *dir_entry;
 
static ssize_t mywrite(struct file *file, const char __user *ubuf,size_t count, loff_t *ppos) 
{
	pr_info("write handler\n");
	return count;
}
 
static ssize_t myread(struct file *file, char __user *user_buffer,size_t read_size, loff_t *offset) 
{
	size_t bytes_read; /*number of bytes actually read*/
	int maxbytes; /* number of bytes from offset to MAX_SIZE */
	int bytes_to_read; /* number of bytes to read */

	pr_info("%s\n", __func__);
	maxbytes = MAX_SIZE - *offset;

	if(maxbytes > read_size)
		bytes_to_read = read_size;
	else
		bytes_to_read = maxbytes;

	if (bytes_to_read == 0) {
		pr_info("trying to read past end of device,"
				"aborting !!\n");
		return 0; /* Causes read() to return EOF */
	}
	pr_info("buffer_index:%d\t offset:%llu\t bytes_to_read:%d\t read_size:%lu\n",
			buffer_index, *offset, bytes_to_read, read_size);
	//copy_to_user returns number of bytes that could not be copied. On success, this will be zero.
	bytes_read = bytes_to_read - copy_to_user(user_buffer, buffer + *offset, bytes_to_read);
	*offset += bytes_read;
	pr_info("\n%s:bytes read=%lu, offset=%d\n", __func__, bytes_read, *(int *)offset);
	return bytes_read;

}

static struct file_operations proc_ops = 
{
	.owner = THIS_MODULE,
	.read = myread,
	.write = mywrite,
};

static int proc_init(void)
{
	dir_entry = proc_mkdir("embedded", NULL);

	proc_create("ldd",0646,dir_entry,&proc_ops);
	pr_info("ldd proc entry created\n");
	return 0;
}

static void proc_cleanup(void)
{
	remove_proc_entry("ldd", dir_entry);
	remove_proc_entry("embedded", NULL);
}

module_init(proc_init);
module_exit(proc_cleanup);
