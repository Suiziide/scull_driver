#ifndef __KERNEL__
#  define __KERNEL__
#endif
#ifndef MODULE
#  define MODULE
#endif

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>	
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/semaphore.h>
#include <asm/switch_to.h>
#include <linux/cdev.h>

/* Prototypes - this would normally go in a .h file */
static int dm510_open( struct inode*, struct file* );
static int dm510_release( struct inode*, struct file* );
static ssize_t dm510_read( struct file*, char*, size_t, loff_t* );
static ssize_t dm510_write( struct file*, const char*, size_t, loff_t* );
long dm510_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

#define DEVICE_NAME "dm510_dev" /* Dev name as it appears in /proc/devices */
#define MAJOR_NUMBER 255
#define MIN_MINOR_NUMBER 0
#define MAX_MINOR_NUMBER 1
#define MAX_READERS 2
#define MAX_WRITERS 1
#define BUFFER_SIZE 25
#define DEVICE_COUNT 2

#define DM510_IOC_MAGIC 255
#define DM510_IOCRESET _IO(DM510_IOC_MAGIC, 0)
#define DM510_IOCSBUFFER _IOW(DM510_IOC_MAGIC, 1, int)
#define DM510_IOCSREADERS _IOW(DM510_IOC_MAGIC, 2, int)
#define DM510_IOCTBUFFERSIZE _IO(DM510_IOC_MAGIC, 3)
#define DM510_IOCTMAXREADERS _IO(DM510_IOC_MAGIC, 4)
#define DM510_IOC_MAXNR 4 
/* end of what really should have been in a .h file */

/* file operations struct */
static struct file_operations dm510_fops = {
	.owner   			= THIS_MODULE,
	.read    			= dm510_read,
	.write   			= dm510_write,
	.open    			= dm510_open,
	.release 			= dm510_release,
    .unlocked_ioctl		= dm510_ioctl
};

struct DM510 {
        wait_queue_head_t inq, outq;       /* read and write queues */
        char *buffer, *end;                /* begin of buf, end of buf */
        char *rp, *wp;                     /* own read and write pointers */
        int nreaders, nwriters;            /* number of openings for r/w */
        struct mutex mutex;                /* mutual exclusion semaphore */
        struct cdev cdev;                  /* Char device structure */
};

/* parameters */
static struct DM510 *DM510_devices;
static int DM510_nr_devs = DEVICE_COUNT;	/* num. devices */
int DM510_buffer =  BUFFER_SIZE;		/* buffer size */
int DM510_readers = MAX_READERS;		/* num. of allowed readers */
int DM510_writers = MAX_WRITERS;		/* num. of allowed writers */
dev_t DM510_devno;				/* Our first device number */

static int spacefree(struct DM510 *dev);

/*
 * Set up a cdev entry.
 */
static void dm510_setup_cdev(struct DM510 *dev, int index)
{
	int err, devno = DM510_devno + index;
    
	cdev_init(&dev->cdev, &dm510_fops);
	dev->cdev.owner = THIS_MODULE;
	err = cdev_add (&dev->cdev, devno, 1);
	if (err) {
		printk(KERN_NOTICE "Error %d adding DM510%d", err, index);
	}
}

/* called when module is loaded */
int dm510_init_module( void ) {
	int i, result;
	dev_t firstdev;

	result = register_chrdev_region(firstdev = MKDEV(MAJOR_NUMBER, MIN_MINOR_NUMBER), DM510_nr_devs, "DM510");
	if (result < 0) {
		printk(KERN_NOTICE "Unable to get DM510 region, error %d\n", result);
		return result;
	}
	DM510_devno = firstdev;
	DM510_devices = kmalloc(DM510_nr_devs * sizeof(struct DM510), GFP_KERNEL);
	if (DM510_devices == NULL) {
		unregister_chrdev_region(firstdev, DM510_nr_devs);
		return -ENOMEM;
	}
	memset(DM510_devices, 0, DM510_nr_devs * sizeof(struct DM510));
	for (i = 0; i < DM510_nr_devs; i++) {
		init_waitqueue_head(&(DM510_devices[i].inq));
		init_waitqueue_head(&(DM510_devices[i].outq));
		dm510_setup_cdev(DM510_devices + i, i);

	}
	return 0;
}

/* Called when module is unloaded */
void dm510_cleanup_module( void ) {
	int i;

	if (!DM510_devices) { return; } // no devices to release

	for (i = 0; i < DM510_nr_devs; i++) {
		cdev_del(&DM510_devices[i].cdev);
		kfree(DM510_devices[i].buffer);
	}
	
	kfree(DM510_devices);
	unregister_chrdev_region(DM510_devno, DM510_nr_devs);
	DM510_devices = NULL;
	printk(KERN_INFO "DM510: Module unloaded.\n");
}

/* auxiliary method for getting target of buffer write in the read method */
struct DM510* getTarget(struct DM510 *dev) {
	if (dev == &DM510_devices[0]) {
		return (&DM510_devices[1]);
	} else {
		return (&DM510_devices[0]);
	}
}

// auxiliary method for updating the buffer, its size, end, read, and write pointer for both char devices
int redefinedbuffer(struct DM510 *dev, struct DM510 *target) {
	// if buffer is not defined allocate a buffer for 1st char driver
	if (!dev->buffer) {
		dev->buffer = kmalloc(DM510_buffer, GFP_KERNEL);
		if (!dev->buffer) { return -ENOMEM; }
		dev->rp = dev->wp = dev->buffer; // rd and wr from the beginning
	}

	// if buffer is not defined allocate a buffer for 2nd char driver
	if (!target->buffer) {
		target->buffer = kmalloc(DM510_buffer, GFP_KERNEL);
		if (!target->buffer) { return -ENOMEM; }
		target->rp = target->wp = target->buffer; // rd and wr from the beginning
	}

	// sets endpointer for both char drivers 
	dev->end = dev->buffer + DM510_buffer;
	target->end = target->buffer + DM510_buffer;
	return 0; // success
}


/* Called when a process tries to open the device file */
static int dm510_open( struct inode *inode, struct file *filp ) {
	
	struct DM510 *dev;
	struct DM510 *target;
	int result;

	dev = container_of(inode->i_cdev, struct DM510, cdev);
	target = getTarget(dev);
	filp->private_data = dev;
	
	// get mutex for this char driver
	if (mutex_lock_interruptible(&dev->mutex)) {
		return -ERESTARTSYS;
	}
	
	// get mutex for other char driver
	if (mutex_lock_interruptible(&target->mutex)) {
		return -ERESTARTSYS;
	}

	// if there are too many readers or writers returns
	if ((filp->f_mode & FMODE_READ && dev->nreaders == DM510_readers) || (filp->f_mode & FMODE_WRITE && dev->nwriters == DM510_writers)) {
		mutex_unlock(&dev->mutex);
		mutex_unlock(&target->mutex);
		return -EBUSY;
	} 
	
	if ((result = redefinedbuffer(dev, target)) != 0) {
		mutex_unlock(&dev->mutex);
		mutex_unlock(&target->mutex);
		return result;
	}
	if (filp->f_mode & FMODE_READ) { dev->nreaders++; }
	if (filp->f_mode & FMODE_WRITE)  { dev->nwriters++; }
	mutex_unlock(&dev->mutex);
	mutex_unlock(&target->mutex);
	return nonseekable_open(inode, filp);
}

/* Called when a process closes the device file. */
static int dm510_release( struct inode *inode, struct file *filp ) {
	struct DM510 *dev = filp->private_data;
	mutex_lock(&dev->mutex);
	if (filp->f_mode & FMODE_READ) { dev->nreaders--; }
	if (filp->f_mode & FMODE_WRITE) { dev->nwriters--; }
	if (dev->nreaders + dev->nwriters == 0) { kfree(dev->buffer); dev->buffer = NULL; }
	mutex_unlock(&dev->mutex);
	return 0;
}


/* Called when a process, which already opened the dev file, attempts to read from it. */
static ssize_t dm510_read( struct file *filp,
    char *buf,      /* The buffer to fill with data     */
    size_t count,   /* The max number of bytes to read  */
    loff_t *f_pos )  /* The offset in the file           */
{ 
	struct DM510 *dev = filp->private_data;
	
	if (mutex_lock_interruptible(&dev->mutex)) {
		return -ERESTARTSYS;
	}
	
	while (dev->rp == dev->wp) { /* nothing to read */
		mutex_unlock(&dev->mutex); /* release the lock */
		if (filp->f_flags & O_NONBLOCK) { return -EAGAIN; }
		if (wait_event_interruptible(dev->inq, (dev->rp != dev->wp))) { return -ERESTARTSYS; }
		if (mutex_lock_interruptible(&dev->mutex)) { return -ERESTARTSYS; }
	}
	if (dev->wp > dev->rp) {
		count = min(count, (size_t)(dev->wp - dev->rp));
	} else { /* the write pointer has wrapped, return data up to dev->end */
		count = min(count, (size_t)(dev->end - dev->rp));
	}
	
	if (!access_ok(buf, count) || copy_to_user(buf, dev->rp, count)) {
		mutex_unlock (&dev->mutex);
		return -EFAULT;
	}

	dev->rp += count;
	if (dev->rp == dev->end) { dev->rp = dev->buffer; } // wrapped
	mutex_unlock(&dev->mutex);

	// awake any writers and return
	wake_up_interruptible(&dev->outq);
	return count;	
}

// Auxiliary method for 
static int dm510_getwritespace(struct DM510 *dev, struct file *filp) {
	while (spacefree(dev) == 0) { // buffer is full
		DEFINE_WAIT(wait);
		mutex_unlock(&dev->mutex);
		if (filp->f_flags & O_NONBLOCK) { return -EAGAIN; }
		prepare_to_wait(&dev->outq, &wait, TASK_INTERRUPTIBLE);
		if (spacefree(dev) == 0) { schedule(); }
		finish_wait(&dev->outq, &wait);
		if (signal_pending(current)) { return -ERESTARTSYS; }  // signal: tell the fs layer to handle it
		if (mutex_lock_interruptible(&dev->mutex)) { return -ERESTARTSYS; }
	}
	return 0;
}	

// Auxiliary method for checking how much space is free
static int spacefree(struct DM510 *dev) {
	if (dev->rp == dev->wp) { return DM510_buffer - 1; }
	return ((dev->rp + DM510_buffer - dev->wp) % DM510_buffer) - 1;
}

// Called when a process writes to dev file
static ssize_t dm510_write( struct file *filp,
    const char *buf,/* The buffer to get data from      */
    size_t count,   /* The max number of bytes to write */
    loff_t *f_pos )  /* The offset in the file           */
{
	struct DM510 *dev = filp->private_data;
	int result;
	// gets the target for writing
	struct DM510 *target = getTarget(dev);

	// gets target mutex so no other processe can write to it
	if (mutex_lock_interruptible(&target->mutex)) { return -ERESTARTSYS; }

	// Make sure there's space to write
	result = dm510_getwritespace(target, filp);
	if (result) { return result; } // DM510_getwritespace called mutex_unlock(&target->mutex)

	count = min(count, (size_t)spacefree(target));
	if (target->wp >= target->rp) {
		count = min(count, (size_t)(target->end - target->wp)); // to end-of-buf
	} else { // the write pointer has wrapped, fill up to rp-1
		count = min(count, (size_t)(target->rp - target->wp - 1));
	}

	if (!access_ok(buf, count) || copy_from_user(target->wp, buf, count)) {
		mutex_unlock(&target->mutex);
		return -EFAULT;
	}

	target->wp += count;
	if (target->wp == target->end) {
		target->wp = target->buffer; // wrapped
	}
	mutex_unlock(&target->mutex);
	wake_up_interruptible(&target->inq);  // blocked in read() and select()
	return count;
}

/* called by system call icotl */ 
long dm510_ioctl( 
    struct file *filp, 
    unsigned int cmd,   /* command passed from the user */
    unsigned long arg ) /* argument of the command */
{	
	int err, retval = 0;

	if (_IOC_TYPE(cmd) != DM510_IOC_MAGIC) { 
		return -ENOTTY;
	}
	if (_IOC_NR(cmd) > DM510_IOC_MAXNR) { 
		return -ENOTTY;
	}
	
	if (_IOC_DIR(cmd) & _IOC_READ) {
		err = !access_ok((void __user *) arg, _IOC_SIZE(cmd));
	} else if (_IOC_DIR(cmd) & _IOC_WRITE) {
		err = !access_ok((void __user *) arg, _IOC_SIZE(cmd));
	}

	if (err) { return -EFAULT; }
	switch (cmd) {
		case DM510_IOCRESET: // resets to default values
			DM510_buffer = BUFFER_SIZE;
			DM510_readers = MAX_READERS;
			break;
		case DM510_IOCSBUFFER: // Set buffer to arg
			DM510_buffer = arg;
			for (int i = 0; i < DEVICE_COUNT; ++i) {
				kfree(DM510_devices[i].buffer);
				DM510_devices[i].buffer = NULL;
			}
			retval = redefinedbuffer(&DM510_devices[0], &DM510_devices[1]);
			break;
		case DM510_IOCSREADERS: // set readers to arg
			DM510_readers = arg;
			break;
		case DM510_IOCTBUFFERSIZE: // reads buffersize
			retval = DM510_buffer;
			break;
		case DM510_IOCTMAXREADERS: // reads number of current max readers
			retval = DM510_readers;
			break;
		default:
			return -ENOTTY;
	}
	return retval;
}

module_init( dm510_init_module );
module_exit( dm510_cleanup_module );

MODULE_AUTHOR( "...Steffen Nørgaard Bach, Danny Nicolai Larsen and Mikkel Brix Nielsen." );
MODULE_LICENSE( "GPL" );
