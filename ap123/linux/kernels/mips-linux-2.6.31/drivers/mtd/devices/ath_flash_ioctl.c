/********************************************************************
 *This is flash ioctl for  reading ,writing and erasing application
 *******************************************************************/

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/major.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
//#include <asm-mips/ioctl.h>
//#include <asm-mips/uaccess.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <asm/system.h>

#include <atheros.h>
#include "ath_flash.h"


/*
 * IOCTL Command Codes
 */
#define ATH_FLASH_READ				0x01
#define ATH_FLASH_WRITE				0x02
#define ATH_FLASH_ERASE				0x03
//#define ATH_FLASH_APP_WRITE	      	0x04
//#define ATH_FLASH_APP_ERASE			0x05


#define ATH_IO_MAGIC 				0xB3
#define	ATH_IO_FLASH_READ			_IOR(ATH_IO_MAGIC, ATH_FLASH_READ, char)
#define ATH_IO_FLASH_WRITE			_IOW(ATH_IO_MAGIC, ATH_FLASH_WRITE, char)
#define	ATH_IO_FLASH_ERASE			_IO (ATH_IO_MAGIC, ATH_FLASH_ERASE)
//#define ATH_IO_APP_WRITE				_IOW(ATH_IO_MAGIC, ATH_FLASH_APP_WRITE, char)
//#define ATH_IO_APP_ERASE				_IO(ATH_IO_MAGIC, ATH_FLASH_APP_ERASE)
#define	ATH_IOC_MAXNR				14
#define flash_major      				239
#define flash_minor      				0

int ath_flash_ioctl(struct inode *inode, struct file *file,  unsigned int cmd, unsigned long arg);
int ath_flash_open (struct inode *inode, struct file *file);

struct file_operations flash_device_op = {
        .owner = THIS_MODULE,
        .ioctl = ath_flash_ioctl,
        .open = ath_flash_open,
};

static struct cdev flash_device_cdev = {
		//.kobj   = {.name = "ar7240_flash_chrdev", },
        .owner  = THIS_MODULE,
		.ops = &flash_device_op,
};

typedef struct 
{
	u_int32_t addr;		/* flash r/w addr	*/
	u_int32_t len;		/* r/w length		*/
	u_int8_t* buf;		/* user-space buffer*/
	u_int32_t buflen;	/* buffer length	*/
	u_int32_t reset;	/* reset flag 		*/
}ARG;

//#define ATH_FLASH_IO_BUF_LEN		2048
#define ATH_FLASH_SECTOR_SIZE	(64 * 1024)

int ath_flash_ioctl(struct inode *inode, struct file *file,  unsigned int cmd, unsigned long arg)
{
	struct mtd_info *mtd = (struct mtd_info *)kmalloc(sizeof(struct mtd_info), GFP_KERNEL);
	/* temp buffer for r/w */
	unsigned char *rwBuf = (unsigned char *)kmalloc(ATH_FLASH_SECTOR_SIZE, GFP_KERNEL);
	
	ARG *pArg = (ARG*)arg;
	u_int8_t* usrBuf = pArg->buf;
	u_int32_t usrBufLen = pArg->buflen;
	u_int32_t addr = pArg->addr;
	u_int32_t reset = pArg->reset;
	
	int i;
	int ret = 0;
	size_t retlen;
	int addr_start, addr_end;
	
	int nSector = usrBufLen >> 16; 			/* Divide ATH_FLASH_SECTOR_SIZE */	
	int oddLen = usrBufLen & 0x0000FFFF;	/* odd length (0 ~ ATH_FLASH_SECTOR_SIZE) */

	if ((mtd == NULL) || (rwBuf == NULL))
	{
		goto wrong;
	}

	memset(mtd, 0, sizeof(struct mtd_info));

	mtd->size               =   8 * 1024 * 1024;
	mtd->erasesize          =   ATH_FLASH_SECTOR_SIZE;
	mtd->numeraseregions    =   0;
	mtd->eraseregions       =   NULL;
	mtd->owner              =   THIS_MODULE;

	if (_IOC_TYPE(cmd) != ATH_IO_MAGIC)
	{
		printk("cmd type error!\n");
		goto wrong;
	}
	if (_IOC_NR(cmd) > ATH_IOC_MAXNR)
	{
		printk("cmd NR error!\n");
		goto wrong;
	}
	if (_IOC_DIR(cmd) & _IOC_READ)
	{
		ret = access_ok(VERIFY_WRITE, (void __user *)usrBuf, _IOC_SIZE(cmd));
	}
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
	{
		ret = access_ok(VERIFY_READ, (void __user *)usrBuf, _IOC_SIZE(cmd));
	}
	if (ret < 0)
	{ 
		printk("access no ok!\n");
		goto wrong;
	}

	switch(cmd)
	{
		case ATH_IO_FLASH_READ:
		{
			#if 0
			/* copy from flash to kernel buffer */	
			for (i = 0; i < nSector; i++)
			{
				ret = ath_flash_read(mtd, addr, ATH_FLASH_SECTOR_SIZE, &retlen, rwBuf);
				if ((ret == 0) && (retlen == ATH_FLASH_SECTOR_SIZE))
				{
					/*copy from kernel to user*/
					if (copy_to_user(usrBuf, rwBuf, ATH_FLASH_SECTOR_SIZE))
					{
						printk("read copy_to_user failed\n");
						goto wrong;
					}
				}
				else
				{
					printk("Read failed, ret:%d!\n", ret);
					goto wrong;
				}
				addr += ATH_FLASH_SECTOR_SIZE;
				usrBuf += ATH_FLASH_SECTOR_SIZE;
			}	
		
			if (oddLen) 
			{
				ret = ath_flash_read(mtd, addr, oddLen, &retlen, rwBuf);
                if ((ret == 0) && (retlen == oddLen))
				{
					/*copy from kernel to user*/
					if (copy_to_user(usrBuf, rwBuf, oddLen))
					{
						printk("read copy_to_user failed\n");
						goto wrong;
                    }
                } 
				else 
				{
					printk("Read failed!\n");
					goto wrong;
                }
			}
			#else
			ret = ath_flash_read(mtd, addr, usrBufLen, &retlen, rwBuf);
			if ((ret == 0) && (retlen == usrBufLen))
			{
				/*copy from kernel to user*/
				if (copy_to_user(usrBuf, rwBuf, usrBufLen))
				{
					printk("read copy_to_user failed\n");
					goto wrong;
				}
			}
			else
			{
				printk("Read failed, ret:%d from:%d size:%d\n", ret, addr, usrBufLen);
				goto wrong;
			}
			#endif
			goto good;
			break;
		}
		

		case ATH_IO_FLASH_WRITE:
		{
			addr_start	= addr & 0xFFFF0000;			/* first sector addr to erase */
            addr_end	= (addr + usrBufLen) & 0xFFFF0000;	/* last sector addr to erase */
			
			//local_irq_save(flags);	/* disable interrupts */
			printk("Erase from %#X to %#X:", addr, addr + usrBufLen);
            do
			{
            	ath_spi_sector_erase(addr_start);
				printk(".");
				addr_start += ATH_FLASH_SECTOR_SIZE;
            }while (addr_start < addr_end);

			printk("\n");
			
            ath_spi_done();
			printk("Program from %#X to %#X:", addr, addr + usrBufLen);
			for (i = 0; i < nSector; i++)		
			{
				if (copy_from_user(rwBuf, usrBuf, ATH_FLASH_SECTOR_SIZE))
				{
					printk("config write copy_from_usr failed!\n");
					goto wrong;				
				}
			
				ret =  ath_flash_write(mtd, addr, ATH_FLASH_SECTOR_SIZE, &retlen, rwBuf);
				printk(".");
				if ((ret != 0) && (retlen != ATH_FLASH_SECTOR_SIZE))
				{
					printk("\nWrite to flash failed status:%d retlen:%d\n", ret, retlen);
                	goto wrong;
				}
				addr += ATH_FLASH_SECTOR_SIZE;
				usrBuf += ATH_FLASH_SECTOR_SIZE;
			}

			if (oddLen)
			{
				if (copy_from_user(rwBuf, usrBuf, oddLen))
				{
	                printk("config write copy_from_usr failed!\n");
	                goto wrong;
	            }
				ret = ath_flash_write(mtd, addr, oddLen, &retlen, rwBuf);

	            if ((ret != 0) && (retlen != oddLen))
				{
	                printk("\nWrite to flash failed status:%d retlen:%d\n", ret, retlen);
	                goto wrong;
	            }
			}
			printk("\nwrite successfully\n");
			
			if (reset)
			{
				/* now we reboot in user space */
				//machine_restart("reboot");	/*reboot after success */
				printk("reboot...\n");
			}
			
            goto good;
			break;
		}
		
		case  ATH_FLASH_ERASE:
		{
			goto good;
			break;
		}
		
#if 0
		case	ATH_IO_APP_ERASE:
		{
//			printk("app erase ok!\n");
            goto good;
			break;
		}
	
		
		case ATH_IO_APP_WRITE:
		{
			to = 0x40000;

			if (len%mtd->erasesize) nsect++;
            s_curr = ((unsigned int) 0x40000)/mtd->erasesize; 
            s_last = s_curr + nsect;
            //printk("\nNow erase\n ");
            do {
                ath_spi_sector_erase(s_curr*1024*64);
//              printk("The erase addr is %08x\n",s_curr*1024*64);

            }while(++s_curr < s_last);
            ath_spi_done();
			//printk("App Erase succeeded\n");

			for (i = 0; i<len/buflen; i++)
			{
		    	if (copy_from_user(buffer, argp, buflen)) 
    			{
					printk("write to flash failed when copy_from_user\n");
	   	            goto wrong;
		      	 }
//				printk("The app write len is %08x\n", buflen);
		        status =  ath_flash_write(mtd, to, buflen, &retlen, buffer);
        		if ((status != 0) && (retlen != buflen)) {
	  			    printk("\nWrite to flash failed status:%d retlen:%d\n", status, retlen);
        	        goto wrong;
	        	}
					argp += buflen;
					to += buflen;
			}
		
			if (rwlen)
			{
				if (copy_from_user(buffer, argp, rwlen)) {
                	printk("write to flash failed when copy_from_user\n");
	        	    goto wrong;
        		}
		         printk("The app write len is %08x", rwlen);
        		 status =  ath_flash_write(mtd, to, rwlen, &retlen, buffer);
	        	if ((status != 0) && (retlen != rwlen)) {
        	      	printk("\nWrite to flash failed. status:%d retlen:%d\n", status, retlen);
	        	    goto wrong;
        		}
			
			 }

 			printk("\nWrite to flash successful\n");
	        machine_restart("reboot"); /*reboot after success */
	        goto good;
			break;
		}
	
		default:
		printk("Wrong cmd!\n");
		goto wrong;
#endif	
	}


good:
	kfree(mtd);
	kfree(rwBuf);

	return 0;
wrong:
	if (mtd)
	{
		kfree(mtd);
	}

	if (rwBuf)
	{
		kfree(rwBuf);
	}

	return -1;
	
}


int ath_flash_open (struct inode *inode, struct file *filp)
{
	int minor = iminor(inode);
	int devnum = minor; //>> 1;
	struct mtd_info *mtd;
	printk("Now flash open!\n");
	
	if ((filp->f_mode & 2) && (minor & 1)) {
		printk("You can't open the RO devices RW!\n");
		return -EACCES;
	}

	mtd = get_mtd_device(NULL, devnum);   
	if (!mtd) {
		printk("Can not open mtd!\n");
		return -ENODEV;	
	}
	filp->private_data = mtd;
	return 0;
	
}

int __init ath_flash_chrdev_init (void)
{
    dev_t dev;
    int result;
    int err;
    int ath_flash_major = flash_major;
    int ath_flash_minor = flash_minor;

    if (ath_flash_major) {
        dev = MKDEV(ath_flash_major, ath_flash_minor);
        result = register_chrdev_region(dev, 1, "ar7240_flash_chrdev");
    } else {
        result = alloc_chrdev_region(&dev, ath_flash_minor, 1, "ar7240_flash_chrdev");
        ath_flash_major = MAJOR(dev);
    }

    if (result < 0) {
        printk(KERN_WARNING "ar7240_flash_chrdev : can`t get major %d\n", ath_flash_major);
        return result;
    }
    cdev_init (&flash_device_cdev, &flash_device_op);
    err = cdev_add(&flash_device_cdev, dev, 1);
    if (err) printk(KERN_NOTICE "Error %d adding flash_chrdev ", err);
//    devfs_mk_cdev(dev, S_IFCHR | S_IRUSR | S_IWUSR, "ar7240flash_chrdev");
    return 0;

}

static void __exit cleanup_ath_flash_chrdev_exit (void)
{
//	unregister_chrdev_region(MKDEV(flash_major, flash_minor), 1);
}


module_init(ath_flash_chrdev_init);
module_exit(cleanup_ath_flash_chrdev_exit);
//MODULE_LICENSE("GPL");

