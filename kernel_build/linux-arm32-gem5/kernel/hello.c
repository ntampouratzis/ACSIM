#include <linux/init.h>
#include <linux/module.h>
MODULE_LICENSE("Dual BSD/GPL");

static int hello_init(void)
{
    printk(KERN_ALERT "\n\n\n\n\n\nHello, world\n\n\n\n\n\n");
    return 0;
}

static void hello_exit(void)
{
    printk(KERN_ALERT "\n\n\n\n\n\n\nGoodbye, cruel world\n\n\n\n\n\n");
}

module_init(hello_init);
module_exit(hello_exit);