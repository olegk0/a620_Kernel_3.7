/*
 *
 * Asus 620/620BT PCMCIA/CF support.
 *
 * (c) 2014 olegvedi@gmail.com
 * 
 * Copyright(C)
 *   Nicolas Pouillon, 2004, <nipo@ssji.net>
 *   Vitaliy Sardyko, 2004
 *   Vincent Benony, 2006
 *      Adaptation to A620
 *
 *   Konstantine A. Beklemishev <konstantine@r66.ru>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

#include <pcmcia/ss.h>

#include <asm/irq.h>
#include <linux/irq.h>

//#include <linux/platform_data/pcmcia-pxa2xx_a620.h>

#include "soc_common.h"
#include "pxa2xx_base.h"
#include <mach/asus620.h>

//#define DEBUG 1

#if DEBUG
#  define DPRINTK(fmt, args...)	printk("%s: " fmt, __FUNCTION__ , ## args)
#else
#  define DPRINTK(fmt, args...)
#endif

static struct platform_device *a620_pcmcia_dev;
/*
static inline struct a620_pcmcia_pdata *a620_get_pdata(void)
{
	return a620_pcmcia_dev->dev.platform_data;
}
*/
static int a620_pcmcia_hw_init(struct soc_pcmcia_socket *skt)
{
//    struct a620_pcmcia_pdata *pdata = a620_get_pdata();
//	unsigned long flags;

    DPRINTK("A620 PCMCIA hw_init\n");

//    irq_set_irq_type(gpio_to_irq(GPIO21_A620_CF_VS1_N), IRQ_TYPE_EDGE_BOTH);
//    skt->socket.pci_irq = gpio_to_irq(GPIO7_A620_CF_IRQ);

//	skt->stat[SOC_STAT_CD].irq = gpio_to_irq(GPIO21_A620_CF_VS1_N);
    skt->stat[SOC_STAT_CD].gpio = GPIO21_A620_CF_VS1_N;
    skt->stat[SOC_STAT_CD].name = "CF_CD";
//	skt->stat[SOC_STAT_BVD1].irq = pdata->bvd1_gpio;;
//	skt->stat[SOC_STAT_BVD1].name = "PCMCIA0 STSCHG";
//	skt->stat[SOC_STAT_RDY].gpio = pdata->rdy_gpio;
    skt->stat[SOC_STAT_RDY].gpio = GPIO7_A620_CF_IRQ;
//	skt->stat[SOC_STAT_RDY].gpio = GPIO9_A620_CF_READY_N;
    skt->stat[SOC_STAT_RDY].name = "CF ready";

    return 0;

}

/*
 * Release all resources.
 */
static void a620_pcmcia_hw_shutdown(struct soc_pcmcia_socket *skt)
{
//	struct a620_pcmcia_pdata *pdata = a620_get_pdata();

//	gpio_free(pdata->pwr_gpio);
}

static void a620_pcmcia_socket_state(struct soc_pcmcia_socket *skt,
				      struct pcmcia_state *state)
{
    state->detect = gpio_get_value(GPIO21_A620_CF_VS1_N) ? 0 : 1;
    state->ready  = gpio_get_value(GPIO9_A620_CF_READY_N) ? 0 : 1;
    state->bvd1   = gpio_get_value(GPIO8_A620_CF_BVD1) ? 0 : 1;
    state->bvd2   = 1;
    state->wrprot = 0;
    state->vs_3v  = gpio_get_value(GPIO21_A620_CF_VS1_N) ? 0 : 1;
    state->vs_Xv  = gpio_get_value(GPIO22_A620_CF_VS2_N) ? 0 : 1;
//    DPRINTK ( "detect:%d ready:%d bvd:%d 3X:%d%d  irq:%d\n",
//	      state->detect, state->ready,state->bvd1, state->vs_3v, state->vs_Xv,gpio_get_value(GPIO7_A620_CF_IRQ));

}

static int a620_pcmcia_configure_socket(struct soc_pcmcia_socket *skt,
					 const socket_state_t *state)
{
//	struct a620_pcmcia_pdata *pdata = a620_get_pdata();

    /* Silently ignore Vpp, output enable, speaker enable. */
	DPRINTK ("Reset:%d Output:%d Vcc:%d\n", (state->flags & SS_RESET) ? 1 : 0,
	    !(!(state->flags & SS_OUTPUT_ENA)), state->Vcc);

//	asus620_pcmcia_hw_rst (state->flags & SS_RESET);

	gpio_set_value(GPIO17_A620_CF_ENABLE, (!(state->flags & SS_OUTPUT_ENA)));

        switch(state->Vcc) {
	case 0:
	    gpio_set_value(GPIO14_A620_CF_POWER, 0);
	    break;
        case 33:
	case 50:
	    gpio_set_value(GPIO14_A620_CF_POWER, 1);
	    break;
	default:
	    printk("Unhandled voltage set on PCMCIA: %d\n", state->Vcc);
	}

	DPRINTK("done !\n");
	return 0;

//	gpio_set_value(pdata->pwr_gpio, 0);
}

static struct pcmcia_low_level a620_pcmcia_ops = {
	.owner          	= THIS_MODULE,
	.hw_init        	= a620_pcmcia_hw_init,
	.hw_shutdown		= a620_pcmcia_hw_shutdown,
	.socket_state		= a620_pcmcia_socket_state,
	.configure_socket	= a620_pcmcia_configure_socket,
	.nr         		= 1,
};

static struct platform_device *a620_pcmcia_device;

static int a620_pcmcia_probe(struct platform_device *pdev)
{
	int ret;

	/* I can't imagine more than one device, but you never know... */
	if (a620_pcmcia_dev)
		return -EEXIST;

//	if (!pdev->dev.platform_data)
//		return -EINVAL;

	a620_pcmcia_device = platform_device_alloc("pxa2xx-pcmcia", -1);

	if (!a620_pcmcia_device)
		return -ENOMEM;

	a620_pcmcia_dev = pdev;

	a620_pcmcia_device->dev.parent = &pdev->dev;

	ret = platform_device_add_data(a620_pcmcia_device,
				       &a620_pcmcia_ops,
				       sizeof(a620_pcmcia_ops));

	if (!ret)
		ret = platform_device_add(a620_pcmcia_device);

	if (ret) {
		platform_device_put(a620_pcmcia_device);
		a620_pcmcia_dev = NULL;
	}
DPRINTK("probe ret:%d !\n",ret);
	return ret;
}

static int a620_pcmcia_remove(struct platform_device *pdev)
{
	platform_device_unregister(a620_pcmcia_device);
	a620_pcmcia_dev = NULL;
	return 0;
}

static struct platform_device_id a620_pcmcia_id_table[] = {
	{ .name = "a620-pcmcia", },
	{ },
};

static struct platform_driver a620_pcmcia_driver = {
	.probe		= a620_pcmcia_probe,
	.remove		= a620_pcmcia_remove,
	.driver		= {
		.name	= "asus620-pcmcia",
		.owner	= THIS_MODULE,
	},
	.id_table	= a620_pcmcia_id_table,
};

module_platform_driver(a620_pcmcia_driver);

MODULE_DEVICE_TABLE(platform, a620_pcmcia_id_table);
MODULE_LICENSE("GPL");
