/* drivers/usb/host/ehci-axxia.c
 *
 * USB host controller driver for EHCI compliant device found in LSI Axxia
 * family of SoCs.
 *
 * Copyright (C) 2010 LSI Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/uaccess.h>
#include <linux/io.h>

/* Host interface controller registers
 *
 * Offset 0x0000 - 0x0090 Implementation specific registers
 * Offset 0x0100 -        EHCI standard registers
 *
 */
#define USB_ID			0x0000
#define USB_HWGENERAL		0x0004
#define USB_HWHOST		0x0008
#define USB_HWDEVICE		0x000c
#define USB_HWTXBUF		0x0010
#define USB_HWRXBUF		0x0014
#define USB_SBUSCFG		0x0090

static int axxia_ehci_init(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int retval = 0;
	int len;

	/* EHCI registers start at offset 0x100 */
	ehci->caps = hcd->regs + 0x100;
	len = HC_LENGTH(ehci, ehci_readl(ehci, &ehci->caps->hc_capbase));
	ehci->regs = hcd->regs + 0x100 + len;

	/* Configure other settings */
	ehci->hcs_params = ehci_readl(ehci, &ehci->caps->hcs_params);
	ehci->sbrn = 0x20;
	hcd->has_tt = 1;

	/* Data structure init */
	retval = ehci_init(hcd);
	if (retval)
		return retval;
	hcd->self.sg_tablesize = 0;

	/* Reset is only allowed on a stopped controller */
	ehci_halt(ehci);

	/* Reset controller */
	return ehci_reset(ehci);
}

static int axxia_ehci_run(struct usb_hcd *hcd)
{
	/* Setup AMBA interface to force INCR16 busts when possible in order to
	 * maximize throughput. */
	writel(3, hcd->regs + USB_SBUSCFG);

	return ehci_run(hcd);
}

static const struct hc_driver ehci_axxia_hcd = {
	.description		= "ehci_axxia_hcd",
	.product_desc		= "Axxia EHCI USB Host Controller",
	.hcd_priv_size		= sizeof(struct ehci_hcd),
	.irq			= ehci_irq,
	.flags			= HCD_MEMORY | HCD_USB2,
	.reset			= axxia_ehci_init,
	.start			= axxia_ehci_run,
	.stop			= ehci_stop,
	.shutdown		= ehci_shutdown,
	.urb_enqueue		= ehci_urb_enqueue,
	.urb_dequeue		= ehci_urb_dequeue,
	.endpoint_disable	= ehci_endpoint_disable,
	.get_frame_number	= ehci_get_frame,
	.hub_status_data	= ehci_hub_status_data,
	.hub_control		= ehci_hub_control,
#if defined(CONFIG_PM)
	.bus_suspend		= ehci_bus_suspend,
	.bus_resume		= ehci_bus_resume,
#endif
	.relinquish_port	= ehci_relinquish_port,
	.port_handed_over	= ehci_port_handed_over,
};

static int ehci_axxia_probe(struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	struct resource *res;
	void __iomem *base;
	int irq;
	int err;

	if (usb_disabled())
		return -ENODEV;

	if (of_property_read_bool(pdev->dev.of_node, "dma-coherent"))
		set_dma_ops(&pdev->dev, &arm_coherent_dma_ops);

	err = dma_coerce_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (err)
		return err;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		err = irq;
		goto fail_create_hcd;
	}

	err = irq_set_irq_type(irq, IRQ_TYPE_LEVEL_HIGH);
	if (err)
		goto fail_create_hcd;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base)) {
		err = PTR_ERR(base);
		goto fail_create_hcd;
	}

	hcd = usb_create_hcd(&ehci_axxia_hcd, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		err = -ENOMEM;
		goto fail_create_hcd;
	}

	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);
	hcd->regs = base;

	err = usb_add_hcd(hcd, irq, 0);
	if (err)
		goto fail_put_hcd;

	platform_set_drvdata(pdev, hcd);
	return 0;

fail_put_hcd:
	usb_put_hcd(hcd);
fail_create_hcd:
	dev_err(&pdev->dev, "failed probe, err %d\n", err);
	return err;
}

static int ehci_axxia_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

MODULE_ALIAS("platform:axxia-ehci");

static struct of_device_id ehci_axxia_match[] = {
	{ .compatible = "lsi,axxia-ehci" },
	{ .compatible = "lsi,acp-usb" },
	{ /* end of list */ }
};

static struct platform_driver ehci_axxia_driver = {
	.probe = ehci_axxia_probe,
	.remove = ehci_axxia_remove,
	.driver = {
		.name = "axxia-ehci",
		.of_match_table = ehci_axxia_match,
	},
};
