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
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>

#include "ehci.h"

/*
 * Host interface controller registers
 *
 * Offset
 * 0x0000  Implementation specific registers
 * 0x0100  EHCI standard registers
 */
#define USB_ID			0x0000
#define USB_HWGENERAL		0x0004
#define USB_HWHOST		0x0008
#define USB_HWDEVICE		0x000c
#define USB_HWTXBUF		0x0010
#define USB_HWRXBUF		0x0014
#define USB_SBUSCFG		0x0090
#define USB_CAPLENGTH		0x0100

static const char hcd_name[] = "ehci-axxia";
static struct hc_driver __read_mostly axxia_hc_driver;

static int axxia_ehci_reset(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int retval = 0;

	ehci->caps = hcd->regs + USB_CAPLENGTH;
	hcd->has_tt = 1;

	retval = ehci_setup(hcd);
	if (retval)
		return retval;

	/* Setup AMBA interface to force INCR16 busts when possible in order to
	 * maximize throughput. */
	writel(3, hcd->regs + USB_SBUSCFG);

	return 0;
}

static int ehci_axxia_probe(struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	struct resource *res;
	void __iomem *base;
	int irq;
	int err;

	hcd = usb_create_hcd(&axxia_hc_driver, &pdev->dev,
			     dev_name(&pdev->dev));
	if (!hcd)
		return -ENOMEM;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		err = irq;
		goto fail;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base)) {
		err = PTR_ERR(base);
		goto fail;
	}

	platform_set_drvdata(pdev, hcd);
	hcd->regs = base;
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);

	err = usb_add_hcd(hcd, irq, 0);
	if (err)
		goto fail;

	return 0;

fail:
	usb_put_hcd(hcd);
	dev_err(&pdev->dev, "probe failed, err %d\n", err);
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

static struct of_device_id ehci_axxia_match[] = {
	{ .compatible = "lsi,axxia-ehci" },
	{ .compatible = "lsi,acp-usb" },
	{ /* end of list */ }
};

static struct platform_driver axxia_ehci_driver = {
	.probe = ehci_axxia_probe,
	.remove = ehci_axxia_remove,
	.driver = {
		.name = hcd_name,
		.of_match_table = ehci_axxia_match,
	},
};

static const struct ehci_driver_overrides axxia_overrides __initconst = {
	.reset = axxia_ehci_reset,
};

static int __init axxia_ehci_init(void)
{
	if (usb_disabled())
		return -ENODEV;
	ehci_init_driver(&axxia_hc_driver, &axxia_overrides);
	return platform_driver_register(&axxia_ehci_driver);
}
module_init(axxia_ehci_init);
