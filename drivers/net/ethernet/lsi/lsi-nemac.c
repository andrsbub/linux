/*
 * LSI NEMAC Gigabit Ethernet driver
 *
 * Copyright (C) 2014 LSI Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/phy_fixed.h>
#include <net/ip.h>
#include <net/ipv6.h>


/* NEMAC Registers */

/* Global */
#define VER		0x0000
#define SCRATCH		0x0004
#define PRESENCE	0x0008
#define INT_STAT	0x0010
#define INT_EN		0x0014
#define INT_FORCE	0x0018
#define SWRESET		0x0020
#define ACTIVE		0x0024
#define LOS_CTRL	0x0028
#define LOS_DBG		0x002c

/* PTI */
#define INT_STAT	0x0100
#define INT_EN		0x0104
#define INT_FORCE	0x0108
#define TX_UNDRUN_STAT	0x010c
#define TX_OVRRUN_STAT	0x0110
#define RX_OVRRUN_STAT	0x0114
#define CONFIG		0x0118
#define BURST_LEN	0x011c
#define GMAC_TX_HI_WM	0x0120
#define GMAC_TX_LO_WM	0x0124
#define GMAC_RX_HI_WM	0x0128
#define GMAC_RX_LO_WM	0x012c
#define XGMAC_TX_HI_WM	0x0130
#define XGMAC_TX_LO_WM	0x0134
#define XGMAC_RX_HI_WM	0x0138
#define XGMAC_RX_LO_WM	0x013c
#define TX_TS_ADDR	0x0140
#define TX_TS_DATA_HI	0x0144
#define TX_TS_DATA_LO	0x0148
#define TX_TS_STAT	0x014c
#define RX_TS_STAT	0x0150

/* GMAC_DP */
#define TX_FIFO_STAT	0x0160
#define TX_MAX_USED	0x0164
#define TX_TRUNCATED	0x0168
#define TX_OVERRUNS	0x016c
#define RX_FIFO_STAT	0x0170
#define RX_MAX_USED	0x0174
#define RX_TRUNCATED	0x0178
#define RX_DROPPED	0x017c

/* XGMAC_DP */
#define TX_FIFO_STAT	0x02e0
#define TX_MAX_USED	0x02e4
#define TX_TRUNCATED	0x02e8
#define TX_OVERRUNS	0x02ec
#define RX_FIFO_STAT	0x02f0
#define RX_MAX_USED	0x02f4
#define RX_TRUNCATED	0x02f8
#define RX_DROPPED	0x02fc

/* GMAC */
#define PAUSE		0x0300
#define ADDR_LO		0x0304
#define ADDR_HI		0x0308
#define TIMING		0x030c
#define FEATURE		0x0310
#define STATUS		0x0314
#define PRBS_ERR	0x0318
#define SGMII_EN	0x031c
#define PRBS_CTRL	0x0320
#define ANEG_CTRL	0x0324
#define ANEG_STAT	0x0328
#define SGMII_STAT	0x032c
#define FILTER_CTRL	0x0330
#define MCAST_HASH(_n)	(0x0334 + 4 * (_n)) /* 0..3 */
#define UCAST_ADDR(_n)	(0x0344 + 4 * (_n)) /* 0..4 */
#define C37_AN_INT_MASK	0x0360
#define C73_AN_CTRL	0x0364
#define C73_AN_ADV_LO	0x036c
#define C73_AN_ADV_HI	0x0370
#define C73_AN_LP_LO	0x0374
#define C73_AN_LP_HI	0x0378
#define C73_AN_XNP_TX_LO	0x037c
#define C73_AN_XNP_TX_HI	0x0380
#define C73_AN_XNP_ABILITY_LO	0x0384
#define C73_AN_XNP_ABILITY_HI	0x0388
#define C73_AN_INT_MASK	0x038c
#define C73_AN_DEBUG	0x0390
#define UCAST_MASK_LOWER	0x0394
#define UCAST_MASK_UPPER	0x0398
#define PFC_EN		0x039c
#define PFC_CTRL	0x03a0
#define PFC_STAT	0x03a4
#define TS_MD		0x03a8
#define TS_MD_EXT	0x03b4

/* STATS */
#define INT_STAT	0x0d00
#define INT_EN		0x0d04
#define INT_FORCE	0x0d08
#define GMAC_CONFIG	0x0d0c
#define XGMAC_CONFIG	0x0d3c
#define SNAPSHOT	0x0d40
#define TX_STATS_LO(_n)	(0x0e00 + 8 * (_n) + 0)
#define TX_STATS_HI(_n)	(0x0e00 + 8 * (_n) + 4)
#define RX_STATS_LO(_n)	(0x0f00 + 8 * (_n) + 0)
#define RX_STATS_HI(_n)	(0x0f00 + 8 * (_n) + 4)

/* Driver private structure */
struct nemac_priv {
	void __iomem		*base;
	struct napi_struct	napi ____cacheline_aligned;
	struct net_device	*netdev;
	struct platform_device	*pdev;
	int			irq;

	/* PHY device */
	struct device_node	*phy_dn;
	struct phy_device	*phydev;
	phy_interface_t		phy_interface;
	int			old_pause;
	int			old_link;
	int			old_duplex;
};

/*
 * Ethtool operations
 */

static int nemac_set_settings(struct net_device *dev,
				    struct ethtool_cmd *cmd)
{
	struct nemac_priv *priv = netdev_priv(dev);

	if (!netif_running(dev))
		return -EINVAL;

	return phy_ethtool_sset(priv->phydev, cmd);
}

static int nemac_get_settings(struct net_device *dev,
					struct ethtool_cmd *cmd)
{
	struct nemac_priv *priv = netdev_priv(dev);

	if (!netif_running(dev))
		return -EINVAL;

	return phy_ethtool_gset(priv->phydev, cmd);
}

static void nemac_get_drvinfo(struct net_device *dev,
					struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, KBUILD_MODNAME, sizeof(info->driver));
	strlcpy(info->version, "0.1", sizeof(info->version));
	strlcpy(info->bus_info, "platform", sizeof(info->bus_info));
	info->n_stats = BCM_SYSPORT_STATS_LEN;
}

static u32 nemac_get_msglvl(struct net_device *dev)
{
	struct nemac_priv *priv = netdev_priv(dev);

	return priv->msg_enable;
}

static void nemac_set_msglvl(struct net_device *dev, u32 enable)
{
	struct nemac_priv *priv = netdev_priv(dev);

	priv->msg_enable = enable;
}

static void nemac_get_strings(struct net_device *dev,
					u32 stringset, u8 *data)
{
	int i;

	switch (stringset) {
	case ETH_SS_STATS:
		for (i = 0; i < BCM_SYSPORT_STATS_LEN; i++) {
			memcpy(data + i * ETH_GSTRING_LEN,
				nemac_gstrings_stats[i].stat_string,
				ETH_GSTRING_LEN);
		}
		break;
	default:
		break;
	}
}

static void nemac_get_stats(struct net_device *dev,
					struct ethtool_stats *stats, u64 *data)
{
	struct nemac_priv *priv = netdev_priv(dev);
	int i;

	if (netif_running(dev))
		nemac_update_mib_counters(priv);

	for (i =  0; i < BCM_SYSPORT_STATS_LEN; i++) {
		const struct nemac_stats *s;
		char *p;

		s = &nemac_gstrings_stats[i];
		if (s->type == BCM_SYSPORT_STAT_NETDEV)
			p = (char *)&dev->stats;
		else
			p = (char *)priv;
		p += s->stat_offset;
		data[i] = *(u32 *)p;
	}
}

static int nemac_get_sset_count(struct net_device *dev, int string_set)
{
	switch (string_set) {
	case ETH_SS_STATS:
		return BCM_SYSPORT_STATS_LEN;
	default:
		return -EOPNOTSUPP;
	}
}


static struct ethtool_ops nemac_ethtool_ops = {
	.get_settings		= nemac_get_settings,
	.set_settings		= nemac_set_settings,
	.get_drvinfo		= nemac_get_drvinfo,
	.get_msglevel		= nemac_get_msglvl,
	.set_msglevel		= nemac_set_msglvl,
	.get_link		= ethtool_op_get_link,
	.get_strings		= nemac_get_strings,
	.get_ethtool_stats	= nemac_get_stats,
	.get_sset_count		= nemac_get_sset_count,
};

static const struct net_device_ops nemac_netdev_ops = {
	.ndo_start_xmit		= nemac_xmit,
	.ndo_tx_timeout		= nemac_tx_timeout,
	.ndo_open		= nemac_open,
	.ndo_stop		= nemac_stop,
	.ndo_set_features	= nemac_set_features,
	.ndo_set_rx_mode	= nemac_set_rx_mode,
};

static int
nemac_probe(struct platform_device *pdev)
{
	struct device_node * const dn = pdev->dev.of_node;
	struct nemac_priv *priv;
	struct net_device *ndev;
	const void *macaddr;
	struct resource *res;
	u32 txq, rxq;
	int ret;

	ndev = alloc_etherdev(sizeof(*priv));
	if (!ndev)
		return -ENOMEM;

	/* Initialize private members */
	priv = netdev_priv(ndev);

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq <= 0) {
		ret = priv->irq;
		goto err;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->base)) {
		ret = PTR_ERR(priv->base);
		goto err;
	}

	priv->netdev = ndev;
	priv->pdev = pdev;

	priv->phy_interface = of_get_phy_mode(dn);
	if (priv->phy_interface < 0)
		priv->phy_interface = PHY_INTERFACE_MODE_SGMII;

	/* In the case of a fixed PHY, the DT node associated
	 * to the PHY is the Ethernet MAC DT node.
	 */
	if (of_phy_is_fixed_link(dn)) {
		ret = of_phy_register_fixed_link(dn);
		if (ret) {
			dev_err(&pdev->dev, "failed to register fixed PHY\n");
			goto err;
		}

		priv->phy_dn = dn;
	}

	/* Initialize Ethernet MAC address */
	macaddr = of_get_mac_address(dn);
	if (!macaddr || !is_valid_ether_addr(macaddr)) {
		dev_warn(&pdev->dev, "using random Ethernet MAC\n");
		random_ether_addr(dev->dev_addr);
	} else {
		ether_addr_copy(dev->dev_addr, macaddr);
	}

	SET_NETDEV_DEV(ndev, &pdev->dev);
	dev_set_drvdata(&pdev->dev, ndev);
	dev->ethtool_ops = &nemac_ethtool_ops;
	dev->netdev_ops = &nemac_netdev_ops;
	netif_napi_add(dev, &priv->napi, nemac_poll, 64);

	/* HW supported features, none enabled by default */
	dev->hw_features |= NETIF_F_RXCSUM | NETIF_F_HIGHDMA |
				NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM;

	/* Set the needed headroom once and for all */
	BUILD_BUG_ON(sizeof(struct bcm_tsb) != 8);
	dev->needed_headroom += sizeof(struct bcm_tsb);

	/* We are interfaced to a switch which handles the multicast
	 * filtering for us, so we do not support programming any
	 * multicast hash table in this Ethernet MAC.
	 */
	dev->flags &= ~IFF_MULTICAST;

	/* libphy will adjust the link state accordingly */
	netif_carrier_off(dev);

	ret = register_netdev(dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register net_device\n");
		goto err;
	}

	priv->rev = topctrl_readl(priv, REV_CNTL) & REV_MASK;
	dev_info(&pdev->dev,
		"Broadcom SYSTEMPORT" REV_FMT
		" at 0x%p (irqs: %d, %d, TXQs: %d, RXQs: %d)\n",
		(priv->rev >> 8) & 0xff, priv->rev & 0xff,
		priv->base, priv->irq0, priv->irq1, txq, rxq);

	return 0;
err:
	free_netdev(dev);
	return ret;
}

static int
nemact_remove(struct platform_device *pdev)
{
	struct net_device *dev = dev_get_drvdata(&pdev->dev);

	unregister_netdev(dev);
	free_netdev(dev);
	dev_set_drvdata(&pdev->dev, NULL);

	return 0;
}

static const struct of_device_id nemac_of_match[] = {
	{ .compatible = "lsi,nemac" },
	{ }
};

static struct platform_driver nemac_driver = {
	.probe	= nemac_probe,
	.remove	= nemac_remove,
	.driver =  {
		.name = "lsi-nemac",
		.owner = THIS_MODULE,
		.of_match_table = nemac_of_match,
	},
};
module_platform_driver(nemac_driver);

MODULE_AUTHOR("LSI Corporation");
MODULE_DESCRIPTION("LSI NEMAC Ethernet driver");
MODULE_LICENSE("GPL");
