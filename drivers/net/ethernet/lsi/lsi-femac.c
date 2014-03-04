/*
 * Network device driver for LSI Fast-Ethernet controller (FEMAC).
 *
 * Copyright (C) 2013 LSI
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_mdio.h>
#include <linux/dmapool.h>

#define DRVNAME          "lsi-femac"
#define DESCRIPTOR_GRANULARITY   64
#define MAX_FRAME_SIZE         1600
#define NAPI_WEIGHT              64

static unsigned char macaddr[ETH_ALEN];
module_param_array(macaddr, byte, NULL, S_IRUSR);
MODULE_PARM_DESC(macaddr, "Ethernet address");

static int rx_num_desc = 128; /* Must be multiple of DESCRIPTOR_GRANULARITY */
module_param(rx_num_desc, int, S_IRUSR);
MODULE_PARM_DESC(rx_num_desc, "Number of receive descriptors");

static int tx_num_desc = 128; /* Must be multiple of DESCRIPTOR_GRANULARITY */
module_param(tx_num_desc, int, S_IRUSR);
MODULE_PARM_DESC(tx_num_desc, "Number of transmit descriptors");

/**
 * struct dma_desc - Hardware DMA descriptor
 */
struct dma_desc {
	__le32 flags;
#define DMADESC_FILL    (0<<0)
#define DMADESC_BLOCK   (1<<0)
#define DMADESC_SCATTER (2<<0)
#define DMADESC_READ    (0<<2)
#define DMADESC_WRITE   (1<<2)
#define DMADESC_SOP     (1<<3)
#define DMADESC_EOP     (1<<4)
#define DMADESC_INTR    (1<<5)
#define DMADESC_ERROR   (1<<6)
#define DMADESC_SWAP    (1<<7)
	__le16 pdu_len;
	__le16 buf_len;
	__le32 cookie;
	__le32 buf_ptr;
};

/**
 * struct queue_ptr - Holds the state of the RX or TX queue
 * @hw_tail: Tail pointer (written by hardware, pointer to this field is
 *           programmed into the DMAREG_(RX|TX)_TAIL_ADDR). Points to the next
 *           descriptor to be used for reception or transmission.
 * @tail:    Driver tail pointer. Follows hw_tail and points to next descriptor
 *           to be completed.
 * @head:    Head pointer where the drivers puts the new buffers queued for
 *           transmission, and the where fresh RX buffers are added.
 * @size:    Size in bytes of the descriptor ring.
 * @phys:    Physical address of descriptor ring.
 */
struct queue_ptr {
	__le32      hw_tail;
	u32         tail;
	u32         head;
	size_t      size;
	dma_addr_t  phys;
};

struct dbg_counters {
	unsigned long tx_interrupt;
	unsigned long rx_interrupt;
	unsigned long tx_nodesc;
	unsigned long tx_nobuf;
};

#define DBG_INC(_priv, _member) (++(_priv)->counters._member)

/* Device private data */
struct femac_dev {
	struct net_device      *ndev;
	struct device          *dev;

	/* napi */
	struct napi_struct	napi;

	/* I/O addresses */
	void __iomem		*rx_base;
	void __iomem		*tx_base;
	void __iomem		*dma_base;

	/* MAC address (from parameter, device-tree or randomized) */
	unsigned char		mac_addr[ETH_ALEN] __aligned(2);

	/* PHY */
	struct phy_device	*phy_dev;
	int			link;
	int			speed;
	int			duplex;

	/* RX/TX ring */
	size_t                  rx_ring_size;
	dma_addr_t		rx_ring_phys;
	unsigned		rx_num_desc;
	struct dma_desc		*rx_ring;
	struct queue_ptr       *rxq;

	/* TX ring */
	size_t                  tx_ring_size;
	dma_addr_t		tx_ring_phys;
	unsigned		tx_num_desc;
	struct dma_desc		*tx_ring;
	struct queue_ptr       *txq;

	/* DMA pool for tx buffers */
	struct dma_pool        *tx_pool;

	/* Debug counters */
	struct dbg_counters     counters;

	spinlock_t		lock;
};

#define napi_to_priv(_napi) container_of(napi, struct femac_dev, napi)

/* FEMAC Registers
 */

/* SMII Status */
#define RXREG_SMII_STATUS(p)		((p)->rx_base + 0x10)
#define   RX_SMII_STATUS_SPEED		0x01
#define   RX_SMII_STATUS_DUPLEX		0x02
#define   RX_SMII_STATUS_LINK		0x04
#define   RX_SMII_STATUS_JABBER		0x08
#define   RX_SMII_STATUS_FCD		0x10 /* False Carrier Detect */
/* Receive Configuration */
#define RXREG_CONF(p)			((p)->rx_base + 0x004c)
#define   RX_CONF_ENABLE		0x0001
#define   RX_CONF_PAP			0x0002 /* Pass Any Packet */
#define   RX_CONF_JUMBO9K		0x0008
#define   RX_CONF_STRIPCRC		0x0010
#define   RX_CONF_AMT			0x0020 /* Accept All MAC Types */
#define   RX_CONF_AFC			0x0040 /* Accept Flow Control */
#define   RX_CONF_VLAN			0x0200 /* Enable VLAN */
#define   RX_CONF_SPEED			0x0800 /* RX MAC Speed, 0=10M, 1=100M */
#define   RX_CONF_DUPLEX		0x1000 /* 1=Full-duplex */
#define   RX_CONF_LINK			0x2000 /* 1=Enable */
#define   RX_CONF_RXFCE			0x4000 /* RX flow-control */
#define   RX_CONF_TXFCE			0x8000 /* TX flow-control */
/* Receive Statistics */
#define RXREG_STAT_OVERFLOW(p)	((p)->rx_base + 0x278)
#define RXREG_STAT_UNDERSIZE(p)	((p)->rx_base + 0x280)
#define RXREG_STAT_OVERSIZE(p)	((p)->rx_base + 0x2b8)
#define RXREG_STAT_PACKET_OK(p)	((p)->rx_base + 0x2c0)
#define RXREG_STAT_CRC_ERR(p)	((p)->rx_base + 0x2c8)
#define RXREG_STAT_MULTICAST(p)	((p)->rx_base + 0x2d0)
#define RXREG_STAT_BROADCAST(p)	((p)->rx_base + 0x2d8)
#define RXREG_STAT_MACTYPE(p)	((p)->rx_base + 0x2e0)
#define RXREG_STAT_ALIGN_ERR(p)	((p)->rx_base + 0x2e8)
#define RXREG_STAT_BYTES_LO(p)	((p)->rx_base + 0x2f0)
#define RXREG_STAT_BYTES_HI(p)	((p)->rx_base + 0x2f8)
/* Receive Ethernet Mode */
#define RXREG_MODE(p)			((p)->rx_base + 0x0800)
#define   RX_MODE_ETHERNET_MODE_ENABLE	0x00001
/* Receive Soft Reset */
#define RXREG_SOFT_RESET(p)		((p)->rx_base + 0x0808)
#define   RX_SOFT_RESET_MAC_0		0x00001
/* Receive Internal Interrupt Control */
#define RXREG_IIC(p)			((p)->rx_base + 0xc00)
#define   RX_IIC_MAC_0			0x01
/* Receive External Interrupt Control */
#define RXREG_EIC(p)			((p)->rx_base + 0xc04)
#define   RX_EIC_MAC_0_HIGH_LOW		0x10
#define   RX_EIC_MAC_0			0x01
/* Receive Interrupt Status */
#define RXREG_INTERRUPT_STATUS(p)	((p)->rx_base + 0xc20)
#define   RX_INTERRUPT_EXTERNAL_MAC_0	0x10
#define   RX_INTERRUPT_INTERNAL_MAC_0	0x01
/* Transmit Watermark */
#define TXREG_WATERMARK(p)		((p)->tx_base + 0x18)
#define   TX_WATERMARK_DTPA_ASSERT	0x8000
#define   TX_WATERMARK_DTPA_DISABLE	0x4000
#define   TX_WATERMARK_DTPA_HIGH	0x3f00
#define   TX_WATERMARK_DTPA_LOW		0x003f
/* Swap Source Address Registers */
#define TXREG_SOURCE_ADDRESS_2(p)	((p)->tx_base + 0x20)
#define TXREG_SOURCE_ADDRESS_1(p)	((p)->tx_base + 0x24)
#define TXREG_SOURCE_ADDRESS_0(p)	((p)->tx_base + 0x28)
/* Transmit Extended Configuration */
#define TXREG_EXTENDED_CONF(p)		((p)->tx_base + 0x30)
#define   TX_EXTCONF_TRANSMIT_COLLISION_WATERMARK_LEVEL 0xf000
#define   TX_EXTCONF_EXCESSIVE_DEFFERED_PACKET_DROP	0x200
#define   TX_EXTCONF_JUMBO9K				0x100
#define   TX_EXTCONF_LATE_COLLISION_WINDOW_COUNT	0xff
/* Transmit Half Duplex Configuration */
#define TXREG_HALF_DUPLEX_CONF(p)	((p)->tx_base + 0x34)
#define   TX_HALF_DUPLEX_CONF_RANDOM_SEED_VALUE		0xff
/* Transmit Configuration */
#define TXREG_CONF(p)			((p)->tx_base + 0x0050)
#define   TX_CONF_ENABLE_SWAP_SA	0x8000
#define   TX_CONF_LINK			0x2000
#define   TX_CONF_DUPLEX		0x1000
#define   TX_CONF_SPEED			0x0800
#define   TX_CONF_XBK_RST_RX_NTX	0x0600
#define   TX_CONF_IFG_MASK		0x01f0
#define   TX_CONF_IFG(_x)		(((_x) & 0x1f) << 4)
#define   TX_CONF_APP_CRC_ENABLE	0x0004
#define   TX_CONF_PAD_ENABLE		0x0002
#define   TX_CONF_ENABLE		0x0001
/* Transmit Time Value Configuration */
#define TXREG_TIME_VALUE_CONF(p)	((p)->tx_base + 0x5c)
#define   TX_TIME_VALUE_CONF_PAUSE	0xffff
/* Transmit Statistics */
#define TXREG_STAT_UNDERRUN(p)		((p)->tx_base + 0x300)
#define TXREG_STAT_DEFERRED(p)		((p)->tx_base + 0x308)
#define TXREG_STAT_PAUSE(p)		((p)->tx_base + 0x310)
#define TXREG_STAT_PACKET_OK(p)		((p)->tx_base + 0x318)
#define TXREG_STAT_UNDERSIZE(p)		((p)->tx_base + 0x350)
#define TXREG_STAT_BYTES_LO(p)		((p)->tx_base + 0x358)
#define TXREG_STAT_BYTES_HI(p)		((p)->tx_base + 0x360)
#define TXREG_STAT_LATECOLL(p)		((p)->tx_base + 0x368)
#define TXREG_STAT_EXCECOLL(p)		((p)->tx_base + 0x370)
#define TXREG_STAT_EXCEDEFERRED(p)	((p)->tx_base + 0x378)
#define TXREG_STAT_COLLISION_LIMIT(p)	((p)->tx_base + 0x380)
/* Transmit Mode */
#define TXREG_MODE(p)			((p)->tx_base + 0x800)
#define   TX_MODE_ETHERNET_MODE_ENABLE	0x1
/* Transmit Soft Reset */
#define TXREG_SOFT_RESET(p)		((p)->tx_base + 0x808)
#define   TX_SOFT_RESET_MAC_0		0x1
/* Transmit Interrupt Control */
#define TXREG_INT_CONTROL(p)		((p)->tx_base + 0xc00)
#define   TX_INTERRUPT_CONTROL_MAC_0	0x1
/* Transmit Interrupt Status */
#define TXREG_INT_STATUS(p)		((p)->tx_base + 0xc20)
#define   TX_INTERRUPT_STATUS_MAC_0	0x1
/* DMA PCI Control */
#define DMAREG_PCI_CONTROL(p)		((p)->dma_base + 0x00)
/* DMA Control */
#define DMAREG_CONTROL(p)		((p)->dma_base + 0x08)
/* DMA Interrupt Enable/Status */
#define DMAREG_INT_STATUS(p)		((p)->dma_base + 0x18)
#define DMAREG_INT_ENABLE(p)		((p)->dma_base + 0x1c)
#define   DMA_INT_RX			0x2
#define   DMA_INT_TX			0x1
/* DMA RX/TX Queue Base/Size */
#define DMAREG_RX_QUEUE_BASE(p)		((p)->dma_base + 0x30)
#define DMAREG_RX_QUEUE_SIZE(p)		((p)->dma_base + 0x34)
#define DMAREG_TX_QUEUE_BASE(p)		((p)->dma_base + 0x38)
#define DMAREG_TX_QUEUE_SIZE(p)		((p)->dma_base + 0x3c)
/* DMA Tail Pointer Address */
#define DMAREG_RX_TAIL_ADDR(p)		((p)->dma_base + 0x48)
#define DMAREG_TX_TAIL_ADDR(p)		((p)->dma_base + 0x4c)
/* DMA Head/Tail Pointers */
#define DMAREG_RX_HEAD(p)		((p)->dma_base + 0x50)
#define DMAREG_RX_TAIL(p)		((p)->dma_base + 0x54)
#define DMAREG_TX_HEAD(p)		((p)->dma_base + 0x58)
#define DMAREG_TX_TAIL(p)		((p)->dma_base + 0x5c)
#define   DMA_POINTER_GEN		0x100000
#define   DMA_POINTER_MASK		0x0fffff

#define dmaptr_idx(_val) (((_val) & DMA_POINTER_MASK)/sizeof(struct dma_desc))
#define dmaptr_gen(_val) (!!((_val) & DMA_POINTER_GEN))

/* RX/TX-ring
 *
 * tail - Oldest descriptor, i.e. the next descriptor to be processed by RX/TX
 * interrupt. This pointer is only used by the driver (no corresponding
 * hardware register). The interrupt handler will process descriptors from tail
 * to hw_tail.
 *
 * hw_tail - Next descriptor to be processed by hardware. The memory location
 * is updated by the hardware when it switches descriptor (via DMA). A copy of
 * this value is also available in the DMAREG_[RX|TX]_TAIL register.
 *
 * head - Newest descriptor. This is where the driver adds new descriptors
 * (either fresh rx buffers or tx buffers queued for transmission) and the
 * pointer is updated in hardware via the DMAREG_[RX|TX]_HEAD register. The
 * hardware will process descriptors from hw_tail to head. When hw_tail ==
 * head, the ring is empty.
 *
 *		tail	hw_tail		head
 *		|	|		|
 *		V	V		V
 *      +-----+ +-----+ +-----+ +-----+ +-----+ +-----+
 *      |     | |     | |     | |     | |     | |     |
 *      +-----+ +-----+ +-----+ +-----+ +-----+ +-----+
 *
 */

/**
 * queue_get_head - Return next DMA descriptor from head of queue.
 */
static inline struct dma_desc *
queue_get_head(struct dma_desc *ring, const struct queue_ptr *q)
{
	if ((q->head ^ q->tail) == DMA_POINTER_GEN)
		return NULL;
	return &ring[dmaptr_idx(q->head)];
}

/**
 * queue_get_tail - Return next DMA descriptor from tail of queue.
 */
static inline struct dma_desc *
queue_get_tail(struct dma_desc *ring, const struct queue_ptr *q)
{
	if (q->tail == le32_to_cpu(q->hw_tail))
		return NULL;
	return &ring[dmaptr_idx(q->tail)];
}

/**
 * inc_pointer - Helper function to increment a DMA pointer. The counter is in
 * the lower bits and is incremented modulo the size of the ring. The bit
 * DMA_POINTER_GEN is toggled when the counter wraps.
 */
static inline u32
inc_pointer(u32 ptr, u32 size)
{
	u32 newptr = (ptr & DMA_POINTER_MASK) + sizeof(struct dma_desc);

	/* When counter wraps (on size), reset and toggle generation bit.
	 * Otherwise preserve generation bit
	 */
	if (newptr >= size)
		newptr = (ptr & DMA_POINTER_GEN) ^ DMA_POINTER_GEN;
	else
		newptr |= ptr & DMA_POINTER_GEN;

	return newptr;
}

static inline u32
queue_inc_head(struct queue_ptr *q)
{
	q->head = inc_pointer(q->head, q->size);
	return q->head;
}

static inline u32
queue_inc_tail(struct queue_ptr *q)
{
	q->tail = inc_pointer(q->tail, q->size);
	return q->tail;
}

static inline void
pr_queue(const char *tag, const struct queue_ptr *q)
{
	pr_debug("%s tail=%d.%d hw_tail=%d.%d head=%d.%d\n",
		tag,
		dmaptr_gen(q->tail), dmaptr_idx(q->tail),
		dmaptr_gen(q->hw_tail), dmaptr_idx(q->hw_tail),
		dmaptr_gen(q->head), dmaptr_idx(q->head));
}

/**
 * clear_statistics - Counters are cleared on read.
 */
static void
clear_statistics(const struct femac_dev *priv)
{
	int waste;

	waste = readl(RXREG_STAT_OVERFLOW(priv));
	waste = readl(RXREG_STAT_UNDERSIZE(priv));
	waste = readl(RXREG_STAT_OVERSIZE(priv));
	waste = readl(RXREG_STAT_PACKET_OK(priv));
	waste = readl(RXREG_STAT_CRC_ERR(priv));
	waste = readl(RXREG_STAT_MULTICAST(priv));
	waste = readl(RXREG_STAT_BROADCAST(priv));
	waste = readl(RXREG_STAT_MACTYPE(priv));
	waste = readl(RXREG_STAT_ALIGN_ERR(priv));
	waste = readl(RXREG_STAT_BYTES_LO(priv));
	waste = readl(RXREG_STAT_BYTES_HI(priv));

	waste = readl(TXREG_STAT_UNDERRUN(priv));
	waste = readl(TXREG_STAT_DEFERRED(priv));
	waste = readl(TXREG_STAT_PAUSE(priv));
	waste = readl(TXREG_STAT_PACKET_OK(priv));
	waste = readl(TXREG_STAT_UNDERSIZE(priv));
	waste = readl(TXREG_STAT_BYTES_LO(priv));
	waste = readl(TXREG_STAT_BYTES_HI(priv));
	waste = readl(TXREG_STAT_LATECOLL(priv));
	waste = readl(TXREG_STAT_EXCECOLL(priv));
	waste = readl(TXREG_STAT_EXCEDEFERRED(priv));
	waste = readl(TXREG_STAT_COLLISION_LIMIT(priv));
}

static int
enable_rx_tx(struct net_device *device)
{
	struct femac_dev *priv = netdev_priv(device);
	unsigned long rxcfg;
	unsigned long txcfg;

	rxcfg = (RX_CONF_STRIPCRC |
		 RX_CONF_RXFCE    |
		 RX_CONF_TXFCE);

	txcfg = (TX_CONF_ENABLE_SWAP_SA |
		 TX_CONF_APP_CRC_ENABLE |
		 TX_CONF_IFG(0xf)       |
		 TX_CONF_PAD_ENABLE);

	/* Setup the receive and transmit configuration registers according to
	 * status from PHY.
	 */
	if (priv->phy_dev->speed == SPEED_100) {
		rxcfg |= RX_CONF_SPEED;
		txcfg |= TX_CONF_SPEED;
	}
	if (priv->phy_dev->duplex == DUPLEX_FULL) {
		rxcfg |= RX_CONF_DUPLEX;
		txcfg |= TX_CONF_DUPLEX;
	}
	if (priv->phy_dev->link) {
		rxcfg |= (RX_CONF_ENABLE | RX_CONF_LINK);
		txcfg |= (TX_CONF_LINK | TX_CONF_ENABLE);
	}

	writel(rxcfg, RXREG_CONF(priv));
	writel(txcfg, TXREG_CONF(priv));

	if (priv->phy_dev->link) {
		netif_start_queue(device);
		netif_carrier_on(device);
	} else {
		netif_carrier_off(device);
		netif_stop_queue(device);
	}

	return 0;
}

static void
disable_rx_tx(struct femac_dev *priv)
{
	unsigned long txcfg;
	unsigned long rxcfg;

	rxcfg = readl(RXREG_CONF(priv));
	rxcfg &= ~RX_CONF_ENABLE;
	writel(rxcfg, RXREG_CONF(priv));

	txcfg = readl(TXREG_CONF(priv));
	txcfg &= ~TX_CONF_ENABLE;
	writel(txcfg, TXREG_CONF(priv));
}

static ssize_t
femac_show_counters(struct device *dev,
		    struct device_attribute *attr,
		    char *buf)
{
	struct net_device *ndev = container_of(dev, struct net_device, dev);
	struct femac_dev *priv = netdev_priv(ndev);
	ssize_t n = 0;

	n += snprintf(&buf[n], PAGE_SIZE,
			"tx_interrupt: %lu\n"
			"rx_interrupt: %lu\n"
			"tx_nodesc: %lu\n"
			"tx_nobuf: %lu\n",
			priv->counters.tx_interrupt,
			priv->counters.rx_interrupt,
			priv->counters.tx_nodesc,
			priv->counters.tx_nobuf);

	n += snprintf(&buf[n], PAGE_SIZE,
		      "rx_queue: %u.%u / %u.%u / %u.%u\n",
		      dmaptr_gen(priv->rxq->tail),
		      dmaptr_idx(priv->rxq->tail),
		      dmaptr_gen(le32_to_cpu(priv->rxq->hw_tail)),
		      dmaptr_idx(le32_to_cpu(priv->rxq->hw_tail)),
		      dmaptr_gen(priv->rxq->head),
		      dmaptr_idx(priv->rxq->head));

	n += snprintf(&buf[n], PAGE_SIZE,
		      "tx_queue: %u.%u / %u.%u / %u.%u\n",
		      dmaptr_gen(priv->txq->tail),
		      dmaptr_idx(priv->txq->tail),
		      dmaptr_gen(le32_to_cpu(priv->txq->hw_tail)),
		      dmaptr_idx(le32_to_cpu(priv->txq->hw_tail)),
		      dmaptr_gen(priv->txq->head),
		      dmaptr_idx(priv->txq->head));

	return n;
}
static DEVICE_ATTR(counters, S_IRUSR, femac_show_counters, NULL);

static void
set_macaddr(struct femac_dev *priv, u8 *addr)
{
	writel((addr[4] << 8) | addr[5], TXREG_SOURCE_ADDRESS_2(priv));
	writel((addr[2] << 8) | addr[3], TXREG_SOURCE_ADDRESS_1(priv));
	writel((addr[0] << 8) | addr[1], TXREG_SOURCE_ADDRESS_0(priv));
}

/**
 * femac_adjust_link - Called by the PHY driver to update MAC with changes in
 * link state.
 */
static void
femac_adjust_link(struct net_device *ndev)
{
	struct femac_dev *priv = netdev_priv(ndev);
	struct phy_device *phy_dev = priv->phy_dev;
	int status_change = 0;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);

	/* Link on or off change */
	if (phy_dev->link != priv->link) {
		priv->link = phy_dev->link;
		if (phy_dev->link)
			enable_rx_tx(ndev);
		else
			disable_rx_tx(priv);
		status_change = 1;
	}

	spin_unlock_irqrestore(&priv->lock, flags);
	if (status_change)
		phy_print_status(phy_dev);
}

/**
 * alloc_rx_buf - Initialize a dma descritor with a new rx buffer.
 */
static int
alloc_rx_buf(struct femac_dev *priv, struct dma_desc *d)
{
	struct sk_buff *skb;
	dma_addr_t dma_addr;

	skb = netdev_alloc_skb(priv->ndev, MAX_FRAME_SIZE);
	if (!skb)
		return -ENOMEM;
	dma_addr = dma_map_single(priv->dev, skb->data,
				  MAX_FRAME_SIZE, DMA_FROM_DEVICE);
	if (dma_mapping_error(priv->dev, dma_addr)) {
		dev_kfree_skb_any(skb);
		return -ENOMEM;
	}
	d->flags   = cpu_to_le32(DMADESC_WRITE | DMADESC_INTR);
	d->buf_len = cpu_to_le16(MAX_FRAME_SIZE);
	d->pdu_len = d->buf_len;
	d->buf_ptr = cpu_to_le32((u32)dma_addr);
	d->cookie  = (u32) skb;
	return 0;
}

static void
femac_tx_complete(struct femac_dev *priv)
{
	struct dma_desc *desc;
	int complete = 0;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	while ((desc = queue_get_tail(priv->tx_ring, priv->txq)) != NULL) {
		void *buf = (void *)desc->cookie;
		dma_pool_free(priv->tx_pool, buf, le32_to_cpu(desc->buf_ptr));
		queue_inc_tail(priv->txq);
		pr_queue("femac: TX complete", priv->txq);
		++complete;
	}
	spin_unlock_irqrestore(&priv->lock, flags);

	if (complete)
		netif_wake_queue(priv->ndev);
}

static int
femac_rx_packets(struct femac_dev *priv, int max)
{
	struct sk_buff *skb;
	struct dma_desc *desc;
	int num_rx = 0;

	while (num_rx < max) {
		desc = queue_get_tail(priv->rx_ring, priv->rxq);
		if (!desc)
			break;
		queue_inc_tail(priv->rxq);

		pr_debug(DRVNAME " (RX) desc=%p flags=%#x len=%u/%u buf=%#x cookie=%#x\n",
			 desc,
			 le32_to_cpu(desc->flags),
			 le16_to_cpu(desc->buf_len),
			 le16_to_cpu(desc->pdu_len),
			 le32_to_cpu(desc->buf_ptr),
			 desc->cookie);

		dma_unmap_single(priv->dev, le32_to_cpu(desc->buf_ptr),
				 MAX_FRAME_SIZE, DMA_FROM_DEVICE);
		skb = (struct sk_buff *) desc->cookie;

		if (!(le32_to_cpu(desc->flags) & DMADESC_ERROR)) {
			/* No error, pass sk_buff to upper layer */
			skb_put(skb, le16_to_cpu(desc->buf_len));
			skb->protocol = eth_type_trans(skb, priv->ndev);
			netif_receive_skb(skb);
		} else {
			struct net_device_stats *s = &priv->ndev->stats;

			/* Error, free skb and update counters */
			dev_kfree_skb_any(skb);

			s->rx_fifo_errors +=
				readl(RXREG_STAT_OVERFLOW(priv));
			s->rx_crc_errors +=
				readl(RXREG_STAT_CRC_ERR(priv));
			s->rx_frame_errors +=
				readl(RXREG_STAT_ALIGN_ERR(priv));
		}

		/* Add new RX buffers */
		desc = queue_get_head(priv->rx_ring, priv->rxq);
		if (alloc_rx_buf(priv, desc)) {
			dev_warn(priv->dev, "femac: Failed to alloc RX buffer\n");
			break;
		}
		/* Writes to desc must complete before head is advanced */
		wmb();
		writel(queue_inc_head(priv->rxq), DMAREG_RX_HEAD(priv));

		++num_rx;
	}

	return num_rx;
}

static irqreturn_t
femac_isr(int irq, void *device_id)
{
	struct femac_dev *priv = (struct femac_dev *) device_id;
	u32 int_status, int_enable;

	/* Read interrupt status */
	int_status = readl(DMAREG_INT_STATUS(priv));
	int_enable = readl(DMAREG_INT_ENABLE(priv));
	int_status &= int_enable;

	if ((int_status & (DMA_INT_TX | DMA_INT_RX)) == 0)
		return IRQ_NONE;

	if (int_status & DMA_INT_TX)
		DBG_INC(priv, tx_interrupt);
	else
		DBG_INC(priv, rx_interrupt);

	if (napi_schedule_prep(&priv->napi)) {
		/* Disable interrupts */
		writel(0, DMAREG_INT_ENABLE(priv));
		__napi_schedule(&priv->napi);
	} else {
		/* Clear interrupt status */
		writel(0, DMAREG_INT_STATUS(priv));
	}

	return IRQ_HANDLED;
}

static int
femac_poll(struct napi_struct *napi, int budget)
{
	struct femac_dev *priv = napi_to_priv(napi);
	int rcvd;

	/* Clear interrupt status */
	writel(0, DMAREG_INT_STATUS(priv));

	WARN_ON(priv->rxq->head != readl(DMAREG_RX_HEAD(priv)));

	/* Clean TX ring */
	femac_tx_complete(priv);

	/* Process rx_ring */
	rcvd = femac_rx_packets(priv, budget);
	if (rcvd < budget) {
		/* Exit polling mode */
		napi_complete(napi);
		/* Re-enable receive interrupts */
		writel(DMA_INT_RX | DMA_INT_TX, DMAREG_INT_ENABLE(priv));
	}

	return rcvd;
}

/**
 * femac_open - Opens the interface.
 *
 * The interface is opened whenever ifconfig activates it.  The open method
 * should register any system resource it needs (I/O ports, IRQ, DMA, etc.)
 * turn on the hardware, and increment the module usage count.
 */
static int
femac_open(struct net_device *ndev)
{
	struct femac_dev *priv = netdev_priv(ndev);
	struct device_node *phy_np;
	struct phy_device *phy_dev;
	int rc;

	phy_np = of_parse_phandle(priv->dev->of_node, "phy-handle", 0);
	if (!phy_np) {
		dev_err(&ndev->dev, "Missing phy-handle\n");
		return -ENODEV;
	}

	phy_dev = of_phy_connect(ndev, phy_np, femac_adjust_link,
				 0, PHY_INTERFACE_MODE_MII);
	if (IS_ERR_OR_NULL(phy_dev)) {
		dev_err(&ndev->dev, "Could not attach to PHY\n");
		return -ENODEV;
	}

	phy_dev->advertising = phy_dev->supported;
	priv->link = 0;
	priv->phy_dev = phy_dev;

	dev_info(&ndev->dev, "[%s] (phy_addr=%s, irq=%d)\n",
		 phy_dev->drv->name, dev_name(&phy_dev->dev), phy_dev->irq);

	napi_enable(&priv->napi);

	/* Install the interrupt handlers. */
	rc = request_irq(ndev->irq, femac_isr, 0, DRVNAME, priv);
	if (rc) {
		dev_err(&ndev->dev, "Request IRQ%d failed\n", ndev->irq);
		return rc;
	}

	/* enable interrupts */
	writel(DMA_INT_RX | DMA_INT_TX, DMAREG_INT_ENABLE(priv));

	phy_start(priv->phy_dev);

	return 0;
}

/**
 * femac_stop - Stops the interface.
 *
 * The interface is stopped when it is brought down; operations performed at
 * open time should be reversed.
 */
static int
femac_stop(struct net_device *device)
{
	struct femac_dev *priv = netdev_priv(device);

	/* Indicate to the OS that no more packets should be sent.  */
	netif_stop_queue(device);

	phy_disconnect(priv->phy_dev);

	/* Stop the receiver and transmitter.  */
	disable_rx_tx(priv);

	/* Disable NAPI. */
	napi_disable(&priv->napi);

	/* Free the interrupts.  */
	free_irq(device->irq, priv);

	return 0;

}

/**
 * femac_hard_start_xmit - The method initiates the transmission of a packet.
 *
 * The full packet (protocol headers and all) is contained in a socket buffer
 * (sk_buff) structure.
 *
 * Since the FEMAC DMA engine needs 32-bit aligned data, we allocate a new TX
 * buffer from a dma_pool and copy the payload before passing it on to the
 * hardware.
 */
static int
femac_hard_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct femac_dev *priv = netdev_priv(ndev);
	struct dma_desc *desc;
	void *tx_buf;
	dma_addr_t dma_addr;
	unsigned long flags;

	if (skb->len > MAX_FRAME_SIZE) {
		++ndev->stats.tx_dropped;
		goto drop;
	}

	tx_buf = dma_pool_alloc(priv->tx_pool, GFP_ATOMIC, &dma_addr);
	if (!tx_buf) {
		netif_stop_queue(ndev);
		DBG_INC(priv, tx_nobuf);
		++ndev->stats.tx_dropped;
		dev_err_ratelimited(&ndev->dev, "No TX buffers!\n");
		goto drop;
	}

	skb_copy_from_linear_data(skb, tx_buf, skb->len);

	spin_lock_irqsave(&priv->lock, flags);

	desc = queue_get_head(priv->tx_ring, priv->txq);
	pr_debug(DRVNAME " (TX) desc=%p len=%u\n", desc, skb->len);
	if (!desc) {
		DBG_INC(priv, tx_nodesc);
		spin_unlock_irqrestore(&priv->lock, flags);
		dma_pool_free(priv->tx_pool, tx_buf, dma_addr);
		netif_stop_queue(ndev);
		dev_kfree_skb_any(skb);
		++ndev->stats.tx_dropped;
		dev_err_ratelimited(&ndev->dev, "No TX descriptors!\n");
		return NETDEV_TX_BUSY;
	}

	desc->flags = cpu_to_le32(DMADESC_WRITE | DMADESC_INTR |
				  DMADESC_SOP | DMADESC_EOP);
	desc->buf_ptr = cpu_to_le32((u32)dma_addr);
	desc->buf_len = cpu_to_le16(skb->len);
	desc->pdu_len = desc->buf_len;
	desc->cookie  = (u32) tx_buf;
	/* Make sure writes to descriptor completed before starting TX */
	wmb();
	WARN_ON(priv->txq->head != readl(DMAREG_TX_HEAD(priv)));
	writel(queue_inc_head(priv->txq), DMAREG_TX_HEAD(priv));
	ndev->trans_start = jiffies;
	pr_queue("XMIT", priv->txq);

	spin_unlock_irqrestore(&priv->lock, flags);

drop:
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

/**
 * femac_get_stats - Read hardware counters.
 *
 * Whenever an application needs to get statistics for the interface, this
 * method is called.
 */
static struct net_device_stats *
femac_get_stats(struct net_device *ndev)
{
	struct femac_dev *priv = netdev_priv(ndev);
	struct net_device_stats	*s = &ndev->stats;

	s->rx_packets       += readl(RXREG_STAT_PACKET_OK(priv));
	s->tx_packets       += readl(TXREG_STAT_PACKET_OK(priv));
	s->rx_bytes         += readl(RXREG_STAT_BYTES_LO(priv));
	s->tx_bytes         += readl(TXREG_STAT_BYTES_LO(priv));
	s->multicast        += readl(RXREG_STAT_MULTICAST(priv));
	s->multicast        += readl(RXREG_STAT_BROADCAST(priv));
	s->collisions       += readl(TXREG_STAT_LATECOLL(priv));
	s->collisions       += readl(TXREG_STAT_EXCECOLL(priv));
	s->rx_length_errors += readl(RXREG_STAT_UNDERSIZE(priv));
	s->rx_length_errors += readl(RXREG_STAT_OVERSIZE(priv));
	s->tx_fifo_errors   += readl(TXREG_STAT_UNDERRUN(priv));

	s->rx_errors =
		(s->rx_length_errors +
		 s->rx_fifo_errors +
		 s->rx_crc_errors +
		 s->rx_frame_errors);

	s->tx_errors =
		(s->tx_fifo_errors +
		 s->tx_aborted_errors);

	return &ndev->stats;
}

/**
 * femac_set_mac_address
 */
static int
femac_set_mac_address(struct net_device *ndev, void *data)
{
	struct femac_dev *priv = netdev_priv(ndev);
	struct sockaddr *addr = data;

	if (netif_running(ndev))
		return -EBUSY;

	memcpy(ndev->dev_addr, addr->sa_data, ndev->addr_len);
	set_macaddr(priv, addr->sa_data);

	return 0;
}

static const struct net_device_ops femac_netdev_ops = {
	.ndo_open            = femac_open,
	.ndo_stop            = femac_stop,
	.ndo_get_stats       = femac_get_stats,
	.ndo_set_mac_address = femac_set_mac_address,
	.ndo_start_xmit      = femac_hard_start_xmit,
};


/* Ethtool operations */

static int
femac_get_settings(struct net_device *ndev, struct ethtool_cmd *cmd)
{
	struct femac_dev *priv = netdev_priv(ndev);

	if (!priv->phy_dev)
		return -ENODEV;

	return phy_ethtool_gset(priv->phy_dev, cmd);
}

static int
femac_set_settings(struct net_device *ndev, struct ethtool_cmd *cmd)
{
	struct femac_dev *priv = netdev_priv(ndev);

	if (!priv->phy_dev)
		return -ENODEV;

	return phy_ethtool_sset(priv->phy_dev, cmd);
}

static const struct ethtool_ops femac_ethtool_ops = {
	.get_settings = femac_get_settings,
	.set_settings = femac_set_settings
};

/**
 * femac_init - Allocate memory and initialize the hardware
 */
static int
femac_init(struct net_device *ndev)
{
	struct femac_dev *priv = netdev_priv(ndev);
	dma_addr_t dma_phys;
	int i;

	/* Reset the MAC */
	writel(0x80000000, DMAREG_PCI_CONTROL(priv));

	/* The number of rx and tx descriptors must be an even multiple of
	 * DESCRIPTOR_GRANULARITY.
	 */
	priv->rx_num_desc = ALIGN(rx_num_desc, DESCRIPTOR_GRANULARITY);
	priv->tx_num_desc = ALIGN(tx_num_desc, DESCRIPTOR_GRANULARITY);

	/* This needs to be set to something sane for dma_alloc_coherent() */
	ndev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	ndev->dev.dma_mask = &ndev->dev.coherent_dma_mask;

	priv->tx_pool = dma_pool_create("femac_dma", priv->dev,
					   MAX_FRAME_SIZE, 4, 0);
	if (!priv->tx_pool) {
		dev_err(&ndev->dev, "Failed to allocate DMA buffer pool\n");
		return -ENOMEM;
	}

	spin_lock_init(&priv->lock);

	/* Take MAC out of reset */
	writel(0x0, RXREG_SOFT_RESET(priv));
	writel(0x1, RXREG_MODE(priv));
	writel(0x0, TXREG_SOFT_RESET(priv));
	writel(0x1, TXREG_MODE(priv));
	writel(0x300a, TXREG_WATERMARK(priv));

	writel(0x1, TXREG_HALF_DUPLEX_CONF(priv));
	writel(0xffff, TXREG_TIME_VALUE_CONF(priv));
	writel(0x1, TXREG_INT_CONTROL(priv));
	writel(0x5275, TXREG_EXTENDED_CONF(priv));
	writel(0x1, RXREG_EIC(priv));
	writel(0x40010000, DMAREG_PCI_CONTROL(priv));
	writel(0x30000, DMAREG_CONTROL(priv));
	writel(0x280044, priv->dma_base + 0x60);
	writel(0xc0, priv->dma_base + 0x64);

	set_macaddr(priv, ndev->dev_addr);

	/* Initialize RX queue */
	priv->rx_ring_size = priv->rx_num_desc * sizeof(struct dma_desc);
	priv->rx_ring = dma_alloc_coherent(&ndev->dev, priv->rx_ring_size,
					   &priv->rx_ring_phys, GFP_KERNEL);
	priv->rxq = dma_alloc_coherent(&ndev->dev, 2 * sizeof(struct queue_ptr),
				       &dma_phys, GFP_KERNEL);
	priv->rxq->phys = dma_phys;
	writel(priv->rx_ring_phys, DMAREG_RX_QUEUE_BASE(priv));
	writel(priv->rx_ring_size/1024, DMAREG_RX_QUEUE_SIZE(priv));
	writel(priv->rxq->phys, DMAREG_RX_TAIL_ADDR(priv));
	priv->rxq->size = priv->rx_ring_size;
	priv->rxq->hw_tail = readl(DMAREG_RX_TAIL(priv));
	priv->rxq->tail    = priv->rxq->hw_tail;
	priv->rxq->head    = priv->rxq->hw_tail;
	writel(priv->rxq->head, DMAREG_RX_HEAD(priv));
	pr_debug("femac: rx_ring %p rxq %p\n", priv->rx_ring, priv->rxq);

	/* Initialize the descriptors */
	for (i = 0; i < priv->rx_num_desc-1; ++i) {
		struct dma_desc *desc  = &priv->rx_ring[i];
		alloc_rx_buf(priv, desc);
		writel(queue_inc_head(priv->rxq), DMAREG_RX_HEAD(priv));
	}

	/* Initialize TX queue */
	priv->tx_ring_size = priv->tx_num_desc * sizeof(struct dma_desc);
	priv->tx_ring = dma_alloc_coherent(&ndev->dev, priv->tx_ring_size,
					   &priv->tx_ring_phys, GFP_KERNEL);
	priv->txq = &priv->rxq[1];
	priv->txq->phys = dma_phys + sizeof(struct queue_ptr);
	writel(priv->tx_ring_phys, DMAREG_TX_QUEUE_BASE(priv));
	writel(priv->tx_ring_size/1024, DMAREG_TX_QUEUE_SIZE(priv));
	writel(priv->txq->phys, DMAREG_TX_TAIL_ADDR(priv));
	priv->txq->size    = priv->tx_ring_size;
	priv->txq->hw_tail = readl(DMAREG_RX_TAIL(priv));
	priv->txq->head    = priv->txq->hw_tail;
	priv->txq->tail    = priv->txq->hw_tail;
	writel(priv->txq->head, DMAREG_TX_HEAD(priv));
	pr_debug("femac: tx_ring %p txq %p\n", priv->tx_ring, priv->txq);

	/* Clear statistics */
	clear_statistics(priv);

	ether_setup(ndev);
	ndev->netdev_ops = &femac_netdev_ops;
	ndev->watchdog_timeo = msecs_to_jiffies(1000);
	SET_ETHTOOL_OPS(ndev, &femac_ethtool_ops);
	memset(&priv->napi, 0, sizeof(struct napi_struct));
	netif_napi_add(ndev, &priv->napi, femac_poll, NAPI_WEIGHT);

	return 0;
}

/**
 * femac_probe - Create, initalize and register the net_device.
 */
static int
femac_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct net_device *ndev = NULL;
	struct femac_dev *priv = NULL;
	struct resource *res;
	const u32 *field;
	int length;
	int rc = 0;

	/* Allocate space for the net_device. */
	ndev = alloc_etherdev(sizeof(*priv));
	if (!ndev) {
		rc = -ENOMEM;
		goto out;
	}

	priv = netdev_priv(ndev);
	priv->ndev = ndev;
	priv->dev = &pdev->dev;
	strcpy(ndev->name, "eth%d");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->rx_base  = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->rx_base)) {
		rc = PTR_ERR(priv->rx_base);
		goto out;
	}
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	priv->tx_base  = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->tx_base)) {
		rc = PTR_ERR(priv->tx_base);
		goto out;
	}
	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	priv->dma_base  = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->dma_base)) {
		rc = PTR_ERR(priv->dma_base);
		goto out;
	}

	ndev->irq = platform_get_irq(pdev, 0);
	if (ndev->irq < 0) {
		rc = ndev->irq;
		goto out;
	}

	/* Check for mac address property in device-tree */
	field = of_get_property(np, "mac-address", &length);
	if (!field)
		field = of_get_property(np, "local-mac-address", &length);
	if (field && length == ETH_ALEN)
		ether_addr_copy(priv->mac_addr, (const u8 *)field);
	/* MAC address may be overridden via module/cmdline parameter */
	if (is_valid_ether_addr(macaddr))
		ether_addr_copy(priv->mac_addr, macaddr);
	/* Still no valid MAC address, randomize it */
	if (!is_valid_ether_addr(priv->mac_addr))
		random_ether_addr(priv->mac_addr);

	ether_addr_copy(ndev->dev_addr, &priv->mac_addr[0]);
	ether_addr_copy(ndev->perm_addr, &priv->mac_addr[0]);
	ndev->addr_len = ETH_ALEN;

	/* Initialize the device. */
	rc = femac_init(ndev);
	if (rc != 0)
		goto out;

	/* Register the device. */
	rc = register_netdev(ndev);
	if (rc != 0)
		goto out;

	device_create_file(&ndev->dev, &dev_attr_counters);

	netif_carrier_off(ndev);

out:
	if (rc) {
		dev_err(&pdev->dev, "Failed to initialize, error %d\n", rc);
		if (ndev)
			free_netdev(ndev);
	}
	return rc;
}

/**
 * femac_remove - Handle device removal.
 */
static int
femac_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct femac_dev *priv = netdev_priv(ndev);

	device_remove_file(&ndev->dev, &dev_attr_counters);
	dma_pool_destroy(priv->tx_pool);
	dma_free_coherent(&ndev->dev,
			  priv->rx_ring_size,
			  priv->rx_ring,
			  priv->rx_ring_phys);
	dma_free_coherent(&ndev->dev,
			  priv->tx_ring_size,
			  priv->tx_ring,
			  priv->tx_ring_phys);
	dma_free_coherent(&ndev->dev,
			  2 * sizeof(struct queue_ptr),
			  priv->rxq,
			  priv->rxq->phys);
	unregister_netdev(ndev);
	free_netdev(ndev);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int
femac_suspend(struct device *dev)
{
	return -ENOSYS;
}

static int
femac_resume(struct device *dev)
{
	return -ENOSYS;
}

static const struct dev_pm_ops femac_pm_ops = {
	.suspend = femac_suspend,
	.resume  = femac_resume,
};
#endif

static const struct of_device_id femac_id_table[] = {
	{ .compatible = "lsi,femac" },
	{ }
};
MODULE_DEVICE_TABLE(of, femac_id_table);

static struct platform_driver femac_driver = {
	.driver	= {
		.name  = DRVNAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &femac_pm_ops,
#endif
		.of_match_table = femac_id_table,
	},
	.probe = femac_probe,
	.remove	= femac_remove,
};

module_platform_driver(femac_driver);

MODULE_DESCRIPTION("LSI FEMAC Ethernet Driver");
MODULE_LICENSE("GPL");
