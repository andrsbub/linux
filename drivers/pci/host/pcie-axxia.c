/*
 * drivers/pci/host/pcie-axxia.c
 *
 * PCIe support for AXM55xx.
 *
 * Copyright (C) 2014 LSI Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/sizes.h>
#include <linux/slab.h>

/* Controller registers */
#define PCIE_CONFIG              (0x1000)
#define PCIE_STATUS              (0x1004)
#define PCIE_CORE_DEBUG          (0x1008)
#define PCIE_LOOPBACK_FAIL       (0x100C)
#define PCIE_MPAGE_U(n)          (0x1010 + (n * 8)) /* n = 0..7 */
#define   PCIE_MPAGE_FUNC(_n)    (((_n) & 0x07) << 19)
#define   PCIE_MPAGE_BUS(_n)     (((_n) & 0xff) << 11)
#define   PCIE_MPAGE_DEV(_n)     (((_n) & 0x1f) <<  6)
#define   PCIE_MPAGE_TYPE1       (1 << 5)
#define   PCIE_MPAGE_CONFIG      (1 << 4)
#define   PCIE_MPAGE_IO          (1 << 0)
#define PCIE_MPAGE_L(n)          (0x1014 + (n * 8)) /* n = 0..7 */
#define PCIE_TPAGE_BAR0(n)       (0x1050 + (n * 4)) /* n = 0..7 */
#define     PCIE_TPAGE_32        (0<<31) /* AXI 32-bit access */
#define     PCIE_TPAGE_128       (1<<31) /* AXI 128-bit access */
#define PCIE_TPAGE_BAR1(n)       (0x1070 + (n * 4)) /* n = 0..7 */
#define PCIE_TPAGE_BAR2(n)       (0x1090 + (n * 4)) /* n = 0..7 */
#define PCIE_MSG_IN_FIFO         (0x10B0)
#define PCIE_MSG_IN_FIFO_STATUS  (0x10B4)
#define PCIE_MSG_OUT             (0x10B8)
#define PCIE_TRN_ORDER_STATUS    (0x10BC)
#define PCIE_INT0_STATUS         (0x10C0)
#define PCIE_INT0_ENABLE         (0x10C4)
#define PCIE_INT0_FORCE          (0x10C8)
#define    INT0_MSI              BIT(31)
#define    INT0_SYS_ERR          BIT(30)
#define    INT0_PME_MSG_OFF      BIT(29)
#define    INT0_EP_HOT_RESET     BIT(28)
#define    INT0_INT_ASSERTED     BIT(27)
#define    INT0_INT_DEASSERTED   BIT(26)
#define    INT0_PME_TMO_ACK      BIT(25)
#define    INT0_ERR_MSG          BIT(24)
#define    INT0_MR_NO_LEN        BIT(23)
#define    INT0_AHB_STATE_ERR    BIT(22)
#define    INT0_FIFO_ERR         BIT(21)
#define    INT0_MW_RESP_ERR      BIT(20)
#define    INT0_T2A_PARITY_ERR   BIT(19)
#define    INT0_T2A_FN_INDP	 BIT(18)
#define    INT0_T2A_FN_INDP_ERR  BIT(17)
#define    INT0_T2A_IGR_ERR      BIT(16)
#define    INT0_T2A_CPL_TMO_ERR  BIT(15)
#define    INT0_REQ_PROC_ERR     BIT(14)
#define    INT0_T2A_EGR_ERR      BIT(13)
#define    INT0_LINK_FSM_ERR     BIT(12)
#define    INT0_MR_BAR_MISMATCH  BIT(11)
#define    INT0_MST_RD_PKT_DROP  BIT(10)
#define    INT0_MW_BAR_MISMATCH  BIT(9)
#define    INT0_MST_WR_PKT_DROP  BIT(8)
#define    INT0_SLV_RD_TMO       BIT(7)
#define    INT0_SLV_RD_ERR       BIT(6)
#define    INT0_MST_RD_4K_ERR    BIT(5)
#define    INT0_MSG_RCVD         BIT(4)
#define    INT0_MSG_DROP         BIT(3)
#define    INT0_CFG_WR_ID_ERR    BIT(2)
#define    INT0_CFG_WR_PKT_ERR   BIT(1)
#define    INT0_CFG_WR_TMO_ERR   BIT(0)
#define    INT0_ERROR            (INT0_SYS_ERR           | \
				  INT0_PME_MSG_OFF       | \
				  INT0_EP_HOT_RESET      | \
				  INT0_PME_TMO_ACK       | \
				  INT0_ERR_MSG           | \
				  INT0_MR_NO_LEN         | \
				  INT0_AHB_STATE_ERR     | \
				  INT0_FIFO_ERR          | \
				  INT0_MW_RESP_ERR       | \
				  INT0_T2A_PARITY_ERR    | \
				  INT0_T2A_FN_INDP       | \
				  INT0_T2A_FN_INDP_ERR   | \
				  INT0_T2A_IGR_ERR       | \
				  INT0_T2A_CPL_TMO_ERR   | \
				  INT0_REQ_PROC_ERR      | \
				  INT0_T2A_EGR_ERR       | \
				  INT0_LINK_FSM_ERR      | \
				  INT0_MR_BAR_MISMATCH   | \
				  INT0_MST_RD_PKT_DROP   | \
				  INT0_MW_BAR_MISMATCH   | \
				  INT0_MST_WR_PKT_DROP   | \
				  INT0_SLV_RD_TMO        | \
				  INT0_MST_RD_4K_ERR     | \
				  INT0_MSG_DROP          | \
				  INT0_CFG_WR_PKT_ERR    | \
				  INT0_CFG_WR_TMO_ERR)
#define PCIE_PHY_STATUS0         (0x10CC)
#define PCIE_PHY_STATUS1         (0x10D0)
#define PCIE_PHY_CONTROL0        (0x10D4)
#define PCIE_PHY_CONTROL1        (0x10D8)
#define PCIE_PHY_CONTROL2        (0x10DC)
#define PCIE_AXI_MASTER_WR       (0x10EC)
#define PCIE_LINK_STATUS         (0x117C)
#define PCIE_AXI_MSI_ADDR        (0x1190)
#define PCIE_INT1_STATUS         (0x11C4)
#define PCIE_INT1_ENABLE         (0x11C8)
#define PCIE_INT1_FORCE          (0x11CC)
#define PCIE_RC_BAR0_SIZE        (0x11F4)
#define PCIE_MSI0_STATUS         (0x1230)
#define PCIE_MSI0_ENABLE         (0x1234)
#define PCIE_MSI0_FORCE          (0x1238)
#define PCIE_MSI1_STATUS(_grp)   (0x123C+(_grp)*12)
#define PCIE_MSI1_ENABLE(_grp)   (0x1240+(_grp)*12)
#define PCIE_MSI1_FORCE(_grp)    (0x1244+(_grp)*12)

/* Every MPAGE register maps 128MB in the AXI memory range */
#define MPAGE_SIZE (128U<<20)

/* Number of IRQs allocated to MSI */
#define NUM_MSI_IRQ 256

/**
 * struct axxia_msi - MSI releated state
 * @virt: Virtual (CPU space) address for MSI table
 * @phys: Physical address for MSI table
 * @pci_addr: PCI memory space address for MSI table
 * @chip: MSI chip struct
 * @domain: IRQ domain for MSI.
 */
struct axxia_msi {
	void                *virt;
	dma_addr_t          phys;
	u32                 pci_addr;
	struct msi_chip     chip;
	struct irq_domain  *domain;
};

#define chip_to_axxia_msi(_chip) container_of((_chip), struct axxia_msi, chip)

struct axxia_pcie {
	struct device	   *dev;
	char                name[16];
	u8                  root_bus_nr;
	int                 irq[17]; /* 1 legacy, 16 (optional) MSI */
	void __iomem	    *regs;
	void __iomem	    *cfg_base;
	u32                 last_mpage;
	struct device_node  *node;
	struct resource	    *utl_regs;
	struct resource	    *cfg_space;
	/* Outbound range in (physical) CPU addresses */
	struct resource	    io;
	struct resource	    mem;
	struct resource     busn;
	u64		    io_bus_addr;
	u64		    mem_bus_addr;
	/* Inbound PCI base address */
	u64                 pci_bar;
	/* Inbound range in (physical) CPU addresses */
	struct resource	    inbound;
	struct axxia_msi    msi;
	/* Bitmap for allocated MSIs */
	DECLARE_BITMAP(msi_irq_in_use, NUM_MSI_IRQ);
};

#define axxia_msi_to_pcie(_msi) container_of((_msi), struct axxia_pcie, msi)

static inline struct axxia_pcie *
sys_to_pcie(struct pci_sys_data *sys)
{
	return sys->private_data;
}

static void
axxia_fixup_class(struct pci_dev *dev)
{
	/* if we aren't a PCIe don't bother */
	if (!pci_find_capability(dev, PCI_CAP_ID_EXP))
		return;
	/* Make the bridge transparent */
	dev->class = (PCI_CLASS_BRIDGE_PCI << 8) | 0x01;
}

DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_LSI_LOGIC, 0x5101, axxia_fixup_class);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_LSI_LOGIC, 0x5108, axxia_fixup_class);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_LSI_LOGIC, 0x5120, axxia_fixup_class);

/**
 * axxia_pcie_config_addr - Return the configuration access base address
 */
static void __iomem *
axxia_pcie_config_addr(struct pci_bus *bus, unsigned int devfn)
{
	struct axxia_pcie *pcie = sys_to_pcie(bus->sysdata);
	u32 mpage;

	if (bus->number == pcie->root_bus_nr) {
		/* Only one device/function on the root port */
		if (PCI_FUNC(devfn) != 0 || PCI_SLOT(devfn) != 0)
			return NULL;
		return pcie->regs;
	}

	/* Only one device directly attached */
	if (bus->primary == pcie->root_bus_nr && PCI_SLOT(devfn) != 0)
		return NULL;

	/* Build the mpage register */
	mpage = (PCIE_MPAGE_FUNC(PCI_FUNC(devfn)) |
		 PCIE_MPAGE_BUS(bus->number) |
		 PCIE_MPAGE_DEV(PCI_SLOT(devfn)) |
		 PCIE_MPAGE_CONFIG);
	/* Type0 on primary bus, Type1 on subordinate busses */
	if (bus->number - pcie->root_bus_nr > 1)
		mpage |= PCIE_MPAGE_TYPE1;

	if (mpage != pcie->last_mpage) {
		/* Using MPAGE #7 for configuration space access */
		writel(0, pcie->regs + PCIE_MPAGE_U(7));
		writel(mpage, pcie->regs + PCIE_MPAGE_L(7));
		pcie->last_mpage = mpage;
	}

	return pcie->cfg_base;
}

/**
 * axxia_pcie_validate_offset - Avoid bus errors from unimplemented registers
 * in the host controllers configuration space.
 */
static int
axxia_pcie_validate_offset(struct axxia_pcie *pcie, int offset, u32 *val)
{
	offset &= ~3;

	if (offset >= 0x9c || offset == 0x5c) {
		/* Return zero for unimplemented registers */
		*val = 0;
		return false;
	}
	return true;
}

/**
 * axxia_pcie_read_config - Read PCI configuration space
 */
static int
axxia_pcie_read_config(struct pci_bus *bus, unsigned int devfn,
		       int offset, int len, u32 *val)
{
	struct axxia_pcie *pcie = sys_to_pcie(bus->sysdata);
	int byte_offset = offset & 0x3;
	void __iomem *addr;
	u32 val32;

	addr = axxia_pcie_config_addr(bus, devfn);
	if (!addr) {
		*val = ~0;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	/*
	 * Addressing is different for local register access vs. access through
	 * the mempry mapped config space.
	 */
	if (bus->number == 0) {
		addr += offset & ~3;

		if (!axxia_pcie_validate_offset(pcie, offset, val))
			return PCIBIOS_SUCCESSFUL;
	} else {
		/* Configuration space access only supports 32-bit access
		 *
		 *  AXI address   [3:0] not used.
		 *  AXI address   [9:4] register number.
		 *  AXI address [13:10] external register number.
		 *  AXI address [17:14] 1st DWBE (for read)
		 *  AXI address [29:27] selects MPAGE
		 */
		addr += (offset << 2);

		switch (len) {
		case 1:
			addr +=  (1 << byte_offset) << 14;
			break;
		case 2:
			addr += (3 << byte_offset) << 14;
			break;
		default:
			addr += 0xf << 14;
			break;
		}
	}

	/* Read one 32-bit word */
	val32 = readl(addr);

	switch (len) {
	case 1:
		*val = (val32 >> (byte_offset * 8)) & 0xff;
		break;
	case 2:
		*val = (val32 >> (byte_offset * 8)) & 0xffff;
		break;
	default:
		*val = val32;
		break;
	}

	return PCIBIOS_SUCCESSFUL;
}

/**
 * axxia_pcie_write_config - Write PCI configuration space.
 */
static int
axxia_pcie_write_config(struct pci_bus *bus, unsigned int devfn,
			int offset, int len, u32 val)
{
	void __iomem *addr;
	u32 val32;

	addr = axxia_pcie_config_addr(bus, devfn);
	if (!addr)
		return PCIBIOS_DEVICE_NOT_FOUND;

	/*
	 * Addressing is different for local config access vs. access through
	 * the memory mapped config space. We need to translate the offset for
	 * mapped config access.
	 */
	if (bus->number == 0) {
		/* The local registers only supports 32-bit word access, so if
		 * this is a byte or half-word access we need to perform a
		 * read-modify write
		 */
		addr += (offset & ~3);

		if (len < 4) {
			unsigned int byte_shift = (offset & 3) * 8;
			unsigned int mask = (len == 1 ? 0xff : 0xffff);

			val32 = readl(addr);

			val32 &= ~(mask << byte_shift);
			val = val32 | ((val & mask) << byte_shift);
			len = 4;
		}
	} else {
		addr += (offset << 2) + (offset & 0x3);
	}

	switch (len) {
	case 1:
		writeb(val, addr);
		break;
	case 2:
		writew(val, addr);
		break;
	default:
		writel(val, addr);
		break;
	}

	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops axxia_pciex_pci_ops = {
	.read  = axxia_pcie_read_config,
	.write = axxia_pcie_write_config,
};

/*
 * MSI handler
 *
 * This is the handler for PCIE MSI service. It will decode the signalled MSI
 * using the following hierarchy of status bits. This handler is installed as a
 * chained handler for each of the 16 interrupt lines on the top-level
 * interrupt controller. When a pending MSI is found, this handler forwards the
 * interrupt service to the corresponding MSI IRQs
 *
 *                                             PCIE_MSI1_STATUS(group)
 *
 *                 PCIE_MSI0_STATUS                  +----------+
 *                                                   | MSI      |
 *   +----------+    +----------+                    | 0..15    |
 *   | ARM GIC  |    | GROUP    |        /-----------+          |
 *   |          +----+ 0..15    +-------/            |          |
 *   |          |    |          |                    |          |
 *   |          +----+          +-------\            +----------+
 *   |          |    |          |        \
 *   |          +----+          |         \          +----------+
 *   |          |    |          |          \         | MSI      |
 *   |          +----+          |           \        | 16..31   |
 *   |          |    |          |            \-------+          |
 *   |          +----+          |                    |          |
 *   |          |    |          |                    |          |
 *   |          |    |          |                    +----------+
 *   |          | .  |          |
 *   |          | .  |          |                    ...
 *   |          | .  |          |
 *   |          |    |          |                    +----------+
 *   |          |    |          |                    | MSI      |
 *   |          +----+          +--------------------+ 240..255 |
 *   |          |    |          |                    |          |
 *   +----------+    +----------+                    |          |
 *                                                   |          |
 *                                                   +----------+
 */

static void
pcie_msi_dispatch(u32 group, struct axxia_pcie *pcie)
{
	u32 status;

	/* Check second level interrupt status */
	status = readl(pcie->regs + PCIE_MSI1_STATUS(group)) & 0xffff;
	while (status) {
		u32 line = ffs(status) - 1;
		u32 hwirq = (group * 16) + line;
		unsigned int irq;

		/* Clear interrupt on sub-level */
		writel(BIT(line), pcie->regs + PCIE_MSI1_STATUS(group));

		irq = irq_find_mapping(pcie->msi.domain, hwirq);
		if (irq)
			generic_handle_irq(irq);
		else
			dev_warn(pcie->dev, "Unexpected MSI\n");

		status &= ~BIT(line);
	}

	/* Clear interrupt on top-level*/
	writel(1 << group, pcie->regs + PCIE_MSI0_STATUS);
}


static irqreturn_t
pcie_msi_isr(int irqno, void *_dev)
{
	struct axxia_pcie *pcie = _dev;
	u32 group = irqno - pcie->irq[1];
	u32 status;

	/* Check if interrupt is pending */
	status = readl(pcie->regs + PCIE_MSI0_STATUS);
	if (!(status & BIT(group)))
		return IRQ_NONE;

	pcie_msi_dispatch(group, pcie);

	return IRQ_HANDLED;
}

/*
 * pcie_legacy_isr
 *
 * The interrupt line for this handler is shared between the PCIE controller
 * itself (for status and error interrupts) and devices using legacy PCI
 * interupt signalling. Status and error interrupts are serviced here and this
 * handler will return IRQ_HANDLED. If the reason is the assertion of a device
 * legacy interrupt, this handler returns IRQ_NONE the next action on this line
 * will be called (the PCI device driver interrupt service routine).
 */
static irqreturn_t
pcie_legacy_isr(int irq, void *arg)
{
	struct axxia_pcie *pcie = arg;
	irqreturn_t retval = IRQ_HANDLED;
	u32 intr_status;

	/* read the interrupt status register */
	intr_status = readl(pcie->regs + PCIE_INT0_STATUS);

	if (intr_status & INT0_ERROR) {
		dev_err(pcie->dev, "PCIe error %#x\n", intr_status);
	} else if (intr_status & INT0_INT_ASSERTED) {
		/* Empty the message FIFO */
		while ((readl(pcie->regs + PCIE_MSG_IN_FIFO_STATUS) & 1) == 0)
			(void) readl(pcie->regs + PCIE_MSG_IN_FIFO);
		/* Next handler in chain will service this interrupt */
		retval = IRQ_NONE;
	} else if (intr_status & INT0_MSI) {
		u32 msi_status = readl(pcie->regs + PCIE_MSI0_STATUS);
		if (msi_status == 0) {
			retval = IRQ_NONE;
		} else {
			u32 group = ffs(msi_status) - 1;
			pcie_msi_dispatch(group, pcie);
		}
	}

	/*
	 *  We clear all the interrupts in the PEI status, even though
	 *  interrupts from external devices have not yet been handled.
	 *  That should be okay, since the PCI IRQ in the GIC won't be
	 *  re-enabled until all external handlers have been called.
	 */
	writel(intr_status, pcie->regs + PCIE_INT0_STATUS);

	return retval;
}

static int
pcie_map_outbound(struct axxia_pcie *pcie, const struct resource *res, u64 dest)
{
	int mpage = (res->start >> 27) & 7;
	int num_pages = (resource_size(res) + MPAGE_SIZE - 1) / MPAGE_SIZE;
	u32 flags = 0;
	int i;

	dev_dbg(pcie->dev, "Map %pR -> %#llx (mpage %u..%u)\n",
		res, dest, mpage, mpage + num_pages - 1);

	if (resource_type(res) == IORESOURCE_IO)
		flags |= PCIE_MPAGE_IO;

	for (i = mpage; i < mpage+num_pages; ++i) {
		u32 mpage_u = dest >> 32;
		u32 mpage_l = (u32)dest & ~(MPAGE_SIZE-1);
		writel(mpage_u, pcie->regs + PCIE_MPAGE_U(i));
		writel(mpage_l | flags, pcie->regs + PCIE_MPAGE_L(i));
		dest += MPAGE_SIZE;
	}

	return num_pages;
}

/* PCIe setup function */
static int
axxia_pcie_setup(int nr, struct pci_sys_data *sys)
{
	struct axxia_pcie *pcie = sys_to_pcie(sys);
	u32 pci_status, link_state;
	int i, err;
	u32 inbound_size;

	pcie->root_bus_nr = sys->busnr;

	sys->io_offset = pcie->io.start - pcie->io_bus_addr;
	sys->mem_offset = pcie->mem.start - pcie->mem_bus_addr;

	if (pcie->mem.start)
		pci_add_resource_offset(&sys->resources, &pcie->mem,
					sys->mem_offset);
	if (pcie->io.start)
		pci_add_resource_offset(&sys->resources, &pcie->io,
					sys->io_offset);

	pci_add_resource(&sys->resources, &pcie->busn);

	/* Status/error interrupt */
	err = devm_request_irq(pcie->dev, pcie->irq[0], pcie_legacy_isr,
			       IRQF_SHARED, "pcie", pcie);
	if (err) {
		dev_err(pcie->dev, "Failed to request IRQ#%d (%d)\n",
			pcie->irq[0], err);
		goto fail;
	}

	/* Check if the controller is in root-complex mode
	 */
	pci_status = readl(pcie->regs + PCIE_STATUS);
	if ((pci_status & 0x18) != 0x18) {
		dev_err(pcie->dev, "Device is not Root Complex\n");
		goto fail;
	}

	/* Make sure the link is up */
	link_state = (pci_status >> 8) & 0x3f;
	dev_dbg(pcie->dev, "status=0x%08x, link state=%#x\n",
		pci_status, link_state);
	if (link_state != 0xb) {
		/* Reset */
		u32 pci_config = readl(pcie->regs + PCIE_CONFIG);
		pci_config |= 1;
		writel(pci_config, pcie->regs + PCIE_CONFIG);
		dev_warn(pcie->dev, "Link in bad state - resetting\n");
		msleep(100);
		pci_status = readl(pcie->regs + PCIE_STATUS);
		link_state = (pci_status & 0x3f00) >> 8;
		dev_dbg(pcie->dev, "(after reset) link state=%#x\n",
			link_state);
		if (link_state != 0xb) {
			dev_warn(pcie->dev, "Link in bad state - give up!\n");
			goto fail;
		}
	}

	/*
	 * Setup outbound PCI Memory Windows
	 */
	pcie_map_outbound(pcie, &pcie->io, pcie->io_bus_addr);
	pcie_map_outbound(pcie, &pcie->mem, pcie->mem_bus_addr);

	/*
	 * Setup inbound PCI window
	 */

	/* Configure the inbound window size */
	inbound_size = (u32) resource_size(&pcie->inbound);
	writel(~(inbound_size-1), pcie->regs + PCIE_RC_BAR0_SIZE);

	/* Set the BASE0 address to start of PCIe base */
	writel(pcie->pci_bar, pcie->regs + PCI_BASE_ADDRESS_0);
	/* [BAR0..BAR1] is 64-bit window, clear the upper 32-bits */
	writel(0, pcie->regs + PCI_BASE_ADDRESS_1);

	/* Setup TPAGE registers for inbound mapping
	 *
	 * We set the MSB of each TPAGE to select 128-bit AXI access. For the
	 * address field we simply program an incrementing value to map
	 * consecutive pages
	 */
	for (i = 0; i < 8; i++)
		writel(PCIE_TPAGE_128 | i, pcie->regs + PCIE_TPAGE_BAR0(i));


	/* Enable all legacy/status/error interrupts */
	writel(INT0_MSI | INT0_INT_ASSERTED | INT0_ERROR,
	       pcie->regs + PCIE_INT0_ENABLE);

	/* Enable all MSI interrupt groups */
	writel(0xFFFF, pcie->regs + PCIE_MSI0_ENABLE);
	/* Enable all lines in all subgroups */
	for (i = 0; i < 16; i++)
		writel(0xFFFF, pcie->regs + PCIE_MSI1_ENABLE(i));

	return 1;
fail:
	if (pcie->cfg_base)
		iounmap(pcie->cfg_base);
	if (pcie->regs)
		iounmap(pcie->regs);
	return 0;
}

static int
axxia_pcie_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	struct axxia_pcie *pcie = sys_to_pcie(dev->sysdata);
	return pcie->irq[0];
}

static void
axxia_pcie_add_bus(struct pci_bus *bus)
{
	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		struct axxia_pcie *pcie = sys_to_pcie(bus->sysdata);
		bus->msi = &pcie->msi.chip;
	}
}

/* IRQ chip ops for MSI IRQs */
static struct irq_chip axxia_msi_irq_chip = {
	.name = "PCI-MSI",
	.irq_enable  = unmask_msi_irq,
	.irq_disable = mask_msi_irq,
	.irq_mask    = mask_msi_irq,
	.irq_unmask  = unmask_msi_irq,
};

static int
axxia_msi_map(struct irq_domain *d, unsigned int irq, irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &axxia_msi_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, d->host_data);
	set_irq_flags(irq, IRQF_VALID);
	return 0;
}

static const struct irq_domain_ops msi_domain_ops = {
	.map = axxia_msi_map
};

/*
 * Allocate a MSI interrupt number and IRQ (the IRQ is a constant offset from
 * the MSI index). The kernel IRQs for MSI are in a range above hardware IRQs.
 *
 * First call also allocates the 1Kb MSI table and configures the controller.
 */
static int
axxia_setup_msi_irq(struct msi_chip *chip, struct pci_dev *pdev,
		    struct msi_desc *desc)
{
	struct axxia_pcie *pcie = sys_to_pcie(pdev->bus->sysdata);
	struct msi_msg msg;
	int hwirq, irq;

	/* Find available MSI hwirq number */
	do {
		hwirq = find_first_zero_bit(pcie->msi_irq_in_use, NUM_MSI_IRQ);
		if (hwirq >= NUM_MSI_IRQ)
			return -ENOSPC;
	} while (test_and_set_bit(hwirq, pcie->msi_irq_in_use));

	irq = irq_create_mapping(pcie->msi.domain, hwirq);
	if (!irq) {
		dev_err(&pdev->dev, "Failed to create irq mapping\n");
		clear_bit(hwirq, pcie->msi_irq_in_use);
		return -EINVAL;
	}

	irq_set_msi_desc(irq, desc);

	/* Configure PCI device with its MSI address */
	msg.address_hi = 0x0;
	msg.address_lo = pcie->msi.pci_addr + 4 * hwirq;
	msg.data = irq;
	write_msi_msg(irq, &msg);

	return 0;
}

/*
 * Called by the generic MSI layer to free MSI IRQ.
 */
static void
axxia_teardown_msi_irq(struct msi_chip *chip, unsigned int irq)
{
	struct axxia_pcie *pcie = axxia_msi_to_pcie(chip_to_axxia_msi(chip));

	clear_bit(irq_get_irq_data(irq)->hwirq, pcie->msi_irq_in_use);
}

/*
 * Allocate MSI page. A MSI is generated when EP writes to this PCI address.
 * The region must be 1Kb to manage 256 MSIs.
 */
static int
axxia_pcie_enable_msi(struct axxia_pcie *pcie)
{
	struct axxia_msi *msi = &pcie->msi;
	u32 msi_lower;
	int i;
	int err;

	msi->chip.dev = pcie->dev;
	msi->chip.setup_irq = axxia_setup_msi_irq;
	msi->chip.teardown_irq = axxia_teardown_msi_irq;

	/* Create IRQ domain for MSI interrupts */
	msi->domain = irq_domain_add_linear(pcie->dev->of_node, NUM_MSI_IRQ,
					    &msi_domain_ops, &msi->chip);
	if (!msi->domain) {
		dev_err(pcie->dev, "failed to create IRQ domain\n");
		err = -ENOMEM;
		goto fail;
	}

	/* Hook up (up to 16) MSI interrupt handlers */
	for (i = 1; i <= 16; i++) {
		err = devm_request_irq(pcie->dev, pcie->irq[i],
				       pcie_msi_isr, 0,
				       axxia_msi_irq_chip.name, pcie);
		if (err < 0)
			break;
	}

	msi->virt = dma_alloc_coherent(pcie->dev, 1024,
				       &msi->phys, GFP_KERNEL);
	if (!msi->virt) {
		err = -ENOMEM;
		goto fail;
	}

	msi_lower = (u32)msi->phys;
	msi->pci_addr = pcie->inbound.start + msi_lower;
	writel(msi_lower>>10, pcie->regs + PCIE_AXI_MSI_ADDR);

	dev_dbg(pcie->dev, "msi_table @ %p [virt] %#x [pci]\n",
		msi->virt, msi_lower);

	return 0;
fail:
	if (msi->domain)
		irq_domain_remove(msi->domain);
	if (msi->virt)
		dma_free_coherent(pcie->dev, 1024, msi->virt, msi->phys);
	return err;
}

static int
axxia_read_dt(struct platform_device *pdev, struct axxia_pcie *pcie)
{
	struct device_node *np = pcie->node;
	struct of_pci_range_parser parser;
	struct of_pci_range range;
	int i;
	int err;

	/* Fetch address range for PCIE config space and registers */
	pcie->cfg_space = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pcie->utl_regs = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!pcie->cfg_space || !pcie->utl_regs) {
		dev_err(&pdev->dev, "missing 'reg' property\n");
		return -EINVAL;
	}

	/* Fetch interrupts (one legacy + up to 16 IRQs for MSI */
	for (i = 0; i <= 16; i++)
		pcie->irq[i] = platform_get_irq(pdev, i);

	/* Parse ranges property */
	if (of_pci_range_parser_init(&parser, np)) {
		dev_err(&pdev->dev, "missing 'ranges' property\n");
		return -EINVAL;
	}

	for_each_of_pci_range(&parser, &range) {
		switch (range.flags & IORESOURCE_TYPE_BITS) {
		case IORESOURCE_IO:
			of_pci_range_to_resource(&range, np, &pcie->io);
			pcie->io.name = "I/O";
			pcie->io_bus_addr = range.pci_addr;
			break;
		case IORESOURCE_MEM:
			of_pci_range_to_resource(&range, np, &pcie->mem);
			pcie->mem.name = "MEM";
			pcie->mem_bus_addr = range.pci_addr;
			break;
		default:
			dev_err(&pdev->dev, "bad range (%#x)\n", range.flags);
			break;
		}
	}

	err = of_pci_parse_bus_range(np, &pcie->busn);
	if (err < 0) {
		dev_warn(pcie->dev, "No 'bus-range' property, using default\n");
		pcie->busn.name = np->name;
		pcie->busn.start = 0;
		pcie->busn.end = 0xff;
		pcie->busn.flags = IORESOURCE_BUS;
	}

	/*
	 * Inbound PCI memory window
	 */

	/* Default 4GB */
	pcie->inbound.name  = "PCIE DMA";
	pcie->inbound.start = 0x00000000;
	pcie->inbound.end   = 0xffffffff;
	pcie->inbound.flags = IORESOURCE_MEM | IORESOURCE_PREFETCH;

	dev_dbg(pcie->dev, "Inbound %#llx (PCI) -> %#llx..%#llx (CPU)\n",
		pcie->pci_bar, pcie->inbound.start, pcie->inbound.end);

	return 0;
}

static const struct of_device_id axxia_pcie_of_match[] = {
	{ .compatible = "lsi,axm5516-pcie" },
	{ },
};
MODULE_DEVICE_TABLE(of, axxia_pcie_of_match);

static int
axxia_pcie_probe(struct platform_device *pdev)
{
	static int pci_domain;
	struct axxia_pcie *pcie;
	struct hw_pci hw;
	int err;

	if (dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(64)) != 0 &&
	    dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32)) != 0) {
		return -ENODEV;
	}

	pcie = devm_kzalloc(&pdev->dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	pcie->dev = &pdev->dev;
	snprintf(pcie->name, sizeof(pcie->name) - 1, "PCIE%d", pci_domain);
	pcie->node = of_node_get(pdev->dev.of_node);

	/* Parse device tree for all configuration */
	err = axxia_read_dt(pdev, pcie);
	if (err < 0)
		return err;

	if (request_resource(&iomem_resource, pcie->utl_regs)) {
		dev_err(&pdev->dev, "Registers resource request failed\n");
		return -EINVAL;
	}

	if (request_resource(&iomem_resource, pcie->cfg_space)) {
		dev_err(&pdev->dev, "Config resource request failed\n");
		return -EINVAL;
	}

	if (request_resource(&iomem_resource, &pcie->mem)) {
		dev_err(&pdev->dev, "Memory resource request failed\n");
		return -EINVAL;
	}

	/* Map PCIe bridge control registers */
	pcie->regs = devm_ioremap_resource(pcie->dev, pcie->utl_regs);
	if (!pcie->regs) {
		dev_err(&pdev->dev, "Failed to map control registers\n");
		return -ENODEV;
	}

	/* Map range for access to PCI configuration space */
	pcie->cfg_base = devm_ioremap_resource(pcie->dev, pcie->cfg_space);
	if (!pcie->cfg_base) {
		dev_err(&pdev->dev, "Failed to map config space\n");
		return -ENODEV;
	}

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		err = axxia_pcie_enable_msi(pcie);
		if (err < 0) {
			dev_err(&pdev->dev,
				"failed to enable MSI support: %d\n",
				err);
			return err;
		}
	}

	memset(&hw, 0, sizeof(hw));
	hw.nr_controllers = 1;
	hw.domain = pci_domain++;
	hw.private_data = (void **)&pcie;
	hw.setup = axxia_pcie_setup;
	hw.map_irq = axxia_pcie_map_irq;
	hw.add_bus = axxia_pcie_add_bus;
	hw.ops = &axxia_pciex_pci_ops;

	pci_common_init_dev(pcie->dev, &hw);

	return 0;
}

static struct platform_driver axxia_pcie_driver = {
	.driver = {
		.name = "axxia-pcie",
		.owner = THIS_MODULE,
		.of_match_table = axxia_pcie_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = axxia_pcie_probe
};
module_platform_driver(axxia_pcie_driver);

MODULE_DESCRIPTION("Axxia AXM55xx PCIe driver");
MODULE_LICENSE("GPLv2");
