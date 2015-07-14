/* Applied Micro X-Gene SoC Ethernet Driver
 *
 * Copyright (c) 2014, Applied Micro Circuits Corporation
 * Authors: Iyappan Subramanian <isubramanian@apm.com>
 *	    Keyur Chudgar <kchudgar@apm.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "if_xge_bsd_compat.h"
#include "if_xge_var.h"

#include "xgene_enet_main.h"
#include "xgene_enet_hw.h"
#include "xgene_enet_xgmac.h"

static void xgene_enet_wr_csr(struct xgene_enet_pdata *pdata,
			      u32 offset, u32 val)
{
	bus_space_handle_t addr = pdata->eth_csr_addr + offset;

	ENET_CSR_WRITE32(pdata, addr, val);
}

static void xgene_enet_wr_ring_if(struct xgene_enet_pdata *pdata,
				  u32 offset, u32 val)
{
	bus_space_handle_t addr = pdata->eth_ring_if_addr + offset;

	ENET_CSR_WRITE32(pdata, addr, val);
}

static void xgene_enet_wr_diag_csr(struct xgene_enet_pdata *pdata,
				   u32 offset, u32 val)
{
	bus_space_handle_t addr = pdata->eth_diag_csr_addr + offset;

	ENET_CSR_WRITE32(pdata, addr, val);
}

static bool xgene_enet_wr_indirect(struct xgene_enet_pdata *pdata,
    bus_space_handle_t addr, bus_space_handle_t wr,
    bus_space_handle_t cmd, bus_space_handle_t cmd_done,
    u32 wr_addr, u32 wr_data)
{
	u32 done;
	u8 wait = 10;

	ENET_CSR_WRITE32(pdata, addr, wr_addr);
	ENET_CSR_WRITE32(pdata, wr, wr_data);
	ENET_CSR_WRITE32(pdata, cmd, XGENE_ENET_WR_CMD);

	/* wait for write command to complete */
	while (!(done = ENET_CSR_READ32(pdata, cmd_done)) && wait--)
		udelay(1);

	if (!done)
		return false;

	ENET_CSR_WRITE32(pdata, cmd, 0);

	return true;
}

static void xgene_enet_wr_mac(struct xgene_enet_pdata *pdata,
			      u32 wr_addr, u32 wr_data)
{
	bus_space_handle_t addr, wr, cmd, cmd_done;

	addr = pdata->mcx_mac_addr + MAC_ADDR_REG_OFFSET;
	wr = pdata->mcx_mac_addr + MAC_WRITE_REG_OFFSET;
	cmd = pdata->mcx_mac_addr + MAC_COMMAND_REG_OFFSET;
	cmd_done = pdata->mcx_mac_addr + MAC_COMMAND_DONE_REG_OFFSET;

	if (!xgene_enet_wr_indirect(pdata, addr, wr, cmd, cmd_done, wr_addr, wr_data))
		netdev_err(pdata->ndev, "MCX mac write failed, addr: %04x\n",
			   wr_addr);
}

static void xgene_enet_rd_csr(struct xgene_enet_pdata *pdata,
			      u32 offset, u32 *val)
{
	bus_space_handle_t addr = pdata->eth_csr_addr + offset;

	*val = ENET_CSR_READ32(pdata, addr);
}

static void xgene_enet_rd_diag_csr(struct xgene_enet_pdata *pdata,
				   u32 offset, u32 *val)
{
	bus_space_handle_t addr = pdata->eth_diag_csr_addr + offset;

	*val = ENET_CSR_READ32(pdata, addr);
}

static bool xgene_enet_rd_indirect(struct xgene_enet_pdata *pdata,
    bus_space_handle_t addr, bus_space_handle_t rd, bus_space_handle_t cmd,
    bus_space_handle_t cmd_done, u32 rd_addr, u32 *rd_data)
{
	u32 done;
	u8 wait = 10;

	ENET_CSR_WRITE32(pdata, addr, rd_addr);
	ENET_CSR_WRITE32(pdata, cmd, XGENE_ENET_RD_CMD);

	/* wait for read command to complete */
	while (!(done = ENET_CSR_READ32(pdata, cmd_done)) && wait--)
		udelay(1);

	if (!done)
		return false;

	*rd_data = ENET_CSR_READ32(pdata, rd);
	ENET_CSR_WRITE32(pdata, cmd, 0);

	return true;
}

static void xgene_enet_rd_mac(struct xgene_enet_pdata *pdata,
			      u32 rd_addr, u32 *rd_data)
{
	bus_space_handle_t addr, rd, cmd, cmd_done;

	addr = pdata->mcx_mac_addr + MAC_ADDR_REG_OFFSET;
	rd = pdata->mcx_mac_addr + MAC_READ_REG_OFFSET;
	cmd = pdata->mcx_mac_addr + MAC_COMMAND_REG_OFFSET;
	cmd_done = pdata->mcx_mac_addr + MAC_COMMAND_DONE_REG_OFFSET;

	if (!xgene_enet_rd_indirect(pdata, addr, rd, cmd, cmd_done, rd_addr, rd_data))
		netdev_err(pdata->ndev, "MCX mac read failed, addr: %04x\n",
			   rd_addr);
}

static int xgene_enet_ecc_init(struct xgene_enet_pdata *pdata)
{
	u32 data;
	u8 wait = 10;

	xgene_enet_wr_diag_csr(pdata, ENET_CFG_MEM_RAM_SHUTDOWN_ADDR, 0x0);
	do {
#if !defined(__FreeBSD__)
		usleep_range(100, 110);
#else
		udelay(110);
#endif
		xgene_enet_rd_diag_csr(pdata, ENET_BLOCK_MEM_RDY_ADDR, &data);
	} while ((data != 0xffffffff) && wait--);

	if (data != 0xffffffff) {
		netdev_err(pdata->ndev,
		    "Failed to release memory from shutdown\n");
		return -ENODEV;
	}

	return 0;
}

static void xgene_enet_config_ring_if_assoc(struct xgene_enet_pdata *pdata)
{
	xgene_enet_wr_ring_if(pdata, ENET_CFGSSQMIWQASSOC_ADDR, 0);
	xgene_enet_wr_ring_if(pdata, ENET_CFGSSQMIFPQASSOC_ADDR, 0);
	xgene_enet_wr_ring_if(pdata, ENET_CFGSSQMIQMLITEWQASSOC_ADDR, 0);
	xgene_enet_wr_ring_if(pdata, ENET_CFGSSQMIQMLITEFPQASSOC_ADDR, 0);
}

static void xgene_xgmac_reset(struct xgene_enet_pdata *pdata)
{
	xgene_enet_wr_mac(pdata, AXGMAC_CONFIG_0, HSTMACRST);
	xgene_enet_wr_mac(pdata, AXGMAC_CONFIG_0, 0);
}

static void xgene_xgmac_set_mac_addr(struct xgene_enet_pdata *pdata)
{
	u32 addr0, addr1;
	u8 *dev_addr = (uint8_t *)pdata->hwaddr;

	addr0 = (dev_addr[3] << 24) | (dev_addr[2] << 16) |
		(dev_addr[1] << 8) | dev_addr[0];
	addr1 = (dev_addr[5] << 24) | (dev_addr[4] << 16);

	xgene_enet_wr_mac(pdata, HSTMACADR_LSW_ADDR, addr0);
	xgene_enet_wr_mac(pdata, HSTMACADR_MSW_ADDR, addr1);
}

static void xgene_xgmac_get_mac_addr(struct xgene_enet_pdata *pdata,
    u8 *dev_addr)
{
	u32 addr0, addr1;

	xgene_enet_rd_mac(pdata, HSTMACADR_LSW_ADDR, &addr0);
	xgene_enet_rd_mac(pdata, HSTMACADR_MSW_ADDR, &addr1);

	dev_addr[0] = (addr0 >> 0);
	dev_addr[1] = (addr0 >> 8);
	dev_addr[2] = (addr0 >> 16);
	dev_addr[3] = (addr0 >> 24);
	dev_addr[4] = (addr1 >> 16);
	dev_addr[5] = (addr1 >> 24);
}

static u32 xgene_enet_link_status(struct xgene_enet_pdata *pdata)
{
	u32 data;

	xgene_enet_rd_csr(pdata, XG_LINK_STATUS_ADDR, &data);

	return data;
}

static void xgene_xgmac_init(struct xgene_enet_pdata *pdata)
{
	u32 data;

	xgene_xgmac_reset(pdata);

	xgene_enet_rd_mac(pdata, AXGMAC_CONFIG_1, &data);
	data |= HSTPPEN;
	data &= ~HSTLENCHK;
	xgene_enet_wr_mac(pdata, AXGMAC_CONFIG_1, data);

	xgene_enet_wr_mac(pdata, HSTMAXFRAME_LENGTH_ADDR, 0x06000600);
	xgene_xgmac_set_mac_addr(pdata);

	xgene_enet_rd_csr(pdata, XG_RSIF_CONFIG_REG_ADDR, &data);
	data |= CFG_RSIF_FPBUFF_TIMEOUT_EN;
	xgene_enet_wr_csr(pdata, XG_RSIF_CONFIG_REG_ADDR, data);

	xgene_enet_wr_csr(pdata, XG_CFG_BYPASS_ADDR, RESUME_TX);
	xgene_enet_wr_csr(pdata, XGENET_RX_DV_GATE_REG_0_ADDR, 0);
	xgene_enet_rd_csr(pdata, XG_ENET_SPARE_CFG_REG_ADDR, &data);
	data |= BIT(12);
	xgene_enet_wr_csr(pdata, XG_ENET_SPARE_CFG_REG_ADDR, data);
	xgene_enet_wr_csr(pdata, XG_ENET_SPARE_CFG_REG_1_ADDR, 0x82);
}

static void xgene_xgmac_rx_enable(struct xgene_enet_pdata *pdata)
{
	u32 data;

	xgene_enet_rd_mac(pdata, AXGMAC_CONFIG_1, &data);
	xgene_enet_wr_mac(pdata, AXGMAC_CONFIG_1, data | HSTRFEN);
}

static void xgene_xgmac_tx_enable(struct xgene_enet_pdata *pdata)
{
	u32 data;

	xgene_enet_rd_mac(pdata, AXGMAC_CONFIG_1, &data);
	xgene_enet_wr_mac(pdata, AXGMAC_CONFIG_1, data | HSTTFEN);
}

static void xgene_xgmac_rx_disable(struct xgene_enet_pdata *pdata)
{
	u32 data;

	xgene_enet_rd_mac(pdata, AXGMAC_CONFIG_1, &data);
	xgene_enet_wr_mac(pdata, AXGMAC_CONFIG_1, data & ~HSTRFEN);
}

static void xgene_xgmac_tx_disable(struct xgene_enet_pdata *pdata)
{
	u32 data;

	xgene_enet_rd_mac(pdata, AXGMAC_CONFIG_1, &data);
	xgene_enet_wr_mac(pdata, AXGMAC_CONFIG_1, data & ~HSTTFEN);
}

static int xgene_enet_reset(struct xgene_enet_pdata *pdata)
{
	if (!xgene_ring_mgr_init(pdata))
		return -ENODEV;

#if !defined(__FreeBSD__)
	clk_prepare_enable(pdata->clk);
	clk_disable_unprepare(pdata->clk);
	clk_prepare_enable(pdata->clk);
#endif
	xgene_enet_ecc_init(pdata);
	xgene_enet_config_ring_if_assoc(pdata);

	return 0;
}

static void xgene_enet_xgcle_bypass(struct xgene_enet_pdata *pdata,
				    u32 dst_ring_num, u16 bufpool_id)
{
	u32 cb, fpsel;

	xgene_enet_rd_csr(pdata, XCLE_BYPASS_REG0_ADDR, &cb);
	cb |= CFG_CLE_BYPASS_EN0;
	CFG_CLE_IP_PROTOCOL0_SET(&cb, 3);
	xgene_enet_wr_csr(pdata, XCLE_BYPASS_REG0_ADDR, cb);

	fpsel = xgene_enet_ring_bufnum(bufpool_id) - 0x20;
	xgene_enet_rd_csr(pdata, XCLE_BYPASS_REG1_ADDR, &cb);
	CFG_CLE_DSTQID0_SET(&cb, dst_ring_num);
	CFG_CLE_FPSEL0_SET(&cb, fpsel);
	xgene_enet_wr_csr(pdata, XCLE_BYPASS_REG1_ADDR, cb);
}

#if !defined(__FreeBSD__)
static void xgene_enet_shutdown(struct xgene_enet_pdata *pdata)
{
	clk_disable_unprepare(pdata->clk);
}

static void xgene_enet_link_state(struct work_struct *work)
{
	struct xgene_enet_pdata *pdata = container_of(to_delayed_work(work),
					 struct xgene_enet_pdata, link_work);
	struct net_device *ndev = pdata->ndev;
	u32 link_status, poll_interval;

	link_status = xgene_enet_link_status(pdata);
	if (link_status) {
		if (!netif_carrier_ok(ndev)) {
			netif_carrier_on(ndev);
			xgene_xgmac_init(pdata);
			xgene_xgmac_rx_enable(pdata);
			xgene_xgmac_tx_enable(pdata);
			netdev_info(ndev, "Link is Up - 10Gbps\n");
		}
		poll_interval = PHY_POLL_LINK_ON;
	} else {
		if (netif_carrier_ok(ndev)) {
			xgene_xgmac_rx_disable(pdata);
			xgene_xgmac_tx_disable(pdata);
			netif_carrier_off(ndev);
			netdev_info(ndev, "Link is Down\n");
		}
		poll_interval = PHY_POLL_LINK_OFF;
	}

	schedule_delayed_work(&pdata->link_work, poll_interval);
}
#endif

struct xgene_mac_ops xgene_xgmac_ops = {
	.init = xgene_xgmac_init,
	.reset = xgene_xgmac_reset,
	.rx_enable = xgene_xgmac_rx_enable,
	.tx_enable = xgene_xgmac_tx_enable,
	.rx_disable = xgene_xgmac_rx_disable,
	.tx_disable = xgene_xgmac_tx_disable,
	.set_mac_addr = xgene_xgmac_set_mac_addr,
	.get_mac_addr = xgene_xgmac_get_mac_addr,
#if !defined(__FreeBSD__)
	.link_state = xgene_enet_link_state
#else
	.link_state = xgene_enet_link_status
#endif
};

struct xgene_port_ops xgene_xgport_ops = {
	.reset = xgene_enet_reset,
	.cle_bypass = xgene_enet_xgcle_bypass,
#if !defined(__FreeBSD__)
	.shutdown = xgene_enet_shutdown,
#else
	.shutdown = NULL,
#endif
};
