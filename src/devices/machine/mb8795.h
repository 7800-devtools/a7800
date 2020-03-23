// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
#ifndef MAME_MACHINE_MB8795_H
#define MAME_MACHINE_MB8795_H

#define MCFG_MB8795_ADD(_tag, _tx_irq, _rx_irq, _tx_drq, _rx_drq)    \
	MCFG_DEVICE_ADD(_tag, MB8795, 0)                                 \
	downcast<mb8795_device *>(device)->set_irq_cb(_tx_irq, _rx_irq); \
	downcast<mb8795_device *>(device)->set_drq_cb(_tx_drq, _rx_drq);

#define MCFG_MB8795_TX_IRQ_CALLBACK(_write) \
	devcb = &mb8795_device::set_tx_irq_wr_callback(*device, DEVCB_##_write);

#define MCFG_MB8795_RX_IRQ_CALLBACK(_write) \
	devcb = &mb8795_device::set_rx_irq_wr_callback(*device, DEVCB_##_write);

#define MCFG_MB8795_TX_DRQ_CALLBACK(_write) \
	devcb = &mb8795_device::set_tx_drq_wr_callback(*device, DEVCB_##_write);

#define MCFG_MB8795_RX_DRQ_CALLBACK(_write) \
	devcb = &mb8795_device::set_rx_drq_wr_callback(*device, DEVCB_##_write);

class mb8795_device :   public device_t,
						public device_network_interface
{
public:
	mb8795_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_tx_irq_wr_callback(device_t &device, Object &&cb) { return downcast<mb8795_device &>(device).irq_tx_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_rx_irq_wr_callback(device_t &device, Object &&cb) { return downcast<mb8795_device &>(device).irq_rx_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_tx_drq_wr_callback(device_t &device, Object &&cb) { return downcast<mb8795_device &>(device).drq_tx_cb.set_callback(std::forward<Object>(cb)); }
	template <class Object> static devcb_base &set_rx_drq_wr_callback(device_t &device, Object &&cb) { return downcast<mb8795_device &>(device).drq_rx_cb.set_callback(std::forward<Object>(cb)); }

	DECLARE_ADDRESS_MAP(map, 8);

	DECLARE_READ8_MEMBER(txstat_r);
	DECLARE_WRITE8_MEMBER(txstat_w);
	DECLARE_READ8_MEMBER(txmask_r);
	DECLARE_WRITE8_MEMBER(txmask_w);
	DECLARE_READ8_MEMBER(rxstat_r);
	DECLARE_WRITE8_MEMBER(rxstat_w);
	DECLARE_READ8_MEMBER(rxmask_r);
	DECLARE_WRITE8_MEMBER(rxmask_w);
	DECLARE_READ8_MEMBER(txmode_r);
	DECLARE_WRITE8_MEMBER(txmode_w);
	DECLARE_READ8_MEMBER(rxmode_r);
	DECLARE_WRITE8_MEMBER(rxmode_w);
	DECLARE_WRITE8_MEMBER(reset_w);
	DECLARE_READ8_MEMBER(tdc_lsb_r);
	DECLARE_READ8_MEMBER(mac_r);
	DECLARE_WRITE8_MEMBER(mac_w);

	void tx_dma_w(uint8_t data, bool eof);
	void rx_dma_r(uint8_t &data, bool &eof);

protected:
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

	virtual void recv_cb(uint8_t *buf, int len) override;

private:
	enum { TIMER_TX, TIMER_RX };

	// Lifted from netbsd
	enum {
		EN_TXS_READY        = 0x80, /* ready for packet */
		EN_TXS_BUSY         = 0x40, /* receive carrier detect */
		EN_TXS_TXRECV       = 0x20, /* transmission received */
		EN_TXS_SHORTED      = 0x10, /* possible coax short */
		EN_TXS_UNDERFLOW    = 0x08, /* underflow on xmit */
		EN_TXS_COLLERR      = 0x04, /* collision detected */
		EN_TXS_COLLERR16    = 0x02, /* 16th collision error */
		EN_TXS_PARERR       = 0x01, /* parity error in tx data */

		EN_RXS_OK           = 0x80, /* packet received ok */
		EN_RXS_RESET        = 0x10, /* reset packet received */
		EN_RXS_SHORT        = 0x08, /* < minimum length */
		EN_RXS_ALIGNERR     = 0x04, /* alignment error */
		EN_RXS_CRCERR       = 0x02, /* CRC error */
		EN_RXS_OVERFLOW     = 0x01, /* receiver FIFO overflow */

		EN_TMD_COLLMASK     = 0xf0, /* collision count */
		EN_TMD_COLLSHIFT    =    4,
		EN_TMD_PARIGNORE    = 0x08, /* ignore parity */
		EN_TMD_TURBO1       = 0x04,
		EN_TMD_LB_DISABLE   = 0x02, /* loop back disabled */
		EN_TMD_DISCONTENT   = 0x01, /* disable contention (rx carrier) */

		EN_RMD_TEST         = 0x80, /* must be zero */
		EN_RMD_ADDRSIZE     = 0x10, /* reduces NODE match to 5 chars */
		EN_RMD_SHORTENABLE  = 0x08, /* "rx packets >= 10 bytes" - <? */
		EN_RMD_RESETENABLE  = 0x04, /* detect "reset" ethernet frames */
		EN_RMD_WHATRECV     = 0x03, /* controls what packets are received */
		EN_RMD_RECV_PROMISC = 0x03, /* all packets */
		EN_RMD_RECV_MULTI   = 0x02, /* accept broad/multicasts */
		EN_RMD_RECV_NORMAL  = 0x01, /* accept broad/limited multicasts */
		EN_RMD_RECV_NONE    = 0x00, /* accept no packets */

		EN_RST_RESET        = 0x80 /* reset interface */
	};

	uint8_t mac[6];
	uint8_t txbuf[2000], rxbuf[2000];
	uint8_t txstat, txmask, rxstat, rxmask, txmode, rxmode;
	uint16_t txlen, rxlen, txcount;
	bool drq_tx, drq_rx, irq_tx, irq_rx;
	emu_timer *timer_tx, *timer_rx;

	devcb_write_line irq_tx_cb, irq_rx_cb, drq_tx_cb, drq_rx_cb;

	void check_irq();
	void start_send();
	void receive();
	bool recv_is_broadcast();
	bool recv_is_me();
	bool recv_is_multicast();
	bool recv_is_local_multicast();
};

extern const device_type MB8795;
DECLARE_DEVICE_TYPE(MB8795, mb8795_device)

#endif // MAME_MACHINE_MB8795_H
