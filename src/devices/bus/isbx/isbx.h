// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Intel Multibus I/O Expansion Bus IEEE-P959 (iSBX) emulation

**********************************************************************

                   +12V       1      2       -12V
                    GND       3      4       +5V
                  RESET       5      6       MCLK
                    MA2       7      8       MPST/
                    MA1       9      10      reserved
                    MA0      11      12      MINTR1
                 /IOWRT      13      14      MINTR0
                  /IORD      15      16      MWAIT/
                    GND      17      18      +5V
                    MD7      19      20      MCS1/
                    MD6      21      22      MCS0/
                    MD5      23      24      reserved
                    MD4      25      26      TDMA
                    MD3      27      28      OPT1
                    MD2      29      30      OPT0
                    MD1      31      32      MDACK/
                    MD0      33      34      MDRQT
                    GND      35      36      +5V
                    MDE      37      38      MDF
                    MDC      39      40      MDD
                    MDA      41      42      MDB
                    MD8      43      44      MD9

**********************************************************************/

#ifndef MAME_BUS_ISBX_ISBX_SLOT_H
#define MAME_BUS_ISBX_ISBX_SLOT_H

#pragma once




//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_ISBX_SLOT_ADD(_tag, _clock, _slot_intf, _def_slot) \
	MCFG_DEVICE_ADD(_tag, ISBX_SLOT, _clock) \
	MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _def_slot, false)


#define MCFG_ISBX_SLOT_MINTR0_CALLBACK(_mintr0) \
	downcast<isbx_slot_device *>(device)->set_mintr0_callback(DEVCB_##_mintr0);

#define MCFG_ISBX_SLOT_MINTR1_CALLBACK(_mintr1) \
	downcast<isbx_slot_device *>(device)->set_mintr1_callback(DEVCB_##_mintr1);

#define MCFG_ISBX_SLOT_MDRQT_CALLBACK(_mdrqt) \
	downcast<isbx_slot_device *>(device)->set_mdrqt_callback(DEVCB_##_mdrqt);

#define MCFG_ISBX_SLOT_MWAIT_CALLBACK(_mwait) \
	downcast<isbx_slot_device *>(device)->set_mwait_callback(DEVCB_##_mwait);



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> device_isbx_card_interface

class isbx_slot_device;

class device_isbx_card_interface : public device_slot_card_interface
{
public:
	virtual uint8_t mcs0_r(address_space &space, offs_t offset) { return 0xff; }
	virtual void mcs0_w(address_space &space, offs_t offset, uint8_t data) { }
	virtual uint8_t mcs1_r(address_space &space, offs_t offset) { return 0xff; }
	virtual void mcs1_w(address_space &space, offs_t offset, uint8_t data) { }
	virtual uint8_t mdack_r(address_space &space, offs_t offset) { return 0xff; }
	virtual void mdack_w(address_space &space, offs_t offset, uint8_t data) { }
	virtual int opt0_r() { return 1; }
	virtual void opt0_w(int state) { }
	virtual int opt1_r() { return 1; }
	virtual void opt1_w(int state) { }
	virtual void tdma_w(int state) { }
	virtual void mclk_w(int state) { }

protected:
	// construction/destruction
	device_isbx_card_interface(const machine_config &mconfig, device_t &device);

	isbx_slot_device *m_slot;
};


// ======================> isbx_slot_device

class isbx_slot_device : public device_t,
							public device_slot_interface
{
public:
	// construction/destruction
	isbx_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> void set_mintr0_callback(Object &&cb) { m_write_mintr0.set_callback(std::forward<Object>(cb)); }
	template <class Object> void set_mintr1_callback(Object &&cb) { m_write_mintr1.set_callback(std::forward<Object>(cb)); }
	template <class Object> void set_mdrqt_callback(Object &&cb) { m_write_mdrqt.set_callback(std::forward<Object>(cb)); }
	template <class Object> void set_mwait_callback(Object &&cb) { m_write_mwait.set_callback(std::forward<Object>(cb)); }

	// computer interface
	DECLARE_READ8_MEMBER( mcs0_r ) { return m_card ? m_card->mcs0_r(space, offset) : 0xff; }
	DECLARE_WRITE8_MEMBER( mcs0_w ) { if (m_card) m_card->mcs0_w(space, offset, data); }
	DECLARE_READ8_MEMBER( mcs1_r ) { return m_card ? m_card->mcs1_r(space, offset) : 0xff; }
	DECLARE_WRITE8_MEMBER( mcs1_w ) { if (m_card) m_card->mcs1_w(space, offset, data); }
	DECLARE_READ8_MEMBER( mdack_r ) { return m_card ? m_card->mdack_r(space, offset) : 0xff; }
	DECLARE_WRITE8_MEMBER( mdack_w ) { if (m_card) m_card->mdack_w(space, offset, data); }
	DECLARE_READ_LINE_MEMBER( mpst_r ) { return m_card == nullptr; }
	DECLARE_READ_LINE_MEMBER( opt0_r ) { return m_card ? m_card->opt0_r() : 1; }
	DECLARE_WRITE_LINE_MEMBER( opt0_w ) { if (m_card) m_card->opt0_w(state); }
	DECLARE_READ_LINE_MEMBER( opt1_r ) { return m_card ? m_card->opt1_r() : 1; }
	DECLARE_WRITE_LINE_MEMBER( opt1_w ) { if (m_card) m_card->opt1_w(state); }
	DECLARE_WRITE_LINE_MEMBER( tdma_w ) { if (m_card) m_card->tdma_w(state); }
	DECLARE_WRITE_LINE_MEMBER( mclk_w ) { if (m_card) m_card->mclk_w(state); }

	// card interface
	DECLARE_WRITE_LINE_MEMBER( mintr0_w ) { m_write_mintr0(state); }
	DECLARE_WRITE_LINE_MEMBER( mintr1_w ) { m_write_mintr1(state); }
	DECLARE_WRITE_LINE_MEMBER( mdrqt_w ) { m_write_mdrqt(state); }
	DECLARE_WRITE_LINE_MEMBER( mwait_w ) { m_write_mwait(state); }

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override { if (m_card) get_card_device()->reset(); }

	devcb_write_line   m_write_mintr0;
	devcb_write_line   m_write_mintr1;
	devcb_write_line   m_write_mdrqt;
	devcb_write_line   m_write_mwait;

	device_isbx_card_interface *m_card;
};


// device type definition
DECLARE_DEVICE_TYPE(ISBX_SLOT, isbx_slot_device)


SLOT_INTERFACE_EXTERN( isbx_cards );


#endif // MAME_BUS_ISBX_ISBX_SLOT_H
