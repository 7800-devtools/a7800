// license:BSD-3-Clause
// copyright-holders:Curt Coder
/**********************************************************************

    Luxor ABC (Databoard 4680) Bus emulation

**********************************************************************

                              ABC 80

                              A     B
                -12 V   <--   *  1  *   --> -12V
                0 V     ---   *  2  *   --- 0 V
                RESIN_  -->   *  3  *   --> XMEMWR_
                0 V     ---   *  4  *   --> XMEMFL_
                INT_    -->   *  5  *   --> phi
                D7      <->   *  6  *   --- 0 V
                D6      <->   *  7  *   --- 0 V
                D5      <->   *  8  *   --- 0 V
                D4      <->   *  9  *   --- 0 V
                D3      <->   * 10  *   --- 0 V
                D2      <->   * 11  *   --- 0 V
                D1      <->   * 12  *   --- 0 V
                D0      <->   * 13  *   --- 0 V
                              * 14  *   --> A15
                RST_    <--   * 15  *   --> A14
                IN1     <--   * 16  *   --> A13
                IN0     <--   * 17  *   --> A12
                OUT5    <--   * 18  *   --> A11
                OUT4    <--   * 19  *   --> A10
                OUT3    <--   * 20  *   --> A9
                OUT2    <--   * 21  *   --> A8
                OUT0    <--   * 22  *   --> A7
                OUT1    <--   * 23  *   --> A6
                NMI_    -->   * 24  *   --> A5
                INP2_   <--   * 25  *   --> A4
               XINPSTB_ <--   * 26  *   --> A3
              XOUTPSTB_ <--   * 27  *   --> A2
                XM_     -->   * 28  *   --> A1
                RFSH_   <--   * 29  *   --> A0
                RDY     -->   * 30  *   --> MEMRQ_
                +5 V    <--   * 31  *   --> +5 V
                +12 V   <--   * 32  *   --> +12 V

    OUT 0   _OUT    data output
    OUT 1   _CS     card select
    OUT 2   _C1     command 1
    OUT 3   _C2     command 2
    OUT 4   _C3     command 3
    OUT 5   _C4     command 4

    IN 0    _INP    data input
    IN 1    _STAT   status in
    IN 7    RST     reset

**********************************************************************

                             ABC 1600

                              A     B
                -12 V   ---   *  1  *   --- -12V
                0 V     ---   *  2  *   --- 0 V
                BPCLK*  ---   *  3  *   --- BPCLK
                0 V     ---   *  4  *   --- 0 V
                INT*    ---   *  5  *   --- 0 V
                D7      ---   *  6  *   --- 0 V
                D6      ---   *  7  *   ---
                D5      ---   *  8  *   ---
                D4      ---   *  9  *   ---
                D3      ---   * 10  *   --- XINT*5^
                D2      ---   * 11  *   --- XINT*4^
                D1      ---   * 12  *   --- XINT*3^
                D0      ---   * 13  *   --- XINT*2^
                CSB*    ---   * 14  *   --- XCSB*2^
                BRST*   ---   * 15  *   --- XCSB*3^
                STAT*   ---   * 16  *   --- XCSB*4^
                INP*    ---   * 17  *   --- XCSB*5^
                C4*     ---   * 18  *   ---
                C3*     ---   * 19  *   ---
                C2*     ---   * 20  *   ---
                C1*     ---   * 21  *   --- EXP*^
                OUT*    ---   * 22  *   --- BUSEN*^
                CS*     ---   * 23  *   --- DSTB*
                NMI*^   ---   * 24  *   --- 0 V
                OPS*    ---   * 25  *   --- A4
                R/W*    ---   * 26  *   --- A3
                TREN*   ---   * 27  *   --- A2
                TRRQ*   ---   * 28  *   --- A1
                PRAC*   ---   * 29  *   --- A0
                PREN*   ---   * 30  *   --- DIRW/R*
                +5 V    ---   * 31  *   --- +5 V
                +12 V   ---   * 32  *   --- +12 V

                    ^ only connected on BUS0X

**********************************************************************/

#ifndef MAME_DEVICES_ABCBUS_ABCBUS_H
#define MAME_DEVICES_ABCBUS_ABCBUS_H

#pragma once



//**************************************************************************
//  CONSTANTS
//**************************************************************************

#define ABCBUS_TAG          "bus"



//**************************************************************************
//  INTERFACE CONFIGURATION MACROS
//**************************************************************************

#define MCFG_ABCBUS_SLOT_ADD(_tag, _slot_intf, _def_slot) \
	MCFG_DEVICE_ADD(_tag, ABCBUS_SLOT, 0) \
	MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _def_slot, false)


#define MCFG_ABCBUS_SLOT_IRQ_CALLBACK(_irq) \
	devcb = &downcast<abcbus_slot_device *>(device)->set_irq_callback(DEVCB_##_irq);

#define MCFG_ABCBUS_SLOT_NMI_CALLBACK(_nmi) \
	devcb = &downcast<abcbus_slot_device *>(device)->set_nmi_callback(DEVCB_##_nmi);

#define MCFG_ABCBUS_SLOT_RDY_CALLBACK(_rdy) \
	devcb = &downcast<abcbus_slot_device *>(device)->set_rdy_callback(DEVCB_##_rdy);

#define MCFG_ABCBUS_SLOT_RESIN_CALLBACK(_resin) \
	devcb = &downcast<abcbus_slot_device *>(device)->set_resin_callback(DEVCB_##_resin);

#define MCFG_ABCBUS_SLOT_PREN_CALLBACK(_pren) \
	devcb = &downcast<abcbus_slot_device *>(device)->set_pren_callback(DEVCB_##_pren);

#define MCFG_ABCBUS_SLOT_TRRQ_CALLBACK(_trrq) \
	devcb = &downcast<abcbus_slot_device *>(device)->set_trrq_callback(DEVCB_##_trrq);

#define MCFG_ABCBUS_SLOT_XINT2_CALLBACK(_xint2) \
	devcb = &downcast<abcbus_slot_device *>(device)->set_xint2_callback(DEVCB_##_xint2);

#define MCFG_ABCBUS_SLOT_XINT3_CALLBACK(_xint3) \
	devcb = &downcast<abcbus_slot_device *>(device)->set_xint3_callback(DEVCB_##_xint3);

#define MCFG_ABCBUS_SLOT_XINT4_CALLBACK(_xint4) \
	devcb = &downcast<abcbus_slot_device *>(device)->set_xint4_callback(DEVCB_##_xint4);

#define MCFG_ABCBUS_SLOT_XINT5_CALLBACK(_xint5) \
	devcb = &downcast<abcbus_slot_device *>(device)->set_xint5_callback(DEVCB_##_xint5);



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// ======================> device_abcbus_card_interface

class abcbus_slot_device;

class device_abcbus_card_interface : public device_slot_card_interface
{
public:
	// required operation overrides
	virtual void abcbus_cs(uint8_t data) = 0;

	// optional operation overrides
	virtual uint8_t abcbus_inp() { return 0xff; };
	virtual void abcbus_out(uint8_t data) { };
	virtual uint8_t abcbus_stat() { return 0xff; };
	virtual void abcbus_c1(uint8_t data) { };
	virtual void abcbus_c2(uint8_t data) { };
	virtual void abcbus_c3(uint8_t data) { };
	virtual void abcbus_c4(uint8_t data) { };

	// optional operation overrides for ABC 80
	virtual uint8_t abcbus_xmemfl(offs_t offset) { return 0xff; };
	virtual void abcbus_xmemw(offs_t offset, uint8_t data) { };

	// optional operation overrides for ABC 1600
	virtual int abcbus_csb() { return 1; }
	virtual uint8_t abcbus_ops() { return 0xff; };
	virtual void abcbus_tren(int state) { };
	virtual void abcbus_prac(int state) { };
	virtual uint8_t abcbus_exp() { return 0xff; };
	virtual int abcbus_xcsb2() { return 1; };
	virtual int abcbus_xcsb3() { return 1; };
	virtual int abcbus_xcsb4() { return 1; };
	virtual int abcbus_xcsb5() { return 1; };

protected:
	// construction/destruction
	device_abcbus_card_interface(const machine_config &mconfig, device_t &device);

	abcbus_slot_device  *m_slot;

	friend class abcbus_slot_device;
};


// ======================> abcbus_slot_device

class abcbus_slot_device : public device_t,
							public device_slot_interface
{
public:
	// construction/destruction
	abcbus_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> devcb_base &set_irq_callback(Object &&cb) { return m_write_irq.set_callback(std::forward<Object>(cb)); }
	template <class Object> devcb_base &set_nmi_callback(Object &&cb) { return m_write_nmi.set_callback(std::forward<Object>(cb)); }
	template <class Object> devcb_base &set_rdy_callback(Object &&cb) { return m_write_rdy.set_callback(std::forward<Object>(cb)); }
	template <class Object> devcb_base &set_resin_callback(Object &&cb) { return m_write_resin.set_callback(std::forward<Object>(cb)); }
	template <class Object> devcb_base &set_pren_callback(Object &&cb) { return m_write_pren.set_callback(std::forward<Object>(cb)); }
	template <class Object> devcb_base &set_trrq_callback(Object &&cb) { return m_write_trrq.set_callback(std::forward<Object>(cb)); }
	template <class Object> devcb_base &set_xint2_callback(Object &&cb) { return m_write_xint2.set_callback(std::forward<Object>(cb)); }
	template <class Object> devcb_base &set_xint3_callback(Object &&cb) { return m_write_xint3.set_callback(std::forward<Object>(cb)); }
	template <class Object> devcb_base &set_xint4_callback(Object &&cb) { return m_write_xint4.set_callback(std::forward<Object>(cb)); }
	template <class Object> devcb_base &set_xint5_callback(Object &&cb) { return m_write_xint5.set_callback(std::forward<Object>(cb)); }

	// computer interface
	void cs_w(uint8_t data) { if (m_card) m_card->abcbus_cs(data); }
	uint8_t rst_r() { device_reset(); return 0xff; }
	uint8_t inp_r() { return m_card ? m_card->abcbus_inp() : 0xff; }
	void out_w(uint8_t data) { if (m_card) m_card->abcbus_out(data); }
	uint8_t stat_r() { return m_card ? m_card->abcbus_stat() : 0xff; }
	void c1_w(uint8_t data) { if (m_card) m_card->abcbus_c1(data); }
	void c2_w(uint8_t data) { if (m_card) m_card->abcbus_c2(data); }
	void c3_w(uint8_t data) { if (m_card) m_card->abcbus_c3(data); }
	void c4_w(uint8_t data) { if (m_card) m_card->abcbus_c4(data); }
	uint8_t xmemfl_r(offs_t offset) { return m_card ? m_card->abcbus_xmemfl(offset) : 0xff; }
	void xmemw_w(offs_t offset, uint8_t data) { if (m_card) m_card->abcbus_xmemw(offset, data); }
	DECLARE_READ_LINE_MEMBER( csb_r ) { return m_card ? m_card->abcbus_csb() : 1; }
	uint8_t ops_r() { return m_card ? m_card->abcbus_ops() : 0xff; }
	uint8_t exp_r() { return m_card ? m_card->abcbus_exp() : 0xff; }
	DECLARE_READ_LINE_MEMBER( xcsb2_r ) { return m_card ? m_card->abcbus_xcsb2() : 1; }
	DECLARE_READ_LINE_MEMBER( xcsb3_r ) { return m_card ? m_card->abcbus_xcsb3() : 1; }
	DECLARE_READ_LINE_MEMBER( xcsb4_r ) { return m_card ? m_card->abcbus_xcsb4() : 1; }
	DECLARE_READ_LINE_MEMBER( xcsb5_r ) { return m_card ? m_card->abcbus_xcsb5() : 1; }
	DECLARE_WRITE_LINE_MEMBER( tren_w ) { if (m_card) m_card->abcbus_tren(state); }
	DECLARE_WRITE_LINE_MEMBER( prac_w ) { if (m_card) m_card->abcbus_prac(state); }

	DECLARE_WRITE8_MEMBER( cs_w ) { cs_w(data); }
	DECLARE_READ8_MEMBER( rst_r ) { return rst_r(); }
	DECLARE_READ8_MEMBER( inp_r ) { return inp_r(); }
	DECLARE_WRITE8_MEMBER( out_w ) { out_w(data); }
	DECLARE_READ8_MEMBER( stat_r ) { return stat_r(); }
	DECLARE_WRITE8_MEMBER( c1_w ) { c1_w(data); }
	DECLARE_WRITE8_MEMBER( c2_w ) { c2_w(data); }
	DECLARE_WRITE8_MEMBER( c3_w ) { c3_w(data); }
	DECLARE_WRITE8_MEMBER( c4_w ) { c4_w(data); }
	DECLARE_READ8_MEMBER( xmemfl_r ) { return xmemfl_r(offset); }
	DECLARE_WRITE8_MEMBER( xmemw_w ) { xmemw_w(offset, data); }

	DECLARE_READ_LINE_MEMBER( irq_r ) { return m_irq; }
	DECLARE_READ_LINE_MEMBER( nmi_r ) { return m_nmi; }
	DECLARE_READ_LINE_MEMBER( pren_r ) { return m_pren; }
	DECLARE_READ_LINE_MEMBER( trrq_r ) { return m_trrq; }
	DECLARE_READ_LINE_MEMBER( xint2_r ) { return m_xint2; }
	DECLARE_READ_LINE_MEMBER( xint3_r ) { return m_xint3; }
	DECLARE_READ_LINE_MEMBER( xint4_r ) { return m_xint4; }
	DECLARE_READ_LINE_MEMBER( xint5_r ) { return m_xint5; }

	// card interface
	DECLARE_WRITE_LINE_MEMBER( irq_w ) { m_irq = state; m_write_irq(state); }
	DECLARE_WRITE_LINE_MEMBER( nmi_w ) { m_nmi = state; m_write_nmi(state); }
	DECLARE_WRITE_LINE_MEMBER( rdy_w ) { m_write_rdy(state); }
	DECLARE_WRITE_LINE_MEMBER( resin_w ) { m_write_resin(state); }
	DECLARE_WRITE_LINE_MEMBER( pren_w ) { m_pren = state; m_write_pren(state); }
	DECLARE_WRITE_LINE_MEMBER( trrq_w ) { m_trrq = state; m_write_trrq(state); }
	DECLARE_WRITE_LINE_MEMBER( xint2_w ) { m_xint2 = state; m_write_xint2(state); }
	DECLARE_WRITE_LINE_MEMBER( xint3_w ) { m_xint3 = state; m_write_xint3(state); }
	DECLARE_WRITE_LINE_MEMBER( xint4_w ) { m_xint4 = state; m_write_xint4(state); }
	DECLARE_WRITE_LINE_MEMBER( xint5_w ) { m_xint5 = state; m_write_xint5(state); }

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override { if (m_card) get_card_device()->reset(); }

	devcb_write_line   m_write_irq;
	devcb_write_line   m_write_nmi;
	devcb_write_line   m_write_rdy;
	devcb_write_line   m_write_resin;
	devcb_write_line   m_write_pren;
	devcb_write_line   m_write_trrq;
	devcb_write_line   m_write_xint2;
	devcb_write_line   m_write_xint3;
	devcb_write_line   m_write_xint4;
	devcb_write_line   m_write_xint5;

	device_abcbus_card_interface *m_card;

	int m_irq;
	int m_nmi;
	int m_pren;
	int m_trrq;
	int m_xint2;
	int m_xint3;
	int m_xint4;
	int m_xint5;
};


// device type definition
DECLARE_DEVICE_TYPE(ABCBUS_SLOT, abcbus_slot_device)


SLOT_INTERFACE_EXTERN( abc80_cards );
SLOT_INTERFACE_EXTERN( abcbus_cards );
SLOT_INTERFACE_EXTERN( abc1600bus_cards );


typedef device_type_iterator<abcbus_slot_device> abcbus_slot_device_iterator;


#endif // MAME_DEVICES_ABCBUS_ABCBUS_H
