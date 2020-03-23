// license:BSD-3-Clause
// copyright-holders:Wilbert Pol, Miodrag Milanovic, Carl
#ifndef MAME_MACHINE_AT_H
#define MAME_MACHINE_AT_H

#include "machine/mc146818.h"
#include "machine/pic8259.h"
#include "machine/pit8253.h"
#include "machine/am9517a.h"
#include "bus/pc_kbd/pc_kbdc.h"
#include "bus/isa/isa.h"
#include "sound/spkrdev.h"
#include "softlist.h"

class at_mb_device : public device_t
{
public:
	at_mb_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_ADDRESS_MAP(map, 16);

	DECLARE_READ8_MEMBER(page8_r);
	DECLARE_WRITE8_MEMBER(page8_w);
	DECLARE_READ8_MEMBER(portb_r);
	DECLARE_WRITE8_MEMBER(portb_w);
	DECLARE_WRITE8_MEMBER(write_rtc);

	DECLARE_WRITE_LINE_MEMBER(shutdown);

	uint32_t a20_286(bool state);

protected:
	void device_start() override;
	void device_reset() override;
	virtual void device_add_mconfig(machine_config &config) override;

private:
	void set_dma_channel(int channel, int state);
	void speaker_set_spkrdata(uint8_t data);

	required_device<cpu_device> m_maincpu;
	required_device<isa16_device> m_isabus;
	required_device<pic8259_device> m_pic8259_slave;
	required_device<am9517a_device> m_dma8237_1;
	required_device<am9517a_device> m_dma8237_2;
	required_device<pit8254_device> m_pit8254;
	required_device<speaker_sound_device> m_speaker;
	required_device<mc146818_device> m_mc146818;
	uint8_t m_at_spkrdata;
	uint8_t m_pit_out2;
	int m_dma_channel;
	bool m_cur_eop;
	uint8_t m_dma_offset[2][4];
	uint8_t m_at_pages[0x10];
	uint16_t m_dma_high_byte;
	uint8_t m_at_speaker;
	uint8_t m_channel_check;
	uint8_t m_nmi_enabled;

	DECLARE_WRITE_LINE_MEMBER(pit8254_out2_changed);

	DECLARE_WRITE_LINE_MEMBER(dma8237_out_eop);
	DECLARE_READ8_MEMBER(dma8237_0_dack_r);
	DECLARE_READ8_MEMBER(dma8237_1_dack_r);
	DECLARE_READ8_MEMBER(dma8237_2_dack_r);
	DECLARE_READ8_MEMBER(dma8237_3_dack_r);
	DECLARE_READ8_MEMBER(dma8237_5_dack_r);
	DECLARE_READ8_MEMBER(dma8237_6_dack_r);
	DECLARE_READ8_MEMBER(dma8237_7_dack_r);
	DECLARE_WRITE8_MEMBER(dma8237_0_dack_w);
	DECLARE_WRITE8_MEMBER(dma8237_1_dack_w);
	DECLARE_WRITE8_MEMBER(dma8237_2_dack_w);
	DECLARE_WRITE8_MEMBER(dma8237_3_dack_w);
	DECLARE_WRITE8_MEMBER(dma8237_5_dack_w);
	DECLARE_WRITE8_MEMBER(dma8237_6_dack_w);
	DECLARE_WRITE8_MEMBER(dma8237_7_dack_w);
	DECLARE_WRITE_LINE_MEMBER(dack0_w);
	DECLARE_WRITE_LINE_MEMBER(dack1_w);
	DECLARE_WRITE_LINE_MEMBER(dack2_w);
	DECLARE_WRITE_LINE_MEMBER(dack3_w);
	DECLARE_WRITE_LINE_MEMBER(dack4_w);
	DECLARE_WRITE_LINE_MEMBER(dack5_w);
	DECLARE_WRITE_LINE_MEMBER(dack6_w);
	DECLARE_WRITE_LINE_MEMBER(dack7_w);
	DECLARE_READ8_MEMBER(get_slave_ack);
	DECLARE_WRITE_LINE_MEMBER(dma_hrq_changed);

	DECLARE_READ8_MEMBER(dma_read_byte);
	DECLARE_WRITE8_MEMBER(dma_write_byte);
	DECLARE_READ8_MEMBER(dma_read_word);
	DECLARE_WRITE8_MEMBER(dma_write_word);
};

DECLARE_DEVICE_TYPE(AT_MB, at_mb_device)

MACHINE_CONFIG_EXTERN(at_softlists);

#endif // MAME_MACHINE_AT_H
