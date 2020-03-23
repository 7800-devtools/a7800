// license:BSD-3-Clause
// copyright-holders:Mathis Rosenhauer
/*************************************************************************

    Exidy Vertigo hardware

*************************************************************************/

#include "audio/exidy440.h"
#include "machine/pit8253.h"
#include "machine/74148.h"
#include "video/vector.h"

/*************************************
 *
 *  Typedefs
 *
 *************************************/

#define MC_LENGTH 512

struct am2901
{
	uint32_t ram[16];   /* internal ram */
	uint32_t d;         /* direct data D input */
	uint32_t q;         /* Q register */
	uint32_t f;         /* F ALU result */
	uint32_t y;         /* Y output */
};

class vector_generator
{
public:
	uint32_t sreg;      /* shift register */
	uint32_t l1;        /* latch 1 adder operand only */
	uint32_t l2;        /* latch 2 adder operand only */
	uint32_t c_v;       /* vertical position counter */
	uint32_t c_h;       /* horizontal position counter */
	uint32_t c_l;       /* length counter */
	uint32_t adder_s;   /* slope generator result and B input */
	uint32_t adder_a;   /* slope generator A input */
	uint32_t color;     /* color */
	uint32_t intensity; /* intensity */
	uint32_t brez;      /* h/v-counters enable */
	uint32_t vfin;      /* drawing yes/no */
	uint32_t hud1;      /* h-counter up or down (stored in L1) */
	uint32_t hud2;      /* h-counter up or down (stored in L2) */
	uint32_t vud1;      /* v-counter up or down (stored in L1) */
	uint32_t vud2;      /* v-counter up or down (stored in L2) */
	uint32_t hc1;       /* use h- or v-counter in L1 mode */
	uint32_t ven;       /* vector intensity enable */
};

struct microcode
{
	uint32_t x;
	uint32_t a;
	uint32_t b;
	uint32_t inst;
	uint32_t dest;
	uint32_t cn;
	uint32_t mreq;
	uint32_t rsel;
	uint32_t rwrite;
	uint32_t of;
	uint32_t iif;
	uint32_t oa;
	uint32_t jpos;
	uint32_t jmp;
	uint32_t jcon;
	uint32_t ma;
};

struct vproc
{
	uint16_t sram[64]; /* external sram */
	uint16_t ramlatch; /* latch between 2901 and sram */
	uint16_t rom_adr;  /* vector ROM/RAM address latch */
	uint32_t pc;       /* program counter */
	uint32_t ret;      /* return address */

};


class vertigo_state : public driver_device
{
public:
	vertigo_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
			m_maincpu(*this, "maincpu"),
			m_audiocpu(*this, "audiocpu"),
			m_pit(*this, "pit8254"),
			m_custom(*this, "custom"),
			m_ttl74148(*this, "74148"),
			m_vector(*this, "vector"),
			m_vectorram(*this, "vectorram")
	{ }

	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_audiocpu;
	required_device<pit8254_device> m_pit;
	required_device<exidy440_sound_device> m_custom;
	required_device<ttl74148_device> m_ttl74148;
	required_device<vector_device> m_vector;
	required_shared_ptr<uint16_t> m_vectorram;
	attotime m_irq4_time;
	uint8_t m_irq_state;
	uint8_t m_adc_result;
	vproc m_vs;
	am2901 m_bsp;
	vector_generator m_vgen;
	uint16_t *m_vectorrom;
	microcode m_mc[MC_LENGTH];
	DECLARE_READ16_MEMBER(vertigo_io_convert);
	DECLARE_READ16_MEMBER(vertigo_io_adc);
	DECLARE_READ16_MEMBER(vertigo_coin_r);
	DECLARE_WRITE16_MEMBER(vertigo_wsot_w);
	DECLARE_WRITE16_MEMBER(vertigo_audio_w);
	DECLARE_READ16_MEMBER(vertigo_sio_r);
	DECLARE_WRITE16_MEMBER(vertigo_motor_w);
	DECLARE_READ16_MEMBER(vertigo_pit8254_lsb_r);
	DECLARE_WRITE16_MEMBER(vertigo_pit8254_lsb_w);
	virtual void machine_start() override;
	virtual void machine_reset() override;
	INTERRUPT_GEN_MEMBER(vertigo_interrupt);
	TIMER_CALLBACK_MEMBER(sound_command_w);
	DECLARE_WRITE_LINE_MEMBER(v_irq4_w);
	DECLARE_WRITE_LINE_MEMBER(v_irq3_w);
	TTL74148_OUTPUT_CB(update_irq);

	void vertigo_vproc_init();
	void vertigo_vproc_reset();
	void am2901x4 (am2901 *bsp, microcode *mc);
	void vertigo_vgen (vector_generator *vg);
	void vertigo_vproc(int cycles, int irq4);
	void update_irq_encoder(int line, int state);
};
