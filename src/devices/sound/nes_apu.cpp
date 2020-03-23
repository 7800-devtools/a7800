// license:GPL-2.0+
// copyright-holders:Matthew Conte
/*****************************************************************************

  MAME/MESS NES APU CORE

  Based on the Nofrendo/Nosefart NES N2A03 sound emulation core written by
  Matthew Conte (matt@conte.com) and redesigned for use in MAME/MESS by
  Who Wants to Know? (wwtk@mail.com)

  This core is written with the advise and consent of Matthew Conte and is
  released under the GNU Public License.  This core is freely available for
  use in any freeware project, subject to the following terms:

  Any modifications to this code must be duly noted in the source and
  approved by Matthew Conte and myself prior to public submission.

  timing notes:
  master = 21477270
  2A03 clock = master/12
  sequencer = master/89490 or CPU/7457

 *****************************************************************************

   NES_APU.C

   Actual NES APU interface.

   LAST MODIFIED 02/29/2004

   - Based on Matthew Conte's Nofrendo/Nosefart core and redesigned to
     use MAME system calls and to enable multiple APUs.  Sound at this
     point should be just about 100% accurate, though I cannot tell for
     certain as yet.

     A queue interface is also available for additional speed.  However,
     the implementation is not yet 100% (DPCM sounds are inaccurate),
     so it is disabled by default.

 *****************************************************************************

   BUGFIXES:

   - Various bugs concerning the DPCM channel fixed. (Oliver Achten)
   - Fixed $4015 read behaviour. (Oliver Achten)

 *****************************************************************************/

#include "emu.h"
#include "nes_apu.h"

#include "screen.h"



/* INTERNAL FUNCTIONS */

/* INITIALIZE WAVE TIMES RELATIVE TO SAMPLE RATE */
static void create_vbltimes(u32 * table,const u8 *vbl,unsigned int rate)
{
	int i;

	for (i = 0; i < 0x20; i++)
		table[i] = vbl[i] * rate;
}

/* INITIALIZE SAMPLE TIMES IN TERMS OF VSYNCS */
void nesapu_device::create_syncs(unsigned long sps)
{
	int i;
	unsigned long val = sps;

	for (i = 0; i < SYNCS_MAX1; i++)
	{
		m_sync_times1[i] = val;
		val += sps;
	}

	val = 0;
	for (i = 0; i < SYNCS_MAX2; i++)
	{
		m_sync_times2[i] = val;
		m_sync_times2[i] >>= 2;
		val += sps;
	}
}

/* INITIALIZE NOISE LOOKUP TABLE */
static void create_noise(u8 *buf, const int bits, int size)
{
	int m = 0x0011;
	int xor_val, i;

	for (i = 0; i < size; i++)
	{
		xor_val = m & 1;
		m >>= 1;
		xor_val ^= (m & 1);
		m |= xor_val << (bits - 1);

		buf[i] = m;
	}
}

DEFINE_DEVICE_TYPE(NES_APU, nesapu_device, "nesapu", "N2A03 APU")

nesapu_device::nesapu_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, NES_APU, tag, owner, clock),
		device_sound_interface(mconfig, *this),
		m_apu_incsize(0.0),
		m_samps_per_sync(0),
		m_buffer_size(0),
		m_real_rate(0),
		m_stream(nullptr),
		m_irq_handler(*this),
		m_mem_read_cb(*this)
{
	for (auto & elem : m_noise_lut)
	{
		elem = 0;
	}

	for (auto & elem : m_vbl_times)
	{
		elem = 0;
	}

	for (auto & elem : m_sync_times1)
	{
		elem = 0;
	}

	for (auto & elem : m_sync_times2)
	{
		elem = 0;
	}
}

void nesapu_device::device_clock_changed()
{
	calculate_rates();
}

void nesapu_device::calculate_rates()
{
	int rate = clock() / 4;

	screen_device *screen = machine().first_screen();
	if (screen != nullptr)
	{
		m_samps_per_sync = rate / ATTOSECONDS_TO_HZ(machine().first_screen()->frame_period().attoseconds());
		m_real_rate = m_samps_per_sync * ATTOSECONDS_TO_HZ(machine().first_screen()->frame_period().attoseconds());
	}
	else
	{
		m_samps_per_sync = rate / screen_device::DEFAULT_FRAME_RATE;
		m_real_rate = m_samps_per_sync * screen_device::DEFAULT_FRAME_RATE;
	}
	m_buffer_size = m_samps_per_sync;
	m_apu_incsize = float(clock() / (float) m_real_rate);

	create_vbltimes(m_vbl_times,vbl_length,m_samps_per_sync);
	create_syncs(m_samps_per_sync);

	/* Adjust buffer size if 16 bits */
	m_buffer_size+=m_samps_per_sync;

	if (m_stream != nullptr)
		m_stream->set_sample_rate(rate);
	else
		m_stream = machine().sound().stream_alloc(*this, 0, 1, rate);
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void nesapu_device::device_start()
{
	// resolve callbacks
	m_irq_handler.resolve_safe();
	m_mem_read_cb.resolve_safe(0x00);

	create_noise(m_noise_lut, 13, apu_t::NOISE_LONG);

	calculate_rates();

	/* register for save */
	for (int i = 0; i < 2; i++)
	{
		save_item(NAME(m_APU.squ[i].regs), i);
		save_item(NAME(m_APU.squ[i].vbl_length), i);
		save_item(NAME(m_APU.squ[i].freq), i);
		save_item(NAME(m_APU.squ[i].phaseacc), i);
		save_item(NAME(m_APU.squ[i].output_vol), i);
		save_item(NAME(m_APU.squ[i].env_phase), i);
		save_item(NAME(m_APU.squ[i].sweep_phase), i);
		save_item(NAME(m_APU.squ[i].adder), i);
		save_item(NAME(m_APU.squ[i].env_vol), i);
		save_item(NAME(m_APU.squ[i].enabled), i);
	}

	save_item(NAME(m_APU.tri.regs));
	save_item(NAME(m_APU.tri.linear_length));
	save_item(NAME(m_APU.tri.vbl_length));
	save_item(NAME(m_APU.tri.write_latency));
	save_item(NAME(m_APU.tri.phaseacc));
	save_item(NAME(m_APU.tri.output_vol));
	save_item(NAME(m_APU.tri.adder));
	save_item(NAME(m_APU.tri.counter_started));
	save_item(NAME(m_APU.tri.enabled));

	save_item(NAME(m_APU.noi.regs));
	save_item(NAME(m_APU.noi.cur_pos));
	save_item(NAME(m_APU.noi.vbl_length));
	save_item(NAME(m_APU.noi.phaseacc));
	save_item(NAME(m_APU.noi.output_vol));
	save_item(NAME(m_APU.noi.env_phase));
	save_item(NAME(m_APU.noi.env_vol));
	save_item(NAME(m_APU.noi.enabled));

	save_item(NAME(m_APU.dpcm.regs));
	save_item(NAME(m_APU.dpcm.address));
	save_item(NAME(m_APU.dpcm.length));
	save_item(NAME(m_APU.dpcm.bits_left));
	save_item(NAME(m_APU.dpcm.phaseacc));
	save_item(NAME(m_APU.dpcm.output_vol));
	save_item(NAME(m_APU.dpcm.cur_byte));
	save_item(NAME(m_APU.dpcm.enabled));
	save_item(NAME(m_APU.dpcm.irq_occurred));
	save_item(NAME(m_APU.dpcm.vol));

	save_item(NAME(m_APU.regs));

	#ifdef USE_QUEUE
	save_item(NAME(m_APU.queue));
	save_item(NAME(m_APU.head));
	save_item(NAME(m_APU.tail));
	#else
	save_item(NAME(m_APU.buf_pos));
	save_item(NAME(m_APU.step_mode));
	#endif
}

/* TODO: sound channels should *ALL* have DC volume decay */

/* OUTPUT SQUARE WAVE SAMPLE (VALUES FROM -16 to +15) */
s8 nesapu_device::apu_square(apu_t::square_t *chan)
{
	int env_delay;
	int sweep_delay;
	s8 output;

	/* reg0: 0-3=volume, 4=envelope, 5=hold, 6-7=duty cycle
	** reg1: 0-2=sweep shifts, 3=sweep inc/dec, 4-6=sweep length, 7=sweep on
	** reg2: 8 bits of freq
	** reg3: 0-2=high freq, 7-4=vbl length counter
	*/

	if (false == chan->enabled)
		return 0;

	/* enveloping */
	env_delay = m_sync_times1[chan->regs[0] & 0x0F];

	/* decay is at a rate of (env_regs + 1) / 240 secs */
	chan->env_phase -= 4;
	while (chan->env_phase < 0)
	{
		chan->env_phase += env_delay;
		if (chan->regs[0] & 0x20)
			chan->env_vol = (chan->env_vol + 1) & 15;
		else if (chan->env_vol < 15)
			chan->env_vol++;
	}

	/* vbl length counter */
	if (chan->vbl_length > 0 && 0 == (chan->regs [0] & 0x20))
		chan->vbl_length--;

	if (0 == chan->vbl_length)
		return 0;

	/* freqsweeps */
	if ((chan->regs[1] & 0x80) && (chan->regs[1] & 7))
	{
		sweep_delay = m_sync_times1[(chan->regs[1] >> 4) & 7];
		chan->sweep_phase -= 2;
		while (chan->sweep_phase < 0)
		{
			chan->sweep_phase += sweep_delay;
			if (chan->regs[1] & 8)
				chan->freq -= chan->freq >> (chan->regs[1] & 7);
			else
				chan->freq += chan->freq >> (chan->regs[1] & 7);
		}
	}

	if ((0 == (chan->regs[1] & 8) && (chan->freq >> 16) > freq_limit[chan->regs[1] & 7])
			|| (chan->freq >> 16) < 4)
		return 0;

	chan->phaseacc -= (float) m_apu_incsize; /* # of cycles per sample */

	while (chan->phaseacc < 0)
	{
		chan->phaseacc += (chan->freq >> 16);
		chan->adder = (chan->adder + 1) & 0x0F;
	}

	if (chan->regs[0] & 0x10) /* fixed volume */
		output = chan->regs[0] & 0x0F;
	else
		output = 0x0F - chan->env_vol;

	if (chan->adder < (duty_lut[chan->regs[0] >> 6]))
		output = -output;

	return (s8) output;
}

/* OUTPUT TRIANGLE WAVE SAMPLE (VALUES FROM -16 to +15) */
s8 nesapu_device::apu_triangle(apu_t::triangle_t *chan)
{
	int freq;
	s8 output;
	/* reg0: 7=holdnote, 6-0=linear length counter
	** reg2: low 8 bits of frequency
	** reg3: 7-3=length counter, 2-0=high 3 bits of frequency
	*/

	if (false == chan->enabled)
		return 0;

	if (false == chan->counter_started && 0 == (chan->regs[0] & 0x80))
	{
		if (chan->write_latency)
			chan->write_latency--;
		if (0 == chan->write_latency)
			chan->counter_started = true;
	}

	if (chan->counter_started)
	{
		if (chan->linear_length > 0)
			chan->linear_length--;
		if (chan->vbl_length && 0 == (chan->regs[0] & 0x80))
				chan->vbl_length--;

		if (0 == chan->vbl_length)
			return 0;
	}

	if (0 == chan->linear_length)
		return 0;

	freq = (((chan->regs[3] & 7) << 8) + chan->regs[2]) + 1;

	if (freq < 4) /* inaudible */
		return 0;

	chan->phaseacc -= (float) m_apu_incsize; /* # of cycles per sample */
	while (chan->phaseacc < 0)
	{
		chan->phaseacc += freq;
		chan->adder = (chan->adder + 1) & 0x1F;

		output = (chan->adder & 7) << 1;
		if (chan->adder & 8)
			output = 0x10 - output;
		if (chan->adder & 0x10)
			output = -output;

		chan->output_vol = output;
	}

	return (s8) chan->output_vol;
}

/* OUTPUT NOISE WAVE SAMPLE (VALUES FROM -16 to +15) */
s8 nesapu_device::apu_noise(apu_t::noise_t *chan)
{
	int freq, env_delay;
	u8 outvol;
	u8 output;

	/* reg0: 0-3=volume, 4=envelope, 5=hold
	** reg2: 7=small(93 byte) sample,3-0=freq lookup
	** reg3: 7-4=vbl length counter
	*/

	if (false == chan->enabled)
		return 0;

	/* enveloping */
	env_delay = m_sync_times1[chan->regs[0] & 0x0F];

	/* decay is at a rate of (env_regs + 1) / 240 secs */
	chan->env_phase -= 4;
	while (chan->env_phase < 0)
	{
		chan->env_phase += env_delay;
		if (chan->regs[0] & 0x20)
			chan->env_vol = (chan->env_vol + 1) & 15;
		else if (chan->env_vol < 15)
			chan->env_vol++;
	}

	/* length counter */
	if (0 == (chan->regs[0] & 0x20))
	{
		if (chan->vbl_length > 0)
			chan->vbl_length--;
	}

	if (0 == chan->vbl_length)
		return 0;

	freq = noise_freq[chan->regs[2] & 0x0F];
	chan->phaseacc -= (float) m_apu_incsize; /* # of cycles per sample */
	while (chan->phaseacc < 0)
	{
		chan->phaseacc += freq;

		chan->cur_pos++;
		if (apu_t::NOISE_SHORT == chan->cur_pos && (chan->regs[2] & 0x80))
			chan->cur_pos = 0;
		else if (apu_t::NOISE_LONG == chan->cur_pos)
			chan->cur_pos = 0;
	}

	if (chan->regs[0] & 0x10) /* fixed volume */
		outvol = chan->regs[0] & 0x0F;
	else
		outvol = 0x0F - chan->env_vol;

	output = m_noise_lut[chan->cur_pos];
	if (output > outvol)
		output = outvol;

	if (m_noise_lut[chan->cur_pos] & 0x80) /* make it negative */
		output = -output;

	return (s8) output;
}

/* RESET DPCM PARAMETERS */
static inline void apu_dpcmreset(apu_t::dpcm_t *chan)
{
	chan->address = 0xC000 + u16(chan->regs[2] << 6);
	chan->length = u16(chan->regs[3] << 4) + 1;
	chan->bits_left = chan->length << 3;
	chan->irq_occurred = false;
	chan->enabled = true; /* Fixed * Proper DPCM channel ENABLE/DISABLE flag behaviour*/
	chan->vol = 0; /* Fixed * DPCM DAC resets itself when restarted */
}

/* OUTPUT DPCM WAVE SAMPLE (VALUES FROM -64 to +63) */
/* TODO: centerline naughtiness */
s8 nesapu_device::apu_dpcm(apu_t::dpcm_t *chan)
{
	int freq, bit_pos;

	/* reg0: 7=irq gen, 6=looping, 3-0=pointer to clock table
	** reg1: output dc level, 7 bits unsigned
	** reg2: 8 bits of 64-byte aligned address offset : $C000 + (value * 64)
	** reg3: length, (value * 16) + 1
	*/

	if (chan->enabled)
	{
		freq = dpcm_clocks[chan->regs[0] & 0x0F];
		chan->phaseacc -= (float) m_apu_incsize; /* # of cycles per sample */

		while (chan->phaseacc < 0)
		{
			chan->phaseacc += freq;

			if (0 == chan->length)
			{
				chan->enabled = false; /* Fixed * Proper DPCM channel ENABLE/DISABLE flag behaviour*/
				chan->vol=0; /* Fixed * DPCM DAC resets itself when restarted */
				if (chan->regs[0] & 0x40)
					apu_dpcmreset(chan);
				else
				{
					if (chan->regs[0] & 0x80) /* IRQ Generator */
					{
						chan->irq_occurred = true;
						m_irq_handler(true);
					}
					break;
				}
			}


			chan->bits_left--;
			bit_pos = 7 - (chan->bits_left & 7);
			if (7 == bit_pos)
			{
				chan->cur_byte = m_mem_read_cb(chan->address);
				chan->address++;
				chan->length--;
			}

			if (chan->cur_byte & (1 << bit_pos))
//              chan->regs[1]++;
				chan->vol+=2; /* FIXED * DPCM channel only uses the upper 6 bits of the DAC */
			else
//              chan->regs[1]--;
				chan->vol-=2;
		}
	}

	if (chan->vol > 63)
		chan->vol = 63;
	else if (chan->vol < -64)
		chan->vol = -64;

	return (s8) (chan->vol);
}

/* WRITE REGISTER VALUE */
inline void nesapu_device::apu_regwrite(int address, u8 value)
{
	int chan = (address & 4) ? 1 : 0;

	switch (address)
	{
	/* squares */
	case apu_t::WRA0:
	case apu_t::WRB0:
		m_APU.squ[chan].regs[0] = value;
		break;

	case apu_t::WRA1:
	case apu_t::WRB1:
		m_APU.squ[chan].regs[1] = value;
		break;

	case apu_t::WRA2:
	case apu_t::WRB2:
		m_APU.squ[chan].regs[2] = value;
		if (m_APU.squ[chan].enabled)
			m_APU.squ[chan].freq = ((((m_APU.squ[chan].regs[3] & 7) << 8) + value) + 1) << 16;
		break;

	case apu_t::WRA3:
	case apu_t::WRB3:
		m_APU.squ[chan].regs[3] = value;

		if (m_APU.squ[chan].enabled)
		{
			m_APU.squ[chan].vbl_length = m_vbl_times[value >> 3];
			m_APU.squ[chan].env_vol = 0;
			m_APU.squ[chan].freq = ((((value & 7) << 8) + m_APU.squ[chan].regs[2]) + 1) << 16;
		}

		break;

	/* triangle */
	case apu_t::WRC0:
		m_APU.tri.regs[0] = value;

		if (m_APU.tri.enabled)
		{                                          /* ??? */
			if (false == m_APU.tri.counter_started)
				m_APU.tri.linear_length = m_sync_times2[value & 0x7F];
		}

		break;

	case 0x4009:
		/* unused */
		m_APU.tri.regs[1] = value;
		break;

	case apu_t::WRC2:
		m_APU.tri.regs[2] = value;
		break;

	case apu_t::WRC3:
		m_APU.tri.regs[3] = value;

		/* this is somewhat of a hack.  there is some latency on the Real
		** Thing between when trireg0 is written to and when the linear
		** length counter actually begins its countdown.  we want to prevent
		** the case where the program writes to the freq regs first, then
		** to reg 0, and the counter accidentally starts running because of
		** the sound queue's timestamp processing.
		**
		** set to a few NES sample -- should be sufficient
		**
		**    3 * (1789772.727 / 44100) = ~122 cycles, just around one scanline
		**
		** should be plenty of time for the 6502 code to do a couple of table
		** dereferences and load up the other triregs
		*/

	/* used to be 3, but now we run the clock faster, so base it on samples/sync */
		m_APU.tri.write_latency = (m_samps_per_sync + 239) / 240;

		if (m_APU.tri.enabled)
		{
			m_APU.tri.counter_started = false;
			m_APU.tri.vbl_length = m_vbl_times[value >> 3];
			m_APU.tri.linear_length = m_sync_times2[m_APU.tri.regs[0] & 0x7F];
		}

		break;

	/* noise */
	case apu_t::WRD0:
		m_APU.noi.regs[0] = value;
		break;

	case 0x400D:
		/* unused */
		m_APU.noi.regs[1] = value;
		break;

	case apu_t::WRD2:
		m_APU.noi.regs[2] = value;
		break;

	case apu_t::WRD3:
		m_APU.noi.regs[3] = value;

		if (m_APU.noi.enabled)
		{
			m_APU.noi.vbl_length = m_vbl_times[value >> 3];
			m_APU.noi.env_vol = 0; /* reset envelope */
		}
		break;

	/* DMC */
	case apu_t::WRE0:
		m_APU.dpcm.regs[0] = value;
		if (0 == (value & 0x80)) {
			m_irq_handler(false);
			m_APU.dpcm.irq_occurred = false;
		}
		break;

	case apu_t::WRE1: /* 7-bit DAC */
		//m_APU.dpcm.regs[1] = value - 0x40;
		m_APU.dpcm.regs[1] = value & 0x7F;
		m_APU.dpcm.vol = (m_APU.dpcm.regs[1]-64);
		break;

	case apu_t::WRE2:
		m_APU.dpcm.regs[2] = value;
		//apu_dpcmreset(m_APU.dpcm);
		break;

	case apu_t::WRE3:
		m_APU.dpcm.regs[3] = value;
		break;

	case apu_t::IRQCTRL:
		if(value & 0x80)
			m_APU.step_mode = 5;
		else
			m_APU.step_mode = 4;
		break;

	case apu_t::SMASK:
		if (value & 0x01)
			m_APU.squ[0].enabled = true;
		else
		{
			m_APU.squ[0].enabled = false;
			m_APU.squ[0].vbl_length = 0;
		}

		if (value & 0x02)
			m_APU.squ[1].enabled = true;
		else
		{
			m_APU.squ[1].enabled = false;
			m_APU.squ[1].vbl_length = 0;
		}

		if (value & 0x04)
			m_APU.tri.enabled = true;
		else
		{
			m_APU.tri.enabled = false;
			m_APU.tri.vbl_length = 0;
			m_APU.tri.linear_length = 0;
			m_APU.tri.counter_started = false;
			m_APU.tri.write_latency = 0;
		}

		if (value & 0x08)
			m_APU.noi.enabled = true;
		else
		{
			m_APU.noi.enabled = false;
			m_APU.noi.vbl_length = 0;
		}

		if (value & 0x10)
		{
			/* only reset dpcm values if DMA is finished */
			if (false == m_APU.dpcm.enabled)
			{
				m_APU.dpcm.enabled = true;
				apu_dpcmreset(&m_APU.dpcm);
			}
		}
		else
			m_APU.dpcm.enabled = false;

		//m_irq_handler(false);
		m_APU.dpcm.irq_occurred = false;

		break;
	default:
#ifdef MAME_DEBUG
logerror("invalid apu write: $%02X at $%04X\n", value, address);
#endif
		break;
	}
}



/* READ VALUES FROM REGISTERS */
inline u8 nesapu_device::apu_read(int address)
{
	if (address == 0x15) /*FIXED* Address $4015 has different behaviour*/
	{
		int readval = 0;
		if (m_APU.squ[0].vbl_length > 0)
			readval |= 0x01;

		if (m_APU.squ[1].vbl_length > 0)
			readval |= 0x02;

		if (m_APU.tri.vbl_length > 0)
			readval |= 0x04;

		if (m_APU.noi.vbl_length > 0)
			readval |= 0x08;

		if (m_APU.dpcm.enabled == true)
			readval |= 0x10;

		if (m_APU.dpcm.irq_occurred == true)
			readval |= 0x80;

		return readval;
	}
	else
		return m_APU.regs[address];
}

/* WRITE VALUE TO TEMP REGISTRY AND QUEUE EVENT */
inline void nesapu_device::apu_write(int address, u8 value)
{
	m_APU.regs[address]=value;
	m_stream->update();
	apu_regwrite(address,value);
}

/* EXTERNAL INTERFACE FUNCTIONS */

/* REGISTER READ/WRITE FUNCTIONS */
READ8_MEMBER( nesapu_device::read ) {return apu_read(offset);}
WRITE8_MEMBER( nesapu_device::write ) {apu_write(offset,data);}


//-------------------------------------------------
//  sound_stream_update - handle a stream update
//-------------------------------------------------

void nesapu_device::sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples)
{
	int accum;
	memset( outputs[0], 0, samples*sizeof(*outputs[0]) );

	while (samples--)
	{
		accum = apu_square(&m_APU.squ[0]);
		accum += apu_square(&m_APU.squ[1]);
		accum += apu_triangle(&m_APU.tri);
		accum += apu_noise(&m_APU.noi);
		accum += apu_dpcm(&m_APU.dpcm);

		/* 8-bit clamps */
		if (accum > 127)
			accum = 127;
		else if (accum < -128)
			accum = -128;

		*(outputs[0]++)=accum<<8;
	}
}
