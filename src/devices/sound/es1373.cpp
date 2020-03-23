// license:BSD-3-Clause
// copyright-holders:Ted Green
#include "emu.h"
#include "es1373.h"

#include "speaker.h"


#define LOG_ES            (0)
#define LOG_ES_REG        (0)
#define LOG_ES_FILE       (0)


/* Ensonic ES1373 registers 0x00-0x3f */
#define ES_INT_CS_CTRL          (0x00/4)
#define ES_INT_CS_STATUS        (0x04/4)
#define ES_UART_DATA            (0x08/4)
#define ES_UART_STATUS          (0x09/4)
#define ES_UART_CTRL            (0x09/4)
#define ES_UART_RSVD            (0x0A/4)
#define ES_MEM_PAGE             (0x0C/4)
#define ES_SRC_IF               (0x10/4)
#define ES_CODEC                (0x14/4)
#define ES_LEGACY               (0x18/4)
#define ES_CHAN_CTRL            (0x1C/4)
#define ES_SERIAL_CTRL          (0x20/4)
#define ES_DAC1_CNT             (0x24/4)
#define ES_DAC2_CNT             (0x28/4)
#define ES_ADC_CNT              (0x2C/4)
#define ES_HOST_IF0             (0x30/4)
#define ES_HOST_IF1             (0x34/4)
#define ES_HOST_IF2             (0x38/4)
#define ES_HOST_IF3             (0x3C/4)

// Interrupt/Chip Select Control Register (ES_INT_CS_CTRL) bits
#define ICCTRL_ADC_STOP_MASK   0x00002000
#define ICCTRL_DAC1_EN_MASK    0x00000040
#define ICCTRL_DAC2_EN_MASK    0x00000020
#define ICCTRL_ADC_EN_MASK     0x00000010
#define ICCTRL_UART_EN_MASK    0x00000008
#define ICCTRL_JYSTK_EN_MASK   0x00000004

// Interrupt/Chip Select Status Register (ES_INT_CS_STATUS) bits
#define ICSTATUS_INTR_MASK        0x80000000
#define ICSTATUS_DAC1_INT_MASK    0x00000004
#define ICSTATUS_DAC2_INT_MASK    0x00000002
#define ICSTATUS_ADC_INT_MASK     0x00000001

// Serial Interface Control Register (ES_SERIAL_CTRL) bits
#define SCTRL_P2_END_MASK     0x00380000
#define SCTRL_P2_START_MASK   0x00070000
#define SCTRL_R1_LOOP_MASK    0x00008000
#define SCTRL_P2_LOOP_MASK    0x00004000
#define SCTRL_P1_LOOP_MASK    0x00002000
#define SCTRL_P2_PAUSE_MASK   0x00001000
#define SCTRL_P1_PAUSE_MASK   0x00000800
#define SCTRL_R1_INT_EN_MASK  0x00000400
#define SCTRL_P2_INT_EN_MASK  0x00000200
#define SCTRL_P1_INT_EN_MASK  0x00000100
#define SCTRL_P1_RELOAD_MASK  0x00000080
#define SCTRL_P2_STOP_MASK    0x00000040
#define SCTRL_R1_S_MASK       0x00000030
#define SCTRL_P2_S_MASK       0x0000000C
#define SCTRL_P1_S_MASK       0x00000003

#define SCTRL_8BIT_MONO             0x0
#define SCTRL_8BIT_STEREO           0x1
#define SCTRL_16BIT_MONO            0x2
#define SCTRL_16BIT_STEREO      0x3

#define ES_PCI_READ 0
#define ES_PCI_WRITE 1

MACHINE_CONFIG_MEMBER( es1373_device::device_add_mconfig )
	MCFG_SPEAKER_STANDARD_STEREO("lspeaker", "rspeaker")
MACHINE_CONFIG_END

DEFINE_DEVICE_TYPE(ES1373, es1373_device, "es1373", "Creative Labs Ensoniq AudioPCI97 ES1373")

DEVICE_ADDRESS_MAP_START(map, 32, es1373_device)
	AM_RANGE(0x00, 0x3f) AM_READWRITE  (reg_r,  reg_w)
ADDRESS_MAP_END

es1373_device::es1373_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: pci_device(mconfig, ES1373, tag, owner, clock),
		device_sound_interface(mconfig, *this), m_stream(nullptr),
		m_eslog(nullptr), m_tempCount(0), m_timer(nullptr), m_memory_space(nullptr), m_cpu_tag(nullptr), m_cpu(nullptr),
		m_irq_num(-1)
{
}

void es1373_device::set_irq_info(const char *tag, const int irq_num)
{
	m_cpu_tag = tag;
	m_irq_num = irq_num;
}

//-------------------------------------------------
//  device_stop - device-specific stop
//-------------------------------------------------
void es1373_device::device_stop()
{
	/* debugging */
	if (LOG_ES_FILE && m_eslog)
	{
		fclose(m_eslog);
		m_eslog = nullptr;
	}
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------
void es1373_device::device_start()
{
	m_cpu = machine().device<cpu_device>(m_cpu_tag);
	pci_device::device_start();
	add_map(0x40, M_IO, FUNC(es1373_device::map));

	// create the stream
	m_stream = machine().sound().stream_alloc(*this, 0, 2, 44100/2);

	m_timer = timer_alloc(0, nullptr);
	m_timer->adjust(attotime::zero, 0, attotime::from_hz(44100/2/16));

	// Save states
	save_item(NAME(m_ac97_regs));
	save_item(NAME(m_es_regs));
	save_item(NAME(m_sound_cache));
	save_item(NAME(m_src_ram));
	save_item(NAME(m_dac1.number));
	save_item(NAME(m_dac1.enable));
	save_item(NAME(m_dac1.int_en));
	save_item(NAME(m_dac1.loop_en));
	save_item(NAME(m_dac1.initialized));
	save_item(NAME(m_dac1.format));
	save_item(NAME(m_dac1.buf_wptr));
	save_item(NAME(m_dac1.buf_rptr));
	save_item(NAME(m_dac1.buf_count));
	save_item(NAME(m_dac1.buf_size));
	save_item(NAME(m_dac1.pci_addr));
	save_item(NAME(m_dac1.pci_count));
	save_item(NAME(m_dac1.pci_size));
	save_item(NAME(m_dac2.number));
	save_item(NAME(m_dac2.enable));
	save_item(NAME(m_dac2.int_en));
	save_item(NAME(m_dac2.loop_en));
	save_item(NAME(m_dac2.initialized));
	save_item(NAME(m_dac2.format));
	save_item(NAME(m_dac2.buf_wptr));
	save_item(NAME(m_dac2.buf_rptr));
	save_item(NAME(m_dac2.buf_count));
	save_item(NAME(m_dac2.buf_size));
	save_item(NAME(m_dac2.pci_addr));
	save_item(NAME(m_dac2.pci_count));
	save_item(NAME(m_dac2.pci_size));
	save_item(NAME(m_adc.number));
	save_item(NAME(m_adc.enable));
	save_item(NAME(m_adc.int_en));
	save_item(NAME(m_adc.loop_en));
	save_item(NAME(m_adc.initialized));
	save_item(NAME(m_adc.format));
	save_item(NAME(m_adc.buf_wptr));
	save_item(NAME(m_adc.buf_rptr));
	save_item(NAME(m_adc.buf_count));
	save_item(NAME(m_adc.buf_size));
	save_item(NAME(m_adc.pci_addr));
	save_item(NAME(m_adc.pci_count));
	save_item(NAME(m_adc.pci_size));
	machine().save().register_postload(save_prepost_delegate(FUNC(es1373_device::postload), this));
}

void es1373_device::postload()
{
	remap_cb();
}

void es1373_device::device_reset()
{
	// debugging
	m_tempCount = 0;
	if (LOG_ES_FILE && m_eslog)
	{
		fclose(m_eslog);
		m_eslog = nullptr;
	}
	if (LOG_ES_FILE && !m_eslog)
		m_eslog = fopen("es.log", "w");

	pci_device::device_reset();
	memset(m_es_regs, 0, sizeof(m_es_regs));
	memset(m_ac97_regs, 0, sizeof(m_ac97_regs));
	m_ac97_regs[0] = 0x0800;
	// Reset ADC channel info
	m_adc.number = 0;
	m_adc.enable = false;
	m_adc.initialized = false;
	m_adc.buf_rptr = 0x20;
	m_adc.buf_wptr = 0x20;
	// Reset DAC1 channel info
	m_dac1.number = 1;
	m_dac1.enable = false;
	m_dac1.initialized = false;
	m_dac1.buf_rptr = 0x0;
	m_dac1.buf_wptr = 0x0;
	// Reset DAC2 channel info
	m_dac2.number = 2;
	m_dac2.enable = false;
	m_dac2.initialized = false;
	m_dac2.buf_rptr = 0x10;
	m_dac2.buf_wptr = 0x10;  // Start PCI writing to bottom half of buffer

	m_stream->update();
}

void es1373_device::map_extra(uint64_t memory_window_start, uint64_t memory_window_end, uint64_t memory_offset, address_space *memory_space,
							uint64_t io_window_start, uint64_t io_window_end, uint64_t io_offset, address_space *io_space)
{
	m_memory_space = memory_space;
}

//-------------------------------------------------
//  device_timer - called when our device timer expires
//-------------------------------------------------
void es1373_device::device_timer(emu_timer &timer, device_timer_id tid, int param, void *ptr)
{
	m_stream->update();
}

//-------------------------------------------------
//  sound_stream_update - handle update requests for
//  our sound stream
//-------------------------------------------------
void es1373_device::sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples)
{
	if (m_dac1.enable) {
		logerror("%s: sound_stream_update DAC1 not implemented yet\n", tag());
	}

	if (m_dac2.enable) {
		send_audio_out(m_dac2, ICSTATUS_DAC2_INT_MASK, outputs[0], outputs[1], samples);
	}

	if (m_adc.enable) {
		if (m_adc.format!=SCTRL_16BIT_MONO) {
			logerror("%s: sound_stream_update Only SCTRL_16BIT_MONO recorded supported\n", tag());
		} else {
			for (int i=0; i<samples; i++) {
				if (m_adc.buf_count<=m_adc.buf_size) {
					if (LOG_ES)
						logerror("%s: ADC buf_count: %i buf_size: %i buf_rptr: %i buf_wptr: %i\n", machine().describe_context(),
							m_adc.buf_count, m_adc.buf_size, m_adc.buf_rptr, m_adc.buf_wptr);
					if ((m_adc.buf_count&0x1)) {
						m_adc.buf_wptr++;
					}
					m_adc.buf_count++;
					if (m_adc.buf_count>m_adc.buf_size) {
						if (m_adc.loop_en) {
							// Keep playing
							m_adc.buf_count = 0;
							if (LOG_ES)
								logerror("%X: send_audio_out ADC clearing buf_count\n", machine().device("maincpu")->safe_pc());
						}
						if (m_adc.int_en) {
							m_es_regs[ES_INT_CS_STATUS] |= ICSTATUS_ADC_INT_MASK;
							if (LOG_ES)
								logerror("%X: send_audio_out Setting ADC interrupt\n", machine().device("maincpu")->safe_pc());
						}
					}
					if (!(m_adc.buf_count&1) && !(m_adc.buf_wptr&0xf)) {
						m_adc.buf_wptr -= 0x10;
					}
					// PCI Write Transfer
					if (command & 0x4) {
						if ((m_adc.buf_rptr&8)^(m_adc.buf_wptr&8)) {
							transfer_pci_audio(m_adc, ES_PCI_WRITE);
						}
					}
				}
			}
		}
	}
	if (m_es_regs[ES_INT_CS_STATUS]&(ICSTATUS_DAC1_INT_MASK|ICSTATUS_DAC2_INT_MASK|ICSTATUS_ADC_INT_MASK)) {
		m_es_regs[ES_INT_CS_STATUS] |= ICSTATUS_INTR_MASK;
		// Assert interrupt
		//m_cpu->set_input_line(ES_IRQ_NUM, ASSERT_LINE);
		if (m_irq_num!=-1) {
			m_cpu->set_input_line(m_irq_num, ASSERT_LINE);
		}
	}
}

//-------------------------------------------------
//  send_audio_out - Sends channel audio output data
//-------------------------------------------------
void es1373_device::send_audio_out(chan_info& chan, uint32_t intr_mask, stream_sample_t *outL, stream_sample_t *outR, int samples)
{
	// Only transfer PCI data if bus mastering is enabled
	// Fill initial half buffer
	if (1 && (command & 0x4) && (!chan.initialized)) {
		chan.initialized = true;
		transfer_pci_audio(chan, ES_PCI_READ);
	}
	//uint32_t sample_size = calc_size(chan.format);
	// Send data to sound stream
	bool buf_row_done;
	for (int i=0; i<samples; i++) {
		buf_row_done = false;
		if (chan.buf_count<=chan.buf_size) {
			// Only transfer PCI data if bus mastering is enabled
			// Fill half-buffer when read pointer is at start of next half
			//if ((command & 0x4) && ((chan.buf_rptr&8)^(chan.buf_wptr&8)) && !(m_es_regs[ES_INT_CS_STATUS] & intr_mask)) {
			if ((command & 0x4) && ((chan.buf_rptr&8)^(chan.buf_wptr&8))) {
				transfer_pci_audio(chan, ES_PCI_READ);
			}
			if (LOG_ES && i==0)
				logerror("%X: chan: %X samples: %i buf_count: %X buf_size: %X buf_rptr: %X buf_wptr: %X\n",
					machine().device("maincpu")->safe_pc(), chan.number, samples, chan.buf_count, chan.buf_size, chan.buf_rptr, chan.buf_wptr);
			// Buffer is 4 bytes per location, need to switch on sample mode
			switch (chan.format) {
				case SCTRL_8BIT_MONO:
					logerror("es1373_device::send_audio_out SCTRL_8BIT_MONO not implemented yet\n");
					break;
				case SCTRL_8BIT_STEREO:
					logerror("es1373_device::send_audio_out SCTRL_8BIT_STEREO not implemented yet\n");
					break;
				case SCTRL_16BIT_MONO:
						// The sound cache is 32 bit wide fifo, so each entry is two mono 16 bit samples
						if ((chan.buf_count&0x1)) {
							// Read high 16 bits
							outL[i] = outR[i] = (int16_t)(m_sound_cache[chan.buf_rptr]>>16);
							chan.buf_rptr++;
							buf_row_done = true;
						} else {
							// Read low 16 bits
							outL[i] = outR[i] = (int16_t)(m_sound_cache[chan.buf_rptr]&0xffff);
						}
					break;
				case SCTRL_16BIT_STEREO:
						// The sound cache is 32 bit wide fifo, so each entry is one stereo 16 bit sample
						outL[i] = (int16_t) m_sound_cache[chan.buf_rptr]&0xffff;
						outR[i] = (int16_t) m_sound_cache[chan.buf_rptr]>>16;
						chan.buf_rptr++;
						buf_row_done = true;
					break;
			}
			if (LOG_ES_FILE && m_tempCount<1000000) {
				m_tempCount++;
				//logerror("es1373_device::sound_stream_update count: %i samp16: %X\n", i, samp16);
				//if (LOG_ES_FILE && m_eslog)
					//fprintf(m_eslog, "%i\n", samp16);
			}
			chan.buf_count++;
			if (chan.buf_count > chan.buf_size) {
				if (chan.loop_en) {
					// Keep playing
					//chan.buf_count -= 1;  // Should check SCTRL_P2_END_MASK
					chan.buf_count = 0;
					//chan.buf_rptr -= 1;
					if (LOG_ES)
						logerror("%X: send_audio_out DAC2 clearing buf_count\n", machine().device("maincpu")->safe_pc());
				}
				if (chan.int_en) {
					m_es_regs[ES_INT_CS_STATUS] |= intr_mask;
					if (LOG_ES)
						logerror("%X: send_audio_out Setting DAC2 interrupt\n", machine().device("maincpu")->safe_pc());
				}
			}
			if (buf_row_done && !(chan.buf_rptr&0xf)) {
				chan.buf_rptr -= 0x10;
			}
		} else {
			// Send zeros?
			outL[i] = outR[i] = 0;
		}
	}
}

void es1373_device::transfer_pci_audio(chan_info& chan, int type)
{
	uint32_t pci_addr, data;
	pci_addr = chan.pci_addr + (chan.pci_count<<2);
	if (LOG_ES)
		logerror("%s: transfer_pci_audio start chan: %X pci_addr: %08X pci_count: %X pci_size: %X buf_rptr: %X buf_wptr: %X\n",
			machine().describe_context(), chan.number, pci_addr, chan.pci_count, chan.pci_size, chan.buf_rptr, chan.buf_wptr);
	// Always transfer 8 longwords
	for (int i=0; i<8; i++) {
		pci_addr = chan.pci_addr + (chan.pci_count<<2);
		if (type==ES_PCI_READ) {
			data = m_memory_space->read_dword(pci_addr, 0xffffffff);
			m_sound_cache[chan.buf_wptr++] = data;
			if (!(chan.buf_wptr&0xf)) {
				chan.buf_wptr -= 0x10;
			}
		} else {
			data = m_sound_cache[chan.buf_rptr++];
			m_memory_space->write_dword(pci_addr, data);
			if (!(chan.buf_rptr&0xf)) {
				chan.buf_rptr -= 0x10;
			}
		}
		if (chan.pci_count==chan.pci_size) {
			chan.pci_count = 0;
		} else {
			chan.pci_count++;
		}
	}
}

uint32_t es1373_device::calc_size(const uint8_t &format)
{
	switch (format) {
		case SCTRL_8BIT_MONO:
			return 1;
			break;
		case SCTRL_8BIT_STEREO:
			return 2;
			break;
		case SCTRL_16BIT_MONO:
			return 2;
			break;
		case SCTRL_16BIT_STEREO:
			return 4;
			break;
	}
	logerror("%s: calc_size Invalid format = %X specified\n", tag(), format);
	return 0;
}

READ32_MEMBER (es1373_device::reg_r)
{
	uint32_t result = m_es_regs[offset];
	switch (offset) {
		case ES_CODEC:
			break;
		case ES_DAC2_CNT:
				result = ((m_dac2.buf_size-m_dac2.buf_count)<<16) | m_dac2.buf_size;
			break;
		case ES_HOST_IF0: // 0x30
			result = m_sound_cache[(m_es_regs[ES_MEM_PAGE]<<2) | 0x0];
			switch (m_es_regs[ES_MEM_PAGE]&0xf) {
				case 0xc:
					result = m_dac1.pci_addr;
					break;
				case 0xd:
					result = m_adc.pci_addr;
					break;
				default:
					break;
			}
			break;
		case ES_HOST_IF1: // 0x34
			result = m_sound_cache[(m_es_regs[ES_MEM_PAGE]<<2) | 0x1];
			switch (m_es_regs[ES_MEM_PAGE]&0xf) {
				case 0xc:
					result = (m_dac1.pci_count<<16) | m_dac1.pci_size;
					break;
				case 0xd:
					result = (m_adc.pci_count<<16) | m_adc.pci_size;
					break;
				default:
					break;
			}
			break;
		case ES_HOST_IF2: // 0x38
			result = m_sound_cache[(m_es_regs[ES_MEM_PAGE]<<2) | 0x2];
			switch (m_es_regs[ES_MEM_PAGE]&0xf) {
				case 0xc:
					result = m_dac2.pci_addr;
					break;
				default:
					break;
			}
			break;
		case ES_HOST_IF3: // 0x3C
			result = m_sound_cache[(m_es_regs[ES_MEM_PAGE]<<2) | 0x3];
			switch (m_es_regs[ES_MEM_PAGE]&0xf) {
				case 0xc:
					result = ((m_dac2.pci_count)<<16) | m_dac2.pci_size;
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}
	if (LOG_ES_REG)
		logerror("%08X:ES1373 read from offset %02X = %08X & %08X\n", machine().device("maincpu")->safe_pc(), offset*4, result, mem_mask);
	return result;
}

WRITE32_MEMBER(es1373_device::reg_w)
{
	COMBINE_DATA(&m_es_regs[offset]);
	switch (offset) {
		case ES_INT_CS_CTRL:
				m_dac1.enable = (m_es_regs[ES_INT_CS_CTRL] & ICCTRL_DAC1_EN_MASK);
				m_dac2.enable = (m_es_regs[ES_INT_CS_CTRL] & ICCTRL_DAC2_EN_MASK);
				m_adc.enable = (m_es_regs[ES_INT_CS_CTRL] & ICCTRL_ADC_EN_MASK);
			break;
		case ES_SRC_IF:
			if (data&(1<<24)) {
				// Write to Sample Rate Converter Ram
				m_src_ram[(data>>25)&0x7F] = data&0xFFFF;
			} else {
				// Read From Sample Rate Converter Ram
				m_es_regs[offset] = (data&0xFFFF0000) | m_src_ram[(data>>25)&0x7F];
			}
			break;
		case ES_CODEC:
			if (data&(1<<23)) {
				// Read from AC97 codec registers
				m_es_regs[offset] = (data&0xFFFF0000) | m_ac97_regs[(data>>16)&0x7f] | 0x80000000;
			} else {
				// Write to AC97 codec registers
				m_ac97_regs[(data>>16)&0x7f] = data&0xFFFF;
			}
			break;
		case ES_SERIAL_CTRL:
				m_adc.loop_en  = !(m_es_regs[ES_SERIAL_CTRL] & SCTRL_R1_LOOP_MASK);
				m_dac2.loop_en = !(m_es_regs[ES_SERIAL_CTRL] & SCTRL_P2_LOOP_MASK);
				m_dac1.loop_en = !(m_es_regs[ES_SERIAL_CTRL] & SCTRL_P1_LOOP_MASK);
				m_adc.int_en  = m_es_regs[ES_SERIAL_CTRL] & SCTRL_R1_INT_EN_MASK;
				m_dac2.int_en = m_es_regs[ES_SERIAL_CTRL] & SCTRL_P2_INT_EN_MASK;
				m_dac1.int_en = m_es_regs[ES_SERIAL_CTRL] & SCTRL_P1_INT_EN_MASK;
				m_adc.format = (m_es_regs[ES_SERIAL_CTRL] & SCTRL_R1_S_MASK)>>4;
				m_dac2.format = (m_es_regs[ES_SERIAL_CTRL] & SCTRL_P2_S_MASK)>>2;
				m_dac1.format = (m_es_regs[ES_SERIAL_CTRL] & SCTRL_P1_S_MASK)>>0;
				if (!m_adc.int_en) m_es_regs[ES_INT_CS_STATUS]  &= ~ICSTATUS_ADC_INT_MASK;
				if (!m_dac1.int_en) m_es_regs[ES_INT_CS_STATUS] &= ~ICSTATUS_DAC1_INT_MASK;
				if (!m_dac2.int_en) m_es_regs[ES_INT_CS_STATUS] &= ~ICSTATUS_DAC2_INT_MASK;
				// Clear the summary interrupt and irq line
				if (!(m_es_regs[ES_INT_CS_STATUS]&(ICSTATUS_DAC1_INT_MASK|ICSTATUS_DAC2_INT_MASK|ICSTATUS_ADC_INT_MASK))) {
					// Deassert interrupt
					if (m_es_regs[ES_INT_CS_STATUS]&ICSTATUS_INTR_MASK && m_irq_num!=-1) {
						m_cpu->set_input_line(m_irq_num, CLEAR_LINE);
						m_es_regs[ES_INT_CS_STATUS] &= ~ICSTATUS_INTR_MASK;
						if (0 && LOG_ES_REG)
							logerror("%X: es1373_device::reg_w Clearing interrupt\n", machine().device("maincpu")->safe_pc());
					}
				}
				if (0 && LOG_ES_REG)
					logerror("%s: es1373_device::reg_w adc_int_en: %i dac1_int_en: %i dac2_int_en: %i\n", tag(), m_adc.int_en, m_dac1.int_en, m_dac2.int_en);
			break;
		case ES_DAC2_CNT:
				m_dac2.buf_count = 0;
				m_dac2.buf_size = data&0xffff;
			break;
		case ES_HOST_IF0: // 0x30
			m_sound_cache[(m_es_regs[ES_MEM_PAGE]<<2) | 0x0] = data;
			switch (m_es_regs[ES_MEM_PAGE]&0xf) {
				case 0xc:
					m_dac1.pci_addr = data;
					break;
				case 0xd:
					m_adc.pci_addr = data;
					break;
				default:
					break;
			}
			break;
		case ES_HOST_IF1: // 0x34
			m_sound_cache[(m_es_regs[ES_MEM_PAGE]<<2) | 0x1] = data;
			switch (m_es_regs[ES_MEM_PAGE]&0xf) {
				case 0xc:
					m_dac1.pci_count = (data>>16)&0xffff;
					m_dac1.pci_size = data&0xffff;
					break;
				case 0xd:
					m_adc.pci_count = (data>>16)&0xffff;
					m_adc.pci_size = data&0xffff;
					break;
				default:
					break;
			}
			break;
		case ES_HOST_IF2: // 0x38
			m_sound_cache[(m_es_regs[ES_MEM_PAGE]<<2) | 0x2] = data;
			switch (m_es_regs[ES_MEM_PAGE]&0xf) {
				case 0xc:
					m_dac2.pci_addr = data;
					break;
				default:
					break;
			}
			break;
		case ES_HOST_IF3: // 0x3C
			m_sound_cache[(m_es_regs[ES_MEM_PAGE]<<2) | 0x3] = data;
			switch (m_es_regs[ES_MEM_PAGE]&0xf) {
				case 0xc:
					m_dac2.pci_count = (data>>16)&0xffff;
					m_dac2.pci_size = data&0xffff;
					if (LOG_ES_REG)
						logerror("%08X:ES1373 write to offset %02X = %08X & %08X\n", machine().device("maincpu")->safe_pc(), offset*4, data, mem_mask);
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}

	if (LOG_ES_REG)
		logerror("%08X:ES1373 write to offset %02X = %08X & %08X\n", machine().device("maincpu")->safe_pc(), offset*4, data, mem_mask);

}
