// license:BSD-3-Clause
// copyright-holders:R. Belmont, O. Galibert
/***************************************************************************

    sis85c496.cpp - SiS 85C496/497 PCI chipset
    by R. Belmont (based on i82439hx.cpp/i82439tx.cpp by O. Galibert)

    Unlike Intel chipsets, the southbridge is not a PCI device;
    it connects via a proprietary bus to the northbridge, and the two
    chips appear to software/the BIOS as a single chip.  Thus we emulate
    them in a single file.

***************************************************************************/

#include "emu.h"
#include "sis85c496.h"
#include "bus/pc_kbd/keyboards.h"
#include "speaker.h"

DEFINE_DEVICE_TYPE(SIS85C496, sis85c496_host_device, "sis85c496", "SiS 85C496/497 chipset")

DEVICE_ADDRESS_MAP_START(config_map, 32, sis85c496_host_device)
	AM_RANGE(0x40, 0x43) AM_READWRITE8(dram_config_r, dram_config_w, 0x000000ff)
	AM_RANGE(0x44, 0x47) AM_READWRITE16(shadow_config_r, shadow_config_w, 0x0000ffff)
	AM_RANGE(0x58, 0x5b) AM_READWRITE8(smram_ctrl_r, smram_ctrl_w, 0x00ff0000)
	AM_RANGE(0xc8, 0xcb) AM_READWRITE(mailbox_r, mailbox_w)
	AM_RANGE(0xd0, 0xd3) AM_READWRITE8(bios_config_r, bios_config_w, 0x000000ff)
	AM_RANGE(0xd0, 0xd3) AM_READWRITE8(isa_decoder_r, isa_decoder_w, 0x0000ff00)

	AM_INHERIT_FROM(pci_host_device::config_map)
ADDRESS_MAP_END

DEVICE_ADDRESS_MAP_START(internal_io_map, 32, sis85c496_host_device)
	AM_RANGE(0x0000, 0x001f) AM_DEVREADWRITE8("dma8237_1", am9517a_device, read, write, 0xffffffff)
	AM_RANGE(0x0020, 0x003f) AM_DEVREADWRITE8("pic8259_master", pic8259_device, read, write, 0xffffffff)
	AM_RANGE(0x0040, 0x005f) AM_DEVREADWRITE8("pit8254",   pit8254_device, read, write, 0xffffffff)
	AM_RANGE(0x0060, 0x0063) AM_READWRITE8(at_keybc_r, at_keybc_w, 0xffffffff);
	AM_RANGE(0x0064, 0x0067) AM_DEVREADWRITE8("keybc", at_keyboard_controller_device, status_r, command_w, 0xffffffff);
	AM_RANGE(0x0070, 0x007f) AM_DEVREADWRITE8("rtc", ds12885_device, read, write, 0xffffffff);
	AM_RANGE(0x0080, 0x009f) AM_READWRITE8(at_page8_r, at_page8_w, 0xffffffff);
	AM_RANGE(0x00a0, 0x00bf) AM_DEVREADWRITE8("pic8259_slave", pic8259_device, read, write, 0xffffffff)
	AM_RANGE(0x00c0, 0x00df) AM_READWRITE8(at_dma8237_2_r, at_dma8237_2_w, 0xffffffff);
	AM_RANGE(0x00e0, 0x00ef) AM_NOP

	AM_INHERIT_FROM(pci_host_device::io_configuration_access_map)
ADDRESS_MAP_END

MACHINE_CONFIG_MEMBER(sis85c496_host_device::device_add_mconfig)
	MCFG_DEVICE_ADD("pit8254", PIT8254, 0)
	MCFG_PIT8253_CLK0(4772720/4) /* heartbeat IRQ */
	MCFG_PIT8253_OUT0_HANDLER(WRITELINE(sis85c496_host_device, at_pit8254_out0_changed))
	MCFG_PIT8253_CLK1(4772720/4) /* dram refresh */
	MCFG_PIT8253_OUT1_HANDLER(WRITELINE(sis85c496_host_device, at_pit8254_out1_changed))
	MCFG_PIT8253_CLK2(4772720/4) /* pio port c pin 4, and speaker polling enough */
	MCFG_PIT8253_OUT2_HANDLER(WRITELINE(sis85c496_host_device, at_pit8254_out2_changed))

	MCFG_DEVICE_ADD( "dma8237_1", AM9517A, XTAL_14_31818MHz/3 )
	MCFG_I8237_OUT_HREQ_CB(DEVWRITELINE("dma8237_2", am9517a_device, dreq0_w))
	MCFG_I8237_OUT_EOP_CB(WRITELINE(sis85c496_host_device, at_dma8237_out_eop))
	MCFG_I8237_IN_MEMR_CB(READ8(sis85c496_host_device, pc_dma_read_byte))
	MCFG_I8237_OUT_MEMW_CB(WRITE8(sis85c496_host_device, pc_dma_write_byte))
	MCFG_I8237_IN_IOR_0_CB(READ8(sis85c496_host_device, pc_dma8237_0_dack_r))
	MCFG_I8237_IN_IOR_1_CB(READ8(sis85c496_host_device, pc_dma8237_1_dack_r))
	MCFG_I8237_IN_IOR_2_CB(READ8(sis85c496_host_device, pc_dma8237_2_dack_r))
	MCFG_I8237_IN_IOR_3_CB(READ8(sis85c496_host_device, pc_dma8237_3_dack_r))
	MCFG_I8237_OUT_IOW_0_CB(WRITE8(sis85c496_host_device, pc_dma8237_0_dack_w))
	MCFG_I8237_OUT_IOW_1_CB(WRITE8(sis85c496_host_device, pc_dma8237_1_dack_w))
	MCFG_I8237_OUT_IOW_2_CB(WRITE8(sis85c496_host_device, pc_dma8237_2_dack_w))
	MCFG_I8237_OUT_IOW_3_CB(WRITE8(sis85c496_host_device, pc_dma8237_3_dack_w))
	MCFG_I8237_OUT_DACK_0_CB(WRITELINE(sis85c496_host_device, pc_dack0_w))
	MCFG_I8237_OUT_DACK_1_CB(WRITELINE(sis85c496_host_device, pc_dack1_w))
	MCFG_I8237_OUT_DACK_2_CB(WRITELINE(sis85c496_host_device, pc_dack2_w))
	MCFG_I8237_OUT_DACK_3_CB(WRITELINE(sis85c496_host_device, pc_dack3_w))

	MCFG_DEVICE_ADD( "dma8237_2", AM9517A, XTAL_14_31818MHz/3 )
	MCFG_I8237_OUT_HREQ_CB(WRITELINE(sis85c496_host_device, pc_dma_hrq_changed))
	MCFG_I8237_IN_MEMR_CB(READ8(sis85c496_host_device, pc_dma_read_word))
	MCFG_I8237_OUT_MEMW_CB(WRITE8(sis85c496_host_device, pc_dma_write_word))
	MCFG_I8237_IN_IOR_1_CB(READ8(sis85c496_host_device, pc_dma8237_5_dack_r))
	MCFG_I8237_IN_IOR_2_CB(READ8(sis85c496_host_device, pc_dma8237_6_dack_r))
	MCFG_I8237_IN_IOR_3_CB(READ8(sis85c496_host_device, pc_dma8237_7_dack_r))
	MCFG_I8237_OUT_IOW_1_CB(WRITE8(sis85c496_host_device, pc_dma8237_5_dack_w))
	MCFG_I8237_OUT_IOW_2_CB(WRITE8(sis85c496_host_device, pc_dma8237_6_dack_w))
	MCFG_I8237_OUT_IOW_3_CB(WRITE8(sis85c496_host_device, pc_dma8237_7_dack_w))
	MCFG_I8237_OUT_DACK_0_CB(WRITELINE(sis85c496_host_device, pc_dack4_w))
	MCFG_I8237_OUT_DACK_1_CB(WRITELINE(sis85c496_host_device, pc_dack5_w))
	MCFG_I8237_OUT_DACK_2_CB(WRITELINE(sis85c496_host_device, pc_dack6_w))
	MCFG_I8237_OUT_DACK_3_CB(WRITELINE(sis85c496_host_device, pc_dack7_w))

	MCFG_PIC8259_ADD( "pic8259_master", INPUTLINE(":maincpu", 0), VCC, READ8(sis85c496_host_device, get_slave_ack) )
	MCFG_PIC8259_ADD( "pic8259_slave", DEVWRITELINE("pic8259_master", pic8259_device, ir2_w), GND, NOOP)

	MCFG_DEVICE_ADD("keybc", AT_KEYBOARD_CONTROLLER, XTAL_12MHz)
	MCFG_AT_KEYBOARD_CONTROLLER_SYSTEM_RESET_CB(INPUTLINE(":maincpu", INPUT_LINE_RESET))
	MCFG_AT_KEYBOARD_CONTROLLER_GATE_A20_CB(INPUTLINE(":maincpu", INPUT_LINE_A20))
	MCFG_AT_KEYBOARD_CONTROLLER_INPUT_BUFFER_FULL_CB(DEVWRITELINE("pic8259_master", pic8259_device, ir1_w))
	MCFG_AT_KEYBOARD_CONTROLLER_KEYBOARD_CLOCK_CB(DEVWRITELINE("pc_kbdc", pc_kbdc_device, clock_write_from_mb))
	MCFG_AT_KEYBOARD_CONTROLLER_KEYBOARD_DATA_CB(DEVWRITELINE("pc_kbdc", pc_kbdc_device, data_write_from_mb))
	MCFG_DEVICE_ADD("pc_kbdc", PC_KBDC, 0)
	MCFG_PC_KBDC_OUT_CLOCK_CB(DEVWRITELINE("keybc", at_keyboard_controller_device, keyboard_clock_w))
	MCFG_PC_KBDC_OUT_DATA_CB(DEVWRITELINE("keybc", at_keyboard_controller_device, keyboard_data_w))
	MCFG_PC_KBDC_SLOT_ADD("pc_kbdc", "kbd", pc_at_keyboards, STR_KBD_MICROSOFT_NATURAL)

	MCFG_DS12885_ADD("rtc")
	MCFG_MC146818_IRQ_HANDLER(DEVWRITELINE("pic8259_slave", pic8259_device, ir0_w))
	MCFG_MC146818_CENTURY_INDEX(0x32)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_ADD("speaker", SPEAKER_SOUND, 0)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.50)
MACHINE_CONFIG_END


sis85c496_host_device::sis85c496_host_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: pci_host_device(mconfig, SIS85C496, tag, owner, clock),
	m_maincpu(*this, ":maincpu"),
	m_pic8259_master(*this, "pic8259_master"),
	m_pic8259_slave(*this, "pic8259_slave"),
	m_dma8237_1(*this, "dma8237_1"),
	m_dma8237_2(*this, "dma8237_2"),
	m_pit8254(*this, "pit8254"),
	m_keybc(*this, "keybc"),
	m_speaker(*this, "speaker"),
	m_ds12885(*this, "rtc"),
	m_pc_kbdc(*this, "pc_kbdc"),
	m_at_spkrdata(0), m_pit_out2(0), m_dma_channel(0), m_cur_eop(false), m_dma_high_byte(0), m_at_speaker(0), m_refresh(false), m_channel_check(0), m_nmi_enabled(0)
{
}

void sis85c496_host_device::set_cpu_tag(const char *_cpu_tag)
{
	cpu_tag = _cpu_tag;
}

void sis85c496_host_device::set_ram_size(int _ram_size)
{
	ram_size = _ram_size;
}

void sis85c496_host_device::device_start()
{
	pci_host_device::device_start();

	cpu = machine().device<cpu_device>(cpu_tag);
	memory_space = &cpu->space(AS_PROGRAM);
	io_space = &cpu->space(AS_IO);

	memory_window_start = 0;
	memory_window_end   = 0xffffffff;
	memory_offset       = 0;
	io_window_start = 0;
	io_window_end   = 0xffff;
	io_offset       = 0;
	status = 0x0010;

	m_bios_config = 0x78;
	m_dram_config = 0;
	m_isa_decoder = 0xff;
	m_shadctrl = 0;
	m_smramctrl = 0;

	ram.resize(ram_size/4);
}

void sis85c496_host_device::reset_all_mappings()
{
	pci_host_device::reset_all_mappings();
}

void sis85c496_host_device::device_reset()
{
	pci_host_device::device_reset();

	m_at_spkrdata = 0;
	m_pit_out2 = 1;
	m_dma_channel = -1;
	m_cur_eop = false;
	m_nmi_enabled = 0;
	m_refresh = false;

	m_bios_config = 0x78;
	m_dram_config = 0;
	m_isa_decoder = 0xff;
	m_shadctrl = 0;
	m_smramctrl = 0;
}

void sis85c496_host_device::map_bios(address_space *memory_space, uint32_t start, uint32_t end)
{
	uint32_t mask = m_region->bytes() - 1;
	memory_space->install_rom(start, end, m_region->base() + (start & mask));
}

void sis85c496_host_device::map_shadowram(address_space *memory_space, offs_t addrstart, offs_t addrend, void *baseptr)
{
	if (m_shadctrl & 0x100) // write protected?
	{
		memory_space->install_rom(addrstart, addrend, baseptr);
	}
	else
	{
		memory_space->install_ram(addrstart, addrend, baseptr);
	}
}

void sis85c496_host_device::map_extra(uint64_t memory_window_start, uint64_t memory_window_end, uint64_t memory_offset, address_space *memory_space,
									 uint64_t io_window_start, uint64_t io_window_end, uint64_t io_offset, address_space *io_space)
{
	logerror("SiS496: mapping!\n");
	io_space->install_device(0, 0xffff, *this, &sis85c496_host_device::internal_io_map);

	// is SMRAM at e0000?  overrides shadow if so
	if ((m_smramctrl & 0x16) == 0x16)
	{
		if (m_smramctrl & 0x08)
		{
			memory_space->install_ram(0x000e0000, 0x000effff, &ram[0x000b0000/4]);
			logerror("Sis496: SMRAM at Exxxx, phys Bxxxx\n");
		}
		else
		{
			memory_space->install_ram(0x000e0000, 0x000effff, &ram[0x000a0000/4]);
			logerror("Sis496: SMRAM at Exxxx, phys Axxxx\n");
		}

		// map the high BIOS at FFFExxxx if enabled
		if (m_bios_config & 0x40)
		{
			map_bios(memory_space, 0xfffe0000, 0xfffeffff);
		}
	}
	else
	{
		// does shadow RAM actually require this to be set?  can't tell w/Megatouch BIOS.
		if (m_bios_config & 0x40)
		{
			logerror("SiS496: BIOS at Exxxx\n");
			map_bios(memory_space, 0xfffe0000, 0xfffeffff);

			if ((m_shadctrl & 0x30) == 0)
			{
				map_bios(memory_space, 0x000e0000, 0x000effff);
			}
			else    // at least one 32K block has shadow memory
			{
				if (m_shadctrl & 0x20)
				{
					logerror("SiS496: shadow RAM at e8000\n");
					map_shadowram(memory_space, 0x000e8000, 0x000effff, &ram[0x000e8000/4]);
				}

				if (m_shadctrl & 0x10)
				{
					logerror("SiS496: shadow RAM at e0000\n");
					map_shadowram(memory_space, 0x000e0000, 0x000e7fff, &ram[0x000e0000/4]);
				}
			}
		}
	}
	if (m_bios_config & 0x20)
	{
		map_bios(memory_space, 0xffff0000, 0xffffffff);

		if ((m_shadctrl & 0xc0) == 0)
		{
			map_bios(memory_space, 0x000f0000, 0x000fffff);
			logerror("SiS496: BIOS at Fxxxx\n");
		}
		else    // at least one 32K block has shadow memory
		{
			if (m_shadctrl & 0x80)
			{
				logerror("SiS496: shadow RAM at f8000\n");
				map_shadowram(memory_space, 0x000f8000, 0x000fffff, &ram[0x000f8000/4]);
			}

			if (m_shadctrl & 0x40)
			{
				logerror("SiS496: shadow RAM at f0000\n");
				map_shadowram(memory_space, 0x000f0000, 0x000f7fff, &ram[0x000f0000/4]);
			}
		}
	}

	if (m_shadctrl & 0x08)
	{
		logerror("SiS496: shadow RAM at d8000\n");
		memory_space->install_ram(0x000d8000, 0x000dffff, &ram[0x000d8000/4]);
	}
	if (m_shadctrl & 0x04)
	{
		logerror("SiS496: shadow RAM at d0000\n");
		memory_space->install_ram(0x000d0000, 0x000d7fff, &ram[0x000d0000/4]);
	}
	if (m_shadctrl & 0x02)
	{
		logerror("SiS496: shadow RAM at c8000\n");
		memory_space->install_ram(0x000c8000, 0x000cffff, &ram[0x000c8000/4]);
	}
	if (m_shadctrl & 0x01)
	{
		logerror("SiS496: shadow RAM at d8000\n");
		memory_space->install_ram(0x000c0000, 0x000c7fff, &ram[0x000c0000/4]);
	}

	// is SMRAM enabled at 6xxxx?
	if ((m_smramctrl & 0x12) == 0x02)
	{
		fatalerror("Sis486: SMRAM enabled at 6xxxx, not yet supported!\n");
	}

	if (m_isa_decoder & 0x01)
	{
		logerror("SiS496: ISA base 640K enabled\n");
		memory_space->install_ram(0x00000000, 0x0009ffff, &ram[0x00000000/4]);
	}

	// 32 megs of RAM (todo: don't hardcode)
	memory_space->install_ram(0x00100000, 0x01ffffff, &ram[0x00100000/4]);
}

// Southbridge
READ8_MEMBER( sis85c496_host_device::get_slave_ack )
{
	if (offset==2) // IRQ = 2
		return m_pic8259_slave->acknowledge();

	return 0x00;
}

void sis85c496_host_device::at_speaker_set_spkrdata(uint8_t data)
{
	m_at_spkrdata = data ? 1 : 0;
	m_speaker->level_w(m_at_spkrdata & m_pit_out2);
}



WRITE_LINE_MEMBER( sis85c496_host_device::at_pit8254_out0_changed )
{
	if (m_pic8259_master)
		m_pic8259_master->ir0_w(state);
}

WRITE_LINE_MEMBER( sis85c496_host_device::at_pit8254_out1_changed )
{
	if(state)
		m_refresh = !m_refresh;
}

WRITE_LINE_MEMBER( sis85c496_host_device::at_pit8254_out2_changed )
{
	m_pit_out2 = state ? 1 : 0;
	m_speaker->level_w(m_at_spkrdata & m_pit_out2);
}

READ8_MEMBER( sis85c496_host_device::at_page8_r )
{
	uint8_t data = m_at_pages[offset % 0x10];

	switch(offset % 8)
	{
	case 1:
		data = m_dma_offset[BIT(offset, 3)][2];
		break;
	case 2:
		data = m_dma_offset[BIT(offset, 3)][3];
		break;
	case 3:
		data = m_dma_offset[BIT(offset, 3)][1];
		break;
	case 7:
		data = m_dma_offset[BIT(offset, 3)][0];
		break;
	}
	return data;
}


WRITE8_MEMBER( sis85c496_host_device::at_page8_w )
{
	m_at_pages[offset % 0x10] = data;

	switch(offset % 8)
	{
	case 0:
		//m_boot_state_hook((offs_t)0, data);
		break;
	case 1:
		m_dma_offset[BIT(offset, 3)][2] = data;
		break;
	case 2:
		m_dma_offset[BIT(offset, 3)][3] = data;
		break;
	case 3:
		m_dma_offset[BIT(offset, 3)][1] = data;
		break;
	case 7:
		m_dma_offset[BIT(offset, 3)][0] = data;
		break;
	}
}


WRITE_LINE_MEMBER( sis85c496_host_device::pc_dma_hrq_changed )
{
	m_maincpu->set_input_line(INPUT_LINE_HALT, state ? ASSERT_LINE : CLEAR_LINE);

	/* Assert HLDA */
	m_dma8237_2->hack_w( state );
}

READ8_MEMBER(sis85c496_host_device::pc_dma_read_byte)
{
	address_space& prog_space = m_maincpu->space(AS_PROGRAM); // get the right address space
	if(m_dma_channel == -1)
		return 0xff;
	uint8_t result;
	offs_t page_offset = ((offs_t) m_dma_offset[0][m_dma_channel]) << 16;

	result = prog_space.read_byte(page_offset + offset);
	return result;
}


WRITE8_MEMBER(sis85c496_host_device::pc_dma_write_byte)
{
	address_space& prog_space = m_maincpu->space(AS_PROGRAM); // get the right address space
	if(m_dma_channel == -1)
		return;
	offs_t page_offset = ((offs_t) m_dma_offset[0][m_dma_channel]) << 16;

	prog_space.write_byte(page_offset + offset, data);
}


READ8_MEMBER(sis85c496_host_device::pc_dma_read_word)
{
	address_space& prog_space = m_maincpu->space(AS_PROGRAM); // get the right address space
	if(m_dma_channel == -1)
		return 0xff;
	uint16_t result;
	offs_t page_offset = ((offs_t) m_dma_offset[1][m_dma_channel & 3]) << 16;

	result = prog_space.read_word((page_offset & 0xfe0000) | (offset << 1));
	m_dma_high_byte = result & 0xFF00;

	return result & 0xFF;
}


WRITE8_MEMBER(sis85c496_host_device::pc_dma_write_word)
{
	address_space& prog_space = m_maincpu->space(AS_PROGRAM); // get the right address space
	if(m_dma_channel == -1)
		return;
	offs_t page_offset = ((offs_t) m_dma_offset[1][m_dma_channel & 3]) << 16;

	prog_space.write_word((page_offset & 0xfe0000) | (offset << 1), m_dma_high_byte | data);
}


READ8_MEMBER( sis85c496_host_device::pc_dma8237_0_dack_r ) { return 0; } //m_isabus->dack_r(0); }
READ8_MEMBER( sis85c496_host_device::pc_dma8237_1_dack_r ) { return 0; } //m_isabus->dack_r(1); }
READ8_MEMBER( sis85c496_host_device::pc_dma8237_2_dack_r ) { return 0; } //m_isabus->dack_r(2); }
READ8_MEMBER( sis85c496_host_device::pc_dma8237_3_dack_r ) { return 0; } //m_isabus->dack_r(3); }
READ8_MEMBER( sis85c496_host_device::pc_dma8237_5_dack_r ) { return 0; } //m_isabus->dack_r(5); }
READ8_MEMBER( sis85c496_host_device::pc_dma8237_6_dack_r ) { return 0; } //m_isabus->dack_r(6); }
READ8_MEMBER( sis85c496_host_device::pc_dma8237_7_dack_r ) { return 0; } //m_isabus->dack_r(7); }


WRITE8_MEMBER( sis85c496_host_device::pc_dma8237_0_dack_w ){ } //m_isabus->dack_w(0, data); }
WRITE8_MEMBER( sis85c496_host_device::pc_dma8237_1_dack_w ){ } //m_isabus->dack_w(1, data); }
WRITE8_MEMBER( sis85c496_host_device::pc_dma8237_2_dack_w ){ } //m_isabus->dack_w(2, data); }
WRITE8_MEMBER( sis85c496_host_device::pc_dma8237_3_dack_w ){ } //m_isabus->dack_w(3, data); }
WRITE8_MEMBER( sis85c496_host_device::pc_dma8237_5_dack_w ){ } //m_isabus->dack_w(5, data); }
WRITE8_MEMBER( sis85c496_host_device::pc_dma8237_6_dack_w ){ } //m_isabus->dack_w(6, data); }
WRITE8_MEMBER( sis85c496_host_device::pc_dma8237_7_dack_w ){ } //m_isabus->dack_w(7, data); }

WRITE_LINE_MEMBER( sis85c496_host_device::at_dma8237_out_eop )
{
	m_cur_eop = state == ASSERT_LINE;
	//if(m_dma_channel != -1)
//      m_isabus->eop_w(m_dma_channel, m_cur_eop ? ASSERT_LINE : CLEAR_LINE );
}

void sis85c496_host_device::pc_select_dma_channel(int channel, bool state)
{
	if(!state) {
		m_dma_channel = channel;
		//if(m_cur_eop)
//          m_isabus->eop_w(channel, ASSERT_LINE );

	} else if(m_dma_channel == channel) {
		m_dma_channel = -1;
		//if(m_cur_eop)
//          m_isabus->eop_w(channel, CLEAR_LINE );
	}
}


WRITE_LINE_MEMBER( sis85c496_host_device::pc_dack0_w ) { pc_select_dma_channel(0, state); }
WRITE_LINE_MEMBER( sis85c496_host_device::pc_dack1_w ) { pc_select_dma_channel(1, state); }
WRITE_LINE_MEMBER( sis85c496_host_device::pc_dack2_w ) { pc_select_dma_channel(2, state); }
WRITE_LINE_MEMBER( sis85c496_host_device::pc_dack3_w ) { pc_select_dma_channel(3, state); }
WRITE_LINE_MEMBER( sis85c496_host_device::pc_dack4_w ) { m_dma8237_1->hack_w( state ? 0 : 1); } // it's inverted
WRITE_LINE_MEMBER( sis85c496_host_device::pc_dack5_w ) { pc_select_dma_channel(5, state); }
WRITE_LINE_MEMBER( sis85c496_host_device::pc_dack6_w ) { pc_select_dma_channel(6, state); }
WRITE_LINE_MEMBER( sis85c496_host_device::pc_dack7_w ) { pc_select_dma_channel(7, state); }

READ8_MEMBER( sis85c496_host_device::at_portb_r )
{
	uint8_t data = m_at_speaker;
	data &= ~0xd0; /* AT BIOS don't likes this being set */

	/* 0x10 is the dram refresh line bit on the 5170, just a timer here, 15.085us. */
	data |= m_refresh ? 0x10 : 0;

	if (m_pit_out2)
		data |= 0x20;
	else
		data &= ~0x20; /* ps2m30 wants this */

	return data;
}

WRITE8_MEMBER( sis85c496_host_device::at_portb_w )
{
	m_at_speaker = data;
	m_pit8254->write_gate2(BIT(data, 0));
	at_speaker_set_spkrdata( BIT(data, 1));
	m_channel_check = BIT(data, 3);
	//m_isabus->set_nmi_state((m_nmi_enabled==0) && (m_channel_check==0));
}

READ8_MEMBER( sis85c496_host_device::at_dma8237_2_r )
{
	return m_dma8237_2->read( space, offset / 2);
}

WRITE8_MEMBER( sis85c496_host_device::at_dma8237_2_w )
{
	m_dma8237_2->write( space, offset / 2, data);
}

READ8_MEMBER( sis85c496_host_device::at_keybc_r )
{
	switch (offset)
	{
	case 0: return m_keybc->data_r(space, 0);
	case 1: return at_portb_r(space, 0);
	}

	return 0xff;
}

WRITE8_MEMBER( sis85c496_host_device::at_keybc_w )
{
	switch (offset)
	{
	case 0: m_keybc->data_w(space, 0, data); break;
	case 1: at_portb_w(space, 0, data); break;
	}
}


WRITE8_MEMBER( sis85c496_host_device::write_rtc )
{
	if (offset==0) {
		m_nmi_enabled = BIT(data,7);
		//m_isabus->set_nmi_state((m_nmi_enabled==0) && (m_channel_check==0));
		m_ds12885->write(space,0,data);
	}
	else {
		m_ds12885->write(space,offset,data);
	}
}

/*

after decompress to shadow RAM:

config_write 00:05.0:40 00000004 @ 000000ff
[:pci:05.0] ':maincpu' (000FF6D8): unmapped configuration_space memory write to 28058 = 00120000 & 00FF0000 SMRAM: e0000 to SMRAM, enable
config_write 00:05.0:58 00120000 @ 00ff0000
[:pci:05.0] ':maincpu' (000FF6D8): unmapped configuration_space memory write to 28058 = 00040000 & 00FF0000 SMRAM: always enable
config_write 00:05.0:58 00040000 @ 00ff0000
[:pci:05.0] ':maincpu' (000FF6D8): unmapped configuration_space memory write to 28058 = 00000000 & 00FF0000 SMRAM: disable
config_write 00:05.0:58 00000000 @ 00ff0000
[:pci:05.0] ':maincpu' (000FF6D8): unmapped configuration_space memory write to 280A0 = 0000FF00 & 0000FF00 SMI: clear all requests
config_write 00:05.0:a0 0000ff00 @ 0000ff00
[:pci:05.0] ':maincpu' (000FF6D8): unmapped configuration_space memory write to 280A0 = 000000FF & 000000FF SMI: clear all requests
config_write 00:05.0:a0 000000ff @ 000000ff
[:pci:05.0] ':maincpu' (000FF6D8): unmapped configuration_space memory write to 2808C = 00000500 & 0000FF00 SMI: timer count
config_write 00:05.0:8c 00000500 @ 0000ff00
[:pci:05.0] ':maincpu' (000FF6D8): unmapped configuration_space memory write to 2809C = 00020000 & 00FF0000 SMI: start countdown timer
config_write 00:05.0:9c 00020000 @ 00ff0000
[:pci:05.0] ':maincpu' (000FF6D8): unmapped configuration_space memory write to 28084 = 00000006 & 000000FF clear deturbo and break switch blocks
config_write 00:05.0:84 00000006 @ 000000ff
[:pci:05.0] ':maincpu' (000FF6D8): unmapped configuration_space memory write to 28080 = 00000004 & 000000FF enable soft-SMI
config_write 00:05.0:80 00000004 @ 000000ff
[:pci:05.0] ':maincpu' (000FF6D8): unmapped configuration_space memory write to 280A0 = 00100000 & 00FF0000 select software SMI request
config_write 00:05.0:a0 00100000 @ 00ff0000
[:pci:05.0] ':maincpu' (000FF6D8): unmapped configuration_space memory write to 2809C = 00010000 & 00FF0000 assert SMI
config_write 00:05.0:9c 00010000 @ 00ff0000
[:pci:05.0] ':maincpu' (000FF6D8): unmapped configuration_space memory write to 280C4 = 00080000 & 00FF0000 IRQ routing: undocumented value
config_write 00:05.0:c4 00080000 @ 00ff0000
[:pci:05.0] ':maincpu' (000FF6D8): unmapped configuration_space memory write to 28080 = 00000000 & 000000FF clear all SMI
config_write 00:05.0:80 00000000 @ 000000ff



*/
