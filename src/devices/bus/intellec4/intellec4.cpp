// license:BSD-3-Clause
// copyright-holders:Vas Crabb

#include "emu.h"
#include "intellec4.h"

#include <algorithm>


DEFINE_DEVICE_TYPE_NS(INTELLEC4_UNIV_SLOT, bus::intellec4, univ_slot_device, "intlc4univslot", "INTELLEC 4/MOD 40 Universal Slot")
DEFINE_DEVICE_TYPE_NS(INTELLEC4_UNIV_BUS,  bus::intellec4, univ_bus_device,  "intlc4univbus",  "INTELLEC 4/MOD 40 Universal Bus")


namespace bus { namespace intellec4 {

/***********************************************************************
    SLOT DEVICE
***********************************************************************/

univ_slot_device::univ_slot_device(machine_config const &mconfig, char const *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, INTELLEC4_UNIV_SLOT, tag, owner, clock)
	, device_slot_interface(mconfig, *this)
	, m_bus(*this, finder_base::DUMMY_TAG)
{
}


/*----------------------------------
  configuration helpers
----------------------------------*/

void univ_slot_device::set_bus_tag(device_t &device, char const *tag)
{
	downcast<univ_slot_device &>(device).m_bus.set_tag(tag);
}


/*----------------------------------
  device_t implementation
----------------------------------*/

void univ_slot_device::device_validity_check(validity_checker &valid) const
{
	device_t *const card(get_card_device());
	if (card && !dynamic_cast<device_univ_card_interface *>(card))
		osd_printf_error("Card device %s (%s) does not implement device_univ_card_interface\n", card->tag(), card->name());
}

void univ_slot_device::device_start()
{
	device_t *const card_device(get_card_device());
	device_univ_card_interface *const univ_card(dynamic_cast<device_univ_card_interface *>(card_device));
	if (card_device && !univ_card)
		throw emu_fatalerror("univ_slot_device: card device %s (%s) does not implement device_univ_card_interface\n", card_device->tag(), card_device->name());
	if (univ_card)
		univ_card->set_bus(*m_bus);
}



/***********************************************************************
    BUS DEVICE
***********************************************************************/

univ_bus_device::univ_bus_device(machine_config const &mconfig, char const *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, INTELLEC4_UNIV_BUS, tag, owner, clock)
	, m_rom_device(*this, finder_base::DUMMY_TAG)
	, m_rom_ports_device(*this, finder_base::DUMMY_TAG)
	, m_memory_device(*this, finder_base::DUMMY_TAG)
	, m_status_device(*this, finder_base::DUMMY_TAG)
	, m_ram_ports_device(*this, finder_base::DUMMY_TAG)
	, m_rom_space(-1)
	, m_rom_ports_space(-1)
	, m_memory_space(-1)
	, m_status_space(-1)
	, m_ram_ports_space(-1)
	, m_test_out_cb(*this)
	, m_stop_out_cb(*this)
	, m_reset_4002_out_cb(*this)
	, m_user_reset_out_cb(*this)
	, m_test(0U)
	, m_stop(0U)
	, m_reset_4002(0U)
	, m_user_reset(0U)
{
	std::fill(std::begin(m_cards), std::end(m_cards), nullptr);
}


/*----------------------------------
  address space configuration
----------------------------------*/

void univ_bus_device::set_rom_space(device_t &device, char const *tag, int space)
{
	univ_bus_device &bus(downcast<univ_bus_device &>(device));
	bus.m_rom_device.set_tag(tag);
	bus.m_rom_space = space;
}

void univ_bus_device::set_rom_ports_space(device_t &device, char const *tag, int space)
{
	univ_bus_device &bus(downcast<univ_bus_device &>(device));
	bus.m_rom_ports_device.set_tag(tag);
	bus.m_rom_ports_space = space;
}

void univ_bus_device::set_memory_space(device_t &device, char const *tag, int space)
{
	univ_bus_device &bus(downcast<univ_bus_device &>(device));
	bus.m_memory_device.set_tag(tag);
	bus.m_memory_space = space;
}

void univ_bus_device::set_status_space(device_t &device, char const *tag, int space)
{
	univ_bus_device &bus(downcast<univ_bus_device &>(device));
	bus.m_status_device.set_tag(tag);
	bus.m_status_space = space;
}

void univ_bus_device::set_ram_ports_space(device_t &device, char const *tag, int space)
{
	univ_bus_device &bus(downcast<univ_bus_device &>(device));
	bus.m_ram_ports_device.set_tag(tag);
	bus.m_ram_ports_space = space;
}


/*----------------------------------
  input lines
----------------------------------*/

WRITE_LINE_MEMBER(univ_bus_device::sync_in)
{
	for (device_univ_card_interface *card : m_cards)
	{
		if (card)
			card->sync_in(state);
		else
			break;
	}
}

WRITE_LINE_MEMBER(univ_bus_device::stop_acknowledge_in)
{
	for (device_univ_card_interface *card : m_cards)
	{
		if (card)
			card->stop_acknowledge_in(state);
		else
			break;
	}
}

WRITE_LINE_MEMBER(univ_bus_device::cpu_reset_in)
{
	for (device_univ_card_interface *card : m_cards)
	{
		if (card)
			card->cpu_reset_in(state);
		else
			break;
	}
}


/*----------------------------------
  device_t implementation
----------------------------------*/

void univ_bus_device::device_validity_check(validity_checker &valid) const
{
	if (m_rom_device && !m_rom_device->space_config(m_rom_space))
		osd_printf_error("ROM space device %s (%s) lacks address space %d config\n", m_rom_device->device().tag(), m_rom_device->device().name(), m_rom_space);
	if (m_rom_ports_device && !m_rom_ports_device->space_config(m_rom_ports_space))
		osd_printf_error("ROM ports space device %s (%s) lacks address space %d config\n", m_rom_ports_device->device().tag(), m_rom_ports_device->device().name(), m_rom_ports_space);
	if (m_memory_device && !m_memory_device->space_config(m_memory_space))
		osd_printf_error("Memory space device %s (%s) lacks address space %d config\n", m_memory_device->device().tag(), m_memory_device->device().name(), m_memory_space);
	if (m_status_device && !m_status_device->space_config(m_status_space))
		osd_printf_error("Status space device %s (%s) lacks address space %d config\n", m_status_device->device().tag(), m_status_device->device().name(), m_status_space);
	if (m_ram_ports_device && !m_ram_ports_device->space_config(m_ram_ports_space))
		osd_printf_error("RAM ports space device %s (%s) lacks address space %d config\n", m_ram_ports_device->device().tag(), m_ram_ports_device->device().name(), m_ram_ports_space);
}

void univ_bus_device::device_start()
{
	m_test_out_cb.resolve_safe();
	m_stop_out_cb.resolve_safe();
	m_reset_4002_out_cb.resolve_safe();
	m_user_reset_out_cb.resolve_safe();

	save_item(NAME(m_test));
	save_item(NAME(m_stop));
	save_item(NAME(m_reset_4002));
	save_item(NAME(m_user_reset));
}


/*----------------------------------
  helpers for cards
----------------------------------*/

unsigned univ_bus_device::add_card(device_univ_card_interface &card)
{
	for (unsigned i = 0; ARRAY_LENGTH(m_cards) > i; ++i)
	{
		if (!m_cards[i])
		{
			m_cards[i] = &card;
			return i;
		}
	}
	throw emu_fatalerror("univ_bus_device: maximum number of cards (%u) exceeded\n", unsigned(ARRAY_LENGTH(m_cards)));
}

void univ_bus_device::set_test(unsigned index, int state)
{
	assert(ARRAY_LENGTH(m_cards) >= index);
	bool const changed(bool(state) != !BIT(m_test, index));
	if (changed)
	{
		u16 const other(m_test & ~(u16(1U) << index));
		if (state)
			m_test = other;
		else
			m_test |= u16(1U) << index;

		if ((ARRAY_LENGTH(m_cards) != index) && !(other & ~(u16(1U) << ARRAY_LENGTH(m_cards))))
			m_test_out_cb(state);

		for (unsigned card = 0U; (ARRAY_LENGTH(m_cards) > card) && m_cards[card]; ++card)
		{
			if ((index != card) && !(other & ~(u16(1U) << index)))
				m_cards[card]->test_in(state);
		}
	}
}

void univ_bus_device::set_stop(unsigned index, int state)
{
	assert(ARRAY_LENGTH(m_cards) >= index);
	bool const changed(bool(state) != !BIT(m_stop, index));
	if (changed)
	{
		u16 const other(m_stop & ~(u16(1U) << index));
		if (state)
			m_stop = other;
		else
			m_stop |= u16(1U) << index;

		if ((ARRAY_LENGTH(m_cards) != index) && !(other & ~(u16(1U) << ARRAY_LENGTH(m_cards))))
			m_stop_out_cb(state);

		for (unsigned card = 0U; (ARRAY_LENGTH(m_cards) > card) && m_cards[card]; ++card)
		{
			if ((index != card) && !(other & ~(u16(1U) << index)))
				m_cards[card]->stop_in(state);
		}
	}
}

void univ_bus_device::set_reset_4002(unsigned index, int state)
{
	assert(ARRAY_LENGTH(m_cards) >= index);
	bool const changed(bool(state) != !BIT(m_reset_4002, index));
	if (changed)
	{
		u16 const other(m_reset_4002 & ~(u16(1U) << index));
		if (state)
			m_reset_4002 = other;
		else
			m_reset_4002 |= u16(1U) << index;

		if ((ARRAY_LENGTH(m_cards) != index) && !(other & ~(u16(1U) << ARRAY_LENGTH(m_cards))))
			m_reset_4002_out_cb(state);

		for (unsigned card = 0U; (ARRAY_LENGTH(m_cards) > card) && m_cards[card]; ++card)
		{
			if ((index != card) && !(other & ~(u16(1U) << index)))
				m_cards[card]->reset_4002_in(state);
		}
	}
}

void univ_bus_device::set_user_reset(unsigned index, int state)
{
	assert(ARRAY_LENGTH(m_cards) > index);
	bool const changed(bool(state) != !BIT(m_user_reset, index));
	if (changed)
	{
		u16 const other(m_user_reset & ~(u16(1U) << index));
		if (state)
			m_user_reset = other;
		else
			m_user_reset |= u16(1U) << index;

		if (!other)
			m_user_reset_out_cb(state);

		for (unsigned card = 0U; (ARRAY_LENGTH(m_cards) > card) && m_cards[card]; ++card)
		{
			if ((index != card) && !(other & ~(u16(1U) << index)))
				m_cards[card]->user_reset_in(state);
		}
	}
}



/***********************************************************************
    CARD INTERFACE
***********************************************************************/

device_univ_card_interface::device_univ_card_interface(const machine_config &mconfig, device_t &device)
	: device_slot_card_interface(mconfig, device)
	, m_bus(nullptr)
	, m_index(~unsigned(0))
{
}


void device_univ_card_interface::interface_pre_start()
{
	if (!m_bus)
		throw device_missing_dependencies();
}


void device_univ_card_interface::set_bus(univ_bus_device &bus)
{
	m_index = (m_bus = &bus)->add_card(*this);
}

} } // namespace bus::intellec4



#include "insdatastor.h"
#include "prommemory.h"
#include "tapereader.h"

SLOT_INTERFACE_START(intellec4_univ_cards)
	SLOT_INTERFACE("imm4_22", INTELLEC4_INST_DATA_STORAGE)
	SLOT_INTERFACE("imm6_26", INTELLEC4_PROM_MEMORY)
	SLOT_INTERFACE("imm4_90", INTELLEC4_TAPE_READER)
SLOT_INTERFACE_END
