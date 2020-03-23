// license:BSD-3-Clause
// copyright-holders:Vas Crabb
/*
INTELLEC® 4 Universal Slot

                         1    2  /TEST
                  GND    3    4  GND
                   NC    5    6  NC
                   NC    7    8  NC
                   NC    9   10  NC
                  MA0   11   12  MA1
                  MA2   13   14  MA3
                  MA4   15   16  MA5
                  MA6   17   18  MA7
                   C0   19   20  C1
                        21   22  NC
                /MDI0   23   24  NC
                /MDI1   25   26
                /MDI3   27   28
                /MDI2   29   30
                /MDI5   31   32
                /MDI4   33   34
                /MDI7   35   36
                /MDI6   37   38
                        39   40
                 /OUT   41   42  /ENABLE MON PROM
                 -10V   43   44  -10V
           /CPU RESET   45   46  /USER RESET
             /CM-RAM2   47   48  /CM-RAM3
             /CM-RAM0   49   50  /CM-RAM1
                I/O 1   51   52  I/O 0
                I/O 2   53   54  /IN
                  F/L   55   56  I/O 3
                        57   58
                        59   60
                        61   62
                        63   64
                        65   66
                        67   68
                        69   70
                        71   72  /D3
    /STOP ACKNOWLEDGE   73   74  /STOP
                        75   76  /D2
                        77   78
                /SYNC   79   80  /D1
                        81   82  /PROM SEL
                  /D0   83   84
                        85   86
                        87   88  /RESET-4002
                        89   90
                        91   92
              /CM-ROM   93   94  C3
                    W   95   96  C2
              PHASE 2   97   98  PHASE 1
                  +5V   99  100  +5V

NC                      Not connected on backplane
GND                     Common ground
+5V                     Power supply
-10V                    Power supply
PHASE1                  5.185MHz/7 clock phase 1
PHASE2                  5.185MHz/7 clock phase 2
/SYNC                   CPU /SYNC output (instruction cycle synchronisation)
/TEST                   CPU /TEST input, wired-or (cards should pull low to assert)
/STOP                   CPU /STP input, wired-or (cards should pull low to assert)
/STOP ACKNOWLEDGE       CPU /STP ACK output (stop/halt acknowledge)
/CM-ROM                 CPU /CM-ROM0 output (ROM chip select)
/CM-RAM0.../CM-RAM3     CPU /CM-RAM outputs (RAM chip select)
/D0.../D3               CPU data bus
MA0...MA7               A outputs from 4289 on CPU board (low ROM address/SRC address)
C0...C3                 C outputs from 4289 on CPU board (high ROM address/I/O chip select)
W                       PM output from 4289 on CPU board (program memory enable)
F/L                     FL output from 4289 on CPU board (program memory nybble select)
I/O 0...I/O 3           I/O lines to 4289 on CPU board (bidirectional I/O data)
/MDI0.../MDI7           OPR/OPA multiplexer inputs on CPU board (ROM data when onboard monitor PROM is not selected)
/IN                     IN output from 4289 on CPU board (I/O read strobe)
/OUT                    OUT output from 4289 on CPU board (I/O write strobe)
/CPU RESET              CPU/4289 reset output from control board
/RESET-4002             Open collector output from control board
/ENABLE MON PROM        Output from control board
/PROM SEL               Output from control board
/USER RESET             Input to control board, edge sensitive, wired or (cards should pull low to assert)

other pins connected between universal slots but not connected to CPU or control cards
pins 73 and 74 are connected between cards but not otherwise used on MOD 4 systems
interrupt request/acknowledge are supposedly connected somewhere on MOD 40 systems

Cards can install handlers in the ROM, ROM ports, memory, status, and
RAM ports spaces.

For the ROM space, cards can install handlers for monitor and/or PROM
mode:
0x0000-0x0fff   /ENABLE MON PROM asserted
0x1000-0x1fff   /PROM asserted

For the ROM ports space, cards can install handlers for monitor mode,
PROM mode, and/or neither (RAM mode or two switches pressed at once):
0x0000-0x07ff   /ENABLE MON PROM asserted
0x0800-0x0fff   /PROM asserted
0x1000-0x17ff   neither asserted

It's assumed that the memory, status and RAM ports space are only used
for 4002 memories, and cards don't use the program storage selection
lines to enable/disable them.  These spaces correspond directly to the
CPU's view.

Some of the cards can also be used in INTELLEC® 8 systems, using a
different set of pins.  No provisions are made for using the same class
to implement the card in both systems.
*/
#ifndef MAME_BUS_INTELLEC4_INTELLEC4_H
#define MAME_BUS_INTELLEC4_INTELLEC4_H

#pragma once


#define MCFG_INTELLEC4_UNIV_SLOT_ADD(bus_tag, slot_tag, clock, slot_intf, def_slot) \
		MCFG_DEVICE_ADD(slot_tag, INTELLEC4_UNIV_SLOT, clock) \
		MCFG_DEVICE_SLOT_INTERFACE(slot_intf, def_slot, false) \
		bus::intellec4::univ_slot_device::set_bus_tag(*device, "^" bus_tag);

#define MCFG_INTELLEC4_UNIV_SLOT_REMOVE(slot_tag) \
		MCFG_DEVICE_REMOVE(slot_tag)


#define MCFG_INTELLEC4_UNIV_BUS_ROM_SPACE(tag, space) \
		bus::intellec4::univ_bus_device::set_rom_space(*device, "^" tag, space);

#define MCFG_INTELLEC4_UNIV_BUS_ROM_PORTS_SPACE(tag, space) \
		bus::intellec4::univ_bus_device::set_rom_ports_space(*device, "^" tag, space);

#define MCFG_INTELLEC4_UNIV_BUS_MEMORY_SPACE(tag, space) \
		bus::intellec4::univ_bus_device::set_memory_space(*device, "^" tag, space);

#define MCFG_INTELLEC4_UNIV_BUS_STATUS_SPACE(tag, space) \
		bus::intellec4::univ_bus_device::set_status_space(*device, "^" tag, space);

#define MCFG_INTELLEC4_UNIV_BUS_RAM_PORTS_SPACE(tag, space) \
		bus::intellec4::univ_bus_device::set_ram_ports_space(*device, "^" tag, space);

#define MCFG_INTELLEC4_UNIV_BUS_TEST_CB(obj) \
		bus::intellec4::univ_bus_device::set_test_out_cb(*device, DEVCB_##obj);

#define MCFG_INTELLEC4_UNIV_BUS_STOP_CB(obj) \
		bus::intellec4::univ_bus_device::set_stop_out_cb(*device, DEVCB_##obj);

#define MCFG_INTELLEC4_UNIV_BUS_RESET_4002_CB(obj) \
		bus::intellec4::univ_bus_device::set_reset_4002_out_cb(*device, DEVCB_##obj);

#define MCFG_INTELLEC4_UNIV_BUS_USER_RESET_CB(obj) \
		bus::intellec4::univ_bus_device::set_user_reset_out_cb(*device, DEVCB_##obj);


namespace bus { namespace intellec4 {

class univ_slot_device;
class univ_bus_device;
class device_univ_card_interface;


class univ_slot_device : public device_t, public device_slot_interface
{
public:
	// configuration helpers
	static void set_bus_tag(device_t &device, char const *bus_tag);

	univ_slot_device(machine_config const &mconfig, char const *tag, device_t *owner, uint32_t clock);

protected:
	// device_t implementation
	virtual void device_validity_check(validity_checker &valid) const override ATTR_COLD;
	virtual void device_start() override;

private:
	required_device<univ_bus_device>    m_bus;
};


class univ_bus_device : public device_t
{
public:
	friend class device_univ_card_interface;

	// address space configuration
	static void set_rom_space(device_t &device, char const *tag, int space);
	static void set_rom_ports_space(device_t &device, char const *tag, int space);
	static void set_memory_space(device_t &device, char const *tag, int space);
	static void set_status_space(device_t &device, char const *tag, int space);
	static void set_ram_ports_space(device_t &device, char const *tag, int space);

	// callback configuration
	template <typename Obj> static devcb_base &set_stop_out_cb(device_t &device, Obj &&cb)
	{ return downcast<univ_bus_device &>(device).m_stop_out_cb.set_callback(std::forward<Obj>(cb)); }
	template <typename Obj> static devcb_base &set_test_out_cb(device_t &device, Obj &&cb)
	{ return downcast<univ_bus_device &>(device).m_test_out_cb.set_callback(std::forward<Obj>(cb)); }
	template <typename Obj> static devcb_base &set_reset_4002_out_cb(device_t &device, Obj &&cb)
	{ return downcast<univ_bus_device &>(device).m_reset_4002_out_cb.set_callback(std::forward<Obj>(cb)); }
	template <typename Obj> static devcb_base &set_user_reset_out_cb(device_t &device, Obj &&cb)
	{ return downcast<univ_bus_device &>(device).m_user_reset_out_cb.set_callback(std::forward<Obj>(cb)); }

	univ_bus_device(machine_config const &mconfig, char const *tag, device_t *owner, uint32_t clock);

	// input lines
	DECLARE_WRITE_LINE_MEMBER(sync_in);
	DECLARE_WRITE_LINE_MEMBER(test_in) {set_test(ARRAY_LENGTH(m_cards), state); }
	DECLARE_WRITE_LINE_MEMBER(stop_in) {set_stop(ARRAY_LENGTH(m_cards), state); }
	DECLARE_WRITE_LINE_MEMBER(stop_acknowledge_in);
	DECLARE_WRITE_LINE_MEMBER(cpu_reset_in);
	DECLARE_WRITE_LINE_MEMBER(reset_4002_in) { set_reset_4002(ARRAY_LENGTH(m_cards), state); }

	// output lines
	DECLARE_READ_LINE_MEMBER(test_out) const        { return (m_test        & ~u16(1U)) ? 0 : 1; }
	DECLARE_READ_LINE_MEMBER(stop_out) const        { return (m_stop        & ~u16(1U)) ? 0 : 1; }
	DECLARE_READ_LINE_MEMBER(reset_4002_out) const  { return (m_reset_4002  & ~u16(1U)) ? 0 : 1; }
	DECLARE_READ_LINE_MEMBER(user_reset_out) const  { return (m_user_reset  & ~u16(1U)) ? 0 : 1; }

protected:
	// device_t implementation
	virtual void device_validity_check(validity_checker &valid) const override ATTR_COLD;
	virtual void device_start() override;

private:
	// helpers for cards
	unsigned add_card(device_univ_card_interface &card);
	void set_test(unsigned index, int state);
	void set_stop(unsigned index, int state);
	void set_reset_4002(unsigned index, int state);
	void set_user_reset(unsigned index, int state);

	// finding address spaces
	required_device<device_memory_interface>    m_rom_device, m_rom_ports_device;
	required_device<device_memory_interface>    m_memory_device, m_status_device, m_ram_ports_device;
	int                                         m_rom_space, m_rom_ports_space;
	int                                         m_memory_space, m_status_space, m_ram_ports_space;

	// output line callbacks
	devcb_write_line    m_test_out_cb;
	devcb_write_line    m_stop_out_cb;
	devcb_write_line    m_reset_4002_out_cb;
	devcb_write_line    m_user_reset_out_cb;

	// cards
	device_univ_card_interface  *m_cards[15];

	// packed line states
	u16 m_test, m_stop, m_reset_4002, m_user_reset;
};


class device_univ_card_interface : public device_slot_card_interface
{
protected:
	friend class univ_slot_device;
	friend class univ_bus_device;

	device_univ_card_interface(const machine_config &mconfig, device_t &device);

	// device_interface implementation
	void interface_pre_start() override;

	address_space &rom_space()          { return m_bus->m_rom_device->space(m_bus->m_rom_space); }
	address_space &rom_ports_space()    { return m_bus->m_rom_ports_device->space(m_bus->m_rom_ports_space); }
	address_space &memory_space()       { return m_bus->m_memory_device->space(m_bus->m_memory_space); }
	address_space &status_space()       { return m_bus->m_status_device->space(m_bus->m_status_space); }
	address_space &ram_ports_space()    { return m_bus->m_ram_ports_device->space(m_bus->m_ram_ports_space); }

	DECLARE_WRITE_LINE_MEMBER(test_out)         { m_bus->set_test(m_index, state); }
	DECLARE_WRITE_LINE_MEMBER(stop_out)         { m_bus->set_stop(m_index, state); }
	DECLARE_WRITE_LINE_MEMBER(reset_4002_out)   { m_bus->set_reset_4002(m_index, state); }
	DECLARE_WRITE_LINE_MEMBER(user_reset_out)   { m_bus->set_user_reset(m_index, state); }

	virtual DECLARE_WRITE_LINE_MEMBER(sync_in)              { }
	virtual DECLARE_WRITE_LINE_MEMBER(test_in)              { }
	virtual DECLARE_WRITE_LINE_MEMBER(stop_in)              { }
	virtual DECLARE_WRITE_LINE_MEMBER(stop_acknowledge_in)  { }
	virtual DECLARE_WRITE_LINE_MEMBER(cpu_reset_in)         { }
	virtual DECLARE_WRITE_LINE_MEMBER(reset_4002_in)        { }
	virtual DECLARE_WRITE_LINE_MEMBER(user_reset_in)        { }

private:
	void set_bus(univ_bus_device &bus);

	univ_bus_device *m_bus;
	unsigned        m_index;
};

} } // namespace bus::intellec4


DECLARE_DEVICE_TYPE_NS(INTELLEC4_UNIV_SLOT, bus::intellec4, univ_slot_device)
DECLARE_DEVICE_TYPE_NS(INTELLEC4_UNIV_BUS,  bus::intellec4, univ_bus_device)

SLOT_INTERFACE_EXTERN( intellec4_univ_cards );

#endif // MAME_BUS_INTELLEC4_INTELLEC4_H
