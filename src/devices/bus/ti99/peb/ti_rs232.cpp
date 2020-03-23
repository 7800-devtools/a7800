// license:LGPL-2.1+
// copyright-holders:Michael Zapf
/****************************************************************************

    TI-99 RS232 and Parallel interface card

    TI99 RS232 card ('rs232')
    TMS9902 ('rs232:tms9902_0')
    TMS9902 ('rs232:tms9902_1')
    TI99 RS232 attached serial device ('rs232:serdev0')
    TI99 RS232 attached serial device ('rs232:serdev1')
    TI99 PIO attached parallel device ('rs232:piodev')

    Currently this emulation does not directly interact with the serial
    interface on the host computer. However, using a socket connection it is
    possible to attach an external bridge which interacts with a real UART.

    TI RS232 card wiring
    --------------------
    The card uses this wiring (inverters not included)

     +-----+         Pins of the connector
     | 9902|          (common naming)
     | RIN |---<-------  2  (TD)
     | XOUT|--->-------  3  (RD)
     | RTS |--->-------  8  (DCD)
     | CTS |-<-+------- 20  (DTR)
     | DSR |-<-+    H--> 6  (DSR)
     +-----+     +-----> 5  (CTS)
                /
     +-----+   /
     | CRU |--+
     +-----+

    This wiring is typical for a DCE, not a DTE. The TI RS232 was obviously
    designed to look like a modem. The advantage is that you can use the same
    cables for connecting a modem to the RS232 interface or for connecting
    a second TI via its interface. To connect to a DTE you can use a 1-1
    wiring cable (1 on 1, 2 on 2 ...)

    The TI manual for the RS232 card suggests the following cables:

    TI RS232   -    Modem or other TI RS232
      2 -----<----- 3
      3 ----->----- 2
      6 ----->---- 20         (crossover cable)
     20 -----<----- 6

    TI RS232   -    Terminal (DTE)
      2 ----<------ 2
      3 ---->------ 3
      5 ---->------ 5
      6 ---->------ 6         (1-1 cable)
      8 ---->------ 8
     20 ----<------20

    If we want to use a PC serial interface to play the role of the TI
    interface we have to map the TI wiring to a suitable wiring for PC
    interfaces which are designed as DTEs. This is achieved by the functions
    map_lines_in, map_lines_out.

    Note that we now have to swap the cable types: Use a 1-1 cable to connect
    another TI or a modem on the other end, and use a crossover cable for
    another PC (the usual way of connecting).

    RS232 Over IP protocol
    ----------------------
    This implementation can make use of such an external bridge. Normal data
    are forwarded to the bridge and back, while line control is organized via
    special byte sequences. These sequences are introduced by a 0x1B byte (ESC).

    The protocol has two modes: normal and escape

    normal mode: transmit byte (!= 0x1b) unchanged
    escape mode: entered by ESC, bytes following:
       ESC = plain ESC byte
       length byte[length] = control sequence (length != 0x1b)

       byte[]:
          All configuration settings are related to a specified UART; UARTs may
          differ in their capabilities and may require specific settings
          (e.g. the TMS9902 specifies the line speed by a clock ratio, while
          others may have indexed, fixed rates or use integers)

          (x=unused)

          1ccc xaaa = configuration of parameter ccc; UART type aaa
             1111 xaaa rrrr rrrr rrrr 0000     = config receive rate on aaa
             1110 xaaa rrrr rrrr rrrr 0000     = config transmit rate on aaa
             1101 xaaa xxxx xxbb               = config databits bb (00=5 ... 11=8)
             1100 xaaa xxxx xxss               = config stop bits ss (00=1.5, 01=2, 1x=1)
             1011 xaaa xxxx xxpp               = config parity pp (1x=enable, x1=odd)

          00ab cdef = line state of RTS=a, CTS=b, DSR=c, DCD=d, DTR=e, RI=f
          01gh i000 = exception g=BRK, h=FRMERR, i=PARERR

    The protocol changes back to normal mode after transmitting the control
    sequence.

    Michael Zapf
    February 2012: Rewritten as class

*****************************************************************************/

#include "emu.h"
#include "ti_rs232.h"

DEFINE_DEVICE_TYPE_NS(TI99_RS232,     bus::ti99::peb, ti_rs232_pio_device,      "ti99_rs232",           "TI-99 RS232/PIO interface")
DEFINE_DEVICE_TYPE_NS(TI99_RS232_DEV, bus::ti99::peb, ti_rs232_attached_device, "ti99_rs232_atttached", "TI-99 Serial attached device")
DEFINE_DEVICE_TYPE_NS(TI99_PIO_DEV,   bus::ti99::peb, ti_pio_attached_device,   "ti99_pio_attached",    "TI-99 Parallel attached device")

namespace bus { namespace ti99 { namespace peb {

#define SENILA_0_BIT 0x80
#define SENILA_1_BIT 0x40

#define RECV_MODE_NORMAL 1
#define RECV_MODE_ESC 2
#define RECV_MODE_ESC_LINES 3

#define TRACE_LINES 0
#define TRACE_SETTING 0
#define TRACE_STATE 0
#define TRACE_MAP 0
#define TRACE_IN 0
#define TRACE_OUT 0
#define TRACE_ILA 0

#define ESC 0x1b

ti_rs232_pio_device::ti_rs232_pio_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, TI99_RS232, tag, owner, clock),
	device_ti99_peribox_card_interface(mconfig, *this),
	m_piodev(nullptr),
	m_dsrrom(nullptr),
	m_pio_direction_in(false),
	m_pio_handshakeout(false),
	m_pio_handshakein(false),
	m_pio_spareout(false),
	m_pio_sparein(false),
	m_flag0(false),
	m_led(false),
	m_pio_out_buffer(0),
	m_pio_in_buffer(0),
	m_pio_readable(false),
	m_pio_writable(false),
	m_pio_write(false),
	m_ila(0)
{
}


/**************************************************************************/
/* Ports */

ti_rs232_attached_device::ti_rs232_attached_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, TI99_RS232_DEV, tag, owner, clock),
	device_image_interface(mconfig, *this)
{
}

ti_pio_attached_device::ti_pio_attached_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, TI99_PIO_DEV, tag, owner, clock),
	device_image_interface(mconfig, *this)
{
}

void ti_rs232_attached_device::device_start()
{
}

void ti_pio_attached_device::device_start()
{
}

/*
    Find the index of the image name. We assume the format
    <name><number>, i.e. the number is the longest string from the right
    which can be interpreted as a number.
*/
int ti_rs232_attached_device::get_index_from_tagname()
{
	const char *mytag = tag();
	int maxlen = strlen(mytag);
	int i;
	for (i=maxlen-1; i >=0; i--)
		if (mytag[i] < 48 || mytag[i] > 57) break;

	return atoi(mytag+i+1);
}

/*
    Initialize rs232 unit and open image
*/
image_init_result ti_rs232_attached_device::call_load()
{
	tms9902_device* tms9902;

	int devnumber = get_index_from_tagname();
	if (devnumber==0)
	{
		tms9902 = siblingdevice<tms9902_device>("tms9902_0");
		// Turn on polling
		tms9902->set_clock(true);
	}
	else if (devnumber==1)
	{
		tms9902 = siblingdevice<tms9902_device>("tms9902_1");
		// Turn on polling
		tms9902->set_clock(true);
	}
	else
	{
		logerror("Could not find device tag number\n");
		return image_init_result::FAIL;
	}

	// The following line may cause trouble in the init phase
	// card->incoming_dtr(devnumber, (m_file!=nullptr)? ASSERT_LINE : CLEAR_LINE);

	return image_init_result::PASS;  // OK
}

void ti_rs232_attached_device::call_unload()
{
	tms9902_device* tms9902;

	int devnumber = get_index_from_tagname();
	if (devnumber==0)
	{
		tms9902 = siblingdevice<tms9902_device>("tms9902_0");
		// Turn off polling
		tms9902->set_clock(false);
	}
	else if (devnumber==1)
	{
		tms9902 = siblingdevice<tms9902_device>("tms9902_1");
		// Turn off polling
		tms9902->set_clock(false);
	}
}

/*
    Initialize pio unit and open image
*/
image_init_result ti_pio_attached_device::call_load()
{
	ti_rs232_pio_device* card = static_cast<ti_rs232_pio_device*>(owner());

	// tell whether the image is readable
	card->m_pio_readable = true;
	// tell whether the image is writable
	card->m_pio_writable = !is_readonly();

	if (card->m_pio_write && card->m_pio_writable)
		card->m_pio_handshakein = false;    // receiver ready
	else
		card->m_pio_handshakein = true;

	return image_init_result::PASS;  // OK
}

/*
    close a pio image
*/
void ti_pio_attached_device::call_unload()
{
	ti_rs232_pio_device* card = static_cast<ti_rs232_pio_device*>(owner());

	card->m_pio_writable = false;
	card->m_pio_handshakein = true; /* receiver not ready */
	card->m_pio_sparein = false;
}

/****************************************************************************/

/*
    CRU read
*/
READ8Z_MEMBER(ti_rs232_pio_device::crureadz)
{
	if ((offset & 0xff00)==m_cru_base)
	{
		if ((offset & 0x00c0)==0x0000)
		{
			uint8_t reply = 0x00;
			if (m_pio_direction_in)         reply |= 0x02;
			if (m_pio_handshakein)          reply |= 0x04;
			if (m_pio_sparein)              reply |= 0x08;
			if (m_flag0)                    reply |= 0x10;
			// The CTS line is realized as CRU bits
			// Mind that this line is handled as an output going to the remote CTS
			if ((m_signals[0] & tms9902_device::CTS)!=0)    reply |= 0x20;
			if ((m_signals[1] & tms9902_device::CTS)!=0)    reply |= 0x40;
			if (m_led)                      reply |= 0x80;
			*value = reply;
			return;
		}
		if ((offset & 0x00c0)==0x0040)
		{
			*value = m_uart[0]->cruread(space, offset>>4, 0xff);
			return;
		}
		if ((offset & 0x00c0)==0x0080)
		{
			*value = m_uart[1]->cruread(space, offset>>4, 0xff);
			return;
		}
	}
}

/*
    CRU write
*/
WRITE8_MEMBER(ti_rs232_pio_device::cruwrite)
{
	if ((offset & 0xff00)==m_cru_base)
	{
		if ((offset & 0x00c0)==0x0040)
		{
			m_uart[0]->cruwrite(space, offset>>1, data, 0xff);
			return;
		}
		if ((offset & 0x00c0)==0x0080)
		{
			m_uart[1]->cruwrite(space, offset>>1, data, 0xff);
			return;
		}

		device_image_interface *image = dynamic_cast<device_image_interface *>(m_piodev);

		int bit = (offset & 0x00ff)>>1;
		switch (bit)
		{
		case 0:
			m_selected = (data!=0);
			break;

		case 1:
			m_pio_direction_in = (data!=0);
			break;

		case 2:
			if ((data!=0) != m_pio_handshakeout)
			{
				m_pio_handshakeout = (data!=0);
				if (m_pio_write && m_pio_writable && (!m_pio_direction_in))
				{   /* PIO in output mode */
					if (!m_pio_handshakeout)
					{   /* write data strobe */
						/* write data and acknowledge */
						uint8_t buf = m_pio_out_buffer;
						int ret = image->fwrite(&buf, 1);
						if (ret)
							m_pio_handshakein = 1;
					}
					else
					{
						/* end strobe */
						/* we can write some data: set receiver ready */
						m_pio_handshakein = 0;
					}
				}
				if ((!m_pio_write) && m_pio_readable /*&& pio_direction_in*/)
				{   /* PIO in input mode */
					if (!m_pio_handshakeout)
					{   /* receiver ready */
						/* send data and strobe */
						uint8_t buf;
						if (image->fread(&buf, 1))
							m_pio_in_buffer = buf;
						m_pio_handshakein = 0;
					}
					else
					{
						/* data acknowledge */
						/* we can send some data: set transmitter ready */
						m_pio_handshakein = 1;
					}
				}
			}
			break;

		case 3:
			m_pio_spareout = (data!=0);
			break;

		case 4:
			m_flag0 = (data!=0);
			break;

		case 5:
			// Set the CTS line for RS232/1
			if (TRACE_LINES) logerror("(1/3) Setting CTS* via CRU to %d\n", data);
			output_line_state(0, tms9902_device::CTS, (data==0)? tms9902_device::CTS : 0);
			break;

		case 6:
			// Set the CTS line for RS232/2
			if (TRACE_LINES) logerror("(2/4) Setting CTS* via CRU to %d\n", data);
			output_line_state(1, tms9902_device::CTS, (data==0)? tms9902_device::CTS : 0);
			break;

		case 7:
			m_led = (data!=0);
			break;
		}
		return;
	}
}

/*
    Memory read
*/
READ8Z_MEMBER( ti_rs232_pio_device::readz )
{
	if (m_senila==ASSERT_LINE)
	{
		if (TRACE_ILA) logerror("Sensing ILA\n");
		*value = m_ila;
		// The card ROM must be unselected, or we get two values
		// on the data bus

		// Not sure whether this is correct; there is no software that makes
		// use of it
		m_ila = 0;
	}
	if (((offset & m_select_mask)==m_select_value) && m_selected)
	{
		if ((offset & 0x1000)==0x0000)
		{
			*value = m_dsrrom[offset&0x0fff];
		}
		else
		{
			*value = m_pio_direction_in ? m_pio_in_buffer : m_pio_out_buffer;
		}
	}
}

/*
    Memory write
*/
WRITE8_MEMBER( ti_rs232_pio_device::write )
{
	if (((offset & m_select_mask)==m_select_value) && m_selected)
	{
		if ((offset & 0x1001)==0x1000)
		{
			m_pio_out_buffer = data;
		}
	}
}

/**************************************************************************/

/*
    The DTR line of the interface card is wired to the CTS and DSR
    of the UART.
*/
void ti_rs232_pio_device::incoming_dtr(int uartind, line_state value)
{
	if (TRACE_LINES) logerror("(RS232/%d) Incoming DTR = %d\n", uartind+1, (value==ASSERT_LINE)? 1:0);

	m_uart[uartind]->rcv_cts(value);
	m_uart[uartind]->rcv_dsr(value);
}

/*
    Data transmission
*/
void ti_rs232_pio_device::transmit_data(int uartind, uint8_t value)
{
	uint8_t buf = value;

	device_image_interface *serial;
	serial = dynamic_cast<device_image_interface *>(m_serdev[uartind]);
	if (!serial->exists())
	{
		logerror("(RS232/%d) No serial output attached\n", uartind+1);
		return;
	}

	// Send a double ESC if this is not a control operation
	if (buf==0x1b)
	{
		if (TRACE_OUT) logerror("(RS232/%d) send ESC (requires another ESC)\n", uartind+1);
		serial->fwrite(&buf, 1);
	}
	if (TRACE_OUT)
	{
		char cbuf = (buf < 0x20 || buf > 0x7e)? '.' : (char)buf;
		logerror("(RS232/%d) send %c <%02x>\n", uartind+1, cbuf, buf);
	}
	serial->fwrite(&buf, 1);
}

/*
    Map the DCE-like wiring to a DTE-like wiring and vice versa (mapping==0)
    No handshake

       Emulated           PC serial
       TI RS232           interface
     XOUT  2 TXD ----->-----( 3) ---> TXD
      RIN  3 RXD -----<-----( 2) <--- RXD
      CRU  5 CTS -|       |-( 8) <--- DCD  (cable)
     +12V  6 DSR ----->-----(20) ---> DTR
      RTS  8 DCD ----->-----( 4) ---> RTS
  DSR+CTS 20 DTR -----<-----( 6) <--- DSR
                          |-( 5) <--- CTS

    Alternative mapping: (mapping==1)
    RTS/CTS handshake

       Emulated           PC serial
       TI RS232           interface
     XOUT  2 TXD ----->-----( 3) ---> TXD
      RIN  3 RXD -----<-----( 2) <--- RXD
      CRU  5 CTS ----->-----( 4) ---> RTS
      +12V 6 DSR -|       |-( 6) <--- DSR
      RTS  8 DCD ----->-----(20) ---> DTR
  DSR+CTS 20 DTR -----<-----( 8) <--- DCD
                          |-( 5) <--- CTS

    Yet another mapping: (mapping==2)
    CRU-based handshake

       Emulated           PC serial
       TI RS232           interface
     XOUT  2 TXD ----->-----( 3) ---> TXD
      RIN  3 RXD -----<-----( 2) <--- RXD
      CRU  5 CTS ----->-----(20) ---> DTR
      +12V 6 DSR -|       |-( 6) <--- DSR
      RTS  8 DCD ----->-----( 4) ---> RTS
  DSR+CTS 20 DTR -----<-----( 5) <--- CTS  (cable)
                          |-( 8) <--- DCD

*/
uint8_t ti_rs232_pio_device::map_lines_out(int uartind, uint8_t value)
{
	uint8_t ret = 0;
	int mapping = ioport("SERIALMAP")->read();

	//    00ab cdef = setting line RTS=a, CTS=b, DSR=c, DCD=d, DTR=e, RI=f

	if (TRACE_LINES) logerror("(RS232/%d) out connector pins = 0x%02x; translate for DTE\n", uartind+1, value);

	if (value & tms9902_device::BRK)
	{
		if (TRACE_MAP) logerror("(RS232/%d) Sending BRK\n", uartind+1);
		ret |= tms9902_device::EXCEPT | tms9902_device::BRK;
	}

	if (mapping==0)
	{
		// V1
		if (value & tms9902_device::CTS)
		{
			if (TRACE_MAP) logerror("(RS232/%d) Cannot map CTS line, ignoring\n", uartind+1);
		}
		if (value & tms9902_device::DSR)
		{
			ret |= tms9902_device::DTR;
			if (TRACE_MAP) logerror("(RS232/%d) Setting DTR line\n", uartind+1);
		}
		if (value & tms9902_device::DCD)
		{
			ret |= tms9902_device::RTS;
			if (TRACE_MAP) logerror("(RS232/%d) Setting RTS line\n", uartind+1);
		}
	}
	else
	{
		if (mapping==1)
		{
			// V2
			if (value & tms9902_device::CTS)
			{
				ret |= tms9902_device::RTS;
				if (TRACE_MAP) logerror("(RS232/%d) Setting RTS line\n", uartind+1);
			}
			if (value & tms9902_device::DCD)
			{
				ret |= tms9902_device::DTR;
				if (TRACE_MAP) logerror("(RS232/%d) Setting DTR line\n", uartind+1);
			}
		}
		else
		{
			// v3
			if (value & tms9902_device::CTS)
			{
				ret |= tms9902_device::DTR;
				if (TRACE_MAP) logerror("(RS232/%d) Setting DTR line\n", uartind+1);
			}
			if (value & tms9902_device::DSR)
			{
				if (TRACE_MAP) logerror("(RS232/%d) Cannot map DSR line, ignoring\n", uartind+1);
			}
			if (value & tms9902_device::DCD)
			{
				ret |= tms9902_device::RTS;
				if (TRACE_MAP) logerror("(RS232/%d) Setting RTS line\n", uartind+1);
			}
		}
	}

	return ret;
}

uint8_t ti_rs232_pio_device::map_lines_in(int uartind, uint8_t value)
{
	uint8_t ret = 0;
	int mapping = ioport("SERIALMAP")->read();

	//    00ab cdef = setting line RTS=a, CTS=b, DSR=c, DCD=d, DTR=e, RI=f

	if (TRACE_LINES) logerror("(RS232/%d) in connector pins = 0x%02x; translate from DTE\n", uartind+1, value);

	if (value & tms9902_device::BRK)
	{
		if (TRACE_MAP) logerror("(RS232/%d) Getting BRK\n", uartind+1);
		ret |= tms9902_device::EXCEPT | tms9902_device::BRK;
	}

	if (mapping==0)
	{
		// V1
		if (value & tms9902_device::CTS)
		{
			if (TRACE_MAP) logerror("(RS232/%d) Cannot map CTS line, ignoring\n", uartind+1);
		}
		if (value & tms9902_device::DSR)
		{
			ret |= tms9902_device::DTR;
			if (TRACE_MAP) logerror("(RS232/%d) Setting DTR line\n", uartind+1);
		}
		if (value & tms9902_device::DCD)
		{
			if (TRACE_MAP) logerror("(RS232/%d) Cannot map DCD line, ignoring\n", uartind+1);
		}
	}
	else
	{
		if (mapping==1)
		{
			if (value & tms9902_device::DCD)
			{
				ret |= tms9902_device::DTR;
				if (TRACE_MAP) logerror("(RS232/%d) Setting DTR line\n", uartind+1);
			}
			if (value & tms9902_device::DSR)
			{
				if (TRACE_MAP) logerror("(RS232/%d) Cannot map DSR line, ignoring\n", uartind+1);
			}
			if (value & tms9902_device::CTS)
			{
				if (TRACE_MAP) logerror("(RS232/%d) Cannot map CTS line, ignoring\n", uartind+1);
			}
		}
		else
		{
			if (value & tms9902_device::CTS)
			{
				ret |= tms9902_device::DTR;
				if (TRACE_MAP) logerror("(RS232/%d) Setting DTR line\n", uartind+1);
			}
			if (value & tms9902_device::DSR)
			{
				if (TRACE_MAP) logerror("(RS232/%d) Cannot map DSR line, ignoring\n", uartind+1);
			}
			if (value & tms9902_device::DCD)
			{
				if (TRACE_MAP) logerror("(RS232/%d) Cannot map DCD line, ignoring\n", uartind+1);
			}
		}
	}

	return ret;
}
/*
    Receive a character or a line state from the remote site. This method
    is called by a timer with some sufficiently high polling frequency. Note
    that the control lines are not subject to baud rates.
    The higher polling frequency will cause overloads in the TMS9902 which has
    a one-byte buffer only: Since the data source (e.g. the PC UART and the
    socket connection) may be buffered, we may get a cluster of bytes in rapid
    succession. In order to avoid this, this function uses the parameter
    "baudpoll" which is the ratio of the characters/second and the polling
    frequency. (char/sec is baud rate divided by 10.)
    Whenever we receive a character that is passed to the UART, we have to
    pause for 1/baudpoll iterations before getting the next byte from the
    data source.

    FIXME: This may fail when the emulated system tries to stop the remote
    system by deactivating RTS or DTR, but there are still incoming
    bytes in the socket or PC UART buffer. The buffered bytes may then cause
    an overflow in the emulated UART, since the application program expects
    the remote system to stop sending instantly.
    The only way to handle this is to mirror the activity within the serial
    bridge: Whenever a RTS=0 or DTR=0 is transmitted to the remote site, the
    serial bridge must stop delivering data bytes until the handshake opens the
    channel again.
*/
void ti_rs232_pio_device::receive_data_or_line_state(int uartind)
{
	device_image_interface *serial;
	uint8_t buffer;

	serial = dynamic_cast<device_image_interface *>(m_serdev[uartind]);

	if (!serial->exists())
	{
		logerror("(RS232/%d) No serial input attached\n", uartind+1);
		return;
	}

	double baudpoll = m_uart[uartind]->get_baudpoll();

	// If more than the minimum waiting time since the last data byte has
	// elapsed, we can get a new value.
	if (m_time_hold[uartind] > 1.0)
	{
		// Buffer empty?
		if (m_bufpos[uartind] == m_buflen[uartind])
		{
			// Get all out of sdlsocket
			m_buflen[uartind] = serial->fread(m_recvbuf[uartind].get(), 512);
			m_bufpos[uartind] = 0;
			if (m_buflen[uartind]==0) return;
		}
		buffer = m_recvbuf[uartind][m_bufpos[uartind]++];
	}
	else
	{
		// number of polls has not yet elapsed; we have to wait.
		m_time_hold[uartind] += baudpoll;
		return;
	}

	char cbuf = (buffer < 0x20 || buffer > 0x7e)? '.' : (char)buffer;

	// No config parameters here, only data or line setting
	switch (m_recv_mode[uartind])
	{
	case RECV_MODE_NORMAL:
		if (buffer==0x1b)
		{
			if (TRACE_IN) logerror("(RS232/%d) Received: %c <%02x>, switch to ESC mode\n", uartind+1, cbuf, buffer);
			m_recv_mode[uartind] = RECV_MODE_ESC;
		}
		else
		{
			if (TRACE_IN) logerror("(RS232/%d) Received: %c <%02x>, pass to UART\n", uartind+1, cbuf, buffer);
			m_uart[uartind]->rcv_data(buffer);
			m_time_hold[uartind] = 0.0;
		}
		break;
	case RECV_MODE_ESC:
		if (buffer==0x1b)
		{
			m_recv_mode[uartind] = RECV_MODE_NORMAL;
			if (TRACE_STATE) logerror("(RS232/%d) Received another ESC, passing to UART, leaving ESC mode\n", uartind+1);
			m_uart[uartind]->rcv_data(buffer);
			m_time_hold[uartind] = 0.0;
		}
		else
		{
			// the byte in buffer is the length byte
			if (TRACE_STATE) logerror("(RS232/%d) Received length byte <%02x> in ESC mode\n", uartind+1, buffer);
			if (buffer != 1)
			{
				logerror("(RS232/%d) ** ERROR: Expected length byte 1 but got 0x%02x, leaving ESC mode.\n", uartind+1, buffer);
				m_recv_mode[uartind] = RECV_MODE_NORMAL;
			}
			else
				m_recv_mode[uartind] = RECV_MODE_ESC_LINES;
		}
		break;

	case RECV_MODE_ESC_LINES:
		// Map the real serial interface lines to our emulated connector
		// The mapping is the same for both directions, so we use the same function
		if (buffer & tms9902_device::EXCEPT)
		{
			// Exception states: BRK, FRMERR, PARERR
			if (TRACE_LINES) logerror("(RS232/%d) Received BRK or ERROR <%02x>\n", uartind+1, buffer);
			m_uart[uartind]->rcv_break(((buffer & tms9902_device::BRK)!=0));

			if (buffer & tms9902_device::FRMERR)    m_uart[uartind]->rcv_framing_error();
			if (buffer & tms9902_device::PARERR)    m_uart[uartind]->rcv_parity_error();
		}
		else
		{
			buffer = map_lines_in(uartind, buffer);
			if (TRACE_LINES) logerror("(RS232/%d) Received (remapped) <%02x> in ESC mode\n", uartind+1, buffer);

			// The DTR line on the RS232 connector of the board is wired to both the
			// CTS and the DSR pin of the TMS9902
			// Apart from the data line, DTR is the only input line
			incoming_dtr(uartind,  (buffer & tms9902_device::DTR)? ASSERT_LINE : CLEAR_LINE);
		}

		m_recv_mode[uartind] = RECV_MODE_NORMAL;
		break;

	default:
		logerror("(RS232/%d) Unknown mode: %d\n", uartind+1, m_recv_mode[uartind]);
	}
}

/*
    Control operations like configuration or line changes
*/
void ti_rs232_pio_device::configure_interface(int uartind, int type, int value)
{
	uint8_t bufctrl[4];
	device_image_interface *serial;
	uint8_t esc = ESC;

	serial = dynamic_cast<device_image_interface *>(m_serdev[uartind]);

	if (!serial->exists())
	{
		logerror("(RS232/%d) No serial output attached\n", uartind+1);
		return;
	}

	serial->fwrite(&esc, 1);
	bufctrl[0] = 0x02;
	bufctrl[1] = tms9902_device::CONFIG | tms9902_device::TYPE_TMS9902;

	switch (type) {
	case tms9902_device::RATERECV:
		if (TRACE_SETTING) logerror("(RS232/%d) Send receive rate %04x\n", uartind+1, value);
		// value has 12 bits
		// 1ccc xaaa                         = config adapter type a
		// 1111 xaaa rrrr rrrr rrrr 0000     = config receive rate on a
		// 1110 xaaa rrrr rrrr rrrr 0000     = config transmit rate on a
		bufctrl[0] = 0x03; // length
		bufctrl[1] |= tms9902_device::RATERECV;
		bufctrl[2] = (value & 0x0ff0)>>4;
		bufctrl[3] = (value & 0x0f)<<4;
		break;
	case tms9902_device::RATEXMIT:
		if (TRACE_SETTING) logerror("(RS232/%d) Send transmit rate %04x\n", uartind+1, value);
		bufctrl[0] = 0x03; // length
		bufctrl[1] |= tms9902_device::RATEXMIT;
		bufctrl[2] = (value & 0x0ff0)>>4;
		bufctrl[3] = (value & 0x0f)<<4;
		break;
	case tms9902_device::STOPBITS:
		if (TRACE_SETTING) logerror("(RS232/%d) Send stop bit config %02x\n", uartind+1, value&0x03);
		bufctrl[1] |= tms9902_device::STOPBITS;
		bufctrl[2] = (value & 0x03);
		break;
	case tms9902_device::DATABITS:
		if (TRACE_SETTING) logerror("(RS232/%d) Send data bit config %02x\n", uartind+1, value&0x03);
		bufctrl[1] |= tms9902_device::DATABITS;
		bufctrl[2] = (value & 0x03);
		break;
	case tms9902_device::PARITY:
		if (TRACE_SETTING) logerror("(RS232/%d) Send parity config %02x\n", uartind+1, value&0x03);
		bufctrl[1] |= tms9902_device::PARITY;
		bufctrl[2] = (value & 0x03);
		break;
	default:
		logerror("(RS232/%d) Error - unknown config type %02x\n", uartind+1, type);
	}

	serial->fwrite(bufctrl, bufctrl[0]+1);
}

void ti_rs232_pio_device::set_bit(int uartind, int line, int value)
{
	if (TRACE_LINES)
	{
		switch (line)
		{
		case tms9902_device::CTS: logerror("(RS232/%d) Set CTS(out)=%s\n", uartind+1, (value!=0)? "asserted" : "cleared"); break;
		case tms9902_device::DCD: logerror("(RS232/%d) Set DCD(out)=%s\n", uartind+1, (value!=0)? "asserted" : "cleared"); break;
		case tms9902_device::BRK: logerror("(RS232/%d) Set BRK(out)=%s\n", uartind+1, (value!=0)? "asserted" : "cleared"); break;
		}
	}

	if (value!=0)   m_signals[uartind] |= line;
	else            m_signals[uartind] &= ~line;
}

/*
   Line changes
*/
void ti_rs232_pio_device::output_exception(int uartind, int param, uint8_t value)
{
	device_image_interface *serial;
	uint8_t bufctrl[2];
	uint8_t esc = ESC;

	serial = dynamic_cast<device_image_interface *>(m_serdev[uartind]);

	if (!serial->exists())
	{
		logerror("(RS232/%d) No serial output attached\n", uartind+1);
		return;
	}

	serial->fwrite(&esc, 1);

	bufctrl[0] = 1;
	// 0100 0xxv = exception xx: 02=BRK, 04=FRMERR, 06=PARERR; v=0,1 (only for BRK)
	// BRK is the only output exception
	bufctrl[1] = tms9902_device::EXCEPT | param | (value&1);
	serial->fwrite(bufctrl, 2);
}

/*
   Line changes
*/
void ti_rs232_pio_device::output_line_state(int uartind, int mask, uint8_t value)
{
	device_image_interface *serial;
	uint8_t bufctrl[2];
	uint8_t esc = ESC;

	serial = dynamic_cast<device_image_interface *>(m_serdev[uartind]);

	if (!serial->exists())
	{
		logerror("(RS232/%d) No serial output attached\n", uartind+1);
		return;
	}

	// Send ESC to serial bridge
	serial->fwrite(&esc, 1);

	// Length 1
	bufctrl[0] = 1;

	// 01ab cdef = setting line RTS=a, CTS=b, DSR=c, DCD=d, DTR=e, RI=f

	// The CTS line (coming from a CRU bit) is connected to the CTS pin
	if (mask & tms9902_device::CTS) set_bit(uartind, tms9902_device::CTS, value & tms9902_device::CTS);

	// The RTS line (from 9902) is connected to the DCD pin
	if (mask & tms9902_device::RTS) set_bit(uartind, tms9902_device::DCD, value & tms9902_device::RTS);

	// The DSR pin is hardwired to +5V
	set_bit(uartind, tms9902_device::DSR, 1);

	// As of here, the lines are set according to the schematics of the
	// serial interface.

	// Now translate the signals of the board to those of a DTE-like device
	// so that we can pass the signal to the real PC serial interface
	// (can be imagined as if we emulated the cable)
	bufctrl[1] = map_lines_out(uartind, m_signals[uartind]);
	serial->fwrite(bufctrl, 2);
}

/***********************************************************************
    callbacks
************************************************************************/
/*
    Propagates the /INT signal of the UARTs to the /INT line of the pbox.
*/
WRITE_LINE_MEMBER( ti_rs232_pio_device::int0_callback )
{
	int senila_bit = SENILA_0_BIT;

	if (state==ASSERT_LINE) m_ila |= senila_bit;
	else m_ila &= ~senila_bit;

	m_slot->set_inta(state);
}

WRITE_LINE_MEMBER( ti_rs232_pio_device::int1_callback )
{
	int senila_bit = SENILA_1_BIT;

	if (state==ASSERT_LINE) m_ila |= senila_bit;
	else m_ila &= ~senila_bit;

	m_slot->set_inta(state);
}

/*
    Called from the UART when it wants to receive a character
    However, characters are not passed to it at this point
    Instead, we check for signal line change or data transmission
    and call the respective function
*/
WRITE_LINE_MEMBER( ti_rs232_pio_device::rcv0_callback )
{
	receive_data_or_line_state(0);
}

WRITE_LINE_MEMBER( ti_rs232_pio_device::rcv1_callback )
{
	receive_data_or_line_state(1);
}

WRITE8_MEMBER( ti_rs232_pio_device::xmit0_callback )
{
	transmit_data(0, data);
}

WRITE8_MEMBER( ti_rs232_pio_device::xmit1_callback )
{
	transmit_data(1, data);
}

void ti_rs232_pio_device::ctrl_callback(int uartind, int offset, uint8_t data)
{
	if ((offset & tms9902_device::CONFIG)!=0)
	{
		// We cannot pass the configuration data as they need more than 8 bits.
		// Could be done by a write16 function as well.
		configure_interface(uartind, data, m_uart[uartind]->get_config_value());
	}
	else
	{
		if ((offset & tms9902_device::EXCEPT)!=0)
		{
			output_exception(uartind, offset & ~tms9902_device::EXCEPT, data);
		}
		else
		{
			output_line_state(uartind, offset, data);
		}
	}
}

WRITE8_MEMBER( ti_rs232_pio_device::ctrl0_callback )
{
	ctrl_callback(0, offset, data);
}

WRITE8_MEMBER( ti_rs232_pio_device::ctrl1_callback )
{
	ctrl_callback(1, offset, data);
}

void ti_rs232_pio_device::device_start()
{
	m_dsrrom = memregion(TI99_DSRROM)->base();
	m_uart[0] = subdevice<tms9902_device>("tms9902_0");
	m_uart[1] = subdevice<tms9902_device>("tms9902_1");
	m_serdev[0] = subdevice<ti_rs232_attached_device>("serdev0");
	m_serdev[1] = subdevice<ti_rs232_attached_device>("serdev1");
	m_piodev = subdevice<ti_pio_attached_device>("piodev");
	// Prepare the receive buffers
	m_recvbuf[0] = std::make_unique<uint8_t[]>(512);
	m_recvbuf[1] = std::make_unique<uint8_t[]>(512);
	m_pio_write = true; // required for call_load of pio_attached_device
	m_pio_writable = false;
	m_pio_handshakein = false;

	// We don't save the receive buffers for persistent state
	save_pointer(NAME(m_signals),2);
	save_pointer(NAME(m_recv_mode),2);
	save_pointer(NAME(m_time_hold),2);
	save_item(NAME(m_pio_direction_in));
	save_item(NAME(m_pio_handshakeout));
	save_item(NAME(m_pio_handshakein));
	save_item(NAME(m_pio_spareout));
	save_item(NAME(m_pio_sparein));
	save_item(NAME(m_flag0));
	save_item(NAME(m_led));
	save_item(NAME(m_pio_out_buffer));
	save_item(NAME(m_pio_in_buffer));
	save_item(NAME(m_pio_readable));
	save_item(NAME(m_pio_writable));
	save_item(NAME(m_pio_write));
	save_item(NAME(m_ila));
}

void ti_rs232_pio_device::device_stop()
{
	m_recvbuf[0] = nullptr;
	m_recvbuf[1] = nullptr;
}

void ti_rs232_pio_device::device_reset()
{
	m_pio_direction_in = false;
	m_pio_handshakeout = false;
	m_pio_spareout = false;
	m_flag0 = false;

	set_bit(0, tms9902_device::CTS, 0);
	set_bit(1, tms9902_device::CTS, 0);

	m_led = false;
	m_recv_mode[0] = RECV_MODE_NORMAL;
	m_recv_mode[1] = RECV_MODE_NORMAL;

	m_bufpos[0] = m_bufpos[1] = m_buflen[0] = m_buflen[1] = 0;

	if (m_genmod)
	{
		m_select_mask = 0x1fe000;
		m_select_value = 0x174000;
	}
	else
	{
		m_select_mask = 0x7e000;
		m_select_value = 0x74000;
	}

	m_selected = false;

	m_cru_base = (ioport("CRURS232")->read()==0)? 0x1300 : 0x1500;

	m_time_hold[0] = m_time_hold[1] = 0.0;

	// Both DTRs are pulled up
	incoming_dtr(0, ASSERT_LINE);
	incoming_dtr(1, ASSERT_LINE);
}

ROM_START( ti_rs232 )
	ROM_REGION(0x1000, TI99_DSRROM, 0)
	ROM_LOAD("rs232pio_dsr.u1", 0x0000, 0x1000, CRC(eab382fb) SHA1(ee609a18a21f1a3ddab334e8798d5f2a0fcefa91)) /* TI rs232 DSR ROM */
ROM_END

INPUT_PORTS_START( ti_rs232 )
	PORT_START( "CRURS232"  )
	PORT_DIPNAME( 0x01, 0x00, "TI-RS232 CRU base" )
		PORT_DIPSETTING(    0x00, "1300" )
		PORT_DIPSETTING(    0x00, "1500" )

	PORT_START( "SERIALMAP" )
	PORT_CONFNAME( 0x03, 0x00, "Serial cable pin mapping" )
		PORT_CONFSETTING(    0x00, "6-20" )
		PORT_CONFSETTING(    0x01, "8-20" )
		PORT_CONFSETTING(    0x02, "5-20" )
INPUT_PORTS_END

MACHINE_CONFIG_MEMBER( ti_rs232_pio_device::device_add_mconfig )
	MCFG_DEVICE_ADD("tms9902_0", TMS9902, 3000000)
	MCFG_TMS9902_INT_CB(WRITELINE(ti_rs232_pio_device, int0_callback))            /* called when interrupt pin state changes */
	MCFG_TMS9902_RCV_CB(WRITELINE(ti_rs232_pio_device, rcv0_callback))            /* called when a character is received */
	MCFG_TMS9902_XMIT_CB(WRITE8(ti_rs232_pio_device, xmit0_callback))            /* called when a character is transmitted */
	MCFG_TMS9902_CTRL_CB(WRITE8(ti_rs232_pio_device, ctrl0_callback))
	MCFG_DEVICE_ADD("tms9902_1", TMS9902, 3000000)
	MCFG_TMS9902_INT_CB(WRITELINE(ti_rs232_pio_device, int1_callback))            /* called when interrupt pin state changes */
	MCFG_TMS9902_RCV_CB(WRITELINE(ti_rs232_pio_device, rcv1_callback))            /* called when a character is received */
	MCFG_TMS9902_XMIT_CB(WRITE8(ti_rs232_pio_device, xmit1_callback))            /* called when a character is transmitted */
	MCFG_TMS9902_CTRL_CB(WRITE8(ti_rs232_pio_device, ctrl1_callback))
	MCFG_DEVICE_ADD("serdev0", TI99_RS232_DEV, 0)
	MCFG_DEVICE_ADD("serdev1", TI99_RS232_DEV, 0)
	MCFG_DEVICE_ADD("piodev", TI99_PIO_DEV, 0)
MACHINE_CONFIG_END

const tiny_rom_entry *ti_rs232_pio_device::device_rom_region() const
{
	return ROM_NAME( ti_rs232 );
}

ioport_constructor ti_rs232_pio_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(ti_rs232);
}

} } } // end namespace bus::ti99::peb
