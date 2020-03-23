// license:BSD-3-Clause
// copyright-holders:Couriersud
/***************************************************************************

    Gaelco 3D serial hardware

    Couriersud, early 2010

    Not all lines are fully understood. There is some handshaking going
    on on send which is not fully understood. Those wishing to have a look
    at this:

    Serial send:          0x1fca (radikalb, 68020) 1866e (surfplnt)
    Serial receive:       0x1908 (radikalb, 68020) 185ae

    The receive is interrupt driven (interrupt 6) and the send is
    initiated out of the normal program loop. There is the chance
    of race conditions in mame.

    To run two instances of radikalb on *nix, use the following

    a) Uncomment SHARED_MEM_DRIVER below
    b) Open two terminals
    c) In terminal 1: mkdir /tmp/x1; cd /tmp/x1; /path/to/mame64 -np 2 -mt -rp /mnt/mame/romlib/r -inipath . radikalb -w -nomaximize -inipath .
    d) In terminal 2: mkdir /tmp/x2; cd /tmp/x2; /path/to/mame64 -np 2 -mt -rp /mnt/mame/romlib/r -inipath . radikalb -w -nomaximize -inipath .
    e) Set one instance to be master and one to be slave in service mode
    f) Have fun

***************************************************************************/

#include "emu.h"
#include "gaelco3d.h"

//#define SHARED_MEM_DRIVER      (1)

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#ifdef _MSC_VER
#include <io.h>
#define S_IRWXU (_S_IREAD | _S_IWRITE | _S_IEXEC)
#else
#include <unistd.h>
#endif
#include <errno.h>
#ifdef SHARED_MEM_DRIVER
#include <sys/mman.h>
#endif

#define VERBOSE     (0)
#if VERBOSE
#define LOGMSG(x)   logerror x
#else
#define LOGMSG(x)   do {} while (0);
#endif

/* external status bits */
#define GAELCOSER_STATUS_READY          0x01
#define GAELCOSER_STATUS_RTS            0x02

/* only RTS currently understood ! */
//#define GAELCOSER_STATUS_DTR          0x04

/* internal bits follow ... */
#define GAELCOSER_STATUS_IRQ_ENABLE     0x10
#define GAELCOSER_STATUS_RESET          0x20
#define GAELCOSER_STATUS_SEND           0x40

/*
 * 115200 seems plausible, radikalb won't work below this speed
 * surfplnt will not work below 460800 ....
 * speedup will not work above 115200
 * 10 bits = 8 data + 1 start + 1 stop
 */

#define LINK_BAUD (115200*4)
//Only for speedup
//#define LINK_BAUD (115200)
#define LINK_BITS 10

#define LINK_FREQ (LINK_BAUD / LINK_BITS)

/* Sync up the instances 8 times for each byte transferred */
#define SYNC_MULT (4)

#define SYNC_FREQ (15000000 / 20) //(LINK_FREQ * SYNC_MULT)

/* allow for slight differences in execution speed */

#define LINK_SLACK ((SYNC_MULT / 4) + 1)
#define LINK_SLACK_B ((LINK_SLACK / 3) + 1)



/* code below currently works on unix only */
#ifdef SHARED_MEM_DRIVER
gaelco_serial_device::osd_shared_mem *gaelco_serial_device::osd_sharedmem_alloc(const char *path, int create, size_t size)
{
	int fd;
	osd_shared_mem *os_shmem = (osd_shared_mem *)malloc(sizeof(osd_shared_mem));

	if (create)
	{
		char *buf = (char *) malloc(size);
		memset(buf,0, size);

		fd = open(path, O_RDWR | O_CREAT, S_IRWXU);
		write(fd, buf, size);
		os_shmem->creator = 1;
	}
	else
	{
		fd = open(path, O_RDWR);
		if (fd == -1)
		{
			free(os_shmem);
			return nullptr;
		}
		os_shmem->creator = 0;
	}
	os_shmem->fn = (char *) malloc(strlen(path)+1);
	strcpy(os_shmem->fn, path);

	assert(fd != -1);

	os_shmem->ptr = mmap(nullptr, size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
	os_shmem->size = size;
	close(fd);
	return os_shmem;
}

void gaelco_serial_device::osd_sharedmem_free(osd_shared_mem *os_shmem)
{
	munmap(os_shmem->ptr, os_shmem->size);
	if (os_shmem->creator)
		unlink(os_shmem->fn);
	free(os_shmem->fn);
	free(os_shmem);
}

void *gaelco_serial_device::osd_sharedmem_ptr(osd_shared_mem *os_shmem)
{
	return os_shmem->ptr;
}
#else
gaelco_serial_device::osd_shared_mem *gaelco_serial_device::osd_sharedmem_alloc(const char *path, int create, size_t size)
{
	osd_shared_mem *os_shmem = (osd_shared_mem *) malloc(sizeof(osd_shared_mem));

	os_shmem->creator = 0;

	os_shmem->ptr = (void *) malloc(size);
	os_shmem->size = size;
	return os_shmem;
}

void gaelco_serial_device::osd_sharedmem_free(osd_shared_mem *os_shmem)
{
	free(os_shmem->ptr);
	free(os_shmem);
}

void *gaelco_serial_device::osd_sharedmem_ptr(osd_shared_mem *os_shmem)
{
	return os_shmem->ptr;
}
#endif

/***************************************************************************
    DEVICE INTERFACE
***************************************************************************/

#define PATH_NAME "/tmp/gaelco_serial"

void gaelco_serial_device::buf_reset(buf_t *buf)
{
	buf->stat = GAELCOSER_STATUS_RTS | GAELCOSER_STATUS_RESET;
	buf->data = 0;
	buf->data_cnt = -1;
	buf->cnt = 0;
}

DEFINE_DEVICE_TYPE(GAELCO_SERIAL, gaelco_serial_device, "gaelco_serial", "Gaelco 3D Serial Hardware")

gaelco_serial_device::gaelco_serial_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, GAELCO_SERIAL, tag, owner, clock),
	m_irq_handler(*this),
	m_status(0),
	m_last_in_msg_cnt(0),
	m_slack_cnt(0),
	m_sync_timer(nullptr),
	m_in_ptr(nullptr),
	m_out_ptr(nullptr),
	m_os_shmem(nullptr),
	m_shmem(nullptr)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void gaelco_serial_device::device_start()
{
	/* validate arguments */
	assert(strlen(tag()) < 20);

	m_irq_handler.resolve_safe();
	m_sync_timer = machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(gaelco_serial_device::link_cb), this));

	/* register for save states */
	//save_item(NAME(earom->offset));
	//save_item(NAME(earom->data));

#ifdef SHARED_MEM_DRIVER
	m_sync_timer->adjust(attotime::zero,0,attotime::from_hz(SYNC_FREQ));
#endif

	m_os_shmem = osd_sharedmem_alloc(PATH_NAME, 0, sizeof(shmem_t));
	if (m_os_shmem == nullptr)
	{
		m_os_shmem = osd_sharedmem_alloc(PATH_NAME, 1, sizeof(shmem_t));
		m_shmem = (shmem_t *) osd_sharedmem_ptr(m_os_shmem);

		m_in_ptr = &m_shmem->buf[0];
		m_out_ptr = &m_shmem->buf[1];
	}
	else
	{
		m_shmem = (shmem_t *) osd_sharedmem_ptr(m_os_shmem);
		m_in_ptr = &m_shmem->buf[1];
		m_out_ptr = &m_shmem->buf[0];
	}
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void gaelco_serial_device::device_reset()
{
	m_status = GAELCOSER_STATUS_READY    |GAELCOSER_STATUS_IRQ_ENABLE ;

	m_last_in_msg_cnt = -1;
	m_slack_cnt = LINK_SLACK_B;

	std::lock_guard<std::mutex> guard(m_mutex);
	buf_reset(m_out_ptr);
	buf_reset(m_in_ptr);
}

//-------------------------------------------------
//  device_stop - device-specific stop
//-------------------------------------------------

void gaelco_serial_device::device_stop()
{
	{
		std::lock_guard<std::mutex> guard(m_mutex);
		buf_reset(m_out_ptr);
		buf_reset(m_in_ptr);
	}
	osd_sharedmem_free(m_os_shmem);
}

TIMER_CALLBACK_MEMBER( gaelco_serial_device::set_status_cb )
{
	uint8_t mask = param >> 8;
	uint8_t set = param & 0xff;

	m_status &= mask;
	m_status |= set;
}

void gaelco_serial_device::set_status(uint8_t mask, uint8_t set, int wait)
{
	machine().scheduler().timer_set(attotime::from_hz(wait), timer_expired_delegate(FUNC(gaelco_serial_device::set_status_cb), this), (mask << 8)|set);
}

void gaelco_serial_device::process_in()
{
	int t;

	if ((m_in_ptr->stat & GAELCOSER_STATUS_RESET) != 0)
		m_out_ptr->cnt = 0;

	/* new data available ? */
	t = m_in_ptr->data_cnt;
	if (t != m_last_in_msg_cnt)
	{
		m_last_in_msg_cnt = t;
		if (m_in_ptr->cnt > 10)
		{
			m_status &= ~GAELCOSER_STATUS_READY;
			LOGMSG(("command receive %02x at %d (%d)\n", m_in_ptr->data, m_out_ptr->cnt, m_in_ptr->cnt));
			if ((m_status & GAELCOSER_STATUS_IRQ_ENABLE) != 0)
			{
				m_irq_handler(1);
				LOGMSG(("irq!\n"));
			}
		}
	}
}

void gaelco_serial_device::sync_link()
{
	volatile buf_t *buf = m_in_ptr;
	int breakme = 1;
	do
	{
		std::lock_guard<std::mutex> guard(m_mutex);

		process_in();
		/* HACK: put some timing noise on the line */
		if (buf->cnt + m_slack_cnt > m_out_ptr->cnt)
			breakme = 0;
		/* stop if not connected .. */
		if ((m_out_ptr->stat & GAELCOSER_STATUS_RESET) != 0)
			breakme = 0;

	} while (breakme);

	m_slack_cnt++;
	m_slack_cnt = (m_slack_cnt % LINK_SLACK) + LINK_SLACK_B;

	{
		std::lock_guard<std::mutex> guard(m_mutex);
		m_out_ptr->stat &= ~GAELCOSER_STATUS_RESET;
	}
}

TIMER_CALLBACK_MEMBER( gaelco_serial_device::link_cb )
{
	std::lock_guard<std::mutex> guard(m_mutex);
	m_out_ptr->cnt++;
	sync_link();
}


/***************************************************************************
    INTERFACE FUNCTIONS
***************************************************************************/



WRITE8_MEMBER( gaelco_serial_device::irq_enable )
{
	LOGMSG(("???? irq enable %d\n", data));
}

READ8_MEMBER( gaelco_serial_device::status_r)
{
	uint8_t ret = 0;

	std::lock_guard<std::mutex> guard(m_mutex);

	process_in();
	if ((m_status & GAELCOSER_STATUS_READY) != 0)
		ret |= 0x01;
	if ((m_in_ptr->stat & GAELCOSER_STATUS_RTS) != 0)
		ret |= 0x02;

	return ret;
}

WRITE8_MEMBER( gaelco_serial_device::data_w)
{
	std::lock_guard<std::mutex> guard(m_mutex);

	m_out_ptr->data = data;
	m_status &= ~GAELCOSER_STATUS_READY;
	m_out_ptr->data_cnt++;

	set_status( ~GAELCOSER_STATUS_READY, GAELCOSER_STATUS_READY, LINK_FREQ );

	LOGMSG(("command send %02x at %d\n", data, m_out_ptr->cnt));
}

READ8_MEMBER( gaelco_serial_device::data_r)
{
	uint8_t ret;

	std::lock_guard<std::mutex> guard(m_mutex);
	process_in();
	ret = (m_in_ptr->data & 0xff);

	m_irq_handler(0);
	LOGMSG(("read %02x at %d (%d)\n", ret, m_out_ptr->cnt, m_in_ptr->cnt));

	/* if we are not sending, mark as as ready */
	if ((m_status & GAELCOSER_STATUS_SEND) == 0)
		m_status |= GAELCOSER_STATUS_READY;

	return ret;
}

WRITE8_MEMBER( gaelco_serial_device::unknown_w)
{
	std::lock_guard<std::mutex> guard(m_mutex);
	LOGMSG(("???? unknown serial access %d\n", data));

}

WRITE8_MEMBER( gaelco_serial_device::rts_w )
{
	std::lock_guard<std::mutex> guard(m_mutex);

	if (data == 0)
		m_out_ptr->stat |= GAELCOSER_STATUS_RTS;
	else
	{
		//Commented out for now
		//m_status |= GAELCOSER_STATUS_READY;
		m_out_ptr->stat &= ~GAELCOSER_STATUS_RTS;
	}
}

WRITE8_MEMBER( gaelco_serial_device::tr_w)
{
	LOGMSG(("set transmit %d\n", data));
	std::lock_guard<std::mutex> guard(m_mutex);
	if ((data & 0x01) != 0)
		m_status |= GAELCOSER_STATUS_SEND;
	else
		m_status &= ~GAELCOSER_STATUS_SEND;
}
