// license:BSD-3-Clause
// copyright-holders:Olivier Galibert

#include "emu.h"
#include "ncr5390.h"

#define DELAY_HACK

DEFINE_DEVICE_TYPE(NCR5390, ncr5390_device, "ncr5390", "NCR 5390 SCSI")
DEFINE_DEVICE_TYPE(NCR53C90A, ncr53c90a_device, "ncr53c90a", "NCR 53C90A SCSI")
DEFINE_DEVICE_TYPE(NCR53C94, ncr53c94_device, "ncr53c94", "NCR 53C94 SCSI")

DEVICE_ADDRESS_MAP_START(map, 8, ncr5390_device)
	AM_RANGE(0x0, 0x0) AM_READWRITE(tcounter_lo_r, tcount_lo_w)
	AM_RANGE(0x1, 0x1) AM_READWRITE(tcounter_hi_r, tcount_hi_w)
	AM_RANGE(0x2, 0x2) AM_READWRITE(fifo_r, fifo_w)
	AM_RANGE(0x3, 0x3) AM_READWRITE(command_r, command_w)
	AM_RANGE(0x4, 0x4) AM_READWRITE(status_r, bus_id_w)
	AM_RANGE(0x5, 0x5) AM_READWRITE(istatus_r, timeout_w)
	AM_RANGE(0x6, 0x6) AM_READWRITE(seq_step_r, sync_period_w)
	AM_RANGE(0x7, 0x7) AM_READWRITE(fifo_flags_r, sync_offset_w)
	AM_RANGE(0x8, 0x8) AM_READWRITE(conf_r, conf_w)
	AM_RANGE(0xa, 0xa) AM_WRITE(test_w)
	AM_RANGE(0x9, 0x9) AM_WRITE(clock_w)
ADDRESS_MAP_END

DEVICE_ADDRESS_MAP_START(map, 8, ncr53c90a_device)
	AM_INHERIT_FROM(ncr5390_device::map)

	AM_RANGE(0xb, 0xb) AM_READWRITE(conf2_r, conf2_w)
ADDRESS_MAP_END

DEVICE_ADDRESS_MAP_START(map, 8, ncr53c94_device)
	AM_INHERIT_FROM(ncr53c90a_device::map)

	AM_RANGE(0xc, 0xc) AM_READWRITE(conf3_r, conf3_w)
	AM_RANGE(0xf, 0xf) AM_WRITE(fifo_align_w)
ADDRESS_MAP_END

ncr5390_device::ncr5390_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: nscsi_device(mconfig, type, tag, owner, clock)
	, tm(nullptr), config(0), status(0), istatus(0), clock_conv(0), sync_offset(0), sync_period(0), bus_id(0)
	, select_timeout(0), seq(0), tcount(0), tcounter(0), mode(0), fifo_pos(0), command_pos(0), state(0), xfr_phase(0), command_length(0), dma_dir(0), irq(false), drq(false), test_mode(false)
	, m_irq_handler(*this)
	, m_drq_handler(*this)
{
}

ncr53c90a_device::ncr53c90a_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: ncr5390_device(mconfig, type, tag, owner, clock)
	, config2(0)
{
}

ncr5390_device::ncr5390_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: ncr5390_device(mconfig, NCR5390, tag, owner, clock)
{
}

ncr53c90a_device::ncr53c90a_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: ncr53c90a_device(mconfig, NCR53C90A, tag, owner, clock)
{
}

ncr53c94_device::ncr53c94_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: ncr53c90a_device(mconfig, NCR53C94, tag, owner, clock)
	, config3(0)
{
}

void ncr5390_device::device_start()
{
	nscsi_device::device_start();

	save_item(NAME(command));
	save_item(NAME(config));
	save_item(NAME(status));
	save_item(NAME(istatus));
	save_item(NAME(clock_conv));
	save_item(NAME(sync_offset));
	save_item(NAME(sync_period));
	save_item(NAME(bus_id));
	save_item(NAME(select_timeout));
	save_item(NAME(seq));
	save_item(NAME(fifo));
	save_item(NAME(tcount));
	save_item(NAME(tcounter));
	save_item(NAME(mode));
	save_item(NAME(fifo_pos));
	save_item(NAME(command_pos));
	save_item(NAME(state));
	save_item(NAME(xfr_phase));
	save_item(NAME(command_length));
	save_item(NAME(dma_dir));
	save_item(NAME(irq));
	save_item(NAME(drq));
	save_item(NAME(test_mode));

	m_irq_handler.resolve_safe();
	m_drq_handler.resolve_safe();

	tcount = 0;
	tcounter = 0;
	config = 0;
	status = 0;
	bus_id = 0;
	select_timeout = 0;
	tm = timer_alloc(0);
}

void ncr5390_device::device_reset()
{
	fifo_pos = 0;
	memset(fifo, 0, sizeof(fifo));

	clock_conv = 2;
	sync_period = 5;
	sync_offset = 0;
	seq = 0;
	config &= 7;
	status &= 0x90;
	istatus = 0;
	irq = false;
	m_irq_handler(irq);
	reset_soft();
}

void ncr5390_device::reset_soft()
{
	state = IDLE;
	scsi_bus->ctrl_wait(scsi_refid, S_SEL|S_BSY|S_RST, S_ALL);
	status &= 0xef;
	drq = false;
	test_mode = false;
	m_drq_handler(drq);
	reset_disconnect();
}

void ncr5390_device::reset_disconnect()
{
	command_pos = 0;
	command_length = 0;
	memset(command, 0, sizeof(command));
	mode = MODE_D;
}

void ncr5390_device::scsi_ctrl_changed()
{
	uint32_t ctrl = scsi_bus->ctrl_r();
	if(ctrl & S_RST) {
		logerror("%s: scsi bus reset\n", tag());
		return;
	}

	step(false);
}

void ncr5390_device::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	step(true);
}

void ncr5390_device::step(bool timeout)
{
	uint32_t ctrl = scsi_bus->ctrl_r();
	uint32_t data = scsi_bus->data_r();
	uint8_t c     = command[0] & 0x7f;

	if(0)
		logerror("%s: state=%d.%d %s\n",
					tag(), state & STATE_MASK, (state & SUB_MASK) >> SUB_SHIFT,
					timeout ? "timeout" : "change");

	if(mode == MODE_I && !(ctrl & S_BSY)) {
		state = IDLE;
		istatus |= I_DISCONNECT;
		reset_disconnect();
		check_irq();
	}
	switch(state & SUB_MASK ? state & SUB_MASK : state & STATE_MASK) {
	case IDLE:
		break;

	case ARB_COMPLETE << SUB_SHIFT: {
		if(!timeout)
			break;

		int win;
		for(win=7; win>=0 && !(data & (1<<win)); win--) {};
		if(win != scsi_id) {
			scsi_bus->data_w(scsi_refid, 0);
			scsi_bus->ctrl_w(scsi_refid, 0, S_ALL);
			fatalerror("need to wait for bus free\n");
		}
		state = (state & STATE_MASK) | (ARB_ASSERT_SEL << SUB_SHIFT);
		scsi_bus->ctrl_w(scsi_refid, S_SEL, S_SEL);
		delay(6);
		break;
	}

	case ARB_ASSERT_SEL << SUB_SHIFT:
		if(!timeout)
			break;

		scsi_bus->data_w(scsi_refid, (1<<scsi_id) | (1<<bus_id));
		state = (state & STATE_MASK) | (ARB_SET_DEST << SUB_SHIFT);
		delay_cycles(4);
		break;

	case ARB_SET_DEST << SUB_SHIFT:
		if(!timeout)
			break;

		state = (state & STATE_MASK) | (ARB_RELEASE_BUSY << SUB_SHIFT);
		scsi_bus->ctrl_w(scsi_refid, c == CD_SELECT_ATN || c == CD_SELECT_ATN_STOP ? S_ATN : 0, S_ATN|S_BSY);
		delay(2);
		break;

	case ARB_RELEASE_BUSY << SUB_SHIFT:
		if(!timeout)
			break;

		if(ctrl & S_BSY) {
			state = (state & STATE_MASK) | (ARB_DESKEW_WAIT << SUB_SHIFT);
			if(c == CD_RESELECT)
				scsi_bus->ctrl_w(scsi_refid, S_BSY, S_BSY);
			delay_cycles(2);
		} else {
			state = (state & STATE_MASK) | (ARB_TIMEOUT_BUSY << SUB_SHIFT);
#ifdef DELAY_HACK
			delay(1);
#else
			delay(8192*select_timeout);
#endif
		}
		break;

	case ARB_DESKEW_WAIT << SUB_SHIFT:
		if(!timeout)
			break;

		scsi_bus->data_w(scsi_refid, 0);
		scsi_bus->ctrl_w(scsi_refid, 0, S_SEL);

		if(c == CD_RESELECT) {
			logerror("%s: mode switch to Target\n", tag());
			mode = MODE_T;
		} else {
			logerror("%s: mode switch to Initiator\n", tag());
			mode = MODE_I;
		}
		state &= STATE_MASK;
		step(true);
		break;

	case ARB_TIMEOUT_BUSY << SUB_SHIFT:
		if(timeout) {
			scsi_bus->data_w(scsi_refid, 0);
			logerror("%s: select timeout\n", tag());
			state = (state & STATE_MASK) | (ARB_TIMEOUT_ABORT << SUB_SHIFT);
			delay(1000);
		} else if(ctrl & S_BSY) {
			state = (state & STATE_MASK) | (ARB_DESKEW_WAIT << SUB_SHIFT);
			if(c == CD_RESELECT)
				scsi_bus->ctrl_w(scsi_refid, S_BSY, S_BSY);
			delay_cycles(2);
		}
		break;

	case ARB_TIMEOUT_ABORT << SUB_SHIFT:
		if(!timeout)
			break;

		if(ctrl & S_BSY) {
			state = (state & STATE_MASK) | (ARB_DESKEW_WAIT << SUB_SHIFT);
			if(c == CD_RESELECT)
				scsi_bus->ctrl_w(scsi_refid, S_BSY, S_BSY);
			delay_cycles(2);
		} else {
			scsi_bus->ctrl_w(scsi_refid, 0, S_ALL);
			state = IDLE;
			istatus |= I_DISCONNECT;
			reset_disconnect();
			check_irq();
		}
		break;

	case SEND_WAIT_SETTLE << SUB_SHIFT:
		if(!timeout)
			break;

		state = (state & STATE_MASK) | (SEND_WAIT_REQ_0 << SUB_SHIFT);
		step(false);
		break;

	case SEND_WAIT_REQ_0 << SUB_SHIFT:
		if(ctrl & S_REQ)
			break;
		state = state & STATE_MASK;
		scsi_bus->data_w(scsi_refid, 0);
		scsi_bus->ctrl_w(scsi_refid, 0, S_ACK);
		step(false);
		break;

	case RECV_WAIT_REQ_1 << SUB_SHIFT:
		if(!(ctrl & S_REQ))
			break;

		state = (state & STATE_MASK) | (RECV_WAIT_SETTLE << SUB_SHIFT);
		delay_cycles(sync_period);
		break;

	case RECV_WAIT_SETTLE << SUB_SHIFT:
		if(!timeout)
			break;

		if((state & STATE_MASK) != INIT_XFR_RECV_PAD)
			fifo_push(scsi_bus->data_r());
		scsi_bus->ctrl_w(scsi_refid, S_ACK, S_ACK);
		state = (state & STATE_MASK) | (RECV_WAIT_REQ_0 << SUB_SHIFT);
		step(false);
		break;

	case RECV_WAIT_REQ_0 << SUB_SHIFT:
		if(ctrl & S_REQ)
			break;
		state = state & STATE_MASK;
		step(false);
		break;

	case DISC_SEL_ARBITRATION:
		if(c == CD_SELECT) {
			state = DISC_SEL_WAIT_REQ;
			command_length = derive_msg_size(fifo[0]);
		} else
			state = DISC_SEL_ATN_WAIT_REQ;

		scsi_bus->ctrl_wait(scsi_refid, S_REQ, S_REQ);
		if(ctrl & S_REQ)
			step(false);
		break;

	case DISC_SEL_ATN_WAIT_REQ:
		if(!(ctrl & S_REQ))
			break;
		if((ctrl & S_PHASE_MASK) != S_PHASE_MSG_OUT) {
			function_complete();
			break;
		}
		if(c == CD_SELECT_ATN)
			scsi_bus->ctrl_w(scsi_refid, 0, S_ATN);
		state = DISC_SEL_ATN_SEND_BYTE;
		send_byte();
		break;

	case DISC_SEL_ATN_SEND_BYTE:
		if(c == CD_SELECT_ATN_STOP) {
			seq = 1;
			function_complete();
		} else {
			command_length = derive_msg_size(fifo[0]);
			state = DISC_SEL_WAIT_REQ;
		}
		break;

	case DISC_SEL_WAIT_REQ:
		if(!(ctrl & S_REQ))
			break;
		if((ctrl & S_PHASE_MASK) != S_PHASE_COMMAND) {
			if(!command_length)
				seq = 4;
			scsi_bus->ctrl_wait(scsi_refid, 0, S_REQ);
			function_bus_complete();
			break;
		}
		if(seq < 3)
			seq = 3;
		state = DISC_SEL_SEND_BYTE;
		send_byte();
		break;

	case DISC_SEL_SEND_BYTE:
		if(command_length) {
			command_length--;
			if(!command_length)
				seq = 4;
		}

		state = DISC_SEL_WAIT_REQ;
		break;

	case INIT_CPT_RECV_BYTE_ACK:
		state = INIT_CPT_RECV_WAIT_REQ;
		scsi_bus->ctrl_w(scsi_refid, 0, S_ACK);
		break;

	case INIT_CPT_RECV_WAIT_REQ:
		if(!(ctrl & S_REQ))
			break;

		if((ctrl & S_PHASE_MASK) != S_PHASE_MSG_IN) {
			command_pos = 0;
			bus_complete();
		} else {
			state = INIT_CPT_RECV_BYTE_NACK;
			recv_byte();
		}
		break;

	case INIT_CPT_RECV_BYTE_NACK:
		scsi_bus->ctrl_wait(scsi_refid, 0, S_REQ);
		function_complete();
		break;

	case INIT_MSG_WAIT_REQ:
		if((ctrl & (S_REQ|S_BSY)) == S_BSY)
			break;
		bus_complete();
		break;

	case INIT_XFR:
		switch(xfr_phase) {
		case S_PHASE_DATA_OUT:
		case S_PHASE_COMMAND:
		case S_PHASE_MSG_OUT:
			dma_set(dma_command ? DMA_OUT : DMA_NONE);
			state = INIT_XFR_SEND_BYTE;

			// can't send if the fifo is empty
			if (fifo_pos == 0)
				break;

			// if it's the last message byte, deassert ATN before sending
			if (xfr_phase == S_PHASE_MSG_OUT && ((!dma_command && fifo_pos == 1) || (dma_command && tcounter == 1)))
				scsi_bus->ctrl_w(scsi_refid, 0, S_ATN);

			send_byte();
			break;

		case S_PHASE_DATA_IN:
		case S_PHASE_STATUS:
		case S_PHASE_MSG_IN:
			dma_set(dma_command ? DMA_IN : DMA_NONE);

			// can't receive if the fifo is full
			if (fifo_pos == 16)
				break;

			// if it's the last message byte, ACK remains asserted, terminate with function_complete()
			state = (xfr_phase == S_PHASE_MSG_IN && (!dma_command || tcounter == 1)) ? INIT_XFR_RECV_BYTE_NACK : INIT_XFR_RECV_BYTE_ACK;

			recv_byte();
			break;

		default:
			logerror("%s: xfer on phase %d\n", tag(), scsi_bus->ctrl_r() & S_PHASE_MASK);
			function_complete();
			break;
		}
		break;

	case INIT_XFR_WAIT_REQ:
		if(!(ctrl & S_REQ))
			break;

		// check for command complete
		if ((dma_command && tcounter == 0)                                  // dma in/out: transfer counter == 0
		|| (!dma_command && (xfr_phase & S_INP) == 0 && fifo_pos == 0)      // non-dma out: fifo empty
		|| (!dma_command && (xfr_phase & S_INP) == S_INP && fifo_pos == 1)) // non-dma in: every byte
			state = INIT_XFR_BUS_COMPLETE;
		else
			// check for phase change
			if((ctrl & S_PHASE_MASK) != xfr_phase) {
				command_pos = 0;
				state = INIT_XFR_BUS_COMPLETE;
			} else {
				state = INIT_XFR;
			}
		step(false);
		break;

	case INIT_XFR_SEND_BYTE:
		decrement_tcounter();
		state = INIT_XFR_WAIT_REQ;
		step(false);
		break;

	case INIT_XFR_RECV_BYTE_ACK:
		decrement_tcounter();
		state = INIT_XFR_WAIT_REQ;
		scsi_bus->ctrl_w(scsi_refid, 0, S_ACK);
		break;

	case INIT_XFR_RECV_BYTE_NACK:
		decrement_tcounter();
		state = INIT_XFR_FUNCTION_COMPLETE;
		step(false);
		break;

	case INIT_XFR_FUNCTION_COMPLETE:
		// wait for the fifo to drain
		if (dma_command && fifo_pos)
			break;

		function_complete();
		break;

	case INIT_XFR_BUS_COMPLETE:
		// wait for the fifo to drain
		if (dma_command && fifo_pos)
			break;

		bus_complete();
		break;

	case INIT_XFR_SEND_PAD_WAIT_REQ:
		if(!(ctrl & S_REQ))
			break;

		if((ctrl & S_PHASE_MASK) != xfr_phase) {
			command_pos = 0;
			bus_complete();
		} else {
			state = INIT_XFR_SEND_PAD;
			send_byte();
		}
		break;

	case INIT_XFR_SEND_PAD:
		decrement_tcounter();
		if(tcounter) {
			state = INIT_XFR_SEND_PAD_WAIT_REQ;
			step(false);
		} else
			function_complete();
		break;

	case INIT_XFR_RECV_PAD_WAIT_REQ:
		if(!(ctrl & S_REQ))
			break;

		if((ctrl & S_PHASE_MASK) != xfr_phase) {
			command_pos = 0;
			bus_complete();
		} else {
			state = INIT_XFR_RECV_PAD;
			recv_byte();
		}
		break;

	case INIT_XFR_RECV_PAD:
		decrement_tcounter();
		if(tcounter) {
			state = INIT_XFR_RECV_PAD_WAIT_REQ;
			scsi_bus->ctrl_w(scsi_refid, 0, S_ACK);
			step(false);
		} else
			function_complete();
		break;

	default:
		logerror("%s: step() unexpected state %d.%d\n",
					tag(),
					state & STATE_MASK, (state & SUB_MASK) >> SUB_SHIFT);
		exit(0);
	}
}

void ncr5390_device::send_byte()
{
	if(!fifo_pos)
		fatalerror("ncr5390_device::send_byte - !fifo_pos\n");

	state = (state & STATE_MASK) | (SEND_WAIT_SETTLE << SUB_SHIFT);
	if((state & STATE_MASK) != INIT_XFR_SEND_PAD &&
		((state & STATE_MASK) != DISC_SEL_SEND_BYTE ||
		command_length))
		scsi_bus->data_w(scsi_refid, fifo_pop());
	else
		scsi_bus->data_w(scsi_refid, 0);

	scsi_bus->ctrl_w(scsi_refid, S_ACK, S_ACK);
	scsi_bus->ctrl_wait(scsi_refid, S_REQ, S_REQ);
	delay_cycles(sync_period);
}

void ncr5390_device::recv_byte()
{
	scsi_bus->ctrl_wait(scsi_refid, S_REQ, S_REQ);
	state = (state & STATE_MASK) | (RECV_WAIT_REQ_1 << SUB_SHIFT);
	step(false);
}

void ncr5390_device::function_bus_complete()
{
	state = IDLE;
	istatus |= I_FUNCTION|I_BUS;
	dma_set(DMA_NONE);
	check_irq();
}

void ncr5390_device::function_complete()
{
	state = IDLE;
	istatus |= I_FUNCTION;
	dma_set(DMA_NONE);
	check_irq();
}

void ncr5390_device::bus_complete()
{
	state = IDLE;
	istatus |= I_BUS;
	dma_set(DMA_NONE);
	check_irq();
}

void ncr5390_device::delay(int cycles)
{
	if(!clock_conv)
		return;
	cycles *= clock_conv;
	tm->adjust(clocks_to_attotime(cycles));
}

void ncr5390_device::delay_cycles(int cycles)
{
	tm->adjust(clocks_to_attotime(cycles));
}

READ8_MEMBER(ncr5390_device::tcounter_lo_r)
{
	logerror("%s: tcounter_lo_r %02x (%s)\n", tag(), tcounter & 0xff, machine().describe_context());
	return tcounter;
}

WRITE8_MEMBER(ncr5390_device::tcount_lo_w)
{
	tcount = (tcount & 0xff00) | data;
	logerror("%s: tcount_lo_w %02x (%s)\n", tag(), data, machine().describe_context());
}

READ8_MEMBER(ncr5390_device::tcounter_hi_r)
{
	logerror("%s: tcounter_hi_r %02x (%s)\n", tag(), tcounter >> 8, machine().describe_context());
	return tcounter >> 8;
}

WRITE8_MEMBER(ncr5390_device::tcount_hi_w)
{
	tcount = (tcount & 0x00ff) | (data << 8);
	logerror("%s: tcount_hi_w %02x (%s)\n", tag(), data, machine().describe_context());
}

uint8_t ncr5390_device::fifo_pop()
{
	uint8_t r = fifo[0];
	fifo_pos--;
	memmove(fifo, fifo+1, fifo_pos);
	if((!fifo_pos) && dma_dir == DMA_OUT)
		drq_set();
	return r;
}

void ncr5390_device::fifo_push(uint8_t val)
{
	fifo[fifo_pos++] = val;
	if(!drq && dma_dir == DMA_IN)
		drq_set();
}

READ8_MEMBER(ncr5390_device::fifo_r)
{
	uint8_t r;
	if(fifo_pos) {
		r = fifo[0];
		fifo_pos--;
		memmove(fifo, fifo+1, fifo_pos);
	} else
		r = 0;
	return r;
}

WRITE8_MEMBER(ncr5390_device::fifo_w)
{
	if(fifo_pos != 16)
		fifo[fifo_pos++] = data;
}

READ8_MEMBER(ncr5390_device::command_r)
{
	logerror("%s: command_r (%s)\n", tag(), machine().describe_context());
	return command[0];
}

WRITE8_MEMBER(ncr5390_device::command_w)
{
	logerror("%s: command_w %02x (%s)\n", tag(), data, machine().describe_context());
	if(command_pos == 2) {
		status |= S_GROSS_ERROR;
		check_irq();
		return;
	}
	command[command_pos++] = data;
	if(command_pos == 1)
		start_command();
}

void ncr5390_device::command_pop_and_chain()
{
	if(command_pos) {
		command_pos--;
		if(command_pos) {
			command[0] = command[1];
			start_command();
		}
	}
}

void ncr5390_device::start_command()
{
	uint8_t c = command[0] & 0x7f;
	if(!check_valid_command(c)) {
		logerror("%s: invalid command %02x\n", tag(), command[0]);
		istatus |= I_ILLEGAL;
		check_irq();
		return;
	}

	// for dma commands, reload transfer counter
	dma_command = command[0] & 0x80;
	if (dma_command)
	{
		tcounter = tcount;

		// clear transfer count zero flag when counter is reloaded
		if (tcounter)
			status &= ~S_TC0;
	}

	switch(c) {
	case CM_NOP:
		command_pop_and_chain();
		break;

	case CM_FLUSH_FIFO:
		fifo_pos = 0;
		command_pop_and_chain();
		break;

	case CM_RESET:
		device_reset();
		break;

	case CM_RESET_BUS:
		reset_soft();
		break;

	case CD_RESELECT:
		state = DISC_REC_ARBITRATION;
		arbitrate();
		break;

	case CD_SELECT:
	case CD_SELECT_ATN:
	case CD_SELECT_ATN_STOP:
		seq = 0;
		state = DISC_SEL_ARBITRATION;
		arbitrate();
		break;

	case CD_ENABLE_SEL:
		command_pop_and_chain();
		break;

	case CD_DISABLE_SEL:
		command_pop_and_chain();
		break;

	case CI_XFER:
		state = INIT_XFR;
		xfr_phase = scsi_bus->ctrl_r() & S_PHASE_MASK;
		step(false);
		break;

	case CI_COMPLETE:
		state = INIT_CPT_RECV_BYTE_ACK;
		recv_byte();
		break;

	case CI_MSG_ACCEPT:
		state = INIT_MSG_WAIT_REQ;
		// It's undocumented what the sequence register should contain after a message accept
		// command, but the InterPro boot code expects it to be non-zero; setting it to an
		// arbirary 1 here makes InterPro happy. Also in the InterPro case (perhaps typical),
		// after ACK is asserted the device disconnects and the INIT_MSG_WAIT_REQ state is never
		// entered, meaning we end up with I_DISCONNECT instead of I_BUS interrupt status.
		seq = 1;
		scsi_bus->ctrl_w(scsi_refid, 0, S_ACK);
		step(false);
		break;

	case CI_PAD:
		xfr_phase = scsi_bus->ctrl_r() & S_PHASE_MASK;
		if(xfr_phase & S_INP)
			state = INIT_XFR_RECV_PAD_WAIT_REQ;
		else
			state = INIT_XFR_SEND_PAD_WAIT_REQ;
		scsi_bus->ctrl_w(scsi_refid, 0, S_ACK);
		step(false);
		break;

	default:
		logerror("%s: start unimplemented command %02x\n", tag(), c);
		exit(0);
	}
}

bool ncr5390_device::check_valid_command(uint8_t cmd)
{
	int subcmd = cmd & 15;
	switch((cmd >> 4) & 7) {
	case 0: return subcmd <= 3;
	case 4: return mode == MODE_D && subcmd <= 5;
	case 2: return mode == MODE_T && subcmd <= 13 && subcmd != 6;
	case 1: return mode == MODE_I && (subcmd <= 2 || subcmd == 8 || subcmd == 10);
	}
	return false;
}

int ncr5390_device::derive_msg_size(uint8_t msg_id)
{
	const static int sizes[8] = { 6, 10, 6, 6, 6, 12, 6, 10 };
	return sizes[msg_id >> 5];
}

void ncr5390_device::arbitrate()
{
	state = (state & STATE_MASK) | (ARB_COMPLETE << SUB_SHIFT);
	scsi_bus->data_w(scsi_refid, 1 << scsi_id);
	scsi_bus->ctrl_w(scsi_refid, S_BSY, S_BSY);
	delay(11);
}

void ncr5390_device::check_irq()
{
	bool oldirq = irq;
	irq = istatus != 0;
	if(irq != oldirq)
		m_irq_handler(irq);

}

READ8_MEMBER(ncr5390_device::status_r)
{
	uint32_t ctrl = scsi_bus->ctrl_r();
	uint8_t res = status | (ctrl & S_MSG ? 4 : 0) | (ctrl & S_CTL ? 2 : 0) | (ctrl & S_INP ? 1 : 0);
	logerror("%s: status_r %02x (%s)\n", tag(), res, machine().describe_context());

	return res;
}

WRITE8_MEMBER(ncr5390_device::bus_id_w)
{
	bus_id = data & 7;
	logerror("%s: bus_id=%d\n", tag(), bus_id);
}

READ8_MEMBER(ncr5390_device::istatus_r)
{
	uint8_t res = istatus;

	if (irq)
	{
		status &= ~(S_GROSS_ERROR | S_PARITY | S_TCC);
		istatus = 0;
		seq = 0;
	}
	check_irq();
	if(res)
		command_pop_and_chain();

	logerror("%s: istatus_r %02x (%s)\n", tag(), res, machine().describe_context());
	return res;
}

WRITE8_MEMBER(ncr5390_device::timeout_w)
{
	select_timeout = data;
}

READ8_MEMBER(ncr5390_device::seq_step_r)
{
	logerror("%s: seq_step_r %d (%s)\n", tag(), seq, machine().describe_context());
	return seq;
}

WRITE8_MEMBER(ncr5390_device::sync_period_w)
{
	sync_period = data & 0x1f;
}

READ8_MEMBER(ncr5390_device::fifo_flags_r)
{
	return fifo_pos;
}

WRITE8_MEMBER(ncr5390_device::sync_offset_w)
{
	sync_offset = data & 0x0f;
}

READ8_MEMBER(ncr5390_device::conf_r)
{
	return config;
}

WRITE8_MEMBER(ncr5390_device::conf_w)
{
	config = data;
	scsi_id = data & 7;

	// test mode can only be cleared by hard/soft reset
	if (data & 0x8)
		test_mode = true;
}

WRITE8_MEMBER(ncr5390_device::test_w)
{
	if (test_mode)
		logerror("%s: test_w %d (%s) - test mode not implemented\n", tag(), data, machine().describe_context());
}

WRITE8_MEMBER(ncr5390_device::clock_w)
{
	clock_conv = data & 0x07;
}

void ncr5390_device::dma_set(int dir)
{
	dma_dir = dir;
	if(dma_dir == DMA_OUT && fifo_pos != 16 && tcounter > fifo_pos)
		drq_set();
}

void ncr5390_device::dma_w(uint8_t val)
{
	fifo_push(val);
	if(fifo_pos == 16)
		drq_clear();
	step(false);
}

uint8_t ncr5390_device::dma_r()
{
	uint8_t r = fifo_pop();
	if(!fifo_pos)
		drq_clear();
	step(false);
	return r;
}

void ncr5390_device::drq_set()
{
	if(!drq) {
		drq = true;
		m_drq_handler(drq);
	}
}

void ncr5390_device::drq_clear()
{
	if(drq) {
		drq = false;
		m_drq_handler(drq);
	}
}

void ncr5390_device::decrement_tcounter()
{
	if (!dma_command)
		return;

	tcounter--;
	if (tcounter == 0)
		status |= S_TC0;
}

/*
 * According to the NCR 53C90A, 53C90B data book (http://bitsavers.informatik.uni-stuttgart.de/pdf/ncr/scsi/NCR53C90ab.pdf),
 * the following are the differences from the 53C90:
 *
 *   - Supports three-byte message exchange SCSI-2 tagged queueing
 *   - Added select with ATN3 command
 *   - Added target DMA abort command
 *   - Added interrupt polling bit
 *   - Added second configuration register
 *   - Improved immunity to cable impedance mismatches and improper termination
 *   - Tri-state DMA request output
 *   - Cut leakage current on SCSI input pins when powered off
 *   - Relaxed register timings
 *   - Relaxed DMA timings
 *   - Relaxed CLK duty cycle
 *   - Lengthened read data access time
 *   - NOP required less often
 */

void ncr53c90a_device::device_start()
{
	save_item(NAME(config2));

	config2 = 0;

	ncr5390_device::device_start();
}

void ncr53c90a_device::reset_soft()
{
	config2 = 0;

	ncr5390_device::reset_soft();
}

READ8_MEMBER(ncr53c90a_device::status_r)
{
	uint32_t ctrl = scsi_bus->ctrl_r();
	uint8_t res = (irq ? S_INTERRUPT : 0) | status | (ctrl & S_MSG ? 4 : 0) | (ctrl & S_CTL ? 2 : 0) | (ctrl & S_INP ? 1 : 0);
	logerror("%s: status_r %02x (%s)\n", tag(), res, machine().describe_context());
	if (irq)
		status &= ~(S_GROSS_ERROR | S_PARITY | S_TCC);
	return res;
}


void ncr53c94_device::device_start()
{
	save_item(NAME(config3));

	config3 = 0;

	ncr53c90a_device::device_start();
}

void ncr53c94_device::reset_soft()
{
	config3 = 0;

	ncr53c90a_device::reset_soft();
}
