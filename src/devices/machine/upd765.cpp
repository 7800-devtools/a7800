// license:BSD-3-Clause
// copyright-holders:Olivier Galibert

#include "emu.h"
#include "upd765.h"
#include "debugger.h"

#define LOG 0

DEFINE_DEVICE_TYPE(UPD765A,        upd765a_device,        "upd765a",        "NEC uPD765A FDC")
DEFINE_DEVICE_TYPE(UPD765B,        upd765b_device,        "upd765b",        "NEC uPD765B FDC")
DEFINE_DEVICE_TYPE(I8272A,         i8272a_device,         "i8272a",         "Intel 8272A FDC")
DEFINE_DEVICE_TYPE(UPD72065,       upd72065_device,       "upd72065",       "NEC uPD72065 FDC")
DEFINE_DEVICE_TYPE(SMC37C78,       smc37c78_device,       "smc37c78",       "SMC FDC73C78 FDC")
DEFINE_DEVICE_TYPE(N82077AA,       n82077aa_device,       "n82077aa",       "Intel N82077AA FDC")
DEFINE_DEVICE_TYPE(PC_FDC_SUPERIO, pc_fdc_superio_device, "pc_fdc_superio", "PC FDC SUPERIO")
DEFINE_DEVICE_TYPE(DP8473,         dp8473_device,         "dp8473",         "National Semiconductor DP8473 FDC")
DEFINE_DEVICE_TYPE(PC8477A,        pc8477a_device,        "pc8477a",        "National Semiconductor PC8477A FDC")
DEFINE_DEVICE_TYPE(WD37C65C,       wd37c65c_device,       "wd37c65c",       "Western Digital WD37C65C FDC")
DEFINE_DEVICE_TYPE(MCS3201,        mcs3201_device,        "mcs3201",        "Motorola MCS3201 FDC")
DEFINE_DEVICE_TYPE(TC8566AF,       tc8566af_device,       "tc8566af",       "Toshiba TC8566AF FDC")

DEVICE_ADDRESS_MAP_START(map, 8, upd765a_device)
	AM_RANGE(0x0, 0x0) AM_READ(msr_r)
	AM_RANGE(0x1, 0x1) AM_READWRITE(fifo_r, fifo_w)
ADDRESS_MAP_END

DEVICE_ADDRESS_MAP_START(map, 8, upd765b_device)
	AM_RANGE(0x0, 0x0) AM_READ(msr_r)
	AM_RANGE(0x1, 0x1) AM_READWRITE(fifo_r, fifo_w)
ADDRESS_MAP_END

DEVICE_ADDRESS_MAP_START(map, 8, i8272a_device)
	AM_RANGE(0x0, 0x0) AM_READ(msr_r)
	AM_RANGE(0x1, 0x1) AM_READWRITE(fifo_r, fifo_w)
ADDRESS_MAP_END

DEVICE_ADDRESS_MAP_START(map, 8, upd72065_device)
	AM_RANGE(0x0, 0x0) AM_READ(msr_r)
	AM_RANGE(0x1, 0x1) AM_READWRITE(fifo_r, fifo_w)
ADDRESS_MAP_END

DEVICE_ADDRESS_MAP_START(map, 8, smc37c78_device)
	AM_RANGE(0x2, 0x2) AM_READWRITE(dor_r, dor_w)
	AM_RANGE(0x3, 0x3) AM_READWRITE(tdr_r, tdr_w)
	AM_RANGE(0x4, 0x4) AM_READWRITE(msr_r, dsr_w)
	AM_RANGE(0x5, 0x5) AM_READWRITE(fifo_r, fifo_w)
	AM_RANGE(0x7, 0x7) AM_READWRITE(dir_r, ccr_w)
ADDRESS_MAP_END

DEVICE_ADDRESS_MAP_START(map, 8, n82077aa_device)
	AM_RANGE(0x0, 0x0) AM_READ(sra_r)
	AM_RANGE(0x1, 0x1) AM_READ(srb_r)
	AM_RANGE(0x2, 0x2) AM_READWRITE(dor_r, dor_w)
	AM_RANGE(0x3, 0x3) AM_READWRITE(tdr_r, tdr_w)
	AM_RANGE(0x4, 0x4) AM_READWRITE(msr_r, dsr_w)
	AM_RANGE(0x5, 0x5) AM_READWRITE(fifo_r, fifo_w)
	AM_RANGE(0x7, 0x7) AM_READWRITE(dir_r, ccr_w)
ADDRESS_MAP_END

DEVICE_ADDRESS_MAP_START(map, 8, pc_fdc_superio_device)
	AM_RANGE(0x0, 0x0) AM_READ(sra_r)
	AM_RANGE(0x1, 0x1) AM_READ(srb_r)
	AM_RANGE(0x2, 0x2) AM_READWRITE(dor_r, dor_w)
	AM_RANGE(0x3, 0x3) AM_READWRITE(tdr_r, tdr_w)
	AM_RANGE(0x4, 0x4) AM_READWRITE(msr_r, dsr_w)
	AM_RANGE(0x5, 0x5) AM_READWRITE(fifo_r, fifo_w)
	AM_RANGE(0x7, 0x7) AM_READWRITE(dir_r, ccr_w)
ADDRESS_MAP_END

DEVICE_ADDRESS_MAP_START(map, 8, dp8473_device)
	AM_RANGE(0x0, 0x0) AM_READ(sra_r)
	AM_RANGE(0x1, 0x1) AM_READ(srb_r)
	AM_RANGE(0x2, 0x2) AM_READWRITE(dor_r, dor_w)
	AM_RANGE(0x3, 0x3) AM_READWRITE(tdr_r, tdr_w)
	AM_RANGE(0x4, 0x4) AM_READWRITE(msr_r, dsr_w)
	AM_RANGE(0x5, 0x5) AM_READWRITE(fifo_r, fifo_w)
	AM_RANGE(0x7, 0x7) AM_READWRITE(dir_r, ccr_w)
ADDRESS_MAP_END

DEVICE_ADDRESS_MAP_START(map, 8, pc8477a_device)
	AM_RANGE(0x0, 0x0) AM_READ(sra_r)
	AM_RANGE(0x1, 0x1) AM_READ(srb_r)
	AM_RANGE(0x2, 0x2) AM_READWRITE(dor_r, dor_w)
	AM_RANGE(0x3, 0x3) AM_READWRITE(tdr_r, tdr_w)
	AM_RANGE(0x4, 0x4) AM_READWRITE(msr_r, dsr_w)
	AM_RANGE(0x5, 0x5) AM_READWRITE(fifo_r, fifo_w)
	AM_RANGE(0x7, 0x7) AM_READWRITE(dir_r, ccr_w)
ADDRESS_MAP_END

DEVICE_ADDRESS_MAP_START(map, 8, wd37c65c_device)
	AM_RANGE(0x0, 0x0) AM_READ(msr_r)
	AM_RANGE(0x1, 0x1) AM_READWRITE(fifo_r, fifo_w)
ADDRESS_MAP_END

DEVICE_ADDRESS_MAP_START( map, 8, mcs3201_device )
	AM_RANGE(0x0, 0x0) AM_READ(input_r)
	AM_RANGE(0x2, 0x2) AM_WRITE(dor_w)
	AM_RANGE(0x4, 0x4) AM_READ(msr_r)
	AM_RANGE(0x5, 0x5) AM_READWRITE(fifo_r, fifo_w)
	AM_RANGE(0x7, 0x7) AM_READWRITE(dir_r, ccr_w)
ADDRESS_MAP_END

DEVICE_ADDRESS_MAP_START( map, 8, tc8566af_device )
	AM_RANGE(0x2, 0x2) AM_WRITE(dor_w)
	AM_RANGE(0x3, 0x3) AM_WRITE(cr1_w)
	AM_RANGE(0x4, 0x4) AM_READ(msr_r)
	AM_RANGE(0x5, 0x5) AM_READWRITE(fifo_r, fifo_w)
ADDRESS_MAP_END


constexpr int upd765_family_device::rates[4];

upd765_family_device::upd765_family_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	pc_fdc_interface(mconfig, type, tag, owner, clock),
	intrq_cb(*this),
	drq_cb(*this),
	hdl_cb(*this)
{
	ready_polled = true;
	ready_connected = true;
	select_connected = true;
	external_ready = false;
	dor_reset = 0x00;
	mode = MODE_AT;
}

void upd765_family_device::set_ready_line_connected(bool _ready)
{
	ready_connected = _ready;
}

void upd765_family_device::set_select_lines_connected(bool _select)
{
	select_connected = _select;
}

void upd765_family_device::set_mode(int _mode)
{
	mode = _mode;
}

void upd765_family_device::device_start()
{
	intrq_cb.resolve_safe();
	drq_cb.resolve_safe();
	hdl_cb.resolve_safe();

	for(int i=0; i != 4; i++) {
		char name[2];
		flopi[i].tm = timer_alloc(i);
		flopi[i].id = i;
		if(select_connected) {
			name[0] = '0'+i;
			name[1] = 0;
			floppy_connector *con = subdevice<floppy_connector>(name);
			if(con) {
				flopi[i].dev = con->get_device();
				if (flopi[i].dev != nullptr)
					flopi[i].dev->setup_index_pulse_cb(floppy_image_device::index_pulse_cb(&upd765_family_device::index_callback, this));
			} else
				flopi[i].dev = nullptr;
		} else
			flopi[i].dev = nullptr;

		flopi[i].main_state = IDLE;
		flopi[i].sub_state = IDLE;
		flopi[i].dir = 0;
		flopi[i].counter = 0;
		flopi[i].pcn = 0;
		flopi[i].st0 = 0;
		flopi[i].st0_filled = false;
		flopi[i].live = false;
		flopi[i].index = false;
		flopi[i].ready = false;
	}
	cur_rate = 250000;
	tc = false;

	// reset at upper levels may cause a write to tc ending up with
	// live_sync, which will crash if the live structure isn't
	// initialized enough

	cur_live.tm = attotime::never;
	cur_live.state = IDLE;
	cur_live.next_state = -1;
	cur_live.fi = nullptr;

	if(ready_polled) {
		poll_timer = timer_alloc(TIMER_DRIVE_READY_POLLING);
		poll_timer->adjust(attotime::from_usec(100), 0, attotime::from_usec(1024));
	} else
		poll_timer = nullptr;

	cur_irq = false;
	locked = false;
}

void upd765_family_device::device_reset()
{
	dor = dor_reset;
	locked = false;
	soft_reset();
}

void upd765_family_device::soft_reset()
{
	main_phase = PHASE_CMD;
	for(int i=0; i<4; i++) {
		flopi[i].main_state = IDLE;
		flopi[i].sub_state = IDLE;
		flopi[i].live = false;
		flopi[i].ready = !ready_polled;
		flopi[i].st0 = i;
		flopi[i].st0_filled = false;
	}
	data_irq = false;
	other_irq = false;
	internal_drq = false;
	fifo_pos = 0;
	command_pos = 0;
	result_pos = 0;
	if(!locked)
		fifocfg = FIF_DIS;
	cur_live.fi = nullptr;
	drq = false;
	cur_live.tm = attotime::never;
	cur_live.state = IDLE;
	cur_live.next_state = -1;
	cur_live.fi = nullptr;
	tc_done = false;
	st1 = st2 = st3 = 0x00;

	check_irq();
	if(ready_polled)
		poll_timer->adjust(attotime::from_usec(100), 0, attotime::from_usec(1024));
}

void upd765_family_device::tc_w(bool _tc)
{
	if (LOG) logerror("tc=%d\n", _tc);
	if(tc != _tc && _tc) {
		live_sync();
		tc_done = true;
		tc = _tc;
		if(cur_live.fi)
			general_continue(*cur_live.fi);
	} else
		tc = _tc;
}

void upd765_family_device::ready_w(bool _ready)
{
	external_ready = _ready;
}

bool upd765_family_device::get_ready(int fid)
{
	if(ready_connected)
		return flopi[fid].dev ? !flopi[fid].dev->ready_r() : false;
	return !external_ready;
}

void upd765_family_device::set_floppy(floppy_image_device *flop)
{
	for(auto & elem : flopi) {
		if(elem.dev)
			elem.dev->setup_index_pulse_cb(floppy_image_device::index_pulse_cb());
		elem.dev = flop;
	}
	if(flop)
		flop->setup_index_pulse_cb(floppy_image_device::index_pulse_cb(&upd765_family_device::index_callback, this));
}

READ8_MEMBER(upd765_family_device::sra_r)
{
	uint8_t sra = 0;
	int fid = dor & 3;
	floppy_info &fi = flopi[fid];
	if(fi.dir)
		sra |= 0x01;
	if(fi.index)
		sra |= 0x04;
	if(cur_rate >= 500000)
		sra |= 0x08;
	if(fi.dev && fi.dev->trk00_r())
		sra |= 0x10;
	if(fi.main_state == SEEK_WAIT_STEP_SIGNAL_TIME)
		sra |= 0x20;
	sra |= 0x40;
	if(cur_irq)
		sra |= 0x80;
	if(mode == MODE_M30)
		sra ^= 0x1f;
	return sra;
}

READ8_MEMBER(upd765_family_device::srb_r)
{
	return 0;
}

READ8_MEMBER(upd765_family_device::dor_r)
{
	return dor;
}

WRITE8_MEMBER(upd765_family_device::dor_w)
{
	if (LOG) logerror("dor = %02x\n", data);
	uint8_t diff = dor ^ data;
	dor = data;
	if(diff & 4)
		soft_reset();

	for(int i=0; i<4; i++) {
		floppy_info &fi = flopi[i];
		if(fi.dev)
			fi.dev->mon_w(!(dor & (0x10 << i)));
	}
	check_irq();
}

READ8_MEMBER(upd765_family_device::tdr_r)
{
	return 0;
}

WRITE8_MEMBER(upd765_family_device::tdr_w)
{
}

READ8_MEMBER(upd765_family_device::msr_r)
{
	uint32_t msr = 0;
	switch(main_phase) {
	case PHASE_CMD:
		msr |= MSR_RQM;
		if(command_pos)
			msr |= MSR_CB;
		break;
	case PHASE_EXEC:
		msr |= MSR_CB;
		if(spec & SPEC_ND)
			msr |= MSR_EXM;
		if(internal_drq) {
			msr |= MSR_RQM;
			if(!fifo_write)
				msr |= MSR_DIO;
		}
		break;

	case PHASE_RESULT:
		msr |= MSR_RQM|MSR_DIO|MSR_CB;
		break;
	}
	for(int i=0; i<4; i++)
		if(flopi[i].main_state == RECALIBRATE || flopi[i].main_state == SEEK) {
			msr |= 1<<i;
			//msr |= MSR_CB;
		}

	if(data_irq) {
		data_irq = false;
		check_irq();
	}

	return msr;
}

WRITE8_MEMBER(upd765_family_device::dsr_w)
{
	if (LOG) logerror("dsr_w %02x\n", data);
	if(data & 0x80)
		soft_reset();
	dsr = data & 0x7f;
	cur_rate = rates[dsr & 3];
}

void upd765_family_device::set_rate(int rate)
{
	cur_rate = rate;
}

READ8_MEMBER(upd765_family_device::fifo_r)
{
	uint8_t r = 0xff;
	switch(main_phase) {
	case PHASE_EXEC:
		if(internal_drq)
			return fifo_pop(false);
		if (LOG) logerror("fifo_r in phase %d\n", main_phase);
		break;

	case PHASE_RESULT:
		r = result[0];
		result_pos--;
		memmove(result, result+1, result_pos);
		if(!result_pos)
			main_phase = PHASE_CMD;
		break;
	default:
		if (LOG) logerror("fifo_r in phase %d\n", main_phase);
		break;
	}

	return r;
}

WRITE8_MEMBER(upd765_family_device::fifo_w)
{
	switch(main_phase) {
	case PHASE_CMD: {
		command[command_pos++] = data;
		other_irq = false;
		check_irq();
		int cmd = check_command();
		if(cmd == C_INCOMPLETE)
			break;
		if(cmd == C_INVALID) {
			if (LOG) logerror("Invalid on %02x\n", command[0]);
			main_phase = PHASE_RESULT;
			result[0] = ST0_UNK;
			result_pos = 1;
			command_pos = 0;
			return;
		}
		start_command(cmd);
		break;
	}
	case PHASE_EXEC:
		if(internal_drq) {
			fifo_push(data, false);
			return;
		}
		if (LOG) logerror("fifo_w in phase %d\n", main_phase);
		break;

	default:
		if (LOG) logerror("fifo_w in phase %d\n", main_phase);
		break;
	}
}

uint8_t upd765_family_device::do_dir_r()
{
	floppy_info &fi = flopi[dor & 3];
	if(fi.dev)
		return fi.dev->dskchg_r() ? 0x00 : 0x80;
	return 0x00;
}

READ8_MEMBER(upd765_family_device::dir_r)
{
	return do_dir_r();
}

WRITE8_MEMBER(upd765_family_device::ccr_w)
{
	dsr = (dsr & 0xfc) | (data & 3);
	cur_rate = rates[data & 3];
}

void upd765_family_device::set_drq(bool state)
{
	if(state != drq) {
		drq = state;
		drq_cb(drq);
	}
}

bool upd765_family_device::get_drq() const
{
	return drq;
}

void upd765_family_device::enable_transfer()
{
	if(spec & SPEC_ND) {
		// PIO
		if(!internal_drq) {
			internal_drq = true;
			check_irq();
		}

	} else {
		// DMA
		if(!drq)
			set_drq(true);
	}
}

void upd765_family_device::disable_transfer()
{
	if(spec & SPEC_ND) {
		internal_drq = false;
		check_irq();
	} else
		set_drq(false);
}

void upd765_family_device::fifo_push(uint8_t data, bool internal)
{
	if(fifo_pos == 16) {
		if(internal) {
			if(!(st1 & ST1_OR))
				if (LOG) logerror("Fifo overrun\n");
			st1 |= ST1_OR;
		}
		return;
	}
	fifo[fifo_pos++] = data;
	fifo_expected--;

	int thr = (fifocfg & FIF_THR)+1;
	if(!fifo_write && (!fifo_expected || fifo_pos >= thr || (fifocfg & FIF_DIS)))
		enable_transfer();
	if(fifo_write && (fifo_pos == 16 || !fifo_expected))
		disable_transfer();
}


uint8_t upd765_family_device::fifo_pop(bool internal)
{
	if(!fifo_pos) {
		if(internal) {
			if(!(st1 & ST1_OR))
				if (LOG) logerror("Fifo underrun\n");
			st1 |= ST1_OR;
		}
		return 0;
	}
	uint8_t r = fifo[0];
	fifo_pos--;
	memmove(fifo, fifo+1, fifo_pos);
	if(!fifo_write && !fifo_pos)
		disable_transfer();
	int thr = fifocfg & 15;
	if(fifo_write && fifo_expected && (fifo_pos <= thr || (fifocfg & 0x20)))
		enable_transfer();
	return r;
}

void upd765_family_device::fifo_expect(int size, bool write)
{
	fifo_expected = size;
	fifo_write = write;
	if(fifo_write)
		enable_transfer();
}

READ8_MEMBER(upd765_family_device::mdma_r)
{
	return dma_r();
}

WRITE8_MEMBER(upd765_family_device::mdma_w)
{
	dma_w(data);
}

uint8_t upd765_family_device::dma_r()
{
	return fifo_pop(false);
}

void upd765_family_device::dma_w(uint8_t data)
{
	fifo_push(data, false);
}

void upd765_family_device::live_start(floppy_info &fi, int state)
{
	cur_live.tm = machine().time();
	cur_live.state = state;
	cur_live.next_state = -1;
	cur_live.fi = &fi;
	cur_live.shift_reg = 0;
	cur_live.crc = 0xffff;
	cur_live.bit_counter = 0;
	cur_live.data_separator_phase = false;
	cur_live.data_reg = 0;
	cur_live.previous_type = live_info::PT_NONE;
	cur_live.data_bit_context = false;
	cur_live.byte_counter = 0;
	cur_live.pll.reset(cur_live.tm);
	cur_live.pll.set_clock(attotime::from_hz(mfm ? 2*cur_rate : cur_rate));
	checkpoint_live = cur_live;
	fi.live = true;

	live_run();
}

void upd765_family_device::checkpoint()
{
	if(cur_live.fi)
		cur_live.pll.commit(cur_live.fi->dev, cur_live.tm);
	checkpoint_live = cur_live;
}

void upd765_family_device::rollback()
{
	cur_live = checkpoint_live;
}

void upd765_family_device::live_delay(int state)
{
	cur_live.next_state = state;
	if(cur_live.tm != machine().time())
		cur_live.fi->tm->adjust(cur_live.tm - machine().time());
	else
		live_sync();
}

void upd765_family_device::live_sync()
{
	if(!cur_live.tm.is_never()) {
		if(cur_live.tm > machine().time()) {
			rollback();
			live_run(machine().time());
			cur_live.pll.commit(cur_live.fi->dev, cur_live.tm);
		} else {
			cur_live.pll.commit(cur_live.fi->dev, cur_live.tm);
			if(cur_live.next_state != -1) {
				cur_live.state = cur_live.next_state;
				cur_live.next_state = -1;
			}
			if(cur_live.state == IDLE) {
				cur_live.pll.stop_writing(cur_live.fi->dev, cur_live.tm);
				cur_live.tm = attotime::never;
				cur_live.fi->live = false;
				cur_live.fi = nullptr;
			}
		}
		cur_live.next_state = -1;
		checkpoint();
	}
}

void upd765_family_device::live_abort()
{
	if(!cur_live.tm.is_never() && cur_live.tm > machine().time()) {
		rollback();
		live_run(machine().time());
	}

	if(cur_live.fi) {
		cur_live.pll.stop_writing(cur_live.fi->dev, cur_live.tm);
		cur_live.fi->live = false;
		cur_live.fi = nullptr;
	}

	cur_live.tm = attotime::never;
	cur_live.state = IDLE;
	cur_live.next_state = -1;
}

void upd765_family_device::live_run(attotime limit)
{
	if(cur_live.state == IDLE || cur_live.next_state != -1)
		return;

	if(limit == attotime::never) {
		if(cur_live.fi->dev)
			limit = cur_live.fi->dev->time_next_index();
		if(limit == attotime::never) {
			// Happens when there's no disk or if the fdc is not
			// connected to a drive, hence no index pulse. Force a
			// sync from time to time in that case, so that the main
			// cpu timeout isn't too painful.  Avoids looping into
			// infinity looking for data too.

			limit = machine().time() + attotime::from_msec(1);
			cur_live.fi->tm->adjust(attotime::from_msec(1));
		}
	}

	for(;;) {
		switch(cur_live.state) {
		case SEARCH_ADDRESS_MARK_HEADER:
			if(read_one_bit(limit))
				return;
#if 0
			fprintf(stderr, "%s: shift = %04x data=%02x c=%d\n", tts(cur_live.tm).c_str(), cur_live.shift_reg,
					(cur_live.shift_reg & 0x4000 ? 0x80 : 0x00) |
					(cur_live.shift_reg & 0x1000 ? 0x40 : 0x00) |
					(cur_live.shift_reg & 0x0400 ? 0x20 : 0x00) |
					(cur_live.shift_reg & 0x0100 ? 0x10 : 0x00) |
					(cur_live.shift_reg & 0x0040 ? 0x08 : 0x00) |
					(cur_live.shift_reg & 0x0010 ? 0x04 : 0x00) |
					(cur_live.shift_reg & 0x0004 ? 0x02 : 0x00) |
					(cur_live.shift_reg & 0x0001 ? 0x01 : 0x00),
					cur_live.bit_counter);
#endif

			if(mfm && cur_live.shift_reg == 0x4489) {
				cur_live.crc = 0x443b;
				cur_live.data_separator_phase = false;
				cur_live.bit_counter = 0;
				cur_live.state = READ_HEADER_BLOCK_HEADER;
			}

			if(!mfm && cur_live.shift_reg == 0xf57e) {
				cur_live.crc = 0xef21;
				cur_live.data_separator_phase = false;
				cur_live.bit_counter = 0;
				cur_live.state = READ_ID_BLOCK;
			}
			break;

		case READ_HEADER_BLOCK_HEADER: {
			if(read_one_bit(limit))
				return;
#if 0
			fprintf(stderr, "%s: shift = %04x data=%02x counter=%d\n", tts(cur_live.tm).c_str(), cur_live.shift_reg,
					(cur_live.shift_reg & 0x4000 ? 0x80 : 0x00) |
					(cur_live.shift_reg & 0x1000 ? 0x40 : 0x00) |
					(cur_live.shift_reg & 0x0400 ? 0x20 : 0x00) |
					(cur_live.shift_reg & 0x0100 ? 0x10 : 0x00) |
					(cur_live.shift_reg & 0x0040 ? 0x08 : 0x00) |
					(cur_live.shift_reg & 0x0010 ? 0x04 : 0x00) |
					(cur_live.shift_reg & 0x0004 ? 0x02 : 0x00) |
					(cur_live.shift_reg & 0x0001 ? 0x01 : 0x00),
					cur_live.bit_counter);
#endif
			if(cur_live.bit_counter & 15)
				break;

			int slot = cur_live.bit_counter >> 4;

			if(slot < 3) {
				if(cur_live.shift_reg != 0x4489)
					cur_live.state = SEARCH_ADDRESS_MARK_HEADER;
				break;
			}
			if(cur_live.data_reg != 0xfe) {
				cur_live.state = SEARCH_ADDRESS_MARK_HEADER;
				break;
			}

			cur_live.bit_counter = 0;
			cur_live.state = READ_ID_BLOCK;

			break;
		}

		case READ_ID_BLOCK: {
			if(read_one_bit(limit))
				return;
			if(cur_live.bit_counter & 15)
				break;
			int slot = (cur_live.bit_counter >> 4)-1;

			if(0)
				fprintf(stderr, "%s: slot=%d data=%02x crc=%04x\n", tts(cur_live.tm).c_str(), slot, cur_live.data_reg, cur_live.crc);
			cur_live.idbuf[slot] = cur_live.data_reg;
			if(slot == 5) {
				live_delay(IDLE);
				return;
			}
			break;
		}

		case SEARCH_ADDRESS_MARK_DATA:
			if(read_one_bit(limit))
				return;
#if 0
			fprintf(stderr, "%s: shift = %04x data=%02x c=%d.%x\n", tts(cur_live.tm).c_str(), cur_live.shift_reg,
					(cur_live.shift_reg & 0x4000 ? 0x80 : 0x00) |
					(cur_live.shift_reg & 0x1000 ? 0x40 : 0x00) |
					(cur_live.shift_reg & 0x0400 ? 0x20 : 0x00) |
					(cur_live.shift_reg & 0x0100 ? 0x10 : 0x00) |
					(cur_live.shift_reg & 0x0040 ? 0x08 : 0x00) |
					(cur_live.shift_reg & 0x0010 ? 0x04 : 0x00) |
					(cur_live.shift_reg & 0x0004 ? 0x02 : 0x00) |
					(cur_live.shift_reg & 0x0001 ? 0x01 : 0x00),
					cur_live.bit_counter >> 4, cur_live.bit_counter & 15);
#endif

			if(mfm) {
				// Large tolerance due to perpendicular recording at extended density
				if(cur_live.bit_counter > 62*16) {
					live_delay(SEARCH_ADDRESS_MARK_DATA_FAILED);
					return;
				}

				if(cur_live.bit_counter >= 28*16 && cur_live.shift_reg == 0x4489) {
					cur_live.crc = 0x443b;
					cur_live.data_separator_phase = false;
					cur_live.bit_counter = 0;
					cur_live.state = READ_DATA_BLOCK_HEADER;
				}

			} else {
				if(cur_live.bit_counter > 23*16) {
					live_delay(SEARCH_ADDRESS_MARK_DATA_FAILED);
					return;
				}

				if(cur_live.bit_counter >= 11*16 && (cur_live.shift_reg == 0xf56a || cur_live.shift_reg == 0xf56f)) {
					cur_live.crc = cur_live.shift_reg == 0xf56a ? 0x8fe7 : 0xbf84;
					cur_live.data_separator_phase = false;
					cur_live.bit_counter = 0;
					cur_live.state = READ_SECTOR_DATA;
				}
			}

			break;

		case READ_DATA_BLOCK_HEADER: {
			if(read_one_bit(limit))
				return;
#if 0
			fprintf(stderr, "%s: shift = %04x data=%02x counter=%d\n", tts(cur_live.tm).c_str(), cur_live.shift_reg,
					(cur_live.shift_reg & 0x4000 ? 0x80 : 0x00) |
					(cur_live.shift_reg & 0x1000 ? 0x40 : 0x00) |
					(cur_live.shift_reg & 0x0400 ? 0x20 : 0x00) |
					(cur_live.shift_reg & 0x0100 ? 0x10 : 0x00) |
					(cur_live.shift_reg & 0x0040 ? 0x08 : 0x00) |
					(cur_live.shift_reg & 0x0010 ? 0x04 : 0x00) |
					(cur_live.shift_reg & 0x0004 ? 0x02 : 0x00) |
					(cur_live.shift_reg & 0x0001 ? 0x01 : 0x00),
					cur_live.bit_counter);
#endif
			if(cur_live.bit_counter & 15)
				break;

			int slot = cur_live.bit_counter >> 4;

			if(slot < 3) {
				if(cur_live.shift_reg != 0x4489) {
					live_delay(SEARCH_ADDRESS_MARK_DATA_FAILED);
					return;
				}
				break;
			}
			if(cur_live.data_reg != 0xfb && cur_live.data_reg != 0xf8) {
				live_delay(SEARCH_ADDRESS_MARK_DATA_FAILED);
				return;
			}

			cur_live.bit_counter = 0;
			cur_live.state = READ_SECTOR_DATA;
			break;
		}

		case SEARCH_ADDRESS_MARK_DATA_FAILED:
			st1 |= ST1_MA;
			st2 |= ST2_MD;
			cur_live.state = IDLE;
			return;

		case READ_SECTOR_DATA: {
			if(read_one_bit(limit))
				return;
			if(cur_live.bit_counter & 15)
				break;
			int slot = (cur_live.bit_counter >> 4)-1;
			if(slot < sector_size) {
				// Sector data
				if(cur_live.fi->main_state == SCAN_DATA)
					live_delay(SCAN_SECTOR_DATA_BYTE);
				else
					live_delay(READ_SECTOR_DATA_BYTE);
				return;

			} else if(slot < sector_size+2) {
				// CRC
				if(slot == sector_size+1) {
					live_delay(IDLE);
					return;
				}
			}
			break;
		}

		case READ_SECTOR_DATA_BYTE:
			if(!tc_done)
				fifo_push(cur_live.data_reg, true);
			cur_live.state = READ_SECTOR_DATA;
			checkpoint();
			break;

		case SCAN_SECTOR_DATA_BYTE:
			if(!scan_done) // TODO: handle stp, x68000 sets it to 0xff (as it would dtl)?
			{
				int slot = (cur_live.bit_counter >> 4)-1;
				uint8_t data = fifo_pop(true);
				if(!slot)
					st2 = (st2 & ~(ST2_SN)) | ST2_SH;

				if(data != cur_live.data_reg)
				{
					st2 = (st2 & ~(ST2_SH)) | ST2_SN;
					if((data < cur_live.data_reg) && ((command[0] & 0x1f) == 0x19)) // low
						st2 &= ~ST2_SN;

					if((data > cur_live.data_reg) && ((command[0] & 0x1f) == 0x1d)) // high
						st2 &= ~ST2_SN;
				}
				if((slot == sector_size) && !(st2 & ST2_SN))
				{
					scan_done = true;
					tc_done = true;
				}
			}
			else
			{
				if(fifo_pos)
					fifo_pop(true);
			}
			cur_live.state = READ_SECTOR_DATA;
			checkpoint();
			break;

		case WRITE_SECTOR_SKIP_GAP2:
			cur_live.bit_counter = 0;
			cur_live.byte_counter = 0;
			cur_live.state = WRITE_SECTOR_SKIP_GAP2_BYTE;
			checkpoint();
			break;

		case WRITE_SECTOR_SKIP_GAP2_BYTE:
			if(read_one_bit(limit))
				return;
			if(mfm && cur_live.bit_counter != 22*16)
				break;
			if(!mfm && cur_live.bit_counter != 11*16)
				break;
			cur_live.bit_counter = 0;
			cur_live.byte_counter = 0;
			live_delay(WRITE_SECTOR_DATA);
			return;

		case WRITE_SECTOR_DATA:
			if(mfm) {
				if(cur_live.byte_counter < 12)
					live_write_mfm(0x00);
				else if(cur_live.byte_counter < 15)
					live_write_raw(0x4489);
				else if(cur_live.byte_counter < 16) {
					cur_live.crc = 0xcdb4;
					live_write_mfm(command[0] & 0x08 ? 0xf8 : 0xfb);
				} else if(cur_live.byte_counter < 16+sector_size)
					live_write_mfm(tc_done && !fifo_pos? 0x00 : fifo_pop(true));
				else if(cur_live.byte_counter < 16+sector_size+2)
					live_write_mfm(cur_live.crc >> 8);
				else if(cur_live.byte_counter < 16+sector_size+2+command[7])
					live_write_mfm(0x4e);
				else {
					cur_live.pll.stop_writing(cur_live.fi->dev, cur_live.tm);
					cur_live.state = IDLE;
					return;
				}

			} else {
				if(cur_live.byte_counter < 6)
					live_write_fm(0x00);
				else if(cur_live.byte_counter < 7) {
					cur_live.crc = 0xffff;
					live_write_raw(command[0] & 0x08 ? 0xf56a : 0xf56f);
				} else if(cur_live.byte_counter < 7+sector_size)
					live_write_fm(tc_done && !fifo_pos? 0x00 : fifo_pop(true));
				else if(cur_live.byte_counter < 7+sector_size+2)
					live_write_fm(cur_live.crc >> 8);
				else if(cur_live.byte_counter < 7+sector_size+2+command[7])
					live_write_fm(0xff);
				else {
					cur_live.pll.stop_writing(cur_live.fi->dev, cur_live.tm);
					cur_live.state = IDLE;
					return;
				}
			}
			cur_live.state = WRITE_SECTOR_DATA_BYTE;
			cur_live.bit_counter = 16;
			checkpoint();
			break;

		case WRITE_TRACK_PRE_SECTORS:
			if(!cur_live.byte_counter && command[3])
				fifo_expect(4, true);
			if(mfm) {
				if(cur_live.byte_counter < 80)
					live_write_mfm(0x4e);
				else if(cur_live.byte_counter < 92)
					live_write_mfm(0x00);
				else if(cur_live.byte_counter < 95)
					live_write_raw(0x5224);
				else if(cur_live.byte_counter < 96)
					live_write_mfm(0xfc);
				else if(cur_live.byte_counter < 146)
					live_write_mfm(0x4e);
				else {
					cur_live.state = WRITE_TRACK_SECTOR;
					cur_live.byte_counter = 0;
					break;
				}
			} else {
				if(cur_live.byte_counter < 40)
					live_write_fm(0xff);
				else if(cur_live.byte_counter < 46)
					live_write_fm(0x00);
				else if(cur_live.byte_counter < 47)
					live_write_raw(0xf77a);
				else if(cur_live.byte_counter < 73)
					live_write_fm(0xff);
				else {
					cur_live.state = WRITE_TRACK_SECTOR;
					cur_live.byte_counter = 0;
					break;
				}
			}
			cur_live.state = WRITE_TRACK_PRE_SECTORS_BYTE;
			cur_live.bit_counter = 16;
			checkpoint();
			break;

		case WRITE_TRACK_SECTOR:
			if(!cur_live.byte_counter) {
				command[3]--;
				if(command[3])
					fifo_expect(4, true);
			}
			if(mfm) {
				if(cur_live.byte_counter < 12)
					live_write_mfm(0x00);
				else if(cur_live.byte_counter < 15)
					live_write_raw(0x4489);
				else if(cur_live.byte_counter < 16) {
					cur_live.crc = 0xcdb4;
					live_write_mfm(0xfe);
				} else if(cur_live.byte_counter < 20) {
					uint8_t byte = fifo_pop(true);
					command[12+cur_live.byte_counter-16] = byte;
					live_write_mfm(byte);
					if(cur_live.byte_counter == 19)
						if (LOG) logerror("formatting sector %02x %02x %02x %02x\n",
									command[12], command[13], command[14], command[15]);
				} else if(cur_live.byte_counter < 22)
					live_write_mfm(cur_live.crc >> 8);
				else if(cur_live.byte_counter < 44)
					live_write_mfm(0x4e);
				else if(cur_live.byte_counter < 56)
					live_write_mfm(0x00);
				else if(cur_live.byte_counter < 59)
					live_write_raw(0x4489);
				else if(cur_live.byte_counter < 60) {
					cur_live.crc = 0xcdb4;
					live_write_mfm(0xfb);
				} else if(cur_live.byte_counter < 60+sector_size)
					live_write_mfm(command[5]);
				else if(cur_live.byte_counter < 62+sector_size)
					live_write_mfm(cur_live.crc >> 8);
				else if(cur_live.byte_counter < 62+sector_size+command[4])
					live_write_mfm(0x4e);
				else {
					cur_live.byte_counter = 0;
					cur_live.state = command[3] ? WRITE_TRACK_SECTOR : WRITE_TRACK_POST_SECTORS;
					break;
				}

			} else {
				if(cur_live.byte_counter < 6)
					live_write_fm(0x00);
				else if(cur_live.byte_counter < 7) {
					cur_live.crc = 0xffff;
					live_write_raw(0xf57e);
				} else if(cur_live.byte_counter < 11) {
					uint8_t byte = fifo_pop(true);
					command[12+cur_live.byte_counter-7] = byte;
					live_write_fm(byte);
					if(cur_live.byte_counter == 10)
						if (LOG) logerror("formatting sector %02x %02x %02x %02x\n",
									command[12], command[13], command[14], command[15]);
				} else if(cur_live.byte_counter < 13)
					live_write_fm(cur_live.crc >> 8);
				else if(cur_live.byte_counter < 24)
					live_write_fm(0xff);
				else if(cur_live.byte_counter < 30)
					live_write_fm(0x00);
				else if(cur_live.byte_counter < 31) {
					cur_live.crc = 0xffff;
					live_write_raw(0xf56f);
				} else if(cur_live.byte_counter < 31+sector_size)
					live_write_fm(command[5]);
				else if(cur_live.byte_counter < 33+sector_size)
					live_write_fm(cur_live.crc >> 8);
				else if(cur_live.byte_counter < 33+sector_size+command[4])
					live_write_fm(0xff);
				else {
					cur_live.byte_counter = 0;
					cur_live.state = command[3] ? WRITE_TRACK_SECTOR : WRITE_TRACK_POST_SECTORS;
					break;
				}
			}
			cur_live.state = WRITE_TRACK_SECTOR_BYTE;
			cur_live.bit_counter = 16;
			checkpoint();
			break;

		case WRITE_TRACK_POST_SECTORS:
			if(mfm)
				live_write_mfm(0x4e);
			else
				live_write_fm(0xff);
			cur_live.state = WRITE_TRACK_POST_SECTORS_BYTE;
			cur_live.bit_counter = 16;
			checkpoint();
			break;

		case WRITE_TRACK_PRE_SECTORS_BYTE:
		case WRITE_TRACK_SECTOR_BYTE:
		case WRITE_TRACK_POST_SECTORS_BYTE:
		case WRITE_SECTOR_DATA_BYTE:
			if(write_one_bit(limit))
				return;
			if(cur_live.bit_counter == 0) {
				cur_live.byte_counter++;
				live_delay(cur_live.state-1);
				return;
			}
			break;

		default:
			if (LOG) logerror("%s: Unknown live state %d\n", tts(cur_live.tm).c_str(), cur_live.state);
			return;
		}
	}
}

int upd765_family_device::check_command()
{
	// 0.000010 read track
	// 00000011 specify
	// 00000100 sense drive status
	// ..000101 write data
	// ...00110 read data
	// 00000111 recalibrate
	// 00001000 sense interrupt status
	// ..001001 write deleted data
	// 0.001010 read id
	// ...01100 read deleted data
	// 0.001101 format track
	// 00001110 dumpreg
	// 00101110 save
	// 01001110 restore
	// 10001110 drive specification command
	// 00001111 seek
	// 1.001111 relative seek
	// 00010000 version
	// ...10001 scan equal
	// 00010010 perpendicular mode
	// 00010011 configure
	// 00110011 option
	// .0010100 lock
	// ...10110 verify
	// 00010111 powerdown mode
	// 00011000 part id
	// ...11001 scan low or equal
	// ...11101 scan high or equal

	// MSDOS 6.22 format uses 0xcd to format a track, which makes one
	// think only the bottom 5 bits are decoded.

	switch(command[0] & 0x1f) {
	case 0x02:
		return command_pos == 9 ? C_READ_TRACK         : C_INCOMPLETE;

	case 0x03:
		return command_pos == 3 ? C_SPECIFY            : C_INCOMPLETE;

	case 0x04:
		return command_pos == 2 ? C_SENSE_DRIVE_STATUS : C_INCOMPLETE;

	case 0x05:
	case 0x09:
		return command_pos == 9 ? C_WRITE_DATA         : C_INCOMPLETE;

	case 0x06:
	case 0x0c:
		return command_pos == 9 ? C_READ_DATA          : C_INCOMPLETE;

	case 0x07:
		return command_pos == 2 ? C_RECALIBRATE        : C_INCOMPLETE;

	case 0x08:
		return C_SENSE_INTERRUPT_STATUS;

	case 0x0a:
		return command_pos == 2 ? C_READ_ID            : C_INCOMPLETE;

	case 0x0d:
		return command_pos == 6 ? C_FORMAT_TRACK       : C_INCOMPLETE;

	case 0x0e:
		return C_DUMP_REG;

	case 0x0f:
		return command_pos == 3 ? C_SEEK               : C_INCOMPLETE;

	case 0x11:
		return command_pos == 9 ? C_SCAN_EQUAL         : C_INCOMPLETE;

	case 0x12:
		return command_pos == 2 ? C_PERPENDICULAR      : C_INCOMPLETE;

	case 0x13:
		return command_pos == 4 ? C_CONFIGURE          : C_INCOMPLETE;

	case 0x14:
		return C_LOCK;

	case 0x19:
		return command_pos == 9 ? C_SCAN_LOW           : C_INCOMPLETE;

	case 0x1d:
		return command_pos == 9 ? C_SCAN_HIGH          : C_INCOMPLETE;

	default:
		return C_INVALID;
	}
}

void upd765_family_device::start_command(int cmd)
{
	command_pos = 0;
	result_pos = 0;
	main_phase = PHASE_EXEC;
	tc_done = false;
	switch(cmd) {
	case C_CONFIGURE:
		if (LOG) logerror("command configure %02x %02x %02x\n",
					command[1], command[2], command[3]);
		// byte 1 is ignored, byte 3 is precompensation-related
		fifocfg = command[2];
		precomp = command[3];
		main_phase = PHASE_CMD;
		break;

	case C_DUMP_REG:
		if (LOG) logerror("command dump regs\n");
		main_phase = PHASE_RESULT;
		result[0] = flopi[0].pcn;
		result[1] = flopi[1].pcn;
		result[2] = flopi[2].pcn;
		result[3] = flopi[3].pcn;
		result[4] = (spec & 0xff00) >> 8;
		result[5] = (spec & 0x00ff);
		result[6] = sector_size;
		result[7] = locked ? 0x80 : 0x00;
		result[7] |= (perpmode & 0x30);
		result[8] = fifocfg;
		result[9] = precomp;
		result_pos = 10;
		break;

	case C_FORMAT_TRACK:
		format_track_start(flopi[command[1] & 3]);
		break;

	case C_LOCK:
		locked = command[0] & 0x80;
		main_phase = PHASE_RESULT;
		result[0] = locked ? 0x10 : 0x00;
		result_pos = 1;
		if (LOG) logerror("command lock (%s)\n", locked ? "on" : "off");
		break;

	case C_PERPENDICULAR:
		if (LOG) logerror("command perpendicular\n");
		perpmode = command[1];
		main_phase = PHASE_CMD;
		break;

	case C_READ_DATA:
		read_data_start(flopi[command[1] & 3]);
		break;

	case C_READ_ID:
		read_id_start(flopi[command[1] & 3]);
		break;

	case C_READ_TRACK:
		read_track_start(flopi[command[1] & 3]);
		break;

	case C_SCAN_EQUAL:
	case C_SCAN_LOW:
	case C_SCAN_HIGH:
		scan_start(flopi[command[1] & 3]);
		break;

	case C_RECALIBRATE:
		recalibrate_start(flopi[command[1] & 3]);
		main_phase = PHASE_CMD;
		break;

	case C_SEEK:
		seek_start(flopi[command[1] & 3]);
		main_phase = PHASE_CMD;
		break;

	case C_SENSE_DRIVE_STATUS: {
		floppy_info &fi = flopi[command[1] & 3];
		main_phase = PHASE_RESULT;
		result[0] = command[1] & 7;
		if(fi.ready)
			result[0] |= ST3_RY;
		if(fi.dev)
			result[0] |=
				(fi.dev->wpt_r() ? ST3_WP : 0x00) |
				(fi.dev->trk00_r() ? 0x00 : ST3_T0) |
				(fi.dev->twosid_r() ? 0x00 : ST3_TS);
		if (LOG) logerror("command sense drive status %d (%02x)\n", fi.id, result[0]);
		result_pos = 1;
		break;
	}

	case C_SENSE_INTERRUPT_STATUS: {
		// Documentation is somewhat contradictory w.r.t polling
		// and irq.  PC bios, especially 5150, requires that only
		// one irq happens.  That's also what the ns82077a doc
		// says it does.  OTOH, a number of docs says you need to
		// call SIS 4 times, once per drive...
		//
		// There's also the interaction with the seek irq.  The
		// somewhat borderline tf20 code seems to think that
		// essentially ignoring the polling irq should work.
		//
		// And the pc98 expects to be able to accumulate irq reasons
		// for different drives and things to work.
		//
		// Current hypothesis:
		// - each drive has its own st0 and irq trigger
		// - SIS drops the irq always, but also returns the first full st0 it finds

		main_phase = PHASE_RESULT;

		int fid;
		for(fid=0; fid<4 && !flopi[fid].st0_filled; fid++) {};
		if(fid == 4) {
			result[0] = ST0_UNK;
			result_pos = 1;
			if (LOG) logerror("command sense interrupt status (%02x)\n", result[0]);
			break;
		}

		floppy_info &fi = flopi[fid];
		fi.st0_filled = false;

		result[0] = fi.st0;
		result[1] = fi.pcn;

		if (LOG) logerror("command sense interrupt status (fid=%d %02x %02x)\n", fid, result[0], result[1]);
		result_pos = 2;

		other_irq = false;
		check_irq();
		break;
	}

	case C_SPECIFY:
		if (LOG) logerror("command specify %02x %02x\n",
					command[1], command[2]);
		spec = (command[1] << 8) | command[2];
		main_phase = PHASE_CMD;
		break;

	case C_WRITE_DATA:
		write_data_start(flopi[command[1] & 3]);
		break;

	default:
		fprintf(stderr, "start command %d\n", cmd);
		exit(1);
	}
}

void upd765_family_device::command_end(floppy_info &fi, bool data_completion)
{
	if (LOG) logerror("command done (%s) -", data_completion ? "data" : "seek");
	for(int i=0; i != result_pos; i++)
		if (LOG) logerror(" %02x", result[i]);
	if (LOG) logerror("\n");
	fi.main_state = fi.sub_state = IDLE;
	if(data_completion)
		data_irq = true;
	else
	{
		other_irq = true;
		fi.st0_filled = true;
	}
	check_irq();
}

void upd765_family_device::recalibrate_start(floppy_info &fi)
{
	if (LOG) logerror("command recalibrate\n");
	fi.main_state = RECALIBRATE;
	fi.sub_state = SEEK_WAIT_STEP_TIME_DONE;
	fi.dir = 1;
	fi.counter = 77;
	fi.ready = get_ready(command[1] & 3);
	fi.st0 = (fi.ready ? 0 : ST0_NR);
	seek_continue(fi);
}

void upd765_family_device::seek_start(floppy_info &fi)
{
	if (LOG) logerror("command %sseek %d\n", command[0] & 0x80 ? "relative " : "", command[2]);
	fi.main_state = SEEK;
	fi.sub_state = SEEK_WAIT_STEP_TIME_DONE;
	fi.dir = fi.pcn > command[2] ? 1 : 0;
	fi.ready = get_ready(command[1] & 3);
	fi.st0 = (fi.ready ? 0 : ST0_NR);
	seek_continue(fi);
}

void upd765_family_device::delay_cycles(emu_timer *tm, int cycles)
{
	tm->adjust(attotime::from_double(double(cycles)/cur_rate));
}

void upd765_family_device::seek_continue(floppy_info &fi)
{
	for(;;) {
		switch(fi.sub_state) {
		case SEEK_MOVE:
			if(fi.dev) {
				fi.dev->dir_w(fi.dir);
				fi.dev->stp_w(0);
			}
			fi.sub_state = SEEK_WAIT_STEP_SIGNAL_TIME;
			fi.tm->adjust(attotime::from_nsec(2500));
			return;

		case SEEK_WAIT_STEP_SIGNAL_TIME:
			return;

		case SEEK_WAIT_STEP_SIGNAL_TIME_DONE:
			if(fi.dev)
				fi.dev->stp_w(1);

			if(fi.main_state == SEEK) {
				if(fi.pcn > command[2])
					fi.pcn--;
				else
					fi.pcn++;
			}
			fi.sub_state = SEEK_WAIT_STEP_TIME;
			delay_cycles(fi.tm, 500*(16-(spec >> 12)));
			return;

		case SEEK_WAIT_STEP_TIME:
			return;

		case SEEK_WAIT_STEP_TIME_DONE: {
			bool done = false;
			switch(fi.main_state) {
			case RECALIBRATE:
				fi.counter--;
				done = fi.dev && !fi.dev->trk00_r();
				if(done)
					fi.pcn = 0;
				else if(!fi.counter) {
					fi.st0 |= ST0_FAIL|ST0_SE|ST0_EC | fi.id;
					command_end(fi, false);
					return;
				}
				break;
			case SEEK:
				done = fi.pcn == command[2];
				break;
			}
			if(done) {
				fi.st0 |= ST0_SE | fi.id;
				command_end(fi, false);
				return;
			}
			fi.sub_state = SEEK_MOVE;
			break;
		}
		}
	}
}

void upd765_family_device::read_data_start(floppy_info &fi)
{
	fi.main_state = READ_DATA;
	fi.sub_state = HEAD_LOAD_DONE;
	mfm = command[0] & 0x40;

	if (LOG) logerror("command read%s data%s%s%s%s cmd=%02x sel=%x chrn=(%d, %d, %d, %d) eot=%02x gpl=%02x dtl=%02x rate=%d\n",
				command[0] & 0x08 ? " deleted" : "",
				command[0] & 0x80 ? " mt" : "",
				command[0] & 0x40 ? " mfm" : "",
				command[0] & 0x20 ? " sk" : "",
				fifocfg & 0x40 ? " seek" : "",
				command[0],
				command[1],
				command[2],
				command[3],
				command[4],
				128 << (command[5] & 7),
				command[6],
				command[7],
				command[8],
				cur_rate);

	fi.st0 = command[1] & 7;
	st1 = ST1_MA;
	st2 = 0x00;
	hdl_cb(1);
	fi.ready = get_ready(command[1] & 3);

	if(!fi.ready)
	{
		fi.st0 |= ST0_NR | ST0_FAIL;
		fi.sub_state = COMMAND_DONE;
		st1 = 0;
		st2 = 0;
		read_data_continue(fi);
		return;
	}

	if(fi.dev)
		fi.dev->ss_w(command[1] & 4 ? 1 : 0);
	read_data_continue(fi);
}

void upd765_family_device::scan_start(floppy_info &fi)
{
	fi.main_state = SCAN_DATA;
	fi.sub_state = HEAD_LOAD_DONE;
	mfm = command[0] & 0x40;

	if (LOG) logerror("command scan%s data%s%s%s%s cmd=%02x sel=%x chrn=(%d, %d, %d, %d) eot=%02x gpl=%02x stp=%02x rate=%d\n",
				command[0] & 0x08 ? " deleted" : "",
				command[0] & 0x80 ? " mt" : "",
				command[0] & 0x40 ? " mfm" : "",
				command[0] & 0x20 ? " sk" : "",
				fifocfg & 0x40 ? " seek" : "",
				command[0],
				command[1],
				command[2],
				command[3],
				command[4],
				128 << (command[5] & 7),
				command[6],
				command[7],
				command[8],
				cur_rate);

	fi.st0 = command[1] & 7;
	st1 = ST1_MA;
	st2 = 0x00;
	scan_done = false;
	hdl_cb(1);
	fi.ready = get_ready(command[1] & 3);

	if(!fi.ready)
	{
		fi.st0 |= ST0_NR | ST0_FAIL;
		fi.sub_state = COMMAND_DONE;
		st1 = 0;
		st2 = 0;
		read_data_continue(fi);
		return;
	}

	if(fi.dev)
		fi.dev->ss_w(command[1] & 4 ? 1 : 0);
	read_data_continue(fi);
}

void upd765_family_device::read_data_continue(floppy_info &fi)
{
	for(;;) {
		switch(fi.sub_state) {
		case HEAD_LOAD_DONE:
			if(fi.pcn == command[2] || !(fifocfg & 0x40)) {
				fi.sub_state = SEEK_DONE;
				break;
			}
			fi.st0 |= ST0_SE;
			if(fi.dev) {
				fi.dev->dir_w(fi.pcn > command[2] ? 1 : 0);
				fi.dev->stp_w(0);
			}
			fi.sub_state = SEEK_WAIT_STEP_SIGNAL_TIME;
			fi.tm->adjust(attotime::from_nsec(2500));
			return;

		case SEEK_WAIT_STEP_SIGNAL_TIME:
			return;

		case SEEK_WAIT_STEP_SIGNAL_TIME_DONE:
			if(fi.dev)
				fi.dev->stp_w(1);

			fi.sub_state = SEEK_WAIT_STEP_TIME;
			delay_cycles(fi.tm, 500*(16-(spec >> 12)));
			return;

		case SEEK_WAIT_STEP_TIME:
			return;

		case SEEK_WAIT_STEP_TIME_DONE:
			if(fi.pcn > command[2])
				fi.pcn--;
			else
				fi.pcn++;
			fi.sub_state = HEAD_LOAD_DONE;
			break;

		case SEEK_DONE:
			fi.counter = 0;
			fi.sub_state = SCAN_ID;
			live_start(fi, SEARCH_ADDRESS_MARK_HEADER);
			return;

		case SCAN_ID:
			if(cur_live.crc) {
				fi.st0 |= ST0_FAIL;
				st1 |= ST1_DE|ST1_ND;
				fi.sub_state = COMMAND_DONE;
				break;
			}
			st1 &= ~ST1_MA;
			if(!sector_matches()) {
				if(cur_live.idbuf[0] != command[2]) {
					if(cur_live.idbuf[0] == 0xff)
						st2 |= ST2_WC|ST2_BC;
					else
						st2 |= ST2_WC;
				}
				live_start(fi, SEARCH_ADDRESS_MARK_HEADER);
				return;
			}
			if (LOG) logerror("reading sector %02x %02x %02x %02x\n",
						cur_live.idbuf[0],
						cur_live.idbuf[1],
						cur_live.idbuf[2],
						cur_live.idbuf[3]);
			sector_size = calc_sector_size(cur_live.idbuf[3]);
			if(fi.main_state == SCAN_DATA)
				fifo_expect(sector_size, true);
			else
				fifo_expect(sector_size, false);
			fi.sub_state = SECTOR_READ;
			live_start(fi, SEARCH_ADDRESS_MARK_DATA);
			return;

		case SCAN_ID_FAILED:
			fi.st0 |= ST0_FAIL;
			st1 |= ST1_ND;
			fi.sub_state = COMMAND_DONE;
			break;

		case SECTOR_READ: {
			if(st2 & ST2_MD) {
				fi.st0 |= ST0_FAIL;
				fi.sub_state = COMMAND_DONE;
				break;
			}
			if(cur_live.crc) {
				fi.st0 |= ST0_FAIL;
				st1 |= ST1_DE;
				st2 |= ST2_CM;
				fi.sub_state = COMMAND_DONE;
				break;
			}
			bool done = tc_done;
			if(command[4] == command[6]) {
				if(command[0] & 0x80) {
					command[3] = command[3] ^ 1;
					command[4] = 1;
					if(fi.dev)
						fi.dev->ss_w(command[3] & 1);
				}
				if(!(command[0] & 0x80) || !(command[3] & 1)) {
					if(!tc_done) {
						fi.st0 |= ST0_FAIL;
						st1 |= ST1_EN;
					} else {
						command[2]++;
						command[4] = 1;
					}
					done = true;
				}
			} else
				command[4]++;
			if(!done) {
				fi.sub_state = SEEK_DONE;
				break;
			}
			fi.sub_state = COMMAND_DONE;
			break;
		}

		case COMMAND_DONE:
			main_phase = PHASE_RESULT;
			result[0] = fi.st0;
			result[1] = st1;
			result[2] = st2;
			result[3] = command[2];
			result[4] = command[3];
			result[5] = command[4];
			result[6] = command[5];
			result_pos = 7;
			command_end(fi, true);
			return;

		default:
			if (LOG) logerror("%s: read sector unknown sub-state %d\n", ttsn().c_str(), fi.sub_state);
			return;
		}
	}
}

void upd765_family_device::write_data_start(floppy_info &fi)
{
	fi.main_state = WRITE_DATA;
	fi.sub_state = HEAD_LOAD_DONE;
	mfm = command[0] & 0x40;
	if (LOG) logerror("command write%s data%s%s cmd=%02x sel=%x chrn=(%d, %d, %d, %d) eot=%02x gpl=%02x dtl=%02x rate=%d\n",
				command[0] & 0x08 ? " deleted" : "",
				command[0] & 0x80 ? " mt" : "",
				command[0] & 0x40 ? " mfm" : "",
				command[0],
				command[1],
				command[2],
				command[3],
				command[4],
				128 << (command[5] & 7),
				command[6],
				command[7],
				command[8],
				cur_rate);

	if(fi.dev)
		fi.dev->ss_w(command[1] & 4 ? 1 : 0);

	fi.st0 = command[1] & 7;
	st1 = ST1_MA;
	st2 = 0x00;
	hdl_cb(1);
	fi.ready = get_ready(command[1] & 3);

	if(!fi.ready)
	{
		fi.st0 |= ST0_NR | ST0_FAIL;
		fi.sub_state = COMMAND_DONE;
		st1 = 0;
		st2 = 0;
		write_data_continue(fi);
		return;
	}

	write_data_continue(fi);
}

void upd765_family_device::write_data_continue(floppy_info &fi)
{
	for(;;) {
		switch(fi.sub_state) {
		case HEAD_LOAD_DONE:
			fi.counter = 0;
			fi.sub_state = SCAN_ID;
			live_start(fi, SEARCH_ADDRESS_MARK_HEADER);
			return;

		case SCAN_ID:
			if(!sector_matches()) {
				live_start(fi, SEARCH_ADDRESS_MARK_HEADER);
				return;
			}
			if(cur_live.crc) {
				fi.st0 |= ST0_FAIL;
				st1 |= ST1_DE|ST1_ND;
				fi.sub_state = COMMAND_DONE;
				break;
			}
			st1 &= ~ST1_MA;
			sector_size = calc_sector_size(cur_live.idbuf[3]);
			fifo_expect(sector_size, true);
			fi.sub_state = SECTOR_WRITTEN;
			live_start(fi, WRITE_SECTOR_SKIP_GAP2);
			return;

		case SCAN_ID_FAILED:
			fi.st0 |= ST0_FAIL;
			st1 |= ST1_ND;
			fi.sub_state = COMMAND_DONE;
			break;

		case SECTOR_WRITTEN: {
			bool done = tc_done;
			if(command[4] == command[6]) {
				if(command[0] & 0x80) {
					command[3] = command[3] ^ 1;
					command[4] = 1;
					if(fi.dev)
						fi.dev->ss_w(command[3] & 1);
				}
				if(!(command[0] & 0x80) || !(command[3] & 1)) {
					if(!tc_done) {
						fi.st0 |= ST0_FAIL;
						st1 |= ST1_EN;
					} else {
						command[2]++;
						command[4] = 1;
					}
					done = true;
				}
			} else
				command[4]++;
			if(!done) {
				fi.sub_state = HEAD_LOAD_DONE;
				break;
			}
			fi.sub_state = COMMAND_DONE;
			break;
		}

		case COMMAND_DONE:
			main_phase = PHASE_RESULT;
			result[0] = fi.st0;
			result[1] = st1;
			result[2] = st2;
			result[3] = command[2];
			result[4] = command[3];
			result[5] = command[4];
			result[6] = command[5];
			result_pos = 7;
			command_end(fi, true);
			return;

		default:
			if (LOG) logerror("%s: write sector unknown sub-state %d\n", ttsn().c_str(), fi.sub_state);
			return;
		}
	}
}

void upd765_family_device::read_track_start(floppy_info &fi)
{
	fi.main_state = READ_TRACK;
	fi.sub_state = HEAD_LOAD_DONE;
	mfm = command[0] & 0x40;
	sectors_read = 0;

	if (LOG) logerror("command read track%s cmd=%02x sel=%x chrn=(%d, %d, %d, %d) eot=%02x gpl=%02x dtl=%02x rate=%d\n",
				command[0] & 0x40 ? " mfm" : "",
				command[0],
				command[1],
				command[2],
				command[3],
				command[4],
				128 << (command[5] & 7),
				command[6],
				command[7],
				command[8],
				cur_rate);
	fi.st0 = command[1] & 7;
	st1 = ST1_MA;
	st2 = 0x00;
	hdl_cb(1);
	fi.ready = get_ready(command[1] & 3);

	if(!fi.ready)
	{
		fi.st0 |= ST0_NR | ST0_FAIL;
		fi.sub_state = COMMAND_DONE;
		st1 = 0;
		st2 = 0;
		read_track_continue(fi);
		return;
	}

	if(fi.dev)
		fi.dev->ss_w(command[1] & 4 ? 1 : 0);
	read_track_continue(fi);
}

void upd765_family_device::read_track_continue(floppy_info &fi)
{
	for(;;) {
		switch(fi.sub_state) {
		case HEAD_LOAD_DONE:
			if(fi.pcn == command[2] || !(fifocfg & 0x40)) {
				fi.sub_state = SEEK_DONE;
				break;
			}
			fi.st0 |= ST0_SE;
			if(fi.dev) {
				fi.dev->dir_w(fi.pcn > command[2] ? 1 : 0);
				fi.dev->stp_w(0);
			}
			fi.sub_state = SEEK_WAIT_STEP_SIGNAL_TIME;
			fi.tm->adjust(attotime::from_nsec(2500));
			return;

		case SEEK_WAIT_STEP_SIGNAL_TIME:
			return;

		case SEEK_WAIT_STEP_SIGNAL_TIME_DONE:
			if(fi.dev)
				fi.dev->stp_w(1);

			fi.sub_state = SEEK_WAIT_STEP_TIME;
			delay_cycles(fi.tm, 500*(16-(spec >> 12)));
			return;

		case SEEK_WAIT_STEP_TIME:
			return;

		case SEEK_WAIT_STEP_TIME_DONE:
			if(fi.pcn > command[2])
				fi.pcn--;
			else
				fi.pcn++;
			fi.sub_state = HEAD_LOAD_DONE;
			break;

		case SEEK_DONE:
			fi.counter = 0;
			fi.sub_state = WAIT_INDEX;
			return;

		case WAIT_INDEX:
			return;

		case WAIT_INDEX_DONE:
			if (LOG) logerror("index found, reading track\n");
			fi.sub_state = SCAN_ID;
			live_start(fi, SEARCH_ADDRESS_MARK_HEADER);
			return;

		case SCAN_ID:
			if(cur_live.crc) {
				st1 |= ST1_DE;
			}
			st1 &= ~ST1_MA;
			if (LOG) logerror("reading sector %02x %02x %02x %02x\n",
						cur_live.idbuf[0],
						cur_live.idbuf[1],
						cur_live.idbuf[2],
						cur_live.idbuf[3]);
			if(!sector_matches())
				st1 |= ST1_ND;
			else
				st1 &= ~ST1_ND;

			sector_size = calc_sector_size(cur_live.idbuf[3]);
			fifo_expect(sector_size, false);
			fi.sub_state = SECTOR_READ;
			live_start(fi, SEARCH_ADDRESS_MARK_DATA);
			return;

		case SCAN_ID_FAILED:
			fi.st0 |= ST0_FAIL;
			st1 |= ST1_ND;
			fi.sub_state = COMMAND_DONE;
			break;

		case SECTOR_READ: {
			if(st2 & ST2_MD) {
				fi.st0 |= ST0_FAIL;
				fi.sub_state = COMMAND_DONE;
				break;
			}
			if(cur_live.crc) {
				st1 |= ST1_DE;
				st2 |= ST2_CM;
			}
			bool done = tc_done;
			sectors_read++;
			if(sectors_read == command[6]) {
				if(!tc_done) {
					fi.st0 |= ST0_FAIL;
					st1 |= ST1_EN;
				}
				done = true;
			}
			if(!done) {
				fi.sub_state = WAIT_INDEX_DONE;
				break;
			}
			fi.sub_state = COMMAND_DONE;
			break;
		}

		case COMMAND_DONE:
			main_phase = PHASE_RESULT;
			result[0] = fi.st0;
			result[1] = st1;
			result[2] = st2;
			result[3] = command[2];
			result[4] = command[3];
			result[5] = command[4];
			result[6] = command[5];
			result_pos = 7;
			command_end(fi, true);
			return;

		default:
			if (LOG) logerror("%s: read track unknown sub-state %d\n", ttsn().c_str(), fi.sub_state);
			return;
		}
	}
}

int upd765_family_device::calc_sector_size(uint8_t size)
{
	return size > 7 ? 16384 : 128 << size;
}

void upd765_family_device::format_track_start(floppy_info &fi)
{
	fi.main_state = FORMAT_TRACK;
	fi.sub_state = HEAD_LOAD_DONE;
	mfm = command[0] & 0x40;

	if (LOG) logerror("command format track %s h=%02x n=%02x sc=%02x gpl=%02x d=%02x\n",
				command[0] & 0x40 ? "mfm" : "fm",
				command[1], command[2], command[3], command[4], command[5]);

	hdl_cb(1);
	fi.ready = get_ready(command[1] & 3);

	if(!fi.ready)
	{
		fi.st0 = (command[1] & 7) | ST0_NR | ST0_FAIL;
		fi.sub_state = TRACK_DONE;
		format_track_continue(fi);
		return;
	}
	fi.st0 = command[1] & 7;

	if(fi.dev)
		fi.dev->ss_w(command[1] & 4 ? 1 : 0);
	sector_size = calc_sector_size(command[2]);

	format_track_continue(fi);
}

void upd765_family_device::format_track_continue(floppy_info &fi)
{
	for(;;) {
		switch(fi.sub_state) {
		case HEAD_LOAD_DONE:
			fi.sub_state = WAIT_INDEX;
			break;

		case WAIT_INDEX:
			return;

		case WAIT_INDEX_DONE:
			if (LOG) logerror("index found, writing track\n");
			fi.sub_state = TRACK_DONE;
			cur_live.pll.start_writing(machine().time());
			live_start(fi, WRITE_TRACK_PRE_SECTORS);
			return;

		case TRACK_DONE:
			main_phase = PHASE_RESULT;
			result[0] = fi.st0;
			result[1] = 0;
			result[2] = 0;
			result[3] = 0;
			result[4] = 0;
			result[5] = 0;
			result[6] = command[2];
			result_pos = 7;
			command_end(fi, true);
			return;

		default:
			if (LOG) logerror("%s: format track unknown sub-state %d\n", ttsn().c_str(), fi.sub_state);
			return;
		}
	}
}

void upd765_family_device::read_id_start(floppy_info &fi)
{
	fi.main_state = READ_ID;
	fi.sub_state = HEAD_LOAD_DONE;
	mfm = command[0] & 0x40;

	if (LOG) logerror("command read id%s, rate=%d\n",
				command[0] & 0x40 ? " mfm" : "",
				cur_rate);

	if(fi.dev)
		fi.dev->ss_w(command[1] & 4 ? 1 : 0);

	fi.st0 = command[1] & 7;
	st1 = 0x00;
	st2 = 0x00;

	for(int i=0; i<4; i++)
		cur_live.idbuf[i] = 0x00;

	hdl_cb(1);
	fi.ready = get_ready(command[1] & 3);

	if(!fi.ready)
	{
		fi.st0 |= ST0_NR | ST0_FAIL;
		fi.sub_state = COMMAND_DONE;
		read_id_continue(fi);
		return;
	}

	read_id_continue(fi);
}

void upd765_family_device::read_id_continue(floppy_info &fi)
{
	for(;;) {
		switch(fi.sub_state) {
		case HEAD_LOAD_DONE:
			fi.counter = 0;
			fi.sub_state = SCAN_ID;
			live_start(fi, SEARCH_ADDRESS_MARK_HEADER);
			return;

		case SCAN_ID:
			if(cur_live.crc) {
				fi.st0 |= ST0_FAIL;
				st1 |= ST1_MA|ST1_DE|ST1_ND;
			}
			fi.sub_state = COMMAND_DONE;
			break;

		case SCAN_ID_FAILED:
			fi.st0 |= ST0_FAIL;
			st1 |= ST1_ND|ST1_MA;
			fi.sub_state = COMMAND_DONE;
			break;

		case COMMAND_DONE:
			main_phase = PHASE_RESULT;
			result[0] = fi.st0;
			result[1] = st1;
			result[2] = st2;
			result[3] = cur_live.idbuf[0];
			result[4] = cur_live.idbuf[1];
			result[5] = cur_live.idbuf[2];
			result[6] = cur_live.idbuf[3];
			result_pos = 7;
			command_end(fi, true);
			return;

		default:
			if (LOG) logerror("%s: read id unknown sub-state %d\n", ttsn().c_str(), fi.sub_state);
			return;
		}
	}
}

void upd765_family_device::check_irq()
{
	bool old_irq = cur_irq;
	cur_irq = data_irq || other_irq || internal_drq;
	cur_irq = cur_irq && (dor & 4) && (mode != MODE_AT || (dor & 8));
	if(cur_irq != old_irq) {
		if (LOG) logerror("irq = %d\n", cur_irq);
		intrq_cb(cur_irq);
	}
}

bool upd765_family_device::get_irq() const
{
	return cur_irq;
}

std::string upd765_family_device::tts(attotime t)
{
	char buf[256];
	const char *sign = "";
	if(t.seconds() < 0) {
		t = attotime::zero-t;
		sign = "-";
	}
	int nsec = t.attoseconds() / ATTOSECONDS_PER_NANOSECOND;
	sprintf(buf, "%s%04d.%03d,%03d,%03d", sign, int(t.seconds()), nsec/1000000, (nsec/1000)%1000, nsec % 1000);
	return buf;
}

std::string upd765_family_device::ttsn()
{
	return tts(machine().time());
}

void upd765_family_device::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	if(id == TIMER_DRIVE_READY_POLLING) {
		run_drive_ready_polling();
		return;
	}

	live_sync();

	floppy_info &fi = flopi[id];
	switch(fi.sub_state) {
	case SEEK_WAIT_STEP_SIGNAL_TIME:
		fi.sub_state = SEEK_WAIT_STEP_SIGNAL_TIME_DONE;
		break;
	case SEEK_WAIT_STEP_TIME:
		fi.sub_state = SEEK_WAIT_STEP_TIME_DONE;
		break;
	}

	general_continue(fi);
}

void upd765_family_device::run_drive_ready_polling()
{
	if(main_phase != PHASE_CMD || (fifocfg & FIF_POLL) || command_pos)
		return;

	for(int fid=0; fid<4; fid++) {
		bool ready = get_ready(fid);
		if(ready != flopi[fid].ready) {
			if (LOG) logerror("polled %d : %d -> %d\n", fid, flopi[fid].ready, ready);
			flopi[fid].ready = ready;
			if(!flopi[fid].st0_filled) {
				flopi[fid].st0 = ST0_ABRT | fid;
				flopi[fid].st0_filled = true;
				other_irq = true;
			}
		}
	}

	check_irq();
}

void upd765_family_device::index_callback(floppy_image_device *floppy, int state)
{
	for(auto & fi : flopi) {
		if(fi.dev != floppy)
			continue;

		if(fi.live)
			live_sync();
		fi.index = state;

		if(!state) {
			general_continue(fi);
			continue;
		}

		switch(fi.sub_state) {
		case IDLE:
		case SEEK_MOVE:
		case SEEK_WAIT_STEP_SIGNAL_TIME:
		case SEEK_WAIT_STEP_SIGNAL_TIME_DONE:
		case SEEK_WAIT_STEP_TIME:
		case SEEK_WAIT_STEP_TIME_DONE:
		case HEAD_LOAD_DONE:
		case SCAN_ID_FAILED:
		case SECTOR_READ:
			break;

		case WAIT_INDEX:
			fi.sub_state = WAIT_INDEX_DONE;
			break;

		case SCAN_ID:
			fi.counter++;
			if(fi.counter == 2) {
				fi.sub_state = SCAN_ID_FAILED;
				live_abort();
			}
			break;

		case TRACK_DONE:
			live_abort();
			break;

		default:
			if (LOG) logerror("%s: Index pulse on unknown sub-state %d\n", ttsn().c_str(), fi.sub_state);
			break;
		}

		general_continue(fi);
	}
}


void upd765_family_device::general_continue(floppy_info &fi)
{
	if(fi.live && cur_live.state != IDLE) {
		live_run();
		if(cur_live.state != IDLE)
			return;
	}

	switch(fi.main_state) {
	case IDLE:
		break;

	case RECALIBRATE:
	case SEEK:
		seek_continue(fi);
		break;

	case READ_DATA:
	case SCAN_DATA:
		read_data_continue(fi);
		break;

	case WRITE_DATA:
		write_data_continue(fi);
		break;

	case READ_TRACK:
		read_track_continue(fi);
		break;

	case FORMAT_TRACK:
		format_track_continue(fi);
		break;

	case READ_ID:
		read_id_continue(fi);
		break;

	default:
		if (LOG) logerror("%s: general_continue on unknown main-state %d\n", ttsn().c_str(), fi.main_state);
		break;
	}
}

bool upd765_family_device::read_one_bit(const attotime &limit)
{
	int bit = cur_live.pll.get_next_bit(cur_live.tm, cur_live.fi->dev, limit);
	if(bit < 0)
		return true;
	cur_live.shift_reg = (cur_live.shift_reg << 1) | bit;
	cur_live.bit_counter++;
	if(cur_live.data_separator_phase) {
		cur_live.data_reg = (cur_live.data_reg << 1) | bit;
		if((cur_live.crc ^ (bit ? 0x8000 : 0x0000)) & 0x8000)
			cur_live.crc = (cur_live.crc << 1) ^ 0x1021;
		else
			cur_live.crc = cur_live.crc << 1;
	}
	cur_live.data_separator_phase = !cur_live.data_separator_phase;
	return false;
}

bool upd765_family_device::write_one_bit(const attotime &limit)
{
	bool bit = cur_live.shift_reg & 0x8000;
	if(cur_live.pll.write_next_bit(bit, cur_live.tm, cur_live.fi->dev, limit))
		return true;
	if(cur_live.bit_counter & 1) {
		if((cur_live.crc ^ (bit ? 0x8000 : 0x0000)) & 0x8000)
			cur_live.crc = (cur_live.crc << 1) ^ 0x1021;
		else
			cur_live.crc = cur_live.crc << 1;
	}
	cur_live.shift_reg = cur_live.shift_reg << 1;
	cur_live.bit_counter--;
	return false;
}

void upd765_family_device::live_write_raw(uint16_t raw)
{
	//  if (LOG) logerror("write %04x %04x\n", raw, cur_live.crc);
	cur_live.shift_reg = raw;
	cur_live.data_bit_context = raw & 1;
}

void upd765_family_device::live_write_mfm(uint8_t mfm)
{
	bool context = cur_live.data_bit_context;
	uint16_t raw = 0;
	for(int i=0; i<8; i++) {
		bool bit = mfm & (0x80 >> i);
		if(!(bit || context))
			raw |= 0x8000 >> (2*i);
		if(bit)
			raw |= 0x4000 >> (2*i);
		context = bit;
	}
	cur_live.data_reg = mfm;
	cur_live.shift_reg = raw;
	cur_live.data_bit_context = context;
	//  if (LOG) logerror("write %02x   %04x %04x\n", mfm, cur_live.crc, raw);
}

void upd765_family_device::live_write_fm(uint8_t fm)
{
	uint16_t raw = 0xaaaa;
	for(int i=0; i<8; i++)
		if(fm & (0x80 >> i))
			raw |= 0x4000 >> (2*i);
	cur_live.data_reg = fm;
	cur_live.shift_reg = raw;
	cur_live.data_bit_context = fm & 1;
	//  if (LOG) logerror("write %02x   %04x %04x\n", fm, cur_live.crc, raw);
}

bool upd765_family_device::sector_matches() const
{
	if(0)
		if (LOG) logerror("matching %02x %02x %02x %02x - %02x %02x %02x %02x\n",
					cur_live.idbuf[0], cur_live.idbuf[1], cur_live.idbuf[2], cur_live.idbuf[3],
					command[2], command[3], command[4], command[5]);
	return
		cur_live.idbuf[0] == command[2] &&
		cur_live.idbuf[1] == command[3] &&
		cur_live.idbuf[2] == command[4] &&
		cur_live.idbuf[3] == command[5];
}

upd765a_device::upd765a_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) : upd765_family_device(mconfig, UPD765A, tag, owner, clock)
{
	dor_reset = 0x0c;
}

upd765b_device::upd765b_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) : upd765_family_device(mconfig, UPD765B, tag, owner, clock)
{
	dor_reset = 0x0c;
}

i8272a_device::i8272a_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) : upd765_family_device(mconfig, I8272A, tag, owner, clock)
{
	dor_reset = 0x0c;
}

upd72065_device::upd72065_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) : upd765_family_device(mconfig, UPD72065, tag, owner, clock)
{
	dor_reset = 0x0c;
}

smc37c78_device::smc37c78_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) : upd765_family_device(mconfig, SMC37C78, tag, owner, clock)
{
	ready_connected = false;
	select_connected = true;
}

n82077aa_device::n82077aa_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) : upd765_family_device(mconfig, N82077AA, tag, owner, clock)
{
	ready_connected = false;
	select_connected = true;
}

pc_fdc_superio_device::pc_fdc_superio_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) : upd765_family_device(mconfig, PC_FDC_SUPERIO, tag, owner, clock)
{
	ready_polled = false;
	ready_connected = false;
	select_connected = true;
}

dp8473_device::dp8473_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) : upd765_family_device(mconfig, DP8473, tag, owner, clock)
{
	ready_polled = false;
	ready_connected = false;
	select_connected = true;
}

pc8477a_device::pc8477a_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) : upd765_family_device(mconfig, PC8477A, tag, owner, clock)
{
	ready_polled = true;
	ready_connected = false;
	select_connected = true;
}

wd37c65c_device::wd37c65c_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) : upd765_family_device(mconfig, WD37C65C, tag, owner, clock)
{
	ready_polled = true;
	ready_connected = false;
	select_connected = true;
}

mcs3201_device::mcs3201_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	upd765_family_device(mconfig, MCS3201, tag, owner, clock),
	m_input_handler(*this)
{
	dor_reset = 0x0c;
	ready_polled = false;
	ready_connected = false;
	select_connected = true;
}

void mcs3201_device::device_start()
{
	upd765_family_device::device_start();
	m_input_handler.resolve_safe(0);
}

READ8_MEMBER( mcs3201_device::input_r )
{
	return m_input_handler();
}

tc8566af_device::tc8566af_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: upd765_family_device(mconfig, TC8566AF, tag, owner, clock)
	, m_cr1(0)
{
	ready_polled = true;
	ready_connected = true;
	select_connected = true;
}

void tc8566af_device::device_start()
{
	upd765_family_device::device_start();
	save_item(NAME(m_cr1));
}

WRITE8_MEMBER(tc8566af_device::cr1_w)
{
	m_cr1 = data;

	if (m_cr1 & 0x02)
	{
		// Not sure if this inverted or not
		tc_w((m_cr1 & 0x01) ? true : false);
	}
}
