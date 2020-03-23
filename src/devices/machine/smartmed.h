// license:BSD-3-Clause
// copyright-holders:Raphael Nabet
/*
    smartmed.h: header file for smartmed.c
*/

#ifndef MAME_MACHINE_SMARTMEDIA_H
#define MAME_MACHINE_SMARTMEDIA_H

#pragma once

#include "formats/imageutl.h"
#include "softlist_dev.h"

//#define SMARTMEDIA_IMAGE_SAVE

#define MCFG_NAND_TYPE(type) \
	nand_device::set_nand_type(*device, (nand_device::chip::type));

#define MCFG_NAND_RNB_CALLBACK(write) \
		devcb = &nand_device::set_rnb_wr_callback(*device, DEVCB_##write);

/***************************************************************************
    TYPE DEFINITIONS
***************************************************************************/

// ======================> nand_device
class nand_device : public device_t
{
public:
	// "Sequential Row Read is available only on K9F5608U0D_Y,P,V,F or K9F5608D0D_Y,P"
	enum class chip
	{
		K9F5608U0D = 0,   // K9F5608U0D
		K9F5608U0D_J,     // K9F5608U0D-Jxxx
		K9F5608U0B,       // K9F5608U0B
		K9F1G08U0B,       // K9F1G08U0B
		K9LAG08U0M        // K9LAG08U0M
	};

	// construction/destruction
	nand_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	template <class Object> static devcb_base &set_rnb_wr_callback(device_t &device, Object &&cb) { return downcast<nand_device &>(device).m_write_rnb.set_callback(std::forward<Object>(cb)); }

	static void set_nand_type(device_t &device, chip type)
	{
		nand_device &dev = downcast<nand_device &>(device);
		switch (type)
		{
		case chip::K9F5608U0D:
			dev.m_id_len = 2;
			dev.m_id[0] = 0xec;
			dev.m_id[1] = 0x75;
			dev.m_page_data_size = 512;
			dev.m_page_total_size = 512 + 16;
			dev.m_log2_pages_per_block = compute_log2(32);
			dev.m_num_pages = 32 * 2048;
			dev.m_col_address_cycles = 1;
			dev.m_row_address_cycles = 2;
			dev.m_sequential_row_read = 1;
			break;
		case chip::K9F5608U0D_J:
		case chip::K9F5608U0B:
			dev.m_id_len = 2;
			dev.m_id[0] = 0xec;
			dev.m_id[1] = 0x75;
			dev.m_page_data_size = 512;
			dev.m_page_total_size = 512 + 16;
			dev.m_log2_pages_per_block = compute_log2(32);
			dev.m_num_pages = 32 * 2048;
			dev.m_col_address_cycles = 1;
			dev.m_row_address_cycles = 2;
			dev.m_sequential_row_read = 0;
			break;
		case chip::K9F1G08U0B:
			dev.m_id_len = 5;
			dev.m_id[0] = 0xec;
			dev.m_id[1] = 0xf1;
			dev.m_id[2] = 0x00;
			dev.m_id[3] = 0x95;
			dev.m_id[4] = 0x40;
			dev.m_page_data_size = 2048;
			dev.m_page_total_size = 2048 + 64;
			dev.m_log2_pages_per_block = compute_log2(64);
			dev.m_num_pages = 64 * 1024;
			dev.m_col_address_cycles = 2;
			dev.m_row_address_cycles = 2;
			dev.m_sequential_row_read = 0;
			break;
		case chip::K9LAG08U0M:
			dev.m_id_len = 5;
			dev.m_id[0] = 0xec;
			dev.m_id[1] = 0xd5;
			dev.m_id[2] = 0x55;
			dev.m_id[3] = 0x25;
			dev.m_id[4] = 0x68;
			dev.m_page_data_size = 2048;
			dev.m_page_total_size = 2048 + 64;
			dev.m_log2_pages_per_block = compute_log2(128);
			dev.m_num_pages = 128 * 8192;
			dev.m_col_address_cycles = 2;
			dev.m_row_address_cycles = 3;
			dev.m_sequential_row_read = 0;
			break;
		default:
			printf("Unknown NAND type!\n");
			dev.m_id_len = 0;
			dev.m_page_data_size = 0;
			dev.m_page_total_size = 0;
			dev.m_log2_pages_per_block = 0;
			dev.m_num_pages = 0;
			dev.m_col_address_cycles = 0;
			dev.m_row_address_cycles = 0;
			dev.m_sequential_row_read = 0;
			break;
		}
	}

	int is_present();
	int is_protected();
	int is_busy();

	uint8_t data_r();
	void command_w(uint8_t data);
	void address_w(uint8_t data);
	void data_w(uint8_t data);

	void set_data_ptr(void *ptr);

protected:
	enum sm_mode_t
	{
		SM_M_INIT,      // initial state
		SM_M_READ,      // read page data
		SM_M_PROGRAM,   // program page data
		SM_M_ERASE,     // erase block data
		SM_M_READSTATUS,// read status
		SM_M_READID,        // read ID
		SM_M_30,
		SM_M_RANDOM_DATA_INPUT,
		SM_M_RANDOM_DATA_OUTPUT
	};

	enum pointer_sm_mode_t
	{
		SM_PM_A,        // accessing first 256-byte half of 512-byte data field
		SM_PM_B,        // accessing second 256-byte half of 512-byte data field
		SM_PM_C         // accessing spare field
	};

	nand_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	int m_page_data_size;   // 256 for a 2MB card, 512 otherwise
	int m_page_total_size;// 264 for a 2MB card, 528 otherwise
	int m_num_pages;        // 8192 for a 4MB card, 16184 for 8MB, 32768 for 16MB,
						// 65536 for 32MB, 131072 for 64MB, 262144 for 128MB...
						// 0 means no card loaded
	int m_log2_pages_per_block; // log2 of number of pages per erase block (usually 4 or 5)

	uint8_t* m_data_ptr;  // FEEPROM data area
	std::unique_ptr<uint8_t[]> m_data_uid_ptr;

	sm_mode_t m_mode;               // current operation mode
	pointer_sm_mode_t m_pointer_mode;       // pointer mode

	unsigned int m_page_addr;       // page address pointer
	int m_byte_addr;        // byte address pointer
	int m_addr_load_ptr;    // address load pointer

	int m_status;           // current status
	int m_accumulated_status;   // accumulated status

	std::unique_ptr<uint8_t[]> m_pagereg;   // page register used by program command
	uint8_t m_id[5];      // chip ID
	uint8_t m_mp_opcode;  // multi-plane operation code

	int m_mode_3065;

	// Palm Z22 NAND has 512 + 16 byte pages but, for some reason, Palm OS writes 512 + 64 bytes when
	// programming a page, so we need to keep track of the number of bytes written so we can ignore the
	// last 48 (64 - 16) bytes or else the first 48 bytes get overwritten
	int m_program_byte_count;

	int m_id_len;
	int m_col_address_cycles;
	int m_row_address_cycles;
	int m_sequential_row_read;

	devcb_write_line m_write_rnb;

#ifdef SMARTMEDIA_IMAGE_SAVE
	int m_image_format;
#endif
};



class smartmedia_image_device : public nand_device, public device_image_interface
{
public:
	// construction/destruction
	smartmedia_image_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// image-level overrides
	virtual iodevice_t image_type() const override { return IO_MEMCARD; }

	virtual bool is_readable()  const override { return 1; }
	virtual bool is_writeable() const override { return 1; }
	virtual bool is_creatable() const override { return 0; }
	virtual bool must_be_loaded() const override { return 0; }
	virtual bool is_reset_on_load() const override { return 0; }
	virtual const char *image_interface() const override { return "sm_memc"; }
	virtual const char *file_extensions() const override { return "smc"; }

	virtual image_init_result call_load() override;
	virtual void call_unload() override;
	virtual const software_list_loader &get_software_list_loader() const override { return image_software_list_loader::instance(); }

protected:
	image_init_result smartmedia_format_1();
	image_init_result smartmedia_format_2();
	int detect_geometry(uint8_t id1, uint8_t id2);
};


// device type definition
DECLARE_DEVICE_TYPE(NAND,       nand_device)
DECLARE_DEVICE_TYPE(SMARTMEDIA, smartmedia_image_device)

#endif // MAME_MACHINE_SMARTMEDIA_H
