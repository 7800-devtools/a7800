// license:BSD-3-Clause
// copyright-holders:David Haywood
#ifndef MAME_MACHINE_315_5881_CRYPT_H
#define MAME_MACHINE_315_5881_CRYPT_H

#pragma once


typedef device_delegate<uint16_t (uint32_t)> sega_m2_read_delegate;

DECLARE_DEVICE_TYPE(SEGA315_5881_CRYPT, sega_315_5881_crypt_device)

#define MCFG_SET_READ_CALLBACK( _class, _method) \
	sega_315_5881_crypt_device::set_read_cb(*device, sega_m2_read_delegate(&_class::_method, #_class "::" #_method, nullptr, (_class *)nullptr));


class sega_315_5881_crypt_device :  public device_t
{
public:
	// construction/destruction
	sega_315_5881_crypt_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);


	uint16_t do_decrypt(uint8_t *&base);
	void set_addr_low(uint16_t data);
	void set_addr_high(uint16_t data);
	void set_subkey(uint16_t data);

	sega_m2_read_delegate m_read;

	static void set_read_cb(device_t &device,sega_m2_read_delegate readcb)
	{
		sega_315_5881_crypt_device &dev = downcast<sega_315_5881_crypt_device &>(device);
		dev.m_read = readcb;
	}

protected:
	virtual void device_start() override;
	virtual void device_reset() override;

private:

	enum {
//        BUFFER_SIZE = 32768, LINE_SIZE = 512,
		BUFFER_SIZE = 2, LINE_SIZE = 512,  // this should be a stream, without any 'BUFFER_SIZE' ? I guess the SH4 DMA implementation isn't on a timer tho?
		FLAG_COMPRESSED = 0x20000
	};

	uint32_t key;

	std::unique_ptr<uint8_t[]> buffer;
	std::unique_ptr<uint8_t[]> line_buffer;
	std::unique_ptr<uint8_t[]> line_buffer_prev;
	uint32_t prot_cur_address;
	uint16_t subkey, dec_hist;
	uint32_t dec_header;

	bool enc_ready;

	int buffer_pos, line_buffer_pos, line_buffer_size, buffer_bit, buffer_bit2;
	uint8_t buffer2[2];
	uint16_t buffer2a;

	int block_size;
	int block_pos;
	int block_numlines;
	int done_compression;

	struct sbox {
		uint8_t table[64];
		int inputs[6];      // positions of the inputs bits, -1 means no input except from key
		int outputs[2];     // positions of the output bits
	};

	static const sbox fn1_sboxes[4][4];
	static const sbox fn2_sboxes[4][4];

	static const int FN1GK = 38;
	static const int FN2GK = 32;
	static const int fn1_game_key_scheduling[FN1GK][2];
	static const int fn2_game_key_scheduling[FN2GK][2];
	static const int fn1_sequence_key_scheduling[20][2];
	static const int fn2_sequence_key_scheduling[16];
	static const int fn2_middle_result_scheduling[16];

	static const uint8_t trees[9][2][32];

	int feistel_function(int input, const struct sbox *sboxes, uint32_t subkeys);
	uint16_t block_decrypt(uint32_t game_key, uint16_t sequence_key, uint16_t counter, uint16_t data);

	uint16_t get_decrypted_16();
	int get_compressed_bit();

	void enc_start();
	void enc_fill();
	void line_fill();

};

#endif // MAME_MACHINE_315_5881_CRYPT_H
