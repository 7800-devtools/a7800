// license:BSD-3-Clause
// copyright-holders:Morten Shearman Kirkegaard, Samuel Neves, Peter Wilhelmsen
/*************************************************************************

    atarixga.h

    Atari XGA encryption FPGAs

*************************************************************************/

#ifndef MAME_MACHINE_ATARIXGA_H
#define MAME_MACHINE_ATARIXGA_H

DECLARE_DEVICE_TYPE(ATARI_136094_0072, atari_136094_0072_device)
DECLARE_DEVICE_TYPE(ATARI_136095_0072, atari_136095_0072_device)

class atari_xga_device : public device_t
{
public:
	virtual DECLARE_WRITE32_MEMBER(write) = 0;
	virtual DECLARE_READ32_MEMBER(read) = 0;

protected:
	// construction/destruction
	atari_xga_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
		: device_t(mconfig, type, tag, owner, clock)
	{
	}

	virtual void device_start() override = 0;
	virtual void device_reset() override = 0;

	std::unique_ptr<uint16_t[]> m_ram; // CY7C185-45PC, only 16-Kbit used
};

class atari_136094_0072_device : public atari_xga_device
{
public:
	atari_136094_0072_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual DECLARE_WRITE32_MEMBER(write) override;
	virtual DECLARE_READ32_MEMBER(read) override;

protected:
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	static const size_t RAM_WORDS = 2048;

	uint16_t powers2(uint8_t k, uint16_t x);
	uint16_t lfsr2(uint16_t x);
	uint16_t lfsr1(uint16_t x);
	uint16_t decipher(uint8_t k, uint16_t c);

	enum fpga_mode
	{
		FPGA_RESET,
		FPGA_SETKEY,
		FPGA_DECIPHER
	};

	fpga_mode m_mode;
	uint16_t m_address;    // last written address
	uint16_t m_ciphertext; // last written ciphertext
};

class atari_136095_0072_device : public atari_xga_device
{
public:
	atari_136095_0072_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	DECLARE_WRITE32_MEMBER(polylsb_write);
	DECLARE_READ32_MEMBER(polylsb_read);

	virtual DECLARE_WRITE32_MEMBER(write) override;
	virtual DECLARE_READ32_MEMBER(read) override;

protected:
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	static const size_t RAM_WORDS = 4096;

	uint16_t powers2(uint8_t k, uint16_t x);
	uint16_t lfsr2(uint16_t x);
	uint16_t lfsr1(uint16_t x);
	uint16_t decipher(uint8_t k, uint16_t c);

	enum fpga_mode
	{
		FPGA_SETKEY,
		FPGA_DECIPHER,
		FPGA_PROCESS,
		FPGA_RESULT
	};

	struct
	{
		uint16_t addr;
		uint32_t data[64];
	} m_update;

	fpga_mode m_mode;
	uint8_t m_poly_lsb;
	uint16_t m_reply;
};


#endif // MAME_MACHINE_ATARIXGA_H
