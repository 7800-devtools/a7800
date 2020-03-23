// license:BSD-3-Clause
// copyright-holders:ElSemi
#ifndef MAME_VIDEO_VRENDER0_H
#define MAME_VIDEO_VRENDER0_H

#pragma once


/***************************************************************************
 TYPE DEFINITIONS
 ***************************************************************************/

class vr0video_device : public device_t
{
public:
	vr0video_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	~vr0video_device() {}

	int vrender0_ProcessPacket(uint32_t PacketPtr, uint16_t *Dest, uint8_t *TEXTURE);

	static void set_cpu_tag(device_t &device, const char *tag) { downcast<vr0video_device &>(device).m_cpu.set_tag(tag); }

protected:
	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	struct RenderStateInfo
	{
		uint32_t Tx;
		uint32_t Ty;
		uint32_t Txdx;
		uint32_t Tydx;
		uint32_t Txdy;
		uint32_t Tydy;
		uint32_t SrcAlphaColor;
		uint32_t SrcBlend;
		uint32_t DstAlphaColor;
		uint32_t DstBlend;
		uint32_t ShadeColor;
		uint32_t TransColor;
		uint32_t TileOffset;
		uint32_t FontOffset;
		uint32_t PalOffset;
		uint32_t PaletteBank;
		uint32_t TextureMode;
		uint32_t PixelFormat;
		uint32_t Width;
		uint32_t Height;
	};

	// internal state
	required_device<cpu_device> m_cpu;

	uint16_t m_InternalPalette[256];
	uint32_t m_LastPalUpdate;

	RenderStateInfo m_RenderState;
};

DECLARE_DEVICE_TYPE(VIDEO_VRENDER0, vr0video_device)


#define MCFG_VIDEO_VRENDER0_CPU(_tag) \
	vr0video_device::set_cpu_tag(*device, "^" _tag);

#endif // MAME_VIDEO_VRENDER0_H
