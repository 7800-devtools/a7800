// license:BSD-3-Clause
// copyright-holders:David Haywood
/* Konami Video Helper functions */

#include "emu.h"
#include "konami_helper.h"

void konami_decode_gfx(device_gfx_interface &gfxdecode, int gfx_index, uint8_t *data, uint32_t total, const gfx_layout *layout, int bpp)
{
	gfx_layout gl;
	device_palette_interface &palette = gfxdecode.palette();

	memcpy(&gl, layout, sizeof(gl));
	gl.total = total;
	gfxdecode.set_gfx(gfx_index, std::make_unique<gfx_element>(&palette, gl, data, 0, palette.entries() >> bpp, 0));
}


/* useful function to sort three tile layers by priority order */
void konami_sortlayers3( int *layer, int *pri )
{
#define SWAP(a,b) \
	if (pri[a] < pri[b]) \
	{ \
		int t; \
		t = pri[a]; pri[a] = pri[b]; pri[b] = t; \
		t = layer[a]; layer[a] = layer[b]; layer[b] = t; \
	}

	SWAP(0,1)
	SWAP(0,2)
	SWAP(1,2)
#undef  SWAP
}

/* useful function to sort four tile layers by priority order */
void konami_sortlayers4( int *layer, int *pri )
{
#define SWAP(a,b) \
	if (pri[a] <= pri[b]) \
	{ \
		int t; \
		t = pri[a]; pri[a] = pri[b]; pri[b] = t; \
		t = layer[a]; layer[a] = layer[b]; layer[b] = t; \
	}

	SWAP(0, 1)
	SWAP(0, 2)
	SWAP(0, 3)
	SWAP(1, 2)
	SWAP(1, 3)
	SWAP(2, 3)
#undef  SWAP
}

/* useful function to sort five tile layers by priority order */
void konami_sortlayers5( int *layer, int *pri )
{
#define SWAP(a,b) \
	if (pri[a] <= pri[b]) \
	{ \
		int t; \
		t = pri[a]; pri[a] = pri[b]; pri[b] = t; \
		t = layer[a]; layer[a] = layer[b]; layer[b] = t; \
	}

	SWAP(0, 1)
	SWAP(0, 2)
	SWAP(0, 3)
	SWAP(0, 4)
	SWAP(1, 2)
	SWAP(1, 3)
	SWAP(1, 4)
	SWAP(2, 3)
	SWAP(2, 4)
	SWAP(3, 4)
#undef  SWAP
}
