// license:BSD-3-Clause
// copyright-holders:Dirk Best
/**********************************************************************

    ISA cards

**********************************************************************/

#include "emu.h"
#include "isa_cards.h"

// video
#include "mda.h"
#include "cga.h"
#include "aga.h"
#include "ega.h"
#include "pgc.h"
#include "vga.h"
#include "vga_ati.h"
#include "svga_cirrus.h"
#include "svga_s3.h"
#include "svga_tseng.h"
#include "svga_trident.h"
#include "num9rev.h"

// storage
#include "fdc.h"
#include "mufdc.h"
#include "hdc.h"
#include "wdxt_gen.h"
#include "ide.h"
#include "xtide.h"
#include "side116.h"
#include "aha1542.h"
#include "wd1002a_wx1.h"
#include "mcd.h"

// sound
#include "adlib.h"
#include "gblaster.h"
#include "gus.h"
#include "ibm_mfc.h"
#include "mpu401.h"
#include "sblaster.h"
#include "ssi2001.h"
#include "stereo_fx.h"
#include "dectalk.h"
#include "sb16.h"

// network
#include "3c503.h"
#include "ne1000.h"
#include "ne2000.h"
#include "3c505.h"

// communication ports
#include "lpt.h"
#include "com.h"
#include "pds.h"

// other
#include "finalchs.h"


SLOT_INTERFACE_START( pc_isa8_cards )
	SLOT_INTERFACE("mda", ISA8_MDA)
	SLOT_INTERFACE("cga", ISA8_CGA)
	SLOT_INTERFACE("cga_ec1841", ISA8_EC1841_0002)
	SLOT_INTERFACE("cga_poisk2", ISA8_CGA_POISK2)
	SLOT_INTERFACE("cga_mc1502", ISA8_CGA_MC1502)
	SLOT_INTERFACE("cga_m24", ISA8_CGA_M24)
	SLOT_INTERFACE("aga", ISA8_AGA)
	SLOT_INTERFACE("aga_pc200", ISA8_AGA_PC200)
	SLOT_INTERFACE("ega", ISA8_EGA)
	SLOT_INTERFACE("pgc", ISA8_PGC)
	SLOT_INTERFACE("vga", ISA8_VGA)
	SLOT_INTERFACE("svga_et4k", ISA8_SVGA_ET4K)
	SLOT_INTERFACE("num9rev",ISA8_NUM_9_REV)
	SLOT_INTERFACE("com", ISA8_COM)
	SLOT_INTERFACE("fdc", ISA8_FDC_SUPERIO)
	SLOT_INTERFACE("fdc_xt", ISA8_FDC_XT)
	SLOT_INTERFACE("fdc_at", ISA8_FDC_AT)
	SLOT_INTERFACE("fdc_smc", ISA8_FDC_SMC)
	SLOT_INTERFACE("fdc_ps2", ISA8_FDC_PS2)
	SLOT_INTERFACE("fdc344", ISA8_FDC344)
	SLOT_INTERFACE("fdcmag", ISA8_FDCMAG)
	SLOT_INTERFACE("wdxt_gen", ISA8_WDXT_GEN)
	SLOT_INTERFACE("finalchs", ISA8_FINALCHS)
	SLOT_INTERFACE("xtide", ISA8_XTIDE)
	SLOT_INTERFACE("side116", ISA8_SIDE116)
	SLOT_INTERFACE("hdc", ISA8_HDC)
	SLOT_INTERFACE("adlib", ISA8_ADLIB)
	SLOT_INTERFACE("hercules", ISA8_HERCULES)
	SLOT_INTERFACE("gblaster", ISA8_GAME_BLASTER)
	SLOT_INTERFACE("sblaster1_0", ISA8_SOUND_BLASTER_1_0)
	SLOT_INTERFACE("sblaster1_5", ISA8_SOUND_BLASTER_1_5)
	SLOT_INTERFACE("stereo_fx", ISA8_STEREO_FX)
	SLOT_INTERFACE("ssi2001", ISA8_SSI2001)
	SLOT_INTERFACE("mpu401", ISA8_MPU401)
	SLOT_INTERFACE("ne1000", NE1000)
	SLOT_INTERFACE("3c503", EL2_3C503)
	SLOT_INTERFACE("lpt", ISA8_LPT)
	SLOT_INTERFACE("ibm_mfc", ISA8_IBM_MFC)
	SLOT_INTERFACE("wd1002a_wx1", ISA8_WD1002A_WX1)
	SLOT_INTERFACE("dectalk", ISA8_DECTALK)
	SLOT_INTERFACE("pds", ISA8_PDS)
SLOT_INTERFACE_END

SLOT_INTERFACE_START( pc_isa16_cards )
	// 8-bit
	SLOT_INTERFACE("mda", ISA8_MDA)
	SLOT_INTERFACE("cga", ISA8_CGA)
	SLOT_INTERFACE("wyse700", ISA8_WYSE700)
	SLOT_INTERFACE("ega", ISA8_EGA)
	SLOT_INTERFACE("pgc", ISA8_PGC)
	SLOT_INTERFACE("vga", ISA8_VGA)
	SLOT_INTERFACE("svga_et4k", ISA8_SVGA_ET4K)
	SLOT_INTERFACE("num9rev",ISA8_NUM_9_REV)
	SLOT_INTERFACE("com", ISA8_COM)
	SLOT_INTERFACE("comat", ISA8_COM_AT)
	SLOT_INTERFACE("fdc", ISA8_FDC_AT)
	SLOT_INTERFACE("fdc344", ISA8_FDC344)
	SLOT_INTERFACE("fdcmag", ISA8_FDCMAG)
	SLOT_INTERFACE("hdc", ISA8_HDC)
	SLOT_INTERFACE("side116", ISA8_SIDE116)
	SLOT_INTERFACE("adlib", ISA8_ADLIB)
	SLOT_INTERFACE("hercules", ISA8_HERCULES)
	SLOT_INTERFACE("gblaster", ISA8_GAME_BLASTER)
	SLOT_INTERFACE("sblaster1_0", ISA8_SOUND_BLASTER_1_0)
	SLOT_INTERFACE("sblaster1_5", ISA8_SOUND_BLASTER_1_5)
	SLOT_INTERFACE("stereo_fx", ISA8_STEREO_FX)
	SLOT_INTERFACE("ssi2001", ISA8_SSI2001)
	SLOT_INTERFACE("ne1000", NE1000)
	SLOT_INTERFACE("3c503", EL2_3C503)
	SLOT_INTERFACE("mpu401", ISA8_MPU401)
	SLOT_INTERFACE("lpt", ISA8_LPT)
	SLOT_INTERFACE("ibm_mfc", ISA8_IBM_MFC)
	SLOT_INTERFACE("fdcsmc", ISA8_FDC_SMC)
	SLOT_INTERFACE("dectalk", ISA8_DECTALK)
	SLOT_INTERFACE("pds", ISA8_PDS)
	// 16-bit
	SLOT_INTERFACE("ide", ISA16_IDE)
	SLOT_INTERFACE("ne2000", NE2000)
	SLOT_INTERFACE("aha1542", AHA1542)
	SLOT_INTERFACE("gus",ISA16_GUS)
	SLOT_INTERFACE("sblaster_16", ISA16_SOUND_BLASTER_16)
	SLOT_INTERFACE("svga_s3", ISA16_SVGA_S3)
	SLOT_INTERFACE("s3virge", ISA16_S3VIRGE)
	SLOT_INTERFACE("s3virgedx", ISA16_S3VIRGEDX)
	SLOT_INTERFACE("dms3d2kp", ISA16_DMS3D2KPRO)
	SLOT_INTERFACE("svga_dm",ISA16_SVGA_CIRRUS)
	SLOT_INTERFACE("clgd542x",ISA16_SVGA_CIRRUS_GD542X)
	SLOT_INTERFACE("gfxultra", ISA16_VGA_GFXULTRA)
	SLOT_INTERFACE("gfxultrap", ISA16_SVGA_GFXULTRAPRO)
	SLOT_INTERFACE("tgui9680",ISA16_SVGA_TGUI9680)
	SLOT_INTERFACE("3c505", ISA16_3C505)
	SLOT_INTERFACE("mach64", ISA16_SVGA_MACH64)
	SLOT_INTERFACE("sb16_lle", ISA16_SB16)
	SLOT_INTERFACE("mcd", ISA16_MCD)
SLOT_INTERFACE_END
