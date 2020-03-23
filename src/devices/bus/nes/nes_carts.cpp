// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
/**********************************************************************

    NES carts

**********************************************************************/

#include "emu.h"
#include "nes_carts.h"

// official PCBs
#include "nxrom.h"
#include "mmc1.h"
#include "mmc2.h"
#include "mmc3.h"
#include "mmc5.h"
#include "bandai.h"
#include "datach.h"
#include "discrete.h"
#include "disksys.h"
#include "event.h"
#include "irem.h"
#include "jaleco.h"
#include "karastudio.h"
#include "konami.h"
#include "namcot.h"
#include "pt554.h"
#include "sunsoft.h"
#include "sunsoft_dcs.h"
#include "taito.h"
// unlicensed/bootleg/pirate PCBs
#include "2a03pur.h"
#include "act53.h"
#include "aladdin.h"
#include "ave.h"
#include "benshieng.h"
#include "camerica.h"
#include "cne.h"
#include "cony.h"
#include "ggenie.h"
#include "hes.h"
#include "henggedianzi.h"
#include "hosenkan.h"
#include "jy.h"
#include "kaiser.h"
#include "legacy.h"
#include "nanjing.h"
#include "ntdec.h"
#include "racermate.h"
#include "rcm.h"
#include "rexsoft.h"
#include "sachen.h"
#include "somari.h"
#include "subor.h"
#include "tengen.h"
#include "txc.h"
#include "waixing.h"
#include "zemina.h"
// misc unlicensed/bootleg/pirate PCBs
#include "bootleg.h"
#include "multigame.h"
#include "pirate.h"
#include "mmc3_clones.h"


SLOT_INTERFACE_START(nes_cart)
// HROM, NROM, RROM, SROM & STROM
	SLOT_INTERFACE_INTERNAL("nrom",             NES_NROM)
// Nintendo Family BASIC pcb (NROM + 2K or 4K WRAM)
	SLOT_INTERFACE_INTERNAL("hvc_basic",        NES_FCBASIC)
// Extended NROM-368 board (NROM with 46K PRG)
	SLOT_INTERFACE_INTERNAL("nrom368",          NES_NROM368)
// Game Genie
	SLOT_INTERFACE_INTERNAL("nrom_gg",          NES_GGENIE)
// UNROM/UOROM
	SLOT_INTERFACE_INTERNAL("uxrom",            NES_UXROM)
	SLOT_INTERFACE_INTERNAL("unrom_cc",         NES_UXROM_CC)
// CNROM
	SLOT_INTERFACE_INTERNAL("cnrom",            NES_CNROM)
// Bandai PT-554 (CNROM boards + special audio chip, used by Aerobics Studio)
	SLOT_INTERFACE_INTERNAL("bandai_pt554",     NES_BANDAI_PT554)
// CPROM
	SLOT_INTERFACE_INTERNAL("cprom",            NES_CPROM)
// AMROM, ANROM, AOROM
	SLOT_INTERFACE_INTERNAL("axrom",            NES_AXROM)
// PxROM
	SLOT_INTERFACE_INTERNAL("pxrom",            NES_PXROM)
// FxROM
	SLOT_INTERFACE_INTERNAL("fxrom",            NES_FXROM)
// BNROM
	SLOT_INTERFACE_INTERNAL("bnrom",            NES_BXROM)
// GNROM & MHROM
	SLOT_INTERFACE_INTERNAL("gxrom",            NES_GXROM)
// UN1ROM
	SLOT_INTERFACE_INTERNAL("un1rom",           NES_UN1ROM)
// SxROM
	SLOT_INTERFACE_INTERNAL("sxrom",            NES_SXROM)
	SLOT_INTERFACE_INTERNAL("sorom",            NES_SOROM)
	SLOT_INTERFACE_INTERNAL("sxrom_a",          NES_SXROM_A)  // in MMC1-A PRG RAM is always enabled
	SLOT_INTERFACE_INTERNAL("sorom_a",          NES_SOROM_A)  // in MMC1-A PRG RAM is always enabled
// TxROM
	SLOT_INTERFACE_INTERNAL("txrom",            NES_TXROM)
// HKROM
	SLOT_INTERFACE_INTERNAL("hkrom",            NES_HKROM)
// TQROM
	SLOT_INTERFACE_INTERNAL("tqrom",            NES_TQROM)
// TxSROM
	SLOT_INTERFACE_INTERNAL("txsrom",           NES_TXSROM)
// ExROM
	SLOT_INTERFACE_INTERNAL("exrom",            NES_EXROM)
// RAM expansion + Disk System add-on
	SLOT_INTERFACE_INTERNAL("disksys",          NES_DISKSYS)
// Nintendo Custom boards
	SLOT_INTERFACE_INTERNAL("pal_zz",           NES_ZZ_PCB)
	SLOT_INTERFACE_INTERNAL("nes_qj",           NES_QJ_PCB)
	SLOT_INTERFACE_INTERNAL("nes_event",        NES_EVENT)
// Discrete Components boards
// IC_74x139x74
	SLOT_INTERFACE_INTERNAL("discrete_74x139",  NES_74X139X74)
// IC_74x377
	SLOT_INTERFACE_INTERNAL("discrete_74x377",  NES_74X377)
// Discrete board IC_74x161x161x32
	SLOT_INTERFACE_INTERNAL("discrete_74x161", NES_74X161X161X32)
// Discrete board IC_74x161x138
	SLOT_INTERFACE_INTERNAL("bitcorp_dis",      NES_74X161X138)
// Bandai boards
	SLOT_INTERFACE_INTERNAL("lz93d50",          NES_LZ93D50)
	SLOT_INTERFACE_INTERNAL("lz93d50_ep1",      NES_LZ93D50_24C01)
	SLOT_INTERFACE_INTERNAL("lz93d50_ep2",      NES_LZ93D50_24C02)
	SLOT_INTERFACE_INTERNAL("fcg",              NES_FCG)
	SLOT_INTERFACE_INTERNAL("fjump2",           NES_FJUMP2)
	SLOT_INTERFACE_INTERNAL("datach",           NES_DATACH)
	SLOT_INTERFACE_INTERNAL("karastudio",       NES_KARAOKESTUDIO)
	SLOT_INTERFACE_INTERNAL("oekakids",         NES_OEKAKIDS)
// Irem boards
	SLOT_INTERFACE_INTERNAL("g101",             NES_G101)
	SLOT_INTERFACE_INTERNAL("lrog017",          NES_LROG017)
	SLOT_INTERFACE_INTERNAL("h3001",            NES_H3001)
	SLOT_INTERFACE_INTERNAL("holydivr",         NES_HOLYDIVR)
	SLOT_INTERFACE_INTERNAL("tam_s1",           NES_TAM_S1)
// Jaleco boards
	SLOT_INTERFACE_INTERNAL("jf11",             NES_JF11)
	SLOT_INTERFACE_INTERNAL("jf13",             NES_JF13)
	SLOT_INTERFACE_INTERNAL("jf16",             NES_JF16)
	SLOT_INTERFACE_INTERNAL("jf17",             NES_JF17)
	SLOT_INTERFACE_INTERNAL("jf17pcm",          NES_JF17_ADPCM)
	SLOT_INTERFACE_INTERNAL("jf19",             NES_JF19)
	SLOT_INTERFACE_INTERNAL("jf19pcm",          NES_JF19_ADPCM)
	SLOT_INTERFACE_INTERNAL("ss88006",          NES_SS88006)
	SLOT_INTERFACE_INTERNAL("jf23",             NES_JF23)
	SLOT_INTERFACE_INTERNAL("jf24",             NES_JF24)
	SLOT_INTERFACE_INTERNAL("jf29",             NES_JF29)
	SLOT_INTERFACE_INTERNAL("jf33",             NES_JF33)
// Konami boards
	SLOT_INTERFACE_INTERNAL("vrc1",             NES_VRC1)
	SLOT_INTERFACE_INTERNAL("vrc2",             NES_VRC2)
	SLOT_INTERFACE_INTERNAL("vrc3",             NES_VRC3)
	SLOT_INTERFACE_INTERNAL("vrc4",             NES_VRC4)
	SLOT_INTERFACE_INTERNAL("vrc6",             NES_VRC6)
	SLOT_INTERFACE_INTERNAL("vrc7",             NES_VRC7)
// Namcot boards
	SLOT_INTERFACE_INTERNAL("namcot_163",       NES_NAMCOT163)
	SLOT_INTERFACE_INTERNAL("namcot_175",       NES_NAMCOT175)
	SLOT_INTERFACE_INTERNAL("namcot_340",       NES_NAMCOT340)
	SLOT_INTERFACE_INTERNAL("namcot_3433",      NES_NAMCOT3433)  // DxROM is a Nintendo board for US versions of the 3433/3443 games
	SLOT_INTERFACE_INTERNAL("namcot_3425",      NES_NAMCOT3425)
	SLOT_INTERFACE_INTERNAL("namcot_3446",      NES_NAMCOT3446)
// Sunsoft boards
	SLOT_INTERFACE_INTERNAL("sunsoft1",         NES_SUNSOFT_1)
	SLOT_INTERFACE_INTERNAL("sunsoft2",         NES_SUNSOFT_2)
	SLOT_INTERFACE_INTERNAL("sunsoft3",         NES_SUNSOFT_3)
	SLOT_INTERFACE_INTERNAL("sunsoft4",         NES_SUNSOFT_4)
	SLOT_INTERFACE_INTERNAL("sunsoft_dcs",      NES_SUNSOFT_DCS)
	SLOT_INTERFACE_INTERNAL("sunsoft_fme7",     NES_SUNSOFT_FME7) // JxROM is a Nintendo board for US versions of the Sunsoft FME7 games
	SLOT_INTERFACE_INTERNAL("sunsoft5a",        NES_SUNSOFT_5)
	SLOT_INTERFACE_INTERNAL("sunsoft5b",        NES_SUNSOFT_5)
// Taito boards
	SLOT_INTERFACE_INTERNAL("tc0190fmc",        NES_TC0190FMC)
	SLOT_INTERFACE_INTERNAL("tc0190fmcp",       NES_TC0190FMC_PAL16R4)
	SLOT_INTERFACE_INTERNAL("tc0350fmr",        NES_TC0190FMC)
	SLOT_INTERFACE_INTERNAL("x1_005",           NES_X1_005)   // two variants exist, depending on pin17 & pin31 connections
	SLOT_INTERFACE_INTERNAL("x1_017",           NES_X1_017)
// Misc pirate boards (by AVE, Camerica, C&E, Nanjing, NTDEC, JY Company, Sachen, Tengen, TXC, Waixing, Henggendianzi, etc.)
	SLOT_INTERFACE_INTERNAL("nina001",          NES_NINA001)
	SLOT_INTERFACE_INTERNAL("nina006",          NES_NINA006)
	SLOT_INTERFACE_INTERNAL("bf9093",           NES_BF9093)
	SLOT_INTERFACE_INTERNAL("bf9096",           NES_BF9096)
	SLOT_INTERFACE_INTERNAL("goldenfive",       NES_GOLDEN5)
	SLOT_INTERFACE_INTERNAL("ade",              NES_ALADDIN)
	SLOT_INTERFACE_INTERNAL("cne_decathl",      NES_CNE_DECATHL)
	SLOT_INTERFACE_INTERNAL("cne_fsb",          NES_CNE_FSB)
	SLOT_INTERFACE_INTERNAL("cne_shlz",         NES_CNE_SHLZ)
	SLOT_INTERFACE_INTERNAL("nanjing",          NES_NANJING) // mapper 163
	SLOT_INTERFACE_INTERNAL("ntdec_asder",      NES_NTDEC_ASDER) // mapper 112
	SLOT_INTERFACE_INTERNAL("ntdec_fh",         NES_NTDEC_FH)    // mapper 193
	SLOT_INTERFACE_INTERNAL("jyc_a",            NES_JY_TYPEA)    // mapper 90
	SLOT_INTERFACE_INTERNAL("jyc_b",            NES_JY_TYPEB)    // mapper 211
	SLOT_INTERFACE_INTERNAL("jyc_c",            NES_JY_TYPEC)    // mapper 209
	SLOT_INTERFACE_INTERNAL("sa009",            NES_SACHEN_SA009)
	SLOT_INTERFACE_INTERNAL("sa0036",           NES_SACHEN_SA0036)
	SLOT_INTERFACE_INTERNAL("sa0037",           NES_SACHEN_SA0037)
	SLOT_INTERFACE_INTERNAL("sa72007",          NES_SACHEN_SA72007)
	SLOT_INTERFACE_INTERNAL("sa72008",          NES_SACHEN_SA72008)
	SLOT_INTERFACE_INTERNAL("tca01",            NES_SACHEN_TCA01)
	SLOT_INTERFACE_INTERNAL("s8259a",           NES_SACHEN_8259A)
	SLOT_INTERFACE_INTERNAL("s8259b",           NES_SACHEN_8259B)
	SLOT_INTERFACE_INTERNAL("s8259c",           NES_SACHEN_8259C)
	SLOT_INTERFACE_INTERNAL("s8259d",           NES_SACHEN_8259D)
	SLOT_INTERFACE_INTERNAL("s74x374",          NES_SACHEN_74X374)
	SLOT_INTERFACE_INTERNAL("s74x374a",         NES_SACHEN_74X374_ALT)  /* FIXME: Made up boards some different handling */
	SLOT_INTERFACE_INTERNAL("tcu01",            NES_SACHEN_TCU01)
	SLOT_INTERFACE_INTERNAL("tcu02",            NES_SACHEN_TCU02)
	SLOT_INTERFACE_INTERNAL("tengen_800008",    NES_TENGEN_800008)   /* FIXME: Is this the same as CNROM? */
	SLOT_INTERFACE_INTERNAL("tengen_800032",    NES_TENGEN_800032)
	SLOT_INTERFACE_INTERNAL("tengen_800037",    NES_TENGEN_800037)
	SLOT_INTERFACE_INTERNAL("txc_22211",        NES_TXC_22211)
	SLOT_INTERFACE_INTERNAL("txc_dumarc",       NES_TXC_DUMARACING)
	SLOT_INTERFACE_INTERNAL("txc_mjblock",      NES_TXC_MJBLOCK)
	SLOT_INTERFACE_INTERNAL("txc_strikew",      NES_TXC_STRIKEW)
	SLOT_INTERFACE_INTERNAL("txc_commandos",    NES_TXC_COMMANDOS)
	SLOT_INTERFACE_INTERNAL("waixing_a",        NES_WAIXING_A)
	SLOT_INTERFACE_INTERNAL("waixing_a1",       NES_WAIXING_A1)   /* FIXME: Made up boards the different CHRRAM banks (see Ji Jia Zhan Shi) */
	SLOT_INTERFACE_INTERNAL("waixing_b",        NES_WAIXING_B)
	SLOT_INTERFACE_INTERNAL("waixing_c",        NES_WAIXING_C)
	SLOT_INTERFACE_INTERNAL("waixing_d",        NES_WAIXING_D)
	SLOT_INTERFACE_INTERNAL("waixing_e",        NES_WAIXING_E)
	SLOT_INTERFACE_INTERNAL("waixing_f",        NES_WAIXING_F)
	SLOT_INTERFACE_INTERNAL("waixing_g",        NES_WAIXING_G)
	SLOT_INTERFACE_INTERNAL("waixing_h",        NES_WAIXING_H)
	SLOT_INTERFACE_INTERNAL("waixing_h1",       NES_WAIXING_H1)   /* FIXME: Made up boards the different WRAM protect banks (see Shen Mi Jin San Jiao) */
	SLOT_INTERFACE_INTERNAL("waixing_i",        NES_WAIXING_I)
	SLOT_INTERFACE_INTERNAL("waixing_j",        NES_WAIXING_J)
	SLOT_INTERFACE_INTERNAL("waixing_sgz",      NES_WAIXING_SGZ)
	SLOT_INTERFACE_INTERNAL("waixing_sgzlz",    NES_WAIXING_SGZLZ)
	SLOT_INTERFACE_INTERNAL("waixing_sec",      NES_WAIXING_SEC)
	SLOT_INTERFACE_INTERNAL("waixing_ffv",      NES_WAIXING_FFV)
	SLOT_INTERFACE_INTERNAL("waixing_wxzs",     NES_WAIXING_WXZS)
	SLOT_INTERFACE_INTERNAL("waixing_wxzs2",    NES_WAIXING_WXZS2)
	SLOT_INTERFACE_INTERNAL("waixing_dq8",      NES_WAIXING_DQ8)
	SLOT_INTERFACE_INTERNAL("waixing_sh2",      NES_WAIXING_SH2)
	SLOT_INTERFACE_INTERNAL("fs304",            NES_WAIXING_FS304)  // used in Zelda 3 by Waixing
	SLOT_INTERFACE_INTERNAL("cony",             NES_CONY)
	SLOT_INTERFACE_INTERNAL("yoko",             NES_YOKO)
	SLOT_INTERFACE_INTERNAL("hengg_srich",      NES_HENGG_SRICH)
	SLOT_INTERFACE_INTERNAL("hengg_xhzs",       NES_HENGG_XHZS)
	SLOT_INTERFACE_INTERNAL("hengg_shjy3",      NES_HENGG_SHJY3) // mapper 253
	SLOT_INTERFACE_INTERNAL("hes",              NES_HES)
	SLOT_INTERFACE_INTERNAL("hosenkan",         NES_HOSENKAN)
	SLOT_INTERFACE_INTERNAL("ks7058",           NES_KS7058)
	SLOT_INTERFACE_INTERNAL("ks202",            NES_KS202)   // mapper 56
	SLOT_INTERFACE_INTERNAL("ks7022",           NES_KS7022)// mapper 175
	SLOT_INTERFACE_INTERNAL("ks7017",           NES_KS7017)
	SLOT_INTERFACE_INTERNAL("ks7032",           NES_KS7032) //  mapper 142
	SLOT_INTERFACE_INTERNAL("ks7012",           NES_KS7012)  // used in Zanac (FDS Conversion)
	SLOT_INTERFACE_INTERNAL("ks7013b",          NES_KS7013B) // used in Highway Star (FDS Conversion)
	SLOT_INTERFACE_INTERNAL("ks7031",           NES_KS7031) //  used in Dracula II (FDS Conversion)
	SLOT_INTERFACE_INTERNAL("ks7016",           NES_KS7016) //  used in Exciting Basket (FDS Conversion)
	SLOT_INTERFACE_INTERNAL("ks7037",           NES_KS7037) //  used in Metroid (FDS Conversion)
	SLOT_INTERFACE_INTERNAL("gs2015",           NES_GS2015)
	SLOT_INTERFACE_INTERNAL("gs2004",           NES_GS2004)
	SLOT_INTERFACE_INTERNAL("gs2013",           NES_GS2013)
	SLOT_INTERFACE_INTERNAL("tf9in1",           NES_TF9IN1)
	SLOT_INTERFACE_INTERNAL("3dblock",          NES_3DBLOCK)    // NROM + IRQ?
	SLOT_INTERFACE_INTERNAL("racermate",        NES_RACERMATE)   // mapper 168
	SLOT_INTERFACE_INTERNAL("agci_50282",       NES_AGCI_50282)
	SLOT_INTERFACE_INTERNAL("dreamtech01",      NES_DREAMTECH01)
	SLOT_INTERFACE_INTERNAL("fukutake",         NES_FUKUTAKE)
	SLOT_INTERFACE_INTERNAL("futuremedia",      NES_FUTUREMEDIA)
	SLOT_INTERFACE_INTERNAL("magicseries",      NES_MAGSERIES)
	SLOT_INTERFACE_INTERNAL("daou_306",         NES_DAOU306)
	SLOT_INTERFACE_INTERNAL("subor0",           NES_SUBOR0)
	SLOT_INTERFACE_INTERNAL("subor1",           NES_SUBOR1)
	SLOT_INTERFACE_INTERNAL("subor2",           NES_SUBOR2)
	SLOT_INTERFACE_INTERNAL("cc21",             NES_CC21)
	SLOT_INTERFACE_INTERNAL("xiaozy",           NES_XIAOZY)
	SLOT_INTERFACE_INTERNAL("edu2k",            NES_EDU2K)
	SLOT_INTERFACE_INTERNAL("t230",             NES_T230)
	SLOT_INTERFACE_INTERNAL("mk2",              NES_MK2)
	SLOT_INTERFACE_INTERNAL("unl_whero",        NES_WHERO)    // mapper 27
	SLOT_INTERFACE_INTERNAL("unl_43272",        NES_43272)    // used in Gaau Hok Gwong Cheung
	SLOT_INTERFACE_INTERNAL("tf1201",           NES_TF1201)
	SLOT_INTERFACE_INTERNAL("unl_cfight",       NES_CITYFIGHT) //  used by City Fighter IV
	SLOT_INTERFACE_INTERNAL("zemina",           NES_ZEMINA)    // mapper 190 - Magic Kid GooGoo
// misc bootleg boards
	SLOT_INTERFACE_INTERNAL("ax5705",           NES_AX5705)
	SLOT_INTERFACE_INTERNAL("sc127",            NES_SC127)
	SLOT_INTERFACE_INTERNAL("mariobaby",        NES_MARIOBABY)
	SLOT_INTERFACE_INTERNAL("asnicol",          NES_ASN)
	SLOT_INTERFACE_INTERNAL("smb3pirate",       NES_SMB3PIRATE)
	SLOT_INTERFACE_INTERNAL("btl_dninja",       NES_BTL_DNINJA)
	SLOT_INTERFACE_INTERNAL("whirl2706",        NES_WHIRLWIND_2706)
	SLOT_INTERFACE_INTERNAL("smb2j",            NES_SMB2J)
	SLOT_INTERFACE_INTERNAL("smb2ja",           NES_SMB2JA)
	SLOT_INTERFACE_INTERNAL("smb2jb",           NES_SMB2JB)
	SLOT_INTERFACE_INTERNAL("09034a",           NES_09034A)
	SLOT_INTERFACE_INTERNAL("tobidase",         NES_TOBIDASE) // mapper 120
	SLOT_INTERFACE_INTERNAL("mmalee2",          NES_MMALEE)    // mapper 55?
	SLOT_INTERFACE_INTERNAL("unl_2708",         NES_2708)    // mapper 103
	SLOT_INTERFACE_INTERNAL("unl_lh32",         NES_LH32)   // used by Monty no Doki Doki Daidassou FDS conversion
	SLOT_INTERFACE_INTERNAL("unl_lh10",         NES_LH10)    // used in Fuuun Shaolin Kyo (FDS Conversion)
	SLOT_INTERFACE_INTERNAL("unl_lh53",         NES_LH53)    // used in Nazo no Murasamejou (FDS Conversion)
	SLOT_INTERFACE_INTERNAL("unl_ac08",         NES_AC08) //  used by Green Beret FDS conversion
	SLOT_INTERFACE_INTERNAL("unl_bb",           NES_UNL_BB) //  used by a few FDS conversions
	SLOT_INTERFACE_INTERNAL("sgpipe",           NES_SHUIGUAN)    // mapper 183
	SLOT_INTERFACE_INTERNAL("rt01",             NES_RT01)
// misc MMC3 clone boards
	SLOT_INTERFACE_INTERNAL("dbz5",             NES_REX_DBZ5)
	SLOT_INTERFACE_INTERNAL("sl1632",           NES_REX_SL1632)
	SLOT_INTERFACE_INTERNAL("somari",           NES_SOMARI) // mapper 116
	SLOT_INTERFACE_INTERNAL("nitra",            NES_NITRA)
	SLOT_INTERFACE_INTERNAL("ks7057",           NES_KS7057) // mapper 196 alt (for Street Fighter VI / Fight Street VI)
	SLOT_INTERFACE_INTERNAL("sbros11",          NES_SBROS11)
	SLOT_INTERFACE_INTERNAL("unl_malisb",       NES_MALISB) //  used by Super Mali Splash Bomb
	SLOT_INTERFACE_INTERNAL("family4646",       NES_FAMILY4646)
	SLOT_INTERFACE_INTERNAL("pikay2k",          NES_PIKAY2K) // mapper 254
	SLOT_INTERFACE_INTERNAL("8237",             NES_8237)
	SLOT_INTERFACE_INTERNAL("8237a",            NES_NROM)    // UNSUPPORTED
	SLOT_INTERFACE_INTERNAL("sg_lionk",         NES_SG_LIONK)
	SLOT_INTERFACE_INTERNAL("sg_boog",          NES_SG_BOOG)
	SLOT_INTERFACE_INTERNAL("kasing",           NES_KASING)
	SLOT_INTERFACE_INTERNAL("kay",              NES_KAY)
	SLOT_INTERFACE_INTERNAL("h2288",            NES_H2288)
	SLOT_INTERFACE_INTERNAL("unl_6035052",      NES_6035052) // mapper 238?
	SLOT_INTERFACE_INTERNAL("txc_tw",           NES_TXC_TW)
	SLOT_INTERFACE_INTERNAL("kof97",            NES_KOF97)
	SLOT_INTERFACE_INTERNAL("kof96",            NES_KOF96)
	SLOT_INTERFACE_INTERNAL("sfight3",          NES_SF3)
	SLOT_INTERFACE_INTERNAL("gouder",           NES_GOUDER)
	SLOT_INTERFACE_INTERNAL("sa9602b",          NES_SA9602B)
	SLOT_INTERFACE_INTERNAL("unl_shero",        NES_SACHEN_SHERO)
// misc multigame cart boards
	SLOT_INTERFACE_INTERNAL("benshieng",        NES_BENSHIENG)
	SLOT_INTERFACE_INTERNAL("action52",         NES_ACTION52)
	SLOT_INTERFACE_INTERNAL("caltron6in1",      NES_CALTRON6IN1)
	SLOT_INTERFACE_INTERNAL("maxi15",           NES_MAXI15)        //  mapper 234
	SLOT_INTERFACE_INTERNAL("rumblestation",    NES_RUMBLESTATION)    // mapper 46
	SLOT_INTERFACE_INTERNAL("svision16",        NES_SVISION16)  // mapper 53
	SLOT_INTERFACE_INTERNAL("n625092",          NES_N625092)
	SLOT_INTERFACE_INTERNAL("a65as",            NES_A65AS)
	SLOT_INTERFACE_INTERNAL("t262",             NES_T262)
	SLOT_INTERFACE_INTERNAL("novel1",           NES_NOVEL1)
	SLOT_INTERFACE_INTERNAL("novel2",           NES_NOVEL2) // mapper 213... same as BMC-NOVELDIAMOND9999999IN1 board?
	SLOT_INTERFACE_INTERNAL("studyngame",       NES_STUDYNGAME) // mapper 39
	SLOT_INTERFACE_INTERNAL("sgun20in1",        NES_SUPERGUN20IN1)
	SLOT_INTERFACE_INTERNAL("bmc_vt5201",       NES_VT5201) // mapper 60 otherwise
	SLOT_INTERFACE_INTERNAL("bmc_d1038",        NES_VT5201) // mapper 60?
	SLOT_INTERFACE_INTERNAL("810544c",          NES_810544C)
	SLOT_INTERFACE_INTERNAL("ntd03",            NES_NTD03)
	SLOT_INTERFACE_INTERNAL("bmc_gb63",         NES_BMC_GB63)
	SLOT_INTERFACE_INTERNAL("bmc_gka",          NES_BMC_GKA)
	SLOT_INTERFACE_INTERNAL("bmc_gkb",          NES_BMC_GKB)
	SLOT_INTERFACE_INTERNAL("bmc_ws",           NES_BMC_WS)
	SLOT_INTERFACE_INTERNAL("bmc_g146",         NES_BMC_G146)
	SLOT_INTERFACE_INTERNAL("bmc_11160",        NES_BMC_11160)
	SLOT_INTERFACE_INTERNAL("bmc_8157",         NES_BMC_8157)
	SLOT_INTERFACE_INTERNAL("bmc_hik300",       NES_BMC_HIK300)
	SLOT_INTERFACE_INTERNAL("bmc_s700",         NES_BMC_S700)
	SLOT_INTERFACE_INTERNAL("bmc_ball11",       NES_BMC_BALL11)
	SLOT_INTERFACE_INTERNAL("bmc_22games",      NES_BMC_22GAMES)
	SLOT_INTERFACE_INTERNAL("bmc_64y2k",        NES_BMC_64Y2K)
	SLOT_INTERFACE_INTERNAL("bmc_12in1",        NES_BMC_12IN1)
	SLOT_INTERFACE_INTERNAL("bmc_20in1",        NES_BMC_20IN1)
	SLOT_INTERFACE_INTERNAL("bmc_21in1",        NES_BMC_21IN1)
	SLOT_INTERFACE_INTERNAL("bmc_31in1",        NES_BMC_31IN1)
	SLOT_INTERFACE_INTERNAL("bmc_35in1",        NES_BMC_35IN1)
	SLOT_INTERFACE_INTERNAL("bmc_36in1",        NES_BMC_36IN1)
	SLOT_INTERFACE_INTERNAL("bmc_64in1",        NES_BMC_64IN1)
	SLOT_INTERFACE_INTERNAL("bmc_70in1",        NES_BMC_70IN1)   // mapper 236?
	SLOT_INTERFACE_INTERNAL("bmc_72in1",        NES_BMC_72IN1)
	SLOT_INTERFACE_INTERNAL("bmc_76in1",        NES_BMC_76IN1)
	SLOT_INTERFACE_INTERNAL("bmc_s42in1",       NES_BMC_76IN1)
	SLOT_INTERFACE_INTERNAL("bmc_110in1",       NES_BMC_110IN1)
	SLOT_INTERFACE_INTERNAL("bmc_150in1",       NES_BMC_150IN1)
	SLOT_INTERFACE_INTERNAL("bmc_190in1",       NES_BMC_190IN1)
	SLOT_INTERFACE_INTERNAL("bmc_800in1",       NES_BMC_800IN1)   // mapper 236?
	SLOT_INTERFACE_INTERNAL("bmc_1200in1",      NES_BMC_1200IN1)
	SLOT_INTERFACE_INTERNAL("bmc_gold150",      NES_BMC_GOLD150) // mapper 235 with 2M PRG
	SLOT_INTERFACE_INTERNAL("bmc_gold260",      NES_BMC_GOLD260) // mapper 235 with 4M PRG
	SLOT_INTERFACE_INTERNAL("bmc_power255",     NES_BMC_CH001)   // mapper 63?
	SLOT_INTERFACE_INTERNAL("bmc_s22games",     NES_BMC_SUPER22) // mapper 233
	SLOT_INTERFACE_INTERNAL("bmc_reset4",       NES_BMC_4IN1RESET) // mapper 60 with 64k prg and 32k chr
	SLOT_INTERFACE_INTERNAL("bmc_reset42",      NES_BMC_42IN1RESET)  // mapper 60? or 226? or 233?
// misc multigame cart MMC3 clone boards
	SLOT_INTERFACE_INTERNAL("fk23c",            NES_FK23C)
	SLOT_INTERFACE_INTERNAL("fk23ca",           NES_FK23CA)
	SLOT_INTERFACE_INTERNAL("s24in1c03",        NES_S24IN1SC03)
	SLOT_INTERFACE_INTERNAL("bmc_15in1",        NES_BMC_15IN1)
	SLOT_INTERFACE_INTERNAL("bmc_sbig7in1",     NES_BMC_SBIG7)
	SLOT_INTERFACE_INTERNAL("bmc_hik8in1",      NES_BMC_HIK8)
	SLOT_INTERFACE_INTERNAL("bmc_hik4in1",      NES_BMC_HIK4)
	SLOT_INTERFACE_INTERNAL("bmc_mario7in1",    NES_BMC_MARIO7IN1)
	SLOT_INTERFACE_INTERNAL("bmc_gold7in1",     NES_BMC_GOLD7IN1)
	SLOT_INTERFACE_INTERNAL("bmc_gc6in1",       NES_BMC_GC6IN1)
	SLOT_INTERFACE_INTERNAL("bmc_411120c",      NES_BMC_411120C)
	SLOT_INTERFACE_INTERNAL("bmc_830118c",      NES_BMC_830118C)
	SLOT_INTERFACE_INTERNAL("pjoy84",           NES_PJOY84)
	SLOT_INTERFACE_INTERNAL("nocash_nochr",     NES_NOCHR)
	SLOT_INTERFACE_INTERNAL("nes_action53",     NES_ACTION53)
	SLOT_INTERFACE_INTERNAL("nes_2a03pur",      NES_2A03PURITANS)
// other unsupported...
	SLOT_INTERFACE_INTERNAL("ninjaryu",         NES_NROM)    // mapper 111 - UNSUPPORTED
	SLOT_INTERFACE_INTERNAL("unl_dance",        NES_NROM)    // UNSUPPORTED
	SLOT_INTERFACE_INTERNAL("onebus",           NES_NROM)    // UNSUPPORTED
	SLOT_INTERFACE_INTERNAL("pec586",           NES_NROM)    // UNSUPPORTED
	SLOT_INTERFACE_INTERNAL("coolboy",          NES_NROM)    // UNSUPPORTED
	SLOT_INTERFACE_INTERNAL("bmc_f15",          NES_NROM)    // UNSUPPORTED
	SLOT_INTERFACE_INTERNAL("bmc_hp898f",       NES_NROM)    // UNSUPPORTED
	SLOT_INTERFACE_INTERNAL("bmc_8in1",         NES_NROM)    // UNSUPPORTED
	SLOT_INTERFACE_INTERNAL("unl_eh8813a",      NES_NROM)    // UNSUPPORTED
	SLOT_INTERFACE_INTERNAL("unl_158b",         NES_NROM)    // UNSUPPORTED
	SLOT_INTERFACE_INTERNAL("unl_drgnfgt",      NES_NROM)    // UNSUPPORTED
// are there dumps of games with these boards?
	SLOT_INTERFACE_INTERNAL("bmc_hik_kof",      NES_NROM) // mapper 251 - UNSUPPORTED
	SLOT_INTERFACE_INTERNAL("bmc_13in1jy110",   NES_NROM) //  [mentioned in FCEUMM source - we need more info] - UNSUPPORTED
	SLOT_INTERFACE_INTERNAL("bmc_gk_192",       NES_NROM) //  [mentioned in FCEUMM source - we need more info] - UNSUPPORTED
	SLOT_INTERFACE_INTERNAL("konami_qtai",      NES_NROM) //  [mentioned in FCEUMM source - we need more info] - UNSUPPORTED
	SLOT_INTERFACE_INTERNAL("unl_3d_block",     NES_NROM) //  [mentioned in FCEUMM source - we need more info] - UNSUPPORTED
	SLOT_INTERFACE_INTERNAL("unl_c_n22m",       NES_NROM) //  [mentioned in FCEUMM source - we need more info] - UNSUPPORTED
	SLOT_INTERFACE_INTERNAL("a9746",            NES_NROM) // mapper 219 - UNSUPPORTED (no dump available)
// legacy boards for FFE copier mappers (are there images available to fix/improve emulation?)
	SLOT_INTERFACE_INTERNAL("ffe3",             NES_FFE3)
	SLOT_INTERFACE_INTERNAL("ffe4",             NES_FFE4)
	SLOT_INTERFACE_INTERNAL("ffe8",             NES_FFE8)
	SLOT_INTERFACE_INTERNAL("test",             NES_NROM)
//
	SLOT_INTERFACE_INTERNAL("unknown",          NES_NROM)  //  a few pirate dumps uses the wrong mapper...
SLOT_INTERFACE_END

SLOT_INTERFACE_START(disksys_only)
	// RAM expansion + Disk System add-on
	SLOT_INTERFACE("disksys",                   NES_DISKSYS)
SLOT_INTERFACE_END
