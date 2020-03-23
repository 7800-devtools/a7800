// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    mamedasm.c

    Generic MAME disassembler.

****************************************************************************/

#include "emu.h"
#include "cpu/sparc/sparcdasm.h"

#include <algorithm>
#include <cstring>

#include <ctype.h>


enum display_type
{
	_8bit,
	_8bitx,
	_16be,
	_16le,
	_24be,
	_24le,
	_32be,
	_32le,
	_40be,
	_40le,
	_48be,
	_48le,
	_56be,
	_56le,
	_64be,
	_64le
};


struct dasm_table_entry
{
	const char *            name;
	display_type            display;
	int8_t                    pcshift;
	cpu_disassemble_func    func;
};


struct options
{
	const char *            filename;
	offs_t                  basepc;
	uint8_t                   norawbytes;
	uint8_t                   lower;
	uint8_t                   upper;
	uint8_t                   flipped;
	int                     mode;
	const dasm_table_entry *dasm;
	uint32_t                  skip;
	uint32_t                  count;
};


CPU_DISASSEMBLE( adsp21xx );
CPU_DISASSEMBLE( alpha8201 );
CPU_DISASSEMBLE( am29000 );
CPU_DISASSEMBLE( amis2000 );
CPU_DISASSEMBLE( apexc );
CPU_DISASSEMBLE( arcompact );
CPU_DISASSEMBLE( arm );
CPU_DISASSEMBLE( arm_be );
CPU_DISASSEMBLE( arm7arm );
CPU_DISASSEMBLE( arm7arm_be );
CPU_DISASSEMBLE( arm7thumb );
CPU_DISASSEMBLE( arm7thumb_be );
CPU_DISASSEMBLE( asap );
CPU_DISASSEMBLE( avr8 );
CPU_DISASSEMBLE( capricorn );
CPU_DISASSEMBLE( ccpu );
CPU_DISASSEMBLE( cdp1801 );
CPU_DISASSEMBLE( cdp1802 );
CPU_DISASSEMBLE( clipper );
CPU_DISASSEMBLE( coldfire );
CPU_DISASSEMBLE( cop410 );
CPU_DISASSEMBLE( cop420 );
CPU_DISASSEMBLE( cop444 );
CPU_DISASSEMBLE( cop424 );
CPU_DISASSEMBLE( cp1610 );
CPU_DISASSEMBLE( cquestlin );
CPU_DISASSEMBLE( cquestrot );
CPU_DISASSEMBLE( cquestsnd );
CPU_DISASSEMBLE( ds5002fp );
CPU_DISASSEMBLE( dsp16a );
CPU_DISASSEMBLE( dsp32c );
CPU_DISASSEMBLE( dsp56k );
CPU_DISASSEMBLE( e0c6200 );
CPU_DISASSEMBLE( esrip );
CPU_DISASSEMBLE( f8 );
CPU_DISASSEMBLE( g65816_generic );
CPU_DISASSEMBLE( h6280 );
CPU_DISASSEMBLE( hc11 );
CPU_DISASSEMBLE( hcd62121 );
CPU_DISASSEMBLE( hd61700 );
CPU_DISASSEMBLE( hd6301 );
CPU_DISASSEMBLE( hd6309 );
CPU_DISASSEMBLE( hd63701 );
CPU_DISASSEMBLE( hmcs40 );
CPU_DISASSEMBLE( hp_hybrid );
CPU_DISASSEMBLE( hp_5061_3001 );
CPU_DISASSEMBLE( hp_nanoprocessor );
CPU_DISASSEMBLE( hyperstone_generic );
CPU_DISASSEMBLE( i4004 );
CPU_DISASSEMBLE( i4040 );
CPU_DISASSEMBLE( i8008 );
CPU_DISASSEMBLE( i8051 );
CPU_DISASSEMBLE( i8052 );
CPU_DISASSEMBLE( i8085 );
CPU_DISASSEMBLE( i8089 );
CPU_DISASSEMBLE( i80c51 );
CPU_DISASSEMBLE( i80c52 );
CPU_DISASSEMBLE( i860 );
CPU_DISASSEMBLE( i960 );
CPU_DISASSEMBLE( ie15 );
CPU_DISASSEMBLE( jaguardsp );
CPU_DISASSEMBLE( jaguargpu );
CPU_DISASSEMBLE( konami );
CPU_DISASSEMBLE( lh5801 );
CPU_DISASSEMBLE( lr35902 );
CPU_DISASSEMBLE( m58846 );
CPU_DISASSEMBLE( m37710_generic );
CPU_DISASSEMBLE( m6800 );
CPU_DISASSEMBLE( m68000 );
CPU_DISASSEMBLE( m68008 );
CPU_DISASSEMBLE( m6801 );
CPU_DISASSEMBLE( m68010 );
CPU_DISASSEMBLE( m6802 );
CPU_DISASSEMBLE( m68020 );
CPU_DISASSEMBLE( m6803 );
CPU_DISASSEMBLE( m68030 );
CPU_DISASSEMBLE( m68040 );
CPU_DISASSEMBLE( m6805 );
CPU_DISASSEMBLE( m146805 );
CPU_DISASSEMBLE( m68hc05 );
CPU_DISASSEMBLE( m6808 );
CPU_DISASSEMBLE( m6809 );
CPU_DISASSEMBLE( m68340 );
CPU_DISASSEMBLE( mb86233 );
CPU_DISASSEMBLE( mb88 );
CPU_DISASSEMBLE( mcs48 );
CPU_DISASSEMBLE( minx );
CPU_DISASSEMBLE( mips3be );
CPU_DISASSEMBLE( mips3le );
CPU_DISASSEMBLE( mn10200 );
CPU_DISASSEMBLE( n8x300 );
CPU_DISASSEMBLE( nec );
CPU_DISASSEMBLE( nsc8105 );
CPU_DISASSEMBLE( pdp1 );
CPU_DISASSEMBLE( pdp8 );
CPU_DISASSEMBLE( pic16c5x );
CPU_DISASSEMBLE( pic16c62x );
CPU_DISASSEMBLE( powerpc );
CPU_DISASSEMBLE( pps4 );
CPU_DISASSEMBLE( psxcpu_generic );
CPU_DISASSEMBLE( r3000be );
CPU_DISASSEMBLE( r3000le );
CPU_DISASSEMBLE( rsp );
CPU_DISASSEMBLE( s2650 );
CPU_DISASSEMBLE( saturn );
CPU_DISASSEMBLE( sc61860 );
CPU_DISASSEMBLE( scmp );
CPU_DISASSEMBLE( scudsp );
CPU_DISASSEMBLE( se3208 );
CPU_DISASSEMBLE( sh2 );
CPU_DISASSEMBLE( sh4 );
CPU_DISASSEMBLE( sh4be );
CPU_DISASSEMBLE( sharc );
CPU_DISASSEMBLE( sm500 );
CPU_DISASSEMBLE( sm510 );
CPU_DISASSEMBLE( sm511 );
CPU_DISASSEMBLE( sm5a );
CPU_DISASSEMBLE( sm8500 );
CPU_DISASSEMBLE( spc700 );
CPU_DISASSEMBLE( ssem );
CPU_DISASSEMBLE( ssp1601 );
CPU_DISASSEMBLE( superfx );
CPU_DISASSEMBLE( t11 );
CPU_DISASSEMBLE( t90 );
CPU_DISASSEMBLE( tlcs900 );
CPU_DISASSEMBLE( tms0980 );
CPU_DISASSEMBLE( tms1000 );
CPU_DISASSEMBLE( tms1100 );
CPU_DISASSEMBLE( tms32010 );
CPU_DISASSEMBLE( tms32025 );
CPU_DISASSEMBLE( tms3203x );
CPU_DISASSEMBLE( tms32051 );
CPU_DISASSEMBLE( tms34010 );
CPU_DISASSEMBLE( tms34020 );
CPU_DISASSEMBLE( tms57002 );
CPU_DISASSEMBLE( tms7000 );
CPU_DISASSEMBLE( tms9900 );
CPU_DISASSEMBLE( tms9980 );
CPU_DISASSEMBLE( tms9995 );
CPU_DISASSEMBLE( tp0320 );
CPU_DISASSEMBLE( tx0_64kw );
CPU_DISASSEMBLE( tx0_8kw );
CPU_DISASSEMBLE( ucom4 );
CPU_DISASSEMBLE( unsp );
CPU_DISASSEMBLE( upd7725 );
CPU_DISASSEMBLE( upd7801 );
CPU_DISASSEMBLE( upd7807 );
CPU_DISASSEMBLE( upd7810 );
CPU_DISASSEMBLE( upd78c05 );
CPU_DISASSEMBLE( upi41 );
CPU_DISASSEMBLE( v60 );
CPU_DISASSEMBLE( v70 );
CPU_DISASSEMBLE( v810 );
CPU_DISASSEMBLE( x86_16 );
CPU_DISASSEMBLE( x86_32 );
CPU_DISASSEMBLE( x86_64 );
CPU_DISASSEMBLE( z180 );
CPU_DISASSEMBLE( z8 );
CPU_DISASSEMBLE( z80 );
CPU_DISASSEMBLE( z8000 );

CPU_DISASSEMBLE( sparcv7 )      { static sparc_disassembler dasm(nullptr, 7);                             return dasm.dasm(stream, pc, big_endianize_int32(*reinterpret_cast<const uint32_t *>(oprom))); }
CPU_DISASSEMBLE( sparcv8 )      { static sparc_disassembler dasm(nullptr, 8);                             return dasm.dasm(stream, pc, big_endianize_int32(*reinterpret_cast<const uint32_t *>(oprom))); }
CPU_DISASSEMBLE( sparcv9 )      { static sparc_disassembler dasm(nullptr, 9);                             return dasm.dasm(stream, pc, big_endianize_int32(*reinterpret_cast<const uint32_t *>(oprom))); }
CPU_DISASSEMBLE( sparcv9vis1 )  { static sparc_disassembler dasm(nullptr, 9, sparc_disassembler::vis_1);  return dasm.dasm(stream, pc, big_endianize_int32(*reinterpret_cast<const uint32_t *>(oprom))); }
CPU_DISASSEMBLE( sparcv9vis2 )  { static sparc_disassembler dasm(nullptr, 9, sparc_disassembler::vis_2);  return dasm.dasm(stream, pc, big_endianize_int32(*reinterpret_cast<const uint32_t *>(oprom))); }
CPU_DISASSEMBLE( sparcv9vis2p ) { static sparc_disassembler dasm(nullptr, 9, sparc_disassembler::vis_2p); return dasm.dasm(stream, pc, big_endianize_int32(*reinterpret_cast<const uint32_t *>(oprom))); }
CPU_DISASSEMBLE( sparcv9vis3 )  { static sparc_disassembler dasm(nullptr, 9, sparc_disassembler::vis_3);  return dasm.dasm(stream, pc, big_endianize_int32(*reinterpret_cast<const uint32_t *>(oprom))); }
CPU_DISASSEMBLE( sparcv9vis3b ) { static sparc_disassembler dasm(nullptr, 9, sparc_disassembler::vis_3b); return dasm.dasm(stream, pc, big_endianize_int32(*reinterpret_cast<const uint32_t *>(oprom))); }


static const dasm_table_entry dasm_table[] =
{
	{ "8x300",       _16be,  0, CPU_DISASSEMBLE_NAME(n8x300) },
	{ "adsp21xx",    _24le, -2, CPU_DISASSEMBLE_NAME(adsp21xx) },
	{ "alpha8201",   _8bit,  0, CPU_DISASSEMBLE_NAME(alpha8201) },
	{ "am29000",     _32be,  0, CPU_DISASSEMBLE_NAME(am29000) },
	{ "amis2000",    _8bit,  0, CPU_DISASSEMBLE_NAME(amis2000) },
	{ "apexc",       _32be,  0, CPU_DISASSEMBLE_NAME(apexc) },
	{ "arcompact",   _16le,  0, CPU_DISASSEMBLE_NAME(arcompact) },
	{ "arm",         _32le,  0, CPU_DISASSEMBLE_NAME(arm) },
	{ "arm_be",      _32be,  0, CPU_DISASSEMBLE_NAME(arm_be) },
	{ "arm7",        _32le,  0, CPU_DISASSEMBLE_NAME(arm7arm) },
	{ "arm7_be",     _32be,  0, CPU_DISASSEMBLE_NAME(arm7arm_be) },
	{ "arm7thumb",   _16le,  0, CPU_DISASSEMBLE_NAME(arm7thumb) },
	{ "arm7thumbb",  _16be,  0, CPU_DISASSEMBLE_NAME(arm7thumb_be) },
	{ "asap",        _32le,  0, CPU_DISASSEMBLE_NAME(asap) },
	{ "avr8",        _16le,  0, CPU_DISASSEMBLE_NAME(avr8) },
	{ "capricorn",   _8bit,  0, CPU_DISASSEMBLE_NAME(capricorn) },
	{ "ccpu",        _8bit,  0, CPU_DISASSEMBLE_NAME(ccpu) },
	{ "cdp1801",     _8bit,  0, CPU_DISASSEMBLE_NAME(cdp1801) },
	{ "cdp1802",     _8bit,  0, CPU_DISASSEMBLE_NAME(cdp1802) },
	{ "clipper",     _16le,  0, CPU_DISASSEMBLE_NAME(clipper) },
	{ "coldfire",    _16be,  0, CPU_DISASSEMBLE_NAME(coldfire) },
	{ "cop410",      _8bit,  0, CPU_DISASSEMBLE_NAME(cop410) },
	{ "cop420",      _8bit,  0, CPU_DISASSEMBLE_NAME(cop420) },
	{ "cop444",      _8bit,  0, CPU_DISASSEMBLE_NAME(cop444) },
	{ "cop424",      _8bit,  0, CPU_DISASSEMBLE_NAME(cop424) },
	{ "cp1610",      _16be, -1, CPU_DISASSEMBLE_NAME(cp1610) },
	{ "cquestlin",   _64be, -3, CPU_DISASSEMBLE_NAME(cquestlin) },
	{ "cquestrot",   _64be, -3, CPU_DISASSEMBLE_NAME(cquestrot) },
	{ "cquestsnd",   _64be, -3, CPU_DISASSEMBLE_NAME(cquestsnd) },
	{ "ds5002fp",    _8bit,  0, CPU_DISASSEMBLE_NAME(ds5002fp) },
	{ "dsp16a",      _16le, -1, CPU_DISASSEMBLE_NAME(dsp16a) },
	{ "dsp32c",      _32le,  0, CPU_DISASSEMBLE_NAME(dsp32c) },
	{ "dsp56k",      _16le, -1, CPU_DISASSEMBLE_NAME(dsp56k) },
	{ "e0c6200",     _16be, -1, CPU_DISASSEMBLE_NAME(e0c6200) },
	{ "esrip",       _64be,  0, CPU_DISASSEMBLE_NAME(esrip) },
	{ "f8",          _8bit,  0, CPU_DISASSEMBLE_NAME(f8) },
	{ "g65816",      _8bit,  0, CPU_DISASSEMBLE_NAME(g65816_generic) },
	{ "h6280",       _8bit,  0, CPU_DISASSEMBLE_NAME(h6280) },
//  { "h8",          _16be,  0, CPU_DISASSEMBLE_NAME(h8) },
//  { "h8_24",       _16be,  0, CPU_DISASSEMBLE_NAME(h8_24) },
//  { "h8_32",       _16be,  0, CPU_DISASSEMBLE_NAME(h8_32) },
	{ "hc11",        _8bit,  0, CPU_DISASSEMBLE_NAME(hc11) },
	{ "hcd62121",    _8bit,  0, CPU_DISASSEMBLE_NAME(hcd62121) },
	{ "hd61700",     _8bit,  0, CPU_DISASSEMBLE_NAME(hd61700) },
	{ "hd6301",      _8bit,  0, CPU_DISASSEMBLE_NAME(hd6301) },
	{ "hd6309",      _8bit,  0, CPU_DISASSEMBLE_NAME(hd6309) },
	{ "hd63701",     _8bit,  0, CPU_DISASSEMBLE_NAME(hd63701) },
	{ "hmcs40",      _16le, -1, CPU_DISASSEMBLE_NAME(hmcs40) },
	{ "hp_hybrid",   _16be, -1, CPU_DISASSEMBLE_NAME(hp_hybrid) },
	{ "hp_5061_3001",_16be, -1, CPU_DISASSEMBLE_NAME(hp_5061_3001) },
	{ "hyperstone",  _16be,  0, CPU_DISASSEMBLE_NAME(hyperstone_generic) },
	{ "i4004",       _8bit,  0, CPU_DISASSEMBLE_NAME(i4004) },
	{ "i4040",       _8bit,  0, CPU_DISASSEMBLE_NAME(i4040) },
	{ "i8008",       _8bit,  0, CPU_DISASSEMBLE_NAME(i8008) },
	{ "i8051",       _8bit,  0, CPU_DISASSEMBLE_NAME(i8051) },
	{ "i8052",       _8bit,  0, CPU_DISASSEMBLE_NAME(i8052) },
	{ "i8085",       _8bit,  0, CPU_DISASSEMBLE_NAME(i8085) },
	{ "i8089",       _8bit,  0, CPU_DISASSEMBLE_NAME(i8089) },
	{ "i80c51",      _8bit,  0, CPU_DISASSEMBLE_NAME(i80c51) },
	{ "i80c52",      _8bit,  0, CPU_DISASSEMBLE_NAME(i80c52) },
	{ "i860",        _64le,  0, CPU_DISASSEMBLE_NAME(i860) },
	{ "i960",        _32le,  0, CPU_DISASSEMBLE_NAME(i960) },
	{ "ie15",        _8bit,  0, CPU_DISASSEMBLE_NAME(ie15) },
	{ "jaguardsp",   _16be,  0, CPU_DISASSEMBLE_NAME(jaguardsp) },
	{ "jaguargpu",   _16be,  0, CPU_DISASSEMBLE_NAME(jaguargpu) },
	{ "konami",      _8bit,  0, CPU_DISASSEMBLE_NAME(konami) },
	{ "lh5801",      _8bit,  0, CPU_DISASSEMBLE_NAME(lh5801) },
	{ "lr35902",     _8bit,  0, CPU_DISASSEMBLE_NAME(lr35902) },
	{ "m58846",      _16le, -1, CPU_DISASSEMBLE_NAME(m58846) },
	{ "m37710",      _8bit,  0, CPU_DISASSEMBLE_NAME(m37710_generic) },
	{ "m6800",       _8bit,  0, CPU_DISASSEMBLE_NAME(m6800) },
	{ "m68000",      _16be,  0, CPU_DISASSEMBLE_NAME(m68000) },
	{ "m68008",      _16be,  0, CPU_DISASSEMBLE_NAME(m68008) },
	{ "m6801",       _8bit,  0, CPU_DISASSEMBLE_NAME(m6801) },
	{ "m68010",      _16be,  0, CPU_DISASSEMBLE_NAME(m68010) },
	{ "m6802",       _8bit,  0, CPU_DISASSEMBLE_NAME(m6802) },
	{ "m68020",      _16be,  0, CPU_DISASSEMBLE_NAME(m68020) },
	{ "m6803",       _8bit,  0, CPU_DISASSEMBLE_NAME(m6803) },
	{ "m68030",      _16be,  0, CPU_DISASSEMBLE_NAME(m68030) },
	{ "m68040",      _16be,  0, CPU_DISASSEMBLE_NAME(m68040) },
	{ "m6805",       _8bit,  0, CPU_DISASSEMBLE_NAME(m6805) },
	{ "m146805",     _8bit,  0, CPU_DISASSEMBLE_NAME(m146805) },
	{ "m68hc05",     _8bit,  0, CPU_DISASSEMBLE_NAME(m68hc05) },
	{ "m6808",       _8bit,  0, CPU_DISASSEMBLE_NAME(m6808) },
	{ "m6809",       _8bit,  0, CPU_DISASSEMBLE_NAME(m6809) },
	{ "m68340",      _16be,  0, CPU_DISASSEMBLE_NAME(m68340) },
	{ "mb86233",     _32le, -2, CPU_DISASSEMBLE_NAME(mb86233) },
	{ "mb88",        _8bit,  0, CPU_DISASSEMBLE_NAME(mb88) },
	{ "mcs48",       _8bit,  0, CPU_DISASSEMBLE_NAME(mcs48) },
	{ "minx",        _8bit,  0, CPU_DISASSEMBLE_NAME(minx) },
	{ "mips3be",     _32be,  0, CPU_DISASSEMBLE_NAME(mips3be) },
	{ "mips3le",     _32le,  0, CPU_DISASSEMBLE_NAME(mips3le) },
	{ "mn10200",     _16le,  0, CPU_DISASSEMBLE_NAME(mn10200) },
	{ "nanoprocessor",_8bit, 0, CPU_DISASSEMBLE_NAME(hp_nanoprocessor) },
	{ "nec",         _8bit,  0, CPU_DISASSEMBLE_NAME(nec) },
	{ "nsc8105",     _8bit,  0, CPU_DISASSEMBLE_NAME(nsc8105) },
	{ "pdp1",        _32be,  0, CPU_DISASSEMBLE_NAME(pdp1) },
	{ "pdp8",        _16be,  0, CPU_DISASSEMBLE_NAME(pdp8) },
	{ "pic16c5x",    _16le, -1, CPU_DISASSEMBLE_NAME(pic16c5x) },
	{ "pic16c62x",   _16le, -1, CPU_DISASSEMBLE_NAME(pic16c62x) },
	{ "powerpc",     _32be,  0, CPU_DISASSEMBLE_NAME(powerpc) },
	{ "pps4",        _8bit,  0, CPU_DISASSEMBLE_NAME(pps4) },
	{ "psxcpu",      _32le,  0, CPU_DISASSEMBLE_NAME(psxcpu_generic) },
	{ "r3000be",     _32be,  0, CPU_DISASSEMBLE_NAME(r3000be) },
	{ "r3000le",     _32le,  0, CPU_DISASSEMBLE_NAME(r3000le) },
	{ "rsp",         _32le,  0, CPU_DISASSEMBLE_NAME(rsp) },
	{ "s2650",       _8bit,  0, CPU_DISASSEMBLE_NAME(s2650) },
	{ "saturn",      _8bit,  0, CPU_DISASSEMBLE_NAME(saturn) },
	{ "sc61860",     _8bit,  0, CPU_DISASSEMBLE_NAME(sc61860) },
	{ "scmp",        _8bit,  0, CPU_DISASSEMBLE_NAME(scmp) },
	{ "scudsp",      _32be,  0, CPU_DISASSEMBLE_NAME(scudsp) },
	{ "se3208",      _16le,  0, CPU_DISASSEMBLE_NAME(se3208) },
	{ "sh2",         _16be,  0, CPU_DISASSEMBLE_NAME(sh2) },
	{ "sh4",         _16le,  0, CPU_DISASSEMBLE_NAME(sh4) },
	{ "sh4be",       _16be,  0, CPU_DISASSEMBLE_NAME(sh4be) },
	{ "sharc",       _48le, -2, CPU_DISASSEMBLE_NAME(sharc) },
	{ "sm500",       _8bit,  0, CPU_DISASSEMBLE_NAME(sm500) },
	{ "sm510",       _8bit,  0, CPU_DISASSEMBLE_NAME(sm510) },
	{ "sm511",       _8bit,  0, CPU_DISASSEMBLE_NAME(sm511) },
	{ "sm5a",        _8bit,  0, CPU_DISASSEMBLE_NAME(sm5a) },
	{ "sm8500",      _8bit,  0, CPU_DISASSEMBLE_NAME(sm8500) },
	{ "sparcv7",     _32be,  0, CPU_DISASSEMBLE_NAME(sparcv7) },
	{ "sparcv8",     _32be,  0, CPU_DISASSEMBLE_NAME(sparcv8) },
	{ "sparcv9",     _32be,  0, CPU_DISASSEMBLE_NAME(sparcv9) },
	{ "sparcv9vis1", _32be,  0, CPU_DISASSEMBLE_NAME(sparcv9vis1) },
	{ "sparcv9vis2", _32be,  0, CPU_DISASSEMBLE_NAME(sparcv9vis2) },
	{ "sparcv9vis2p",_32be,  0, CPU_DISASSEMBLE_NAME(sparcv9vis2p) },
	{ "sparcv9vis3", _32be,  0, CPU_DISASSEMBLE_NAME(sparcv9vis3) },
	{ "sparcv9vis3b",_32be,  0, CPU_DISASSEMBLE_NAME(sparcv9vis3b) },
	{ "spc700",      _8bit,  0, CPU_DISASSEMBLE_NAME(spc700) },
	{ "ssem",        _32le,  0, CPU_DISASSEMBLE_NAME(ssem) },
	{ "ssp1601",     _16be, -1, CPU_DISASSEMBLE_NAME(ssp1601) },
//  { "superfx",     _8bit,  0, CPU_DISASSEMBLE_NAME(superfx) },
	{ "t11",         _16le,  0, CPU_DISASSEMBLE_NAME(t11) },
//  { "t90",         _8bit,  0, CPU_DISASSEMBLE_NAME(t90) },
	{ "tlcs900",     _8bit,  0, CPU_DISASSEMBLE_NAME(tlcs900) },
	{ "tms0980",     _16be,  0, CPU_DISASSEMBLE_NAME(tms0980) },
	{ "tms1000",     _8bit,  0, CPU_DISASSEMBLE_NAME(tms1000) },
	{ "tms1100",     _8bit,  0, CPU_DISASSEMBLE_NAME(tms1100) },
	{ "tms32010",    _16be, -1, CPU_DISASSEMBLE_NAME(tms32010) },
	{ "tms32025",    _16be, -1, CPU_DISASSEMBLE_NAME(tms32025) },
	{ "tms3203x",    _32le, -2, CPU_DISASSEMBLE_NAME(tms3203x) },
	{ "tms32051",    _16le, -1, CPU_DISASSEMBLE_NAME(tms32051) },
	{ "tms34010",    _8bit,  3, CPU_DISASSEMBLE_NAME(tms34010) },
	{ "tms34020",    _8bit,  3, CPU_DISASSEMBLE_NAME(tms34020) },
	{ "tms57002",    _32le, -2, CPU_DISASSEMBLE_NAME(tms57002) },
	{ "tms7000",     _8bit,  0, CPU_DISASSEMBLE_NAME(tms7000) },
	{ "tms9900",     _16be,  0, CPU_DISASSEMBLE_NAME(tms9900) },
	{ "tms9980",     _8bit,  0, CPU_DISASSEMBLE_NAME(tms9980) },
	{ "tms9995",     _8bit,  0, CPU_DISASSEMBLE_NAME(tms9995) },
	{ "tp0320",      _16be,  0, CPU_DISASSEMBLE_NAME(tp0320) },
	{ "tx0_64kw",    _32be, -2, CPU_DISASSEMBLE_NAME(tx0_64kw) },
	{ "tx0_8kw",     _32be, -2, CPU_DISASSEMBLE_NAME(tx0_8kw) },
	{ "ucom4",       _8bit,  0, CPU_DISASSEMBLE_NAME(ucom4) },
	{ "unsp",        _16be,  0, CPU_DISASSEMBLE_NAME(unsp) },
	{ "upd7725",     _32be,  0, CPU_DISASSEMBLE_NAME(upd7725) },
	{ "upd7801",     _8bit,  0, CPU_DISASSEMBLE_NAME(upd7801) },
	{ "upd7807",     _8bit,  0, CPU_DISASSEMBLE_NAME(upd7807) },
	{ "upd7810",     _8bit,  0, CPU_DISASSEMBLE_NAME(upd7810) },
	{ "upd78c05",    _8bit,  0, CPU_DISASSEMBLE_NAME(upd78c05) },
	{ "upi41",       _8bit,  0, CPU_DISASSEMBLE_NAME(upi41) },
	{ "v60",         _8bit,  0, CPU_DISASSEMBLE_NAME(v60) },
	{ "v70",         _8bit,  0, CPU_DISASSEMBLE_NAME(v70) },
	{ "v810",        _16le,  0, CPU_DISASSEMBLE_NAME(v810) },
	{ "x86_16",      _8bit,  0, CPU_DISASSEMBLE_NAME(x86_16) },
	{ "x86_32",      _8bit,  0, CPU_DISASSEMBLE_NAME(x86_32) },
	{ "x86_64",      _8bit,  0, CPU_DISASSEMBLE_NAME(x86_64) },
	{ "z180",        _8bit,  0, CPU_DISASSEMBLE_NAME(z180) },
	{ "z8",          _8bit,  0, CPU_DISASSEMBLE_NAME(z8) },
	{ "z80",         _8bit,  0, CPU_DISASSEMBLE_NAME(z80) },
//  { "z8000",       _16be,  0, CPU_DISASSEMBLE_NAME(z8000) },
};


static int parse_options(int argc, char *argv[], options *opts)
{
	bool pending_base = false;
	bool pending_arch = false;
	bool pending_mode = false;
	bool pending_skip = false;
	bool pending_count = false;

	memset(opts, 0, sizeof(*opts));

	// loop through arguments
	for (unsigned arg = 1; arg < argc; arg++)
	{
		char *curarg = argv[arg];

		// is it a switch?
		if (curarg[0] == '-')
		{
			if (pending_base || pending_arch || pending_mode || pending_skip || pending_count)
				goto usage;

			if (tolower((uint8_t)curarg[1]) == 'a')
				pending_arch = true;
			else if (tolower((uint8_t)curarg[1]) == 'b')
				pending_base = true;
			else if (tolower((uint8_t)curarg[1]) == 'f')
				opts->flipped = true;
			else if (tolower((uint8_t)curarg[1]) == 'l')
				opts->lower = true;
			else if (tolower((uint8_t)curarg[1]) == 'm')
				pending_mode = true;
			else if (tolower((uint8_t)curarg[1]) == 's')
				pending_skip = true;
			else if (tolower((uint8_t)curarg[1]) == 'c')
				pending_count = true;
			else if (tolower((uint8_t)curarg[1]) == 'n')
				opts->norawbytes = true;
			else if (tolower((uint8_t)curarg[1]) == 'u')
				opts->upper = true;
			else
				goto usage;
		}

		// base PC
		else if (pending_base)
		{
			int result;
			if (curarg[0] == '0' && curarg[1] == 'x')
				result = sscanf(&curarg[2], "%x", &opts->basepc);
			else if (curarg[0] == '$')
				result = sscanf(&curarg[1], "%x", &opts->basepc);
			else
				result = sscanf(&curarg[0], "%x", &opts->basepc);
			if (result != 1)
				goto usage;
			pending_base = false;
		}

		// mode
		else if (pending_mode)
		{
			if (sscanf(curarg, "%d", &opts->mode) != 1)
				goto usage;
			pending_mode = false;
		}

		// architecture
		else if (pending_arch)
		{
			int curarch;
			for (curarch = 0; curarch < ARRAY_LENGTH(dasm_table); curarch++)
				if (core_stricmp(curarg, dasm_table[curarch].name) == 0)
					break;
			if (curarch == ARRAY_LENGTH(dasm_table))
				goto usage;
			opts->dasm = &dasm_table[curarch];
			pending_arch = false;
		}

		// skip bytes
		else if (pending_skip)
		{
			if (sscanf(curarg, "%d", &opts->skip) != 1)
				goto usage;
			pending_skip = false;
		}

		// size
		else if (pending_count)
		{
			if (sscanf(curarg, "%d", &opts->count) != 1)
				goto usage;
			pending_count = false;
		}

		// filename
		else if (opts->filename == nullptr)
			opts->filename = curarg;

		// fail
		else
			goto usage;
	}

	// if we have a dangling option, error
	if (pending_base || pending_arch || pending_mode || pending_skip || pending_count)
		goto usage;

	// if no file or no architecture, fail
	if (opts->filename == nullptr || opts->dasm == nullptr)
		goto usage;
	return 0;

usage:
	printf("Usage: %s <filename> -arch <architecture> [-basepc <pc>] \n", argv[0]);
	printf("   [-mode <n>] [-norawbytes] [-flipped] [-upper] [-lower]\n");
	printf("   [-skip <n>] [-count <n>]\n");
	printf("\n");
	printf("Supported architectures:");
	const int colwidth = 1 + std::strlen(std::max_element(std::begin(dasm_table), std::end(dasm_table), [](const dasm_table_entry &a, const dasm_table_entry &b) { return std::strlen(a.name) < std::strlen(b.name); })->name);
	const int columns = std::max(1, 80 / colwidth);
	const int numrows = (ARRAY_LENGTH(dasm_table) + columns - 1) / columns;
	for (unsigned curarch = 0; curarch < numrows * columns; curarch++)
	{
		const int row = curarch / columns;
		const int col = curarch % columns;
		const int index = col * numrows + row;
		if (col == 0)
			printf("\n  ");
		printf("%-*s", colwidth, (index < ARRAY_LENGTH(dasm_table)) ? dasm_table[index].name : "");
	}
	printf("\n");
	return 1;
};


int main(int argc, char *argv[])
{
	osd_file::error filerr;
	int displayendian;
	int displaychunk;
	uint32_t curbyte;
	uint32_t length;
	int maxchunks;
	uint32_t curpc;
	options opts;
	int numbytes;
	void *data;
	int result = 0;

	// parse options first
	if (parse_options(argc, argv, &opts))
		return 1;

	// load the file
	filerr = util::core_file::load(opts.filename, &data, length);
	if (filerr != osd_file::error::NONE)
	{
		fprintf(stderr, "Error opening file '%s'\n", opts.filename);
		return 1;
	}

	// precompute parameters
	displaychunk = (opts.dasm->display / 2) + 1;
	displayendian = opts.dasm->display % 2;
	switch (displaychunk)
	{
		case 1:     maxchunks = 6;  break;
		case 2:     maxchunks = 3;  break;
		default:    maxchunks = 1;  break;
	}

	// run it
	try
	{
		if (length > opts.skip)
			length = length - opts.skip;
		if ((length > opts.count) && (opts.count != 0))
			length = opts.count;
		curpc = opts.basepc;

		std::stringstream stream;
		for (curbyte = 0; curbyte < length; curbyte += numbytes)
		{
			uint8_t *oprom = (uint8_t *)data + opts.skip + curbyte;
			uint32_t pcdelta;
			int numchunks;

			// disassemble
			stream.str("");
			pcdelta = (*opts.dasm->func)(nullptr, stream, curpc, oprom, oprom, opts.mode) & DASMFLAG_LENGTHMASK;
			std::string buffer = stream.str();

			if (opts.dasm->pcshift < 0)
				numbytes = pcdelta << -opts.dasm->pcshift;
			else
				numbytes = pcdelta >> opts.dasm->pcshift;

			// force upper or lower
			if (opts.lower)
			{
				std::transform(
					std::begin(buffer),
					std::end(buffer),
					std::begin(buffer),
					[](char c) { return tolower(c); });
			}
			else if (opts.upper)
			{
				std::transform(
					std::begin(buffer),
					std::end(buffer),
					std::begin(buffer),
					[](char c) { return toupper(c); });
			}

			// round to the nearest display chunk
			numbytes = ((numbytes + displaychunk - 1) / displaychunk) * displaychunk;
			if (numbytes == 0)
				numbytes = displaychunk;
			numchunks = numbytes / displaychunk;

			// non-flipped case
			if (!opts.flipped)
			{
				// output the address
				printf("%08X: ", curpc);

				// output the raw bytes
				if (!opts.norawbytes)
				{
					int firstchunks = (numchunks < maxchunks) ? numchunks : maxchunks;
					int chunknum, bytenum;
					for (chunknum = 0; chunknum < firstchunks; chunknum++)
					{
						for (bytenum = 0; bytenum < displaychunk; bytenum++)
							printf("%02X", oprom[displayendian ? (displaychunk - 1 - bytenum) : bytenum]);
						printf(" ");
						oprom += displaychunk;
					}
					for ( ; chunknum < maxchunks; chunknum++)
						printf("%*s ", displaychunk * 2, "");
					printf(" ");
				}

				// output the disassembly
				printf("%s\n", buffer.c_str());

				// output additional raw bytes
				if (!opts.norawbytes && numchunks > maxchunks)
				{
					for (numchunks -= maxchunks; numchunks > 0; numchunks -= maxchunks)
					{
						int firstchunks = (numchunks < maxchunks) ? numchunks : maxchunks;
						int chunknum, bytenum;
						printf("          ");
						for (chunknum = 0; chunknum < firstchunks; chunknum++)
						{
							for (bytenum = 0; bytenum < displaychunk; bytenum++)
								printf("%02X", oprom[displayendian ? (displaychunk - 1 - bytenum) : bytenum]);
							printf(" ");
							oprom += displaychunk;
						}
						printf("\n");
					}
				}
			}

			// flipped case
			else
			{
				// output the disassembly and address
				printf("\t%-40s ; %08X", buffer.c_str(), curpc);

				// output the raw bytes
				if (!opts.norawbytes)
				{
					int chunknum, bytenum;
					printf(": ");
					for (chunknum = 0; chunknum < numchunks; chunknum++)
					{
						for (bytenum = 0; bytenum < displaychunk; bytenum++)
							printf("%02X", oprom[displayendian ? (displaychunk - 1 - bytenum) : bytenum]);
						printf(" ");
						oprom += displaychunk;
					}
				}
				printf("\n");
			}

			// advance
			curpc += pcdelta;
		}
	}
	catch (emu_fatalerror &fatal)
	{
		fprintf(stderr, "%s\n", fatal.string());
		result = 1;
		if (fatal.exitcode() != 0)
			result = fatal.exitcode();
	}
	catch (emu_exception &)
	{
		fprintf(stderr, "Caught unhandled emulator exception\n");
		result = 1;
	}
	catch (tag_add_exception &aex)
	{
		fprintf(stderr, "Tag '%s' already exists in tagged map\n", aex.tag());
		result = 1;
	}
	catch (std::exception &ex)
	{
		fprintf(stderr, "Caught unhandled %s exception: %s\n", typeid(ex).name(), ex.what());
		result = 1;
	}
	catch (...)
	{
		fprintf(stderr, "Caught unhandled exception\n");
		result = 1;
	}

	free(data);

	return result;
}
