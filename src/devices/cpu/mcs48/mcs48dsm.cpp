// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    mcs48dsm.c

    Simple MCS-48/UPI-41 disassembler.
    Written by Aaron Giles

***************************************************************************/

#include "emu.h"


static uint32_t common_dasm(device_t *device, std::ostream &stream, offs_t pc, const uint8_t *oprom, const uint8_t *opram, bool upi41)
{
	const uint8_t *startram = opram;
	uint32_t flags = 0;

	opram++;
	switch (*oprom++)
	{
		case 0x00:      util::stream_format(stream, "nop");                                                 break;
		case 0x02:  if (!upi41)
						util::stream_format(stream, "out  bus,a");
					else
						util::stream_format(stream, "out  dbb,a");
			break;
		case 0x03:      util::stream_format(stream, "add  a,#$%02X", *opram++);                             break;
		case 0x04:      util::stream_format(stream, "jmp  $0%02X", *opram++);                               break;
		case 0x05:      util::stream_format(stream, "en   i");                                              break;
		case 0x07:      util::stream_format(stream, "dec  a");                                              break;
		case 0x08:  if (!upi41)
						util::stream_format(stream, "in   a,bus");
					else
						util::stream_format(stream, "illegal");
			break;
		case 0x09:      util::stream_format(stream, "in   a,p1");                                           break;
		case 0x0a:      util::stream_format(stream, "in   a,p2");                                           break;
		case 0x0c:      util::stream_format(stream, "movd a,p4");                                           break;
		case 0x0d:      util::stream_format(stream, "movd a,p5");                                           break;
		case 0x0e:      util::stream_format(stream, "movd a,p6");                                           break;
		case 0x0f:      util::stream_format(stream, "movd a,p7");                                           break;
		case 0x10:      util::stream_format(stream, "inc  @r0");                                            break;
		case 0x11:      util::stream_format(stream, "inc  @r1");                                            break;
		case 0x12:      util::stream_format(stream, "jb0  $%03X", (pc & 0xf00) | *opram++);                 break;
		case 0x13:      util::stream_format(stream, "addc a,#$%02X", *opram++);                             break;
		case 0x14:      util::stream_format(stream, "call $0%02X", *opram++); flags = DASMFLAG_STEP_OVER;   break;
		case 0x15:      util::stream_format(stream, "dis  i");                                              break;
		case 0x16:      util::stream_format(stream, "jtf  $%03X", (pc & 0xf00) | *opram++);                 break;
		case 0x17:      util::stream_format(stream, "inc  a");                                              break;
		case 0x18:      util::stream_format(stream, "inc  r0");                                             break;
		case 0x19:      util::stream_format(stream, "inc  r1");                                             break;
		case 0x1a:      util::stream_format(stream, "inc  r2");                                             break;
		case 0x1b:      util::stream_format(stream, "inc  r3");                                             break;
		case 0x1c:      util::stream_format(stream, "inc  r4");                                             break;
		case 0x1d:      util::stream_format(stream, "inc  r5");                                             break;
		case 0x1e:      util::stream_format(stream, "inc  r6");                                             break;
		case 0x1f:      util::stream_format(stream, "inc  r7");                                             break;
		case 0x20:      util::stream_format(stream, "xch  a,@r0");                                          break;
		case 0x21:      util::stream_format(stream, "xch  a,@r1");                                          break;
		case 0x22:  if (!upi41)
						util::stream_format(stream, "illegal");
					else
						util::stream_format(stream, "in   a,dbb");
			break;
		case 0x23:      util::stream_format(stream, "mov  a,#$%02X", *opram++);                             break;
		case 0x24:      util::stream_format(stream, "jmp  $1%02X", *opram++);                               break;
		case 0x25:      util::stream_format(stream, "en   tcnti");                                          break;
		case 0x26:      util::stream_format(stream, "jnt0 $%03X", (pc & 0xf00) | *opram++);                 break;
		case 0x27:      util::stream_format(stream, "clr  a");                                              break;
		case 0x28:      util::stream_format(stream, "xch  a,r0");                                           break;
		case 0x29:      util::stream_format(stream, "xch  a,r1");                                           break;
		case 0x2a:      util::stream_format(stream, "xch  a,r2");                                           break;
		case 0x2b:      util::stream_format(stream, "xch  a,r3");                                           break;
		case 0x2c:      util::stream_format(stream, "xch  a,r4");                                           break;
		case 0x2d:      util::stream_format(stream, "xch  a,r5");                                           break;
		case 0x2e:      util::stream_format(stream, "xch  a,r6");                                           break;
		case 0x2f:      util::stream_format(stream, "xch  a,r7");                                           break;
		case 0x30:      util::stream_format(stream, "xchd a,@r0");                                          break;
		case 0x31:      util::stream_format(stream, "xchd a,@r1");                                          break;
		case 0x32:      util::stream_format(stream, "jb1  $%03X", (pc & 0xf00) | *opram++);                 break;
		case 0x34:      util::stream_format(stream, "call $1%02X", *opram++); flags = DASMFLAG_STEP_OVER;   break;
		case 0x35:      util::stream_format(stream, "dis  tcnti");                                          break;
		case 0x36:      util::stream_format(stream, "jt0  $%03X", (pc & 0xf00) | *opram++);                 break;
		case 0x37:      util::stream_format(stream, "cpl  a");                                              break;
		case 0x39:      util::stream_format(stream, "outl p1,a");                                           break;
		case 0x3a:      util::stream_format(stream, "outl p2,a");                                           break;
		case 0x3c:      util::stream_format(stream, "movd p4,a");                                           break;
		case 0x3d:      util::stream_format(stream, "movd p5,a");                                           break;
		case 0x3e:      util::stream_format(stream, "movd p6,a");                                           break;
		case 0x3f:      util::stream_format(stream, "movd p7,a");                                           break;
		case 0x40:      util::stream_format(stream, "orl  a,@r0");                                          break;
		case 0x41:      util::stream_format(stream, "orl  a,@r1");                                          break;
		case 0x42:      util::stream_format(stream, "mov  a,t");                                            break;
		case 0x43:      util::stream_format(stream, "orl  a,#$%02X", *opram++);                             break;
		case 0x44:      util::stream_format(stream, "jmp  $2%02X", *opram++);                               break;
		case 0x45:      util::stream_format(stream, "strt cnt");                                            break;
		case 0x46:      util::stream_format(stream, "jnt1 $%03X", (pc & 0xf00) | *opram++);                 break;
		case 0x47:      util::stream_format(stream, "swap a");                                              break;
		case 0x48:      util::stream_format(stream, "orl  a,r0");                                           break;
		case 0x49:      util::stream_format(stream, "orl  a,r1");                                           break;
		case 0x4a:      util::stream_format(stream, "orl  a,r2");                                           break;
		case 0x4b:      util::stream_format(stream, "orl  a,r3");                                           break;
		case 0x4c:      util::stream_format(stream, "orl  a,r4");                                           break;
		case 0x4d:      util::stream_format(stream, "orl  a,r5");                                           break;
		case 0x4e:      util::stream_format(stream, "orl  a,r6");                                           break;
		case 0x4f:      util::stream_format(stream, "orl  a,r7");                                           break;
		case 0x50:      util::stream_format(stream, "anl  a,@r0");                                          break;
		case 0x51:      util::stream_format(stream, "anl  a,@r1");                                          break;
		case 0x52:      util::stream_format(stream, "jb2  $%03X", (pc & 0xf00) | *opram++);                 break;
		case 0x53:      util::stream_format(stream, "anl  a,#$%02X", *opram++);                             break;
		case 0x54:      util::stream_format(stream, "call $2%02X", *opram++); flags = DASMFLAG_STEP_OVER;   break;
		case 0x55:      util::stream_format(stream, "strt t");                                              break;
		case 0x56:      util::stream_format(stream, "jt1  $%03X", (pc & 0xf00) | *opram++);                 break;
		case 0x57:      util::stream_format(stream, "da   a");                                              break;
		case 0x58:      util::stream_format(stream, "anl  a,r0");                                           break;
		case 0x59:      util::stream_format(stream, "anl  a,r1");                                           break;
		case 0x5a:      util::stream_format(stream, "anl  a,r2");                                           break;
		case 0x5b:      util::stream_format(stream, "anl  a,r3");                                           break;
		case 0x5c:      util::stream_format(stream, "anl  a,r4");                                           break;
		case 0x5d:      util::stream_format(stream, "anl  a,r5");                                           break;
		case 0x5e:      util::stream_format(stream, "anl  a,r6");                                           break;
		case 0x5f:      util::stream_format(stream, "anl  a,r7");                                           break;
		case 0x60:      util::stream_format(stream, "add  a,@r0");                                          break;
		case 0x61:      util::stream_format(stream, "add  a,@r1");                                          break;
		case 0x62:      util::stream_format(stream, "mov  t,a");                                            break;
		case 0x64:      util::stream_format(stream, "jmp  $3%02X", *opram++);                               break;
		case 0x65:      util::stream_format(stream, "stop tcnt");                                           break;
		case 0x67:      util::stream_format(stream, "rrc  a");                                              break;
		case 0x68:      util::stream_format(stream, "add  a,r0");                                           break;
		case 0x69:      util::stream_format(stream, "add  a,r1");                                           break;
		case 0x6a:      util::stream_format(stream, "add  a,r2");                                           break;
		case 0x6b:      util::stream_format(stream, "add  a,r3");                                           break;
		case 0x6c:      util::stream_format(stream, "add  a,r4");                                           break;
		case 0x6d:      util::stream_format(stream, "add  a,r5");                                           break;
		case 0x6e:      util::stream_format(stream, "add  a,r6");                                           break;
		case 0x6f:      util::stream_format(stream, "add  a,r7");                                           break;
		case 0x70:      util::stream_format(stream, "addc a,@r0");                                          break;
		case 0x71:      util::stream_format(stream, "addc a,@r1");                                          break;
		case 0x72:      util::stream_format(stream, "jb3  $%03X", (pc & 0xf00) | *opram++);                 break;
		case 0x74:      util::stream_format(stream, "call $3%02X", *opram++); flags = DASMFLAG_STEP_OVER;   break;
		case 0x75:  if (!upi41)
						util::stream_format(stream, "ent0 clk");
					else
						util::stream_format(stream, "illegal");
			break;
		case 0x76:      util::stream_format(stream, "jf1  $%03X", (pc & 0xf00) | *opram++);                 break;
		case 0x77:      util::stream_format(stream, "rr   a");                                              break;
		case 0x78:      util::stream_format(stream, "addc a,r0");                                           break;
		case 0x79:      util::stream_format(stream, "addc a,r1");                                           break;
		case 0x7a:      util::stream_format(stream, "addc a,r2");                                           break;
		case 0x7b:      util::stream_format(stream, "addc a,r3");                                           break;
		case 0x7c:      util::stream_format(stream, "addc a,r4");                                           break;
		case 0x7d:      util::stream_format(stream, "addc a,r5");                                           break;
		case 0x7e:      util::stream_format(stream, "addc a,r6");                                           break;
		case 0x7f:      util::stream_format(stream, "addc a,r7");                                           break;
		case 0x80:  if (!upi41)
						util::stream_format(stream, "movx a,@r0");
					else
						util::stream_format(stream, "illegal");
			break;
		case 0x81:  if (!upi41)
						util::stream_format(stream, "movx a,@r1");
					else
						util::stream_format(stream, "illegal");
			break;
		case 0x83:      util::stream_format(stream, "ret"); flags = DASMFLAG_STEP_OUT;                      break;
		case 0x84:      util::stream_format(stream, "jmp  $4%02X", *opram++);                               break;
		case 0x85:      util::stream_format(stream, "clr  f0");                                             break;
		case 0x86:  if (!upi41)
						util::stream_format(stream, "jni  $%03X", (pc & 0xf00) | *opram++);
					else
						util::stream_format(stream, "jobf $%03X", (pc & 0xf00) | *opram++);
			break;
		case 0x88:  if (!upi41)
						util::stream_format(stream, "orl  bus,#$%02X", *opram++);
					else
						util::stream_format(stream, "illegal");
			break;
		case 0x89:      util::stream_format(stream, "orl  p1,#$%02X", *opram++);                            break;
		case 0x8a:      util::stream_format(stream, "orl  p2,#$%02X", *opram++);                            break;
		case 0x8c:      util::stream_format(stream, "orld p4,a");                                           break;
		case 0x8d:      util::stream_format(stream, "orld p5,a");                                           break;
		case 0x8e:      util::stream_format(stream, "orld p6,a");                                           break;
		case 0x8f:      util::stream_format(stream, "orld p7,a");                                           break;
		case 0x90:  if (!upi41)
						util::stream_format(stream, "movx @r0,a");
					else
						util::stream_format(stream, "mov  sts,a");
			break;
		case 0x91:  if (!upi41)
						util::stream_format(stream, "movx @r1,a");
					else
						util::stream_format(stream, "illegal");
			break;
		case 0x92:      util::stream_format(stream, "jb4  $%03X", (pc & 0xf00) | *opram++);                 break;
		case 0x93:      util::stream_format(stream, "retr"); flags = DASMFLAG_STEP_OUT;                     break;
		case 0x94:      util::stream_format(stream, "call $4%02X", *opram++); flags = DASMFLAG_STEP_OVER;   break;
		case 0x95:      util::stream_format(stream, "cpl  f0");                                             break;
		case 0x96:      util::stream_format(stream, "jnz  $%03X", (pc & 0xf00) | *opram++);                 break;
		case 0x97:      util::stream_format(stream, "clr  c");                                              break;
		case 0x98:  if (!upi41)
						util::stream_format(stream, "anl  bus,#$%02X", *opram++);
					else
						util::stream_format(stream, "illegal");
			break;
		case 0x99:      util::stream_format(stream, "anl  p1,#$%02X", *opram++);                            break;
		case 0x9a:      util::stream_format(stream, "anl  p2,#$%02X", *opram++);                            break;
		case 0x9c:      util::stream_format(stream, "anld p4,a");                                           break;
		case 0x9d:      util::stream_format(stream, "anld p5,a");                                           break;
		case 0x9e:      util::stream_format(stream, "anld p6,a");                                           break;
		case 0x9f:      util::stream_format(stream, "anld p7,a");                                           break;
		case 0xa0:      util::stream_format(stream, "mov  @r0,a");                                          break;
		case 0xa1:      util::stream_format(stream, "mov  @r1,a");                                          break;
		case 0xa3:      util::stream_format(stream, "movp a,@a");                                           break;
		case 0xa4:      util::stream_format(stream, "jmp  $5%02X", *opram++);                               break;
		case 0xa5:      util::stream_format(stream, "clr  f1");                                             break;
		case 0xa7:      util::stream_format(stream, "cpl  c");                                              break;
		case 0xa8:      util::stream_format(stream, "mov  r0,a");                                           break;
		case 0xa9:      util::stream_format(stream, "mov  r1,a");                                           break;
		case 0xaa:      util::stream_format(stream, "mov  r2,a");                                           break;
		case 0xab:      util::stream_format(stream, "mov  r3,a");                                           break;
		case 0xac:      util::stream_format(stream, "mov  r4,a");                                           break;
		case 0xad:      util::stream_format(stream, "mov  r5,a");                                           break;
		case 0xae:      util::stream_format(stream, "mov  r6,a");                                           break;
		case 0xaf:      util::stream_format(stream, "mov  r7,a");                                           break;
		case 0xb0:      util::stream_format(stream, "mov  @r0,#$%02X", *opram++);                           break;
		case 0xb1:      util::stream_format(stream, "mov  @r1,#$%02X", *opram++);                           break;
		case 0xb2:      util::stream_format(stream, "jb5  $%03X", (pc & 0xf00) | *opram++);                 break;
		case 0xb3:      util::stream_format(stream, "jmpp @a");                                             break;
		case 0xb4:      util::stream_format(stream, "call $5%02X", *opram++); flags = DASMFLAG_STEP_OVER;   break;
		case 0xb5:      util::stream_format(stream, "cpl  f1");                                             break;
		case 0xb6:      util::stream_format(stream, "jf0  $%03X", (pc & 0xf00) | *opram++);                 break;
		case 0xb8:      util::stream_format(stream, "mov  r0,#$%02X", *opram++);                            break;
		case 0xb9:      util::stream_format(stream, "mov  r1,#$%02X", *opram++);                            break;
		case 0xba:      util::stream_format(stream, "mov  r2,#$%02X", *opram++);                            break;
		case 0xbb:      util::stream_format(stream, "mov  r3,#$%02X", *opram++);                            break;
		case 0xbc:      util::stream_format(stream, "mov  r4,#$%02X", *opram++);                            break;
		case 0xbd:      util::stream_format(stream, "mov  r5,#$%02X", *opram++);                            break;
		case 0xbe:      util::stream_format(stream, "mov  r6,#$%02X", *opram++);                            break;
		case 0xbf:      util::stream_format(stream, "mov  r7,#$%02X", *opram++);                            break;
		case 0xc4:      util::stream_format(stream, "jmp  $6%02X", *opram++);                               break;
		case 0xc5:      util::stream_format(stream, "sel  rb0");                                            break;
		case 0xc6:      util::stream_format(stream, "jz   $%03X", (pc & 0xf00) | *opram++);                 break;
		case 0xc7:      util::stream_format(stream, "mov  a,psw");                                          break;
		case 0xc8:      util::stream_format(stream, "dec  r0");                                             break;
		case 0xc9:      util::stream_format(stream, "dec  r1");                                             break;
		case 0xca:      util::stream_format(stream, "dec  r2");                                             break;
		case 0xcb:      util::stream_format(stream, "dec  r3");                                             break;
		case 0xcc:      util::stream_format(stream, "dec  r4");                                             break;
		case 0xcd:      util::stream_format(stream, "dec  r5");                                             break;
		case 0xce:      util::stream_format(stream, "dec  r6");                                             break;
		case 0xcf:      util::stream_format(stream, "dec  r7");                                             break;
		case 0xd0:      util::stream_format(stream, "xrl  a,@r0");                                          break;
		case 0xd1:      util::stream_format(stream, "xrl  a,@r1");                                          break;
		case 0xd2:      util::stream_format(stream, "jb6  $%03X", (pc & 0xf00) | *opram++);                 break;
		case 0xd3:      util::stream_format(stream, "xrl  a,#$%02X", *opram++);                             break;
		case 0xd4:      util::stream_format(stream, "call $6%02X", *opram++); flags = DASMFLAG_STEP_OVER;   break;
		case 0xd5:      util::stream_format(stream, "sel  rb1");                                            break;
		case 0xd6:  if (!upi41)
						util::stream_format(stream, "illegal");
					else
						util::stream_format(stream, "jnibf $%03X", (pc & 0xf00) | *opram++);
			break;
		case 0xd7:      util::stream_format(stream, "mov  psw,a");                                          break;
		case 0xd8:      util::stream_format(stream, "xrl  a,r0");                                           break;
		case 0xd9:      util::stream_format(stream, "xrl  a,r1");                                           break;
		case 0xda:      util::stream_format(stream, "xrl  a,r2");                                           break;
		case 0xdb:      util::stream_format(stream, "xrl  a,r3");                                           break;
		case 0xdc:      util::stream_format(stream, "xrl  a,r4");                                           break;
		case 0xdd:      util::stream_format(stream, "xrl  a,r5");                                           break;
		case 0xde:      util::stream_format(stream, "xrl  a,r6");                                           break;
		case 0xdf:      util::stream_format(stream, "xrl  a,r7");                                           break;
		case 0xe3:      util::stream_format(stream, "movp3 a,@a");                                          break;
		case 0xe4:      util::stream_format(stream, "jmp  $7%02X", *opram++);                               break;
		case 0xe5:  if (!upi41)
						util::stream_format(stream, "sel  mb0");
					else
						util::stream_format(stream, "en   dma");
			break;
		case 0xe6:      util::stream_format(stream, "jnc  $%03X", (pc & 0xf00) | *opram++);                 break;
		case 0xe7:      util::stream_format(stream, "rl   a");                                              break;
		case 0xe8:      util::stream_format(stream, "djnz r0,$%03X", (pc & 0xf00) | *opram++); flags = DASMFLAG_STEP_OVER; break;
		case 0xe9:      util::stream_format(stream, "djnz r1,$%03X", (pc & 0xf00) | *opram++); flags = DASMFLAG_STEP_OVER; break;
		case 0xea:      util::stream_format(stream, "djnz r2,$%03X", (pc & 0xf00) | *opram++); flags = DASMFLAG_STEP_OVER; break;
		case 0xeb:      util::stream_format(stream, "djnz r3,$%03X", (pc & 0xf00) | *opram++); flags = DASMFLAG_STEP_OVER; break;
		case 0xec:      util::stream_format(stream, "djnz r4,$%03X", (pc & 0xf00) | *opram++); flags = DASMFLAG_STEP_OVER; break;
		case 0xed:      util::stream_format(stream, "djnz r5,$%03X", (pc & 0xf00) | *opram++); flags = DASMFLAG_STEP_OVER; break;
		case 0xee:      util::stream_format(stream, "djnz r6,$%03X", (pc & 0xf00) | *opram++); flags = DASMFLAG_STEP_OVER; break;
		case 0xef:      util::stream_format(stream, "djnz r7,$%03X", (pc & 0xf00) | *opram++); flags = DASMFLAG_STEP_OVER; break;
		case 0xf0:      util::stream_format(stream, "mov  a,@r0");                                          break;
		case 0xf1:      util::stream_format(stream, "mov  a,@r1");                                          break;
		case 0xf2:      util::stream_format(stream, "jb7  $%03X", (pc & 0xf00) | *opram++);                 break;
		case 0xf4:      util::stream_format(stream, "call $7%02X", *opram++); flags = DASMFLAG_STEP_OVER;   break;
		case 0xf5:  if (!upi41)
						util::stream_format(stream, "sel  mb1");
					else
						util::stream_format(stream, "en   flags");
			break;
		case 0xf6:      util::stream_format(stream, "jc   $%03X", (pc & 0xf00) | *opram++);                 break;
		case 0xf7:      util::stream_format(stream, "rlc  a");                                              break;
		case 0xf8:      util::stream_format(stream, "mov  a,r0");                                           break;
		case 0xf9:      util::stream_format(stream, "mov  a,r1");                                           break;
		case 0xfa:      util::stream_format(stream, "mov  a,r2");                                           break;
		case 0xfb:      util::stream_format(stream, "mov  a,r3");                                           break;
		case 0xfc:      util::stream_format(stream, "mov  a,r4");                                           break;
		case 0xfd:      util::stream_format(stream, "mov  a,r5");                                           break;
		case 0xfe:      util::stream_format(stream, "mov  a,r6");                                           break;
		case 0xff:      util::stream_format(stream, "mov  a,r7");                                           break;
		default:        util::stream_format(stream, "illegal");                                             break;
	}

	return (opram - startram) | flags | DASMFLAG_SUPPORTED;
}


CPU_DISASSEMBLE( mcs48 )
{
	return common_dasm(device, stream, pc, oprom, opram, false);
}


CPU_DISASSEMBLE( upi41 )
{
	return common_dasm(device, stream, pc, oprom, opram, true);
}
