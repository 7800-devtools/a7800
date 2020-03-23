// license:BSD-3-Clause
// copyright-holders:Ville Linde, Barry Rodewald, Carl, Philip Bennett
#pragma once

#ifndef __I386_H__
#define __I386_H__

#include "i386.h"

//#define DEBUG_MISSING_OPCODE

#define I386OP(XX)      i386_##XX
#define I486OP(XX)      i486_##XX
#define PENTIUMOP(XX)   pentium_##XX
#define MMXOP(XX)       mmx_##XX
#define SSEOP(XX)       sse_##XX

extern int i386_dasm_one(std::ostream &stream, uint32_t pc, const uint8_t *oprom, int mode);

enum SREGS { ES, CS, SS, DS, FS, GS };

enum BREGS
{
	AL = NATIVE_ENDIAN_VALUE_LE_BE(0,3),
	AH = NATIVE_ENDIAN_VALUE_LE_BE(1,2),
	CL = NATIVE_ENDIAN_VALUE_LE_BE(4,7),
	CH = NATIVE_ENDIAN_VALUE_LE_BE(5,6),
	DL = NATIVE_ENDIAN_VALUE_LE_BE(8,11),
	DH = NATIVE_ENDIAN_VALUE_LE_BE(9,10),
	BL = NATIVE_ENDIAN_VALUE_LE_BE(12,15),
	BH = NATIVE_ENDIAN_VALUE_LE_BE(13,14)
};

enum WREGS
{
	AX = NATIVE_ENDIAN_VALUE_LE_BE(0,1),
	CX = NATIVE_ENDIAN_VALUE_LE_BE(2,3),
	DX = NATIVE_ENDIAN_VALUE_LE_BE(4,5),
	BX = NATIVE_ENDIAN_VALUE_LE_BE(6,7),
	SP = NATIVE_ENDIAN_VALUE_LE_BE(8,9),
	BP = NATIVE_ENDIAN_VALUE_LE_BE(10,11),
	SI = NATIVE_ENDIAN_VALUE_LE_BE(12,13),
	DI = NATIVE_ENDIAN_VALUE_LE_BE(14,15)
};

enum DREGS { EAX, ECX, EDX, EBX, ESP, EBP, ESI, EDI };

enum
{
	I386_PC = 0,

	/* 8-bit registers */
	I386_AL,
	I386_AH,
	I386_BL,
	I386_BH,
	I386_CL,
	I386_CH,
	I386_DL,
	I386_DH,

	/* 16-bit registers */
	I386_AX,
	I386_BX,
	I386_CX,
	I386_DX,
	I386_BP,
	I386_SP,
	I386_SI,
	I386_DI,
	I386_IP,

	/* 32-bit registers */
	I386_EAX,
	I386_ECX,
	I386_EDX,
	I386_EBX,
	I386_EBP,
	I386_ESP,
	I386_ESI,
	I386_EDI,
	I386_EIP,

	/* segment registers */
	I386_CS,
	I386_CS_BASE,
	I386_CS_LIMIT,
	I386_CS_FLAGS,
	I386_SS,
	I386_SS_BASE,
	I386_SS_LIMIT,
	I386_SS_FLAGS,
	I386_DS,
	I386_DS_BASE,
	I386_DS_LIMIT,
	I386_DS_FLAGS,
	I386_ES,
	I386_ES_BASE,
	I386_ES_LIMIT,
	I386_ES_FLAGS,
	I386_FS,
	I386_FS_BASE,
	I386_FS_LIMIT,
	I386_FS_FLAGS,
	I386_GS,
	I386_GS_BASE,
	I386_GS_LIMIT,
	I386_GS_FLAGS,

	/* other */
	I386_EFLAGS,

	I386_CR0,
	I386_CR1,
	I386_CR2,
	I386_CR3,
	I386_CR4,

	I386_DR0,
	I386_DR1,
	I386_DR2,
	I386_DR3,
	I386_DR4,
	I386_DR5,
	I386_DR6,
	I386_DR7,

	I386_TR6,
	I386_TR7,

	I386_GDTR_BASE,
	I386_GDTR_LIMIT,
	I386_IDTR_BASE,
	I386_IDTR_LIMIT,
	I386_TR,
	I386_TR_BASE,
	I386_TR_LIMIT,
	I386_TR_FLAGS,
	I386_LDTR,
	I386_LDTR_BASE,
	I386_LDTR_LIMIT,
	I386_LDTR_FLAGS,

	I386_CPL,

	X87_CTRL,
	X87_STATUS,
	X87_TAG,
	X87_ST0,
	X87_ST1,
	X87_ST2,
	X87_ST3,
	X87_ST4,
	X87_ST5,
	X87_ST6,
	X87_ST7,

	SSE_XMM0,
	SSE_XMM1,
	SSE_XMM2,
	SSE_XMM3,
	SSE_XMM4,
	SSE_XMM5,
	SSE_XMM6,
	SSE_XMM7
};

enum
{
	/* mmx registers aliased to x87 ones */
	MMX_MM0=X87_ST0,
	MMX_MM1=X87_ST1,
	MMX_MM2=X87_ST2,
	MMX_MM3=X87_ST3,
	MMX_MM4=X87_ST4,
	MMX_MM5=X87_ST5,
	MMX_MM6=X87_ST6,
	MMX_MM7=X87_ST7
};

enum smram
{
	SMRAM_SMBASE = 0xF8,
	SMRAM_SMREV  = 0xFC,
	SMRAM_IORSRT = 0x100,
	SMRAM_AHALT  = 0x102,
	SMRAM_IOEDI  = 0x104,
	SMRAM_IOECX  = 0x108,
	SMRAM_IOESI  = 0x10C,

	SMRAM_ES     = 0x1A8,
	SMRAM_CS     = 0x1AC,
	SMRAM_SS     = 0x1B0,
	SMRAM_DS     = 0x1B4,
	SMRAM_FS     = 0x1B8,
	SMRAM_GS     = 0x1BC,
	SMRAM_LDTR   = 0x1C0,
	SMRAM_TR     = 0x1C4,
	SMRAM_DR7    = 0x1C8,
	SMRAM_DR6    = 0x1CC,
	SMRAM_EAX    = 0x1D0,
	SMRAM_ECX    = 0x1D4,
	SMRAM_EDX    = 0x1D8,
	SMRAM_EBX    = 0x1DC,
	SMRAM_ESP    = 0x1E0,
	SMRAM_EBP    = 0x1E4,
	SMRAM_ESI    = 0x1E8,
	SMRAM_EDI    = 0x1EC,
	SMRAM_EIP    = 0x1F0,
	SMRAM_EFLAGS = 0x1F4,
	SMRAM_CR3    = 0x1F8,
	SMRAM_CR0    = 0x1FC
};

enum smram_intel_p5
{
	SMRAM_IP5_IOEIP   = 0x110,
	SMRAM_IP5_CR4     = 0x128,
	SMRAM_IP5_ESLIM   = 0x130,
	SMRAM_IP5_ESBASE  = 0x134,
	SMRAM_IP5_ESACC   = 0x138,
	SMRAM_IP5_CSLIM   = 0x13C,
	SMRAM_IP5_CSBASE  = 0x140,
	SMRAM_IP5_CSACC   = 0x144,
	SMRAM_IP5_SSLIM   = 0x148,
	SMRAM_IP5_SSBASE  = 0x14C,
	SMRAM_IP5_SSACC   = 0x150,
	SMRAM_IP5_DSLIM   = 0x154,
	SMRAM_IP5_DSBASE  = 0x158,
	SMRAM_IP5_DSACC   = 0x15C,
	SMRAM_IP5_FSLIM   = 0x160,
	SMRAM_IP5_FSBASE  = 0x164,
	SMRAM_IP5_FSACC   = 0x168,
	SMRAM_IP5_GSLIM   = 0x16C,
	SMRAM_IP5_GSBASE  = 0x170,
	SMRAM_IP5_GSACC   = 0x174,
	SMRAM_IP5_LDTLIM  = 0x178,
	SMRAM_IP5_LDTBASE = 0x17C,
	SMRAM_IP5_LDTACC  = 0x180,
	SMRAM_IP5_GDTLIM  = 0x184,
	SMRAM_IP5_GDTBASE = 0x188,
	SMRAM_IP5_GDTACC  = 0x18C,
	SMRAM_IP5_IDTLIM  = 0x190,
	SMRAM_IP5_IDTBASE = 0x194,
	SMRAM_IP5_IDTACC  = 0x198,
	SMRAM_IP5_TRLIM   = 0x19C,
	SMRAM_IP5_TRBASE  = 0x1A0,
	SMRAM_IP5_TRACC   = 0x1A4
};

/* Protected mode exceptions */
#define FAULT_UD 6   // Invalid Opcode
#define FAULT_NM 7   // Coprocessor not available
#define FAULT_DF 8   // Double Fault
#define FAULT_TS 10  // Invalid TSS
#define FAULT_NP 11  // Segment or Gate not present
#define FAULT_SS 12  // Stack fault
#define FAULT_GP 13  // General Protection Fault
#define FAULT_PF 14  // Page Fault
#define FAULT_MF 16  // Match (Coprocessor) Fault

/* MXCSR Control and Status Register */
#define MXCSR_IE  (1<<0)  // Invalid Operation Flag
#define MXCSR_DE  (1<<1)  // Denormal Flag
#define MXCSR_ZE  (1<<2)  // Divide-by-Zero Flag
#define MXCSR_OE  (1<<3)  // Overflow Flag
#define MXCSR_UE  (1<<4)  // Underflow Flag
#define MXCSR_PE  (1<<5)  // Precision Flag
#define MXCSR_DAZ (1<<6)  // Denormals Are Zeros
#define MXCSR_IM  (1<<7)  // Invalid Operation Mask
#define MXCSR_DM  (1<<8)  // Denormal Operation Mask
#define MXCSR_ZM  (1<<9)  // Divide-by-Zero Mask
#define MXCSR_OM  (1<<10) // Overflow Mask
#define MXCSR_UM  (1<<11) // Underflow Mask
#define MXCSR_PM  (1<<12) // Precision Mask
#define MXCSR_RC  (3<<13) // Rounding Control
#define MXCSR_FZ  (1<<15) // Flush to Zero

union MMX_REG {
	uint32_t d[2];
	int32_t  i[2];
	uint16_t w[4];
	int16_t  s[4];
	uint8_t  b[8];
	int8_t   c[8];
	float  f[2];
	uint64_t q;
	int64_t  l;
};

extern int i386_parity_table[256];

#define FAULT_THROW(fault,error) { throw (uint64_t)(fault | (uint64_t)error << 32); }
#define PF_THROW(error) { m_cr[2] = address; FAULT_THROW(FAULT_PF,error); }

#define PROTECTED_MODE      (m_cr[0] & 0x1)
#define STACK_32BIT         (m_sreg[SS].d)
#define V8086_MODE          (m_VM)
#define NESTED_TASK         (m_NT)
#define WP                  (m_cr[0] & 0x10000)

#define SetOF_Add32(r,s,d)  (m_OF = (((r) ^ (s)) & ((r) ^ (d)) & 0x80000000) ? 1: 0)
#define SetOF_Add16(r,s,d)  (m_OF = (((r) ^ (s)) & ((r) ^ (d)) & 0x8000) ? 1 : 0)
#define SetOF_Add8(r,s,d)   (m_OF = (((r) ^ (s)) & ((r) ^ (d)) & 0x80) ? 1 : 0)

#define SetOF_Sub32(r,s,d)  (m_OF = (((d) ^ (s)) & ((d) ^ (r)) & 0x80000000) ? 1 : 0)
#define SetOF_Sub16(r,s,d)  (m_OF = (((d) ^ (s)) & ((d) ^ (r)) & 0x8000) ? 1 : 0)
#define SetOF_Sub8(r,s,d)   (m_OF = (((d) ^ (s)) & ((d) ^ (r)) & 0x80) ? 1 : 0)

#define SetCF8(x)           {m_CF = ((x) & 0x100) ? 1 : 0; }
#define SetCF16(x)          {m_CF = ((x) & 0x10000) ? 1 : 0; }
#define SetCF32(x)          {m_CF = ((x) & (((uint64_t)1) << 32)) ? 1 : 0; }

#define SetSF(x)            (m_SF = (x))
#define SetZF(x)            (m_ZF = (x))
#define SetAF(x,y,z)        (m_AF = (((x) ^ ((y) ^ (z))) & 0x10) ? 1 : 0)
#define SetPF(x)            (m_PF = i386_parity_table[(x) & 0xFF])

#define SetSZPF8(x)         {m_ZF = ((uint8_t)(x)==0);  m_SF = ((x)&0x80) ? 1 : 0; m_PF = i386_parity_table[x & 0xFF]; }
#define SetSZPF16(x)        {m_ZF = ((uint16_t)(x)==0);  m_SF = ((x)&0x8000) ? 1 : 0; m_PF = i386_parity_table[x & 0xFF]; }
#define SetSZPF32(x)        {m_ZF = ((uint32_t)(x)==0);  m_SF = ((x)&0x80000000) ? 1 : 0; m_PF = i386_parity_table[x & 0xFF]; }

#define MMX(n)              (*((MMX_REG *)(&m_x87_reg[(n)].low)))
#define XMM(n)              m_sse_reg[(n)]

/***********************************************************************************/

struct MODRM_TABLE {
	struct {
		int b;
		int w;
		int d;
	} reg;
	struct {
		int b;
		int w;
		int d;
	} rm;
};

extern MODRM_TABLE i386_MODRM_table[256];

#define REG8(x)         (m_reg.b[x])
#define REG16(x)        (m_reg.w[x])
#define REG32(x)        (m_reg.d[x])

#define LOAD_REG8(x)    (REG8(i386_MODRM_table[x].reg.b))
#define LOAD_REG16(x)   (REG16(i386_MODRM_table[x].reg.w))
#define LOAD_REG32(x)   (REG32(i386_MODRM_table[x].reg.d))
#define LOAD_RM8(x)     (REG8(i386_MODRM_table[x].rm.b))
#define LOAD_RM16(x)    (REG16(i386_MODRM_table[x].rm.w))
#define LOAD_RM32(x)    (REG32(i386_MODRM_table[x].rm.d))

#define STORE_REG8(x, value)    (REG8(i386_MODRM_table[x].reg.b) = value)
#define STORE_REG16(x, value)   (REG16(i386_MODRM_table[x].reg.w) = value)
#define STORE_REG32(x, value)   (REG32(i386_MODRM_table[x].reg.d) = value)
#define STORE_RM8(x, value)     (REG8(i386_MODRM_table[x].rm.b) = value)
#define STORE_RM16(x, value)    (REG16(i386_MODRM_table[x].rm.w) = value)
#define STORE_RM32(x, value)    (REG32(i386_MODRM_table[x].rm.d) = value)

#define SWITCH_ENDIAN_32(x) (((((x) << 24) & (0xff << 24)) | (((x) << 8) & (0xff << 16)) | (((x) >> 8) & (0xff << 8)) | (((x) >> 24) & (0xff << 0))))

/***********************************************************************************/

uint32_t i386_device::i386_translate(int segment, uint32_t ip, int rwn)
{
	// TODO: segment limit access size, execution permission, handle exception thrown from exception handler
	if(PROTECTED_MODE && !V8086_MODE && (rwn != -1))
	{
		if(!(m_sreg[segment].valid))
			FAULT_THROW((segment==SS)?FAULT_SS:FAULT_GP, 0);
		if(i386_limit_check(segment, ip))
			FAULT_THROW((segment==SS)?FAULT_SS:FAULT_GP, 0);
		if((rwn == 0) && ((m_sreg[segment].flags & 8) && !(m_sreg[segment].flags & 2)))
			FAULT_THROW(FAULT_GP, 0);
		if((rwn == 1) && ((m_sreg[segment].flags & 8) || !(m_sreg[segment].flags & 2)))
			FAULT_THROW(FAULT_GP, 0);
	}
	return m_sreg[segment].base + ip;
}

#define VTLB_FLAG_DIRTY 0x100

vtlb_entry i386_device::get_permissions(uint32_t pte, int wp)
{
	vtlb_entry ret = VTLB_READ_ALLOWED | ((pte & 4) ? VTLB_USER_READ_ALLOWED : 0);
	if(!wp)
		ret |= VTLB_WRITE_ALLOWED;
	if(pte & 2)
		ret |= VTLB_WRITE_ALLOWED | ((pte & 4) ? VTLB_USER_WRITE_ALLOWED : 0);
	return ret;
}

bool i386_device::i386_translate_address(int intention, offs_t *address, vtlb_entry *entry)
{
	uint32_t a = *address;
	uint32_t pdbr = m_cr[3] & 0xfffff000;
	uint32_t directory = (a >> 22) & 0x3ff;
	uint32_t table = (a >> 12) & 0x3ff;
	vtlb_entry perm = 0;
	bool ret;
	bool user = (intention & TRANSLATE_USER_MASK) ? true : false;
	bool write = (intention & TRANSLATE_WRITE) ? true : false;
	bool debug = (intention & TRANSLATE_DEBUG_MASK) ? true : false;

	if(!(m_cr[0] & 0x80000000))
	{
		if(entry)
			*entry = 0x77;
		return true;
	}

	uint32_t page_dir = m_program->read_dword(pdbr + directory * 4);
	if(page_dir & 1)
	{
		if ((page_dir & 0x80) && (m_cr[4] & 0x10))
		{
			a = (page_dir & 0xffc00000) | (a & 0x003fffff);
			if(debug)
			{
				*address = a;
				return true;
			}
			perm = get_permissions(page_dir, WP);
			if(write && (!(perm & VTLB_WRITE_ALLOWED) || (user && !(perm & VTLB_USER_WRITE_ALLOWED))))
				ret = false;
			else if(user && !(perm & VTLB_USER_READ_ALLOWED))
				ret = false;
			else
			{
				if(write)
					perm |= VTLB_FLAG_DIRTY;
				if(!(page_dir & 0x40) && write)
					m_program->write_dword(pdbr + directory * 4, page_dir | 0x60);
				else if(!(page_dir & 0x20))
					m_program->write_dword(pdbr + directory * 4, page_dir | 0x20);
				ret = true;
			}
		}
		else
		{
			uint32_t page_entry = m_program->read_dword((page_dir & 0xfffff000) + (table * 4));
			if(!(page_entry & 1))
				ret = false;
			else
			{
				a = (page_entry & 0xfffff000) | (a & 0xfff);
				if(debug)
				{
					*address = a;
					return true;
				}
				perm = get_permissions(page_entry, WP);
				if(write && (!(perm & VTLB_WRITE_ALLOWED) || (user && !(perm & VTLB_USER_WRITE_ALLOWED))))
					ret = false;
				else if(user && !(perm & VTLB_USER_READ_ALLOWED))
					ret = false;
				else
				{
					if(write)
						perm |= VTLB_FLAG_DIRTY;
					if(!(page_dir & 0x20))
						m_program->write_dword(pdbr + directory * 4, page_dir | 0x20);
					if(!(page_entry & 0x40) && write)
						m_program->write_dword((page_dir & 0xfffff000) + (table * 4), page_entry | 0x60);
					else if(!(page_entry & 0x20))
						m_program->write_dword((page_dir & 0xfffff000) + (table * 4), page_entry | 0x20);
					ret = true;
				}
			}
		}
	}
	else
		ret = false;
	if(entry)
		*entry = perm;
	if(ret)
		*address = a;
	return ret;
}

//#define TEST_TLB

bool i386_device::translate_address(int pl, int type, uint32_t *address, uint32_t *error)
{
	if(!(m_cr[0] & 0x80000000)) // Some (very few) old OS's won't work with this
		return true;

	const vtlb_entry *table = vtlb_table();
	uint32_t index = *address >> 12;
	vtlb_entry entry = table[index];
	if(type == TRANSLATE_FETCH)
		type = TRANSLATE_READ;
	if(pl == 3)
		type |= TRANSLATE_USER_MASK;
#ifdef TEST_TLB
	uint32_t test_addr = *address;
#endif

	if(!(entry & VTLB_FLAG_VALID) || ((type & TRANSLATE_WRITE) && !(entry & VTLB_FLAG_DIRTY)))
	{
		if(!i386_translate_address(type, address, &entry))
		{
			*error = ((type & TRANSLATE_WRITE) ? 2 : 0) | ((m_CPL == 3) ? 4 : 0);
			if(entry)
				*error |= 1;
			return false;
		}
		vtlb_dynload(index, *address, entry);
		return true;
	}
	if(!(entry & (1 << type)))
	{
		*error = ((type & TRANSLATE_WRITE) ? 2 : 0) | ((m_CPL == 3) ? 4 : 0) | 1;
		return false;
	}
	*address = (entry & 0xfffff000) | (*address & 0xfff);
#ifdef TEST_TLB
	int test_ret = i386_translate_address(type | TRANSLATE_DEBUG_MASK, &test_addr, nullptr);
	if(!test_ret || (test_addr != *address))
		logerror("TLB-PTE mismatch! %06X %06X %06x\n", *address, test_addr, m_pc);
#endif
	return true;
}

void i386_device::CHANGE_PC(uint32_t pc)
{
	m_pc = i386_translate(CS, pc, -1 );
}

void i386_device::NEAR_BRANCH(int32_t offs)
{
	/* TODO: limit */
	m_eip += offs;
	m_pc += offs;
}

uint8_t i386_device::FETCH()
{
	uint8_t value;
	uint32_t address = m_pc, error;

	if(!translate_address(m_CPL,TRANSLATE_FETCH,&address,&error))
		PF_THROW(error);

	value = m_direct->read_byte(address & m_a20_mask);
#ifdef DEBUG_MISSING_OPCODE
	m_opcode_bytes[m_opcode_bytes_length] = value;
	m_opcode_bytes_length = (m_opcode_bytes_length + 1) & 15;
#endif
	m_eip++;
	m_pc++;
	return value;
}
uint16_t i386_device::FETCH16()
{
	uint16_t value;
	uint32_t address = m_pc, error;

	if( !WORD_ALIGNED(address) ) {       /* Unaligned read */
		value = (FETCH() << 0);
		value |= (FETCH() << 8);
	} else {
		if(!translate_address(m_CPL,TRANSLATE_FETCH,&address,&error))
			PF_THROW(error);
		address &= m_a20_mask;
		value = m_direct->read_word(address);
		m_eip += 2;
		m_pc += 2;
	}
	return value;
}
uint32_t i386_device::FETCH32()
{
	uint32_t value;
	uint32_t address = m_pc, error;

	if( !DWORD_ALIGNED(m_pc) ) {      /* Unaligned read */
		value = (FETCH() << 0);
		value |= (FETCH() << 8);
		value |= (FETCH() << 16);
		value |= (FETCH() << 24);
	} else {
		if(!translate_address(m_CPL,TRANSLATE_FETCH,&address,&error))
			PF_THROW(error);

		address &= m_a20_mask;
		value = m_direct->read_dword(address);
		m_eip += 4;
		m_pc += 4;
	}
	return value;
}

uint8_t i386_device::READ8(uint32_t ea)
{
	uint32_t address = ea, error;

	if(!translate_address(m_CPL,TRANSLATE_READ,&address, &error))
		PF_THROW(error);

	address &= m_a20_mask;
	return m_program->read_byte(address);
}
uint16_t i386_device::READ16(uint32_t ea)
{
	uint16_t value;
	uint32_t address = ea, error;

	if( !WORD_ALIGNED(ea) ) {        /* Unaligned read */
		value = (READ8( address+0 ) << 0);
		value |= (READ8( address+1 ) << 8);
	} else {
		if(!translate_address(m_CPL,TRANSLATE_READ,&address,&error))
			PF_THROW(error);

		address &= m_a20_mask;
		value = m_program->read_word( address );
	}
	return value;
}
uint32_t i386_device::READ32(uint32_t ea)
{
	uint32_t value;
	uint32_t address = ea, error;

	if( !DWORD_ALIGNED(ea) ) {        /* Unaligned read */
		value = (READ8( address+0 ) << 0);
		value |= (READ8( address+1 ) << 8);
		value |= (READ8( address+2 ) << 16),
		value |= (READ8( address+3 ) << 24);
	} else {
		if(!translate_address(m_CPL,TRANSLATE_READ,&address,&error))
			PF_THROW(error);

		address &= m_a20_mask;
		value = m_program->read_dword( address );
	}
	return value;
}

uint64_t i386_device::READ64(uint32_t ea)
{
	uint64_t value;
	uint32_t address = ea, error;

	if( !QWORD_ALIGNED(ea) ) {        /* Unaligned read */
		value = (((uint64_t) READ8( address+0 )) << 0);
		value |= (((uint64_t) READ8( address+1 )) << 8);
		value |= (((uint64_t) READ8( address+2 )) << 16);
		value |= (((uint64_t) READ8( address+3 )) << 24);
		value |= (((uint64_t) READ8( address+4 )) << 32);
		value |= (((uint64_t) READ8( address+5 )) << 40);
		value |= (((uint64_t) READ8( address+6 )) << 48);
		value |= (((uint64_t) READ8( address+7 )) << 56);
	} else {
		if(!translate_address(m_CPL,TRANSLATE_READ,&address,&error))
			PF_THROW(error);

		address &= m_a20_mask;
		value = (((uint64_t) m_program->read_dword( address+0 )) << 0);
		value |= (((uint64_t) m_program->read_dword( address+4 )) << 32);
	}
	return value;
}
uint8_t i386_device::READ8PL0(uint32_t ea)
{
	uint32_t address = ea, error;

	if(!translate_address(0,TRANSLATE_READ,&address,&error))
		PF_THROW(error);

	address &= m_a20_mask;
	return m_program->read_byte(address);
}
uint16_t i386_device::READ16PL0(uint32_t ea)
{
	uint16_t value;
	uint32_t address = ea, error;

	if( !WORD_ALIGNED(ea) ) {        /* Unaligned read */
		value = (READ8PL0( address+0 ) << 0);
		value |= (READ8PL0( address+1 ) << 8);
	} else {
		if(!translate_address(0,TRANSLATE_READ,&address,&error))
			PF_THROW(error);

		address &= m_a20_mask;
		value = m_program->read_word( address );
	}
	return value;
}

uint32_t i386_device::READ32PL0(uint32_t ea)
{
	uint32_t value;
	uint32_t address = ea, error;

	if( !DWORD_ALIGNED(ea) ) {        /* Unaligned read */
		value = (READ8PL0( address+0 ) << 0);
		value |= (READ8PL0( address+1 ) << 8);
		value |= (READ8PL0( address+2 ) << 16);
		value |= (READ8PL0( address+3 ) << 24);
	} else {
		if(!translate_address(0,TRANSLATE_READ,&address,&error))
			PF_THROW(error);

		address &= m_a20_mask;
		value = m_program->read_dword( address );
	}
	return value;
}

void i386_device::WRITE_TEST(uint32_t ea)
{
	uint32_t address = ea, error;
	if(!translate_address(m_CPL,TRANSLATE_WRITE,&address,&error))
		PF_THROW(error);
}

void i386_device::WRITE8(uint32_t ea, uint8_t value)
{
	uint32_t address = ea, error;

	if(!translate_address(m_CPL,TRANSLATE_WRITE,&address,&error))
		PF_THROW(error);

	address &= m_a20_mask;
	m_program->write_byte(address, value);
}
void i386_device::WRITE16(uint32_t ea, uint16_t value)
{
	uint32_t address = ea, error;

	if( !WORD_ALIGNED(ea) ) {        /* Unaligned write */
		WRITE8( address+0, value & 0xff );
		WRITE8( address+1, (value >> 8) & 0xff );
	} else {
		if(!translate_address(m_CPL,TRANSLATE_WRITE,&address,&error))
			PF_THROW(error);

		address &= m_a20_mask;
		m_program->write_word(address, value);
	}
}
void i386_device::WRITE32(uint32_t ea, uint32_t value)
{
	uint32_t address = ea, error;

	if( !DWORD_ALIGNED(ea) ) {        /* Unaligned write */
		WRITE8( address+0, value & 0xff );
		WRITE8( address+1, (value >> 8) & 0xff );
		WRITE8( address+2, (value >> 16) & 0xff );
		WRITE8( address+3, (value >> 24) & 0xff );
	} else {
		if(!translate_address(m_CPL,TRANSLATE_WRITE,&address,&error))
			PF_THROW(error);

		ea &= m_a20_mask;
		m_program->write_dword(address, value);
	}
}

void i386_device::WRITE64(uint32_t ea, uint64_t value)
{
	uint32_t address = ea, error;

	if( !QWORD_ALIGNED(ea) ) {        /* Unaligned write */
		WRITE8( address+0, value & 0xff );
		WRITE8( address+1, (value >> 8) & 0xff );
		WRITE8( address+2, (value >> 16) & 0xff );
		WRITE8( address+3, (value >> 24) & 0xff );
		WRITE8( address+4, (value >> 32) & 0xff );
		WRITE8( address+5, (value >> 40) & 0xff );
		WRITE8( address+6, (value >> 48) & 0xff );
		WRITE8( address+7, (value >> 56) & 0xff );
	} else {
		if(!translate_address(m_CPL,TRANSLATE_WRITE,&address,&error))
			PF_THROW(error);

		ea &= m_a20_mask;
		m_program->write_dword(address+0, value & 0xffffffff);
		m_program->write_dword(address+4, (value >> 32) & 0xffffffff);
	}
}

/***********************************************************************************/

uint8_t i386_device::OR8(uint8_t dst, uint8_t src)
{
	uint8_t res = dst | src;
	m_CF = m_OF = 0;
	SetSZPF8(res);
	return res;
}
uint16_t i386_device::OR16(uint16_t dst, uint16_t src)
{
	uint16_t res = dst | src;
	m_CF = m_OF = 0;
	SetSZPF16(res);
	return res;
}
uint32_t i386_device::OR32(uint32_t dst, uint32_t src)
{
	uint32_t res = dst | src;
	m_CF = m_OF = 0;
	SetSZPF32(res);
	return res;
}

uint8_t i386_device::AND8(uint8_t dst, uint8_t src)
{
	uint8_t res = dst & src;
	m_CF = m_OF = 0;
	SetSZPF8(res);
	return res;
}
uint16_t i386_device::AND16(uint16_t dst, uint16_t src)
{
	uint16_t res = dst & src;
	m_CF = m_OF = 0;
	SetSZPF16(res);
	return res;
}
uint32_t i386_device::AND32(uint32_t dst, uint32_t src)
{
	uint32_t res = dst & src;
	m_CF = m_OF = 0;
	SetSZPF32(res);
	return res;
}

uint8_t i386_device::XOR8(uint8_t dst, uint8_t src)
{
	uint8_t res = dst ^ src;
	m_CF = m_OF = 0;
	SetSZPF8(res);
	return res;
}
uint16_t i386_device::XOR16(uint16_t dst, uint16_t src)
{
	uint16_t res = dst ^ src;
	m_CF = m_OF = 0;
	SetSZPF16(res);
	return res;
}
uint32_t i386_device::XOR32(uint32_t dst, uint32_t src)
{
	uint32_t res = dst ^ src;
	m_CF = m_OF = 0;
	SetSZPF32(res);
	return res;
}

#define SUB8(dst, src) SBB8(dst, src, 0)
uint8_t i386_device::SBB8(uint8_t dst, uint8_t src, uint8_t b)
{
	uint16_t res = (uint16_t)dst - (uint16_t)src - (uint8_t)b;
	SetCF8(res);
	SetOF_Sub8(res,src,dst);
	SetAF(res,src,dst);
	SetSZPF8(res);
	return (uint8_t)res;
}

#define SUB16(dst, src) SBB16(dst, src, 0)
uint16_t i386_device::SBB16(uint16_t dst, uint16_t src, uint16_t b)
{
	uint32_t res = (uint32_t)dst - (uint32_t)src - (uint32_t)b;
	SetCF16(res);
	SetOF_Sub16(res,src,dst);
	SetAF(res,src,dst);
	SetSZPF16(res);
	return (uint16_t)res;
}

#define SUB32(dst, src) SBB32(dst, src, 0)
uint32_t i386_device::SBB32(uint32_t dst, uint32_t src, uint32_t b)
{
	uint64_t res = (uint64_t)dst - (uint64_t)src - (uint64_t) b;
	SetCF32(res);
	SetOF_Sub32(res,src,dst);
	SetAF(res,src,dst);
	SetSZPF32(res);
	return (uint32_t)res;
}

#define ADD8(dst, src) ADC8(dst, src, 0)
uint8_t i386_device::ADC8(uint8_t dst, uint8_t src, uint8_t c)
{
	uint16_t res = (uint16_t)dst + (uint16_t)src + (uint16_t)c;
	SetCF8(res);
	SetOF_Add8(res,src,dst);
	SetAF(res,src,dst);
	SetSZPF8(res);
	return (uint8_t)res;
}

#define ADD16(dst, src) ADC16(dst, src, 0)
uint16_t i386_device::ADC16(uint16_t dst, uint16_t src, uint8_t c)
{
	uint32_t res = (uint32_t)dst + (uint32_t)src + (uint32_t)c;
	SetCF16(res);
	SetOF_Add16(res,src,dst);
	SetAF(res,src,dst);
	SetSZPF16(res);
	return (uint16_t)res;
}

#define ADD32(dst, src) ADC32(dst, src, 0)
uint32_t i386_device::ADC32(uint32_t dst, uint32_t src, uint32_t c)
{
	uint64_t res = (uint64_t)dst + (uint64_t)src + (uint64_t) c;
	SetCF32(res);
	SetOF_Add32(res,src,dst);
	SetAF(res,src,dst);
	SetSZPF32(res);
	return (uint32_t)res;
}

uint8_t i386_device::INC8(uint8_t dst)
{
	uint16_t res = (uint16_t)dst + 1;
	SetOF_Add8(res,1,dst);
	SetAF(res,1,dst);
	SetSZPF8(res);
	return (uint8_t)res;
}
uint16_t i386_device::INC16(uint16_t dst)
{
	uint32_t res = (uint32_t)dst + 1;
	SetOF_Add16(res,1,dst);
	SetAF(res,1,dst);
	SetSZPF16(res);
	return (uint16_t)res;
}
uint32_t i386_device::INC32(uint32_t dst)
{
	uint64_t res = (uint64_t)dst + 1;
	SetOF_Add32(res,1,dst);
	SetAF(res,1,dst);
	SetSZPF32(res);
	return (uint32_t)res;
}

uint8_t i386_device::DEC8(uint8_t dst)
{
	uint16_t res = (uint16_t)dst - 1;
	SetOF_Sub8(res,1,dst);
	SetAF(res,1,dst);
	SetSZPF8(res);
	return (uint8_t)res;
}
uint16_t i386_device::DEC16(uint16_t dst)
{
	uint32_t res = (uint32_t)dst - 1;
	SetOF_Sub16(res,1,dst);
	SetAF(res,1,dst);
	SetSZPF16(res);
	return (uint16_t)res;
}
uint32_t i386_device::DEC32(uint32_t dst)
{
	uint64_t res = (uint64_t)dst - 1;
	SetOF_Sub32(res,1,dst);
	SetAF(res,1,dst);
	SetSZPF32(res);
	return (uint32_t)res;
}



void i386_device::PUSH16(uint16_t value)
{
	uint32_t ea, new_esp;
	if( STACK_32BIT ) {
		new_esp = REG32(ESP) - 2;
		ea = i386_translate(SS, new_esp, 1);
		WRITE16(ea, value );
		REG32(ESP) = new_esp;
	} else {
		new_esp = (REG16(SP) - 2) & 0xffff;
		ea = i386_translate(SS, new_esp, 1);
		WRITE16(ea, value );
		REG16(SP) = new_esp;
	}
}
void i386_device::PUSH32(uint32_t value)
{
	uint32_t ea, new_esp;
	if( STACK_32BIT ) {
		new_esp = REG32(ESP) - 4;
		ea = i386_translate(SS, new_esp, 1);
		WRITE32(ea, value );
		REG32(ESP) = new_esp;
	} else {
		new_esp = (REG16(SP) - 4) & 0xffff;
		ea = i386_translate(SS, new_esp, 1);
		WRITE32(ea, value );
		REG16(SP) = new_esp;
	}
}

void i386_device::PUSH32SEG(uint32_t value)
{
	uint32_t ea, new_esp;
	if( STACK_32BIT ) {
		new_esp = REG32(ESP) - 4;
		ea = i386_translate(SS, new_esp, 1);
		((m_cpu_version & 0xf00) == 0x300) ? WRITE16(ea, value) : WRITE32(ea, value ); // 486 also?
		REG32(ESP) = new_esp;
	} else {
		new_esp = (REG16(SP) - 4) & 0xffff;
		ea = i386_translate(SS, new_esp, 1);
		((m_cpu_version & 0xf00) == 0x300) ? WRITE16(ea, value) : WRITE32(ea, value );
		REG16(SP) = new_esp;
	}
}

void i386_device::PUSH8(uint8_t value)
{
	if( m_operand_size ) {
		PUSH32((int32_t)(int8_t)value);
	} else {
		PUSH16((int16_t)(int8_t)value);
	}
}

uint8_t i386_device::POP8()
{
	uint8_t value;
	uint32_t ea, new_esp;
	if( STACK_32BIT ) {
		new_esp = REG32(ESP) + 1;
		ea = i386_translate(SS, new_esp - 1, 0);
		value = READ8(ea );
		REG32(ESP) = new_esp;
	} else {
		new_esp = REG16(SP) + 1;
		ea = i386_translate(SS, (new_esp - 1) & 0xffff, 0);
		value = READ8(ea );
		REG16(SP) = new_esp;
	}
	return value;
}
uint16_t i386_device::POP16()
{
	uint16_t value;
	uint32_t ea, new_esp;
	if( STACK_32BIT ) {
		new_esp = REG32(ESP) + 2;
		ea = i386_translate(SS, new_esp - 2, 0);
		value = READ16(ea );
		REG32(ESP) = new_esp;
	} else {
		new_esp = REG16(SP) + 2;
		ea = i386_translate(SS, (new_esp - 2) & 0xffff, 0);
		value = READ16(ea );
		REG16(SP) = new_esp;
	}
	return value;
}
uint32_t i386_device::POP32()
{
	uint32_t value;
	uint32_t ea, new_esp;
	if( STACK_32BIT ) {
		new_esp = REG32(ESP) + 4;
		ea = i386_translate(SS, new_esp - 4, 0);
		value = READ32(ea );
		REG32(ESP) = new_esp;
	} else {
		new_esp = REG16(SP) + 4;
		ea = i386_translate(SS, (new_esp - 4) & 0xffff, 0);
		value = READ32(ea );
		REG16(SP) = new_esp;
	}
	return value;
}

void i386_device::BUMP_SI(int adjustment)
{
	if ( m_address_size )
		REG32(ESI) += ((m_DF) ? -adjustment : +adjustment);
	else
		REG16(SI) += ((m_DF) ? -adjustment : +adjustment);
}

void i386_device::BUMP_DI(int adjustment)
{
	if ( m_address_size )
		REG32(EDI) += ((m_DF) ? -adjustment : +adjustment);
	else
		REG16(DI) += ((m_DF) ? -adjustment : +adjustment);
}



/***********************************************************************************
    I/O ACCESS
***********************************************************************************/

void i386_device::check_ioperm(offs_t port, uint8_t mask)
{
	uint8_t IOPL, map;
	uint16_t IOPB;
	uint32_t address;

	if(!PROTECTED_MODE)
		return;

	IOPL = m_IOP1 | (m_IOP2 << 1);
	if(!V8086_MODE && (m_CPL <= IOPL))
		return;

	if((m_task.limit < 0x67) || ((m_task.flags & 0xd) != 9))
		FAULT_THROW(FAULT_GP,0);

	address = m_task.base;
	IOPB = READ16PL0(address+0x66);
	if((IOPB+(port/8)) > m_task.limit)
		FAULT_THROW(FAULT_GP,0);

	map = READ8PL0(address+IOPB+(port/8));
	map >>= (port%8);
	if(map & mask)
		FAULT_THROW(FAULT_GP,0);
}

uint8_t i386_device::READPORT8(offs_t port)
{
	check_ioperm(port, 1);
	return m_io->read_byte(port);
}

void i386_device::WRITEPORT8(offs_t port, uint8_t value)
{
	check_ioperm(port, 1);
	m_io->write_byte(port, value);
}

uint16_t i386_device::READPORT16(offs_t port)
{
	if (port & 1)
	{
		uint16_t value = READPORT8(port);
		value |= (READPORT8(port + 1) << 8);
		return value;
	}
	else
	{
		check_ioperm(port, 3);
		return m_io->read_word(port);
	}
}

void i386_device::WRITEPORT16(offs_t port, uint16_t value)
{
	if (port & 1)
	{
		WRITEPORT8(port, value & 0xff);
		WRITEPORT8(port + 1, (value >> 8) & 0xff);
	}
	else
	{
		check_ioperm(port, 3);
		m_io->write_word(port, value);
	}
}

uint32_t i386_device::READPORT32(offs_t port)
{
	if (port & 3)
	{
		uint32_t value = READPORT8(port);
		value |= (READPORT8(port + 1) << 8);
		value |= (READPORT8(port + 2) << 16);
		value |= (READPORT8(port + 3) << 24);
		return value;
	}
	else
	{
		check_ioperm(port, 0xf);
		return m_io->read_dword(port);
	}
}

void i386_device::WRITEPORT32(offs_t port, uint32_t value)
{
	if (port & 3)
	{
		WRITEPORT8(port, value & 0xff);
		WRITEPORT8(port + 1, (value >> 8) & 0xff);
		WRITEPORT8(port + 2, (value >> 16) & 0xff);
		WRITEPORT8(port + 3, (value >> 24) & 0xff);
	}
	else
	{
		check_ioperm(port, 0xf);
		m_io->write_dword(port, value);
	}
}

/***********************************************************************************
    MSR ACCESS
***********************************************************************************/

// Pentium MSR handling
uint64_t i386_device::pentium_msr_read(uint32_t offset,uint8_t *valid_msr)
{
	switch(offset)
	{
	// Machine Check Exception (TODO)
	case 0x00:
		*valid_msr = 1;
		popmessage("RDMSR: Reading P5_MC_ADDR");
		return 0;
	case 0x01:
		*valid_msr = 1;
		popmessage("RDMSR: Reading P5_MC_TYPE");
		return 0;
	// Time Stamp Counter
	case 0x10:
		*valid_msr = 1;
		popmessage("RDMSR: Reading TSC");
		return m_tsc;
	// Event Counters (TODO)
	case 0x11:  // CESR
		*valid_msr = 1;
		popmessage("RDMSR: Reading CESR");
		return 0;
	case 0x12:  // CTR0
		*valid_msr = 1;
		return m_perfctr[0];
	case 0x13:  // CTR1
		*valid_msr = 1;
		return m_perfctr[1];
	default:
		if(!(offset & ~0xf)) // 2-f are test registers
		{
			*valid_msr = 1;
			logerror("RDMSR: Reading test MSR %x", offset);
			return 0;
		}
		logerror("RDMSR: invalid P5 MSR read %08x at %08x\n",offset,m_pc-2);
		*valid_msr = 0;
		return 0;
	}
	return -1;
}

void i386_device::pentium_msr_write(uint32_t offset, uint64_t data, uint8_t *valid_msr)
{
	switch(offset)
	{
	// Machine Check Exception (TODO)
	case 0x00:
		popmessage("WRMSR: Writing P5_MC_ADDR");
		*valid_msr = 1;
		break;
	case 0x01:
		popmessage("WRMSR: Writing P5_MC_TYPE");
		*valid_msr = 1;
		break;
	// Time Stamp Counter
	case 0x10:
		m_tsc = data;
		popmessage("WRMSR: Writing to TSC");
		*valid_msr = 1;
		break;
	// Event Counters (TODO)
	case 0x11:  // CESR
		popmessage("WRMSR: Writing to CESR");
		*valid_msr = 1;
		break;
	case 0x12:  // CTR0
		m_perfctr[0] = data;
		*valid_msr = 1;
		break;
	case 0x13:  // CTR1
		m_perfctr[1] = data;
		*valid_msr = 1;
		break;
	default:
		if(!(offset & ~0xf)) // 2-f are test registers
		{
			*valid_msr = 1;
			logerror("WRMSR: Writing test MSR %x", offset);
			break;
		}
		logerror("WRMSR: invalid MSR write %08x (%08x%08x) at %08x\n",offset,(uint32_t)(data >> 32),(uint32_t)data,m_pc-2);
		*valid_msr = 0;
		break;
	}
}

// P6 (Pentium Pro, Pentium II, Pentium III) MSR handling
uint64_t i386_device::p6_msr_read(uint32_t offset,uint8_t *valid_msr)
{
	switch(offset)
	{
	// Machine Check Exception (TODO)
	case 0x00:
		*valid_msr = 1;
		popmessage("RDMSR: Reading P5_MC_ADDR");
		return 0;
	case 0x01:
		*valid_msr = 1;
		popmessage("RDMSR: Reading P5_MC_TYPE");
		return 0;
	// Time Stamp Counter
	case 0x10:
		*valid_msr = 1;
		popmessage("RDMSR: Reading TSC");
		return m_tsc;
	// Performance Counters (TODO)
	case 0xc1:  // PerfCtr0
		*valid_msr = 1;
		return m_perfctr[0];
	case 0xc2:  // PerfCtr1
		*valid_msr = 1;
		return m_perfctr[1];
	default:
		logerror("RDMSR: unimplemented register called %08x at %08x\n",offset,m_pc-2);
		*valid_msr = 1;
		return 0;
	}
	return -1;
}

void i386_device::p6_msr_write(uint32_t offset, uint64_t data, uint8_t *valid_msr)
{
	switch(offset)
	{
	// Time Stamp Counter
	case 0x10:
		m_tsc = data;
		popmessage("WRMSR: Writing to TSC");
		*valid_msr = 1;
		break;
	// Performance Counters (TODO)
	case 0xc1:  // PerfCtr0
		m_perfctr[0] = data;
		*valid_msr = 1;
		break;
	case 0xc2:  // PerfCtr1
		m_perfctr[1] = data;
		*valid_msr = 1;
		break;
	default:
		logerror("WRMSR: unimplemented register called %08x (%08x%08x) at %08x\n",offset,(uint32_t)(data >> 32),(uint32_t)data,m_pc-2);
		*valid_msr = 1;
		break;
	}
}


// PIV (Pentium 4+)
uint64_t i386_device::piv_msr_read(uint32_t offset,uint8_t *valid_msr)
{
	switch(offset)
	{
	default:
		logerror("RDMSR: unimplemented register called %08x at %08x\n",offset,m_pc-2);
		*valid_msr = 1;
		return 0;
	}
	return -1;
}

void i386_device::piv_msr_write(uint32_t offset, uint64_t data, uint8_t *valid_msr)
{
	switch(offset)
	{
	default:
		logerror("WRMSR: unimplemented register called %08x (%08x%08x) at %08x\n",offset,(uint32_t)(data >> 32),(uint32_t)data,m_pc-2);
		*valid_msr = 1;
		break;
	}
}

uint64_t i386_device::MSR_READ(uint32_t offset,uint8_t *valid_msr)
{
	uint64_t res;
	uint8_t cpu_type = (m_cpu_version >> 8) & 0x0f;

	*valid_msr = 0;

	switch(cpu_type)
	{
	case 5:  // Pentium
		res = pentium_msr_read(offset,valid_msr);
		break;
	case 6:  // Pentium Pro, Pentium II, Pentium III
		res = p6_msr_read(offset,valid_msr);
		break;
	case 15:  // Pentium 4+
		res = piv_msr_read(offset,valid_msr);
		break;
	default:
		res = 0;
		break;
	}

	return res;
}

void i386_device::MSR_WRITE(uint32_t offset, uint64_t data, uint8_t *valid_msr)
{
	*valid_msr = 0;
	uint8_t cpu_type = (m_cpu_version >> 8) & 0x0f;

	switch(cpu_type)
	{
	case 5:  // Pentium
		pentium_msr_write(offset,data,valid_msr);
		break;
	case 6:  // Pentium Pro, Pentium II, Pentium III
		p6_msr_write(offset,data,valid_msr);
		break;
	case 15:  // Pentium 4+
		piv_msr_write(offset,data,valid_msr);
		break;
	}
}

#endif /* __I386_H__ */
