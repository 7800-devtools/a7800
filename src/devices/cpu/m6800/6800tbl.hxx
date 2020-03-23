// license:BSD-3-Clause
// copyright-holders:Aaron Giles
const m6800_cpu_device::op_func m6800_cpu_device::m6800_insn[0x100] = {
&m6800_cpu_device::illegl1,&m6800_cpu_device::nop,    &m6800_cpu_device::illegl1,&m6800_cpu_device::illegl1,&m6800_cpu_device::illegl1,&m6800_cpu_device::illegl1,&m6800_cpu_device::tap,    &m6800_cpu_device::tpa,
&m6800_cpu_device::inx,    &m6800_cpu_device::dex,    &m6800_cpu_device::clv,    &m6800_cpu_device::sev,    &m6800_cpu_device::clc,    &m6800_cpu_device::sec,    &m6800_cpu_device::cli,    &m6800_cpu_device::sei,
&m6800_cpu_device::sba,    &m6800_cpu_device::cba,    &m6800_cpu_device::illegl1,&m6800_cpu_device::illegl1,&m6800_cpu_device::illegl1,&m6800_cpu_device::illegl1,&m6800_cpu_device::tab,    &m6800_cpu_device::tba,
&m6800_cpu_device::illegl1,&m6800_cpu_device::daa,    &m6800_cpu_device::illegl1,&m6800_cpu_device::aba,    &m6800_cpu_device::illegl1,&m6800_cpu_device::illegl1,&m6800_cpu_device::illegl1,&m6800_cpu_device::illegl1,
&m6800_cpu_device::bra,    &m6800_cpu_device::brn,    &m6800_cpu_device::bhi,    &m6800_cpu_device::bls,    &m6800_cpu_device::bcc,    &m6800_cpu_device::bcs,    &m6800_cpu_device::bne,    &m6800_cpu_device::beq,
&m6800_cpu_device::bvc,    &m6800_cpu_device::bvs,    &m6800_cpu_device::bpl,    &m6800_cpu_device::bmi,    &m6800_cpu_device::bge,    &m6800_cpu_device::blt,    &m6800_cpu_device::bgt,    &m6800_cpu_device::ble,
&m6800_cpu_device::tsx,    &m6800_cpu_device::ins,    &m6800_cpu_device::pula,   &m6800_cpu_device::pulb,   &m6800_cpu_device::des,    &m6800_cpu_device::txs,    &m6800_cpu_device::psha,   &m6800_cpu_device::pshb,
&m6800_cpu_device::illegl1,&m6800_cpu_device::rts,    &m6800_cpu_device::illegl1,&m6800_cpu_device::rti,    &m6800_cpu_device::illegl1,&m6800_cpu_device::illegl1,&m6800_cpu_device::wai,    &m6800_cpu_device::swi,
&m6800_cpu_device::nega,   &m6800_cpu_device::illegl1,&m6800_cpu_device::illegl1,&m6800_cpu_device::coma,   &m6800_cpu_device::lsra,   &m6800_cpu_device::illegl1,&m6800_cpu_device::rora,   &m6800_cpu_device::asra,
&m6800_cpu_device::asla,   &m6800_cpu_device::rola,   &m6800_cpu_device::deca,   &m6800_cpu_device::illegl1,&m6800_cpu_device::inca,   &m6800_cpu_device::tsta,   &m6800_cpu_device::illegl1,&m6800_cpu_device::clra,
&m6800_cpu_device::negb,   &m6800_cpu_device::illegl1,&m6800_cpu_device::illegl1,&m6800_cpu_device::comb,   &m6800_cpu_device::lsrb,   &m6800_cpu_device::illegl1,&m6800_cpu_device::rorb,   &m6800_cpu_device::asrb,
&m6800_cpu_device::aslb,   &m6800_cpu_device::rolb,   &m6800_cpu_device::decb,   &m6800_cpu_device::illegl1,&m6800_cpu_device::incb,   &m6800_cpu_device::tstb,   &m6800_cpu_device::illegl1,&m6800_cpu_device::clrb,
&m6800_cpu_device::neg_ix, &m6800_cpu_device::illegl2,&m6800_cpu_device::illegl2,&m6800_cpu_device::com_ix, &m6800_cpu_device::lsr_ix, &m6800_cpu_device::illegl2,&m6800_cpu_device::ror_ix, &m6800_cpu_device::asr_ix,
&m6800_cpu_device::asl_ix, &m6800_cpu_device::rol_ix, &m6800_cpu_device::dec_ix, &m6800_cpu_device::illegl2,&m6800_cpu_device::inc_ix, &m6800_cpu_device::tst_ix, &m6800_cpu_device::jmp_ix, &m6800_cpu_device::clr_ix,
&m6800_cpu_device::neg_ex, &m6800_cpu_device::illegl3,&m6800_cpu_device::illegl3,&m6800_cpu_device::com_ex, &m6800_cpu_device::lsr_ex, &m6800_cpu_device::illegl3,&m6800_cpu_device::ror_ex, &m6800_cpu_device::asr_ex,
&m6800_cpu_device::asl_ex, &m6800_cpu_device::rol_ex, &m6800_cpu_device::dec_ex, &m6800_cpu_device::illegl3,&m6800_cpu_device::inc_ex, &m6800_cpu_device::tst_ex, &m6800_cpu_device::jmp_ex, &m6800_cpu_device::clr_ex,
&m6800_cpu_device::suba_im,&m6800_cpu_device::cmpa_im,&m6800_cpu_device::sbca_im,&m6800_cpu_device::illegl2,&m6800_cpu_device::anda_im,&m6800_cpu_device::bita_im,&m6800_cpu_device::lda_im, &m6800_cpu_device::sta_im,
&m6800_cpu_device::eora_im,&m6800_cpu_device::adca_im,&m6800_cpu_device::ora_im, &m6800_cpu_device::adda_im,&m6800_cpu_device::cmpx_im,&m6800_cpu_device::bsr,    &m6800_cpu_device::lds_im, &m6800_cpu_device::sts_im,
&m6800_cpu_device::suba_di,&m6800_cpu_device::cmpa_di,&m6800_cpu_device::sbca_di,&m6800_cpu_device::illegl2,&m6800_cpu_device::anda_di,&m6800_cpu_device::bita_di,&m6800_cpu_device::lda_di, &m6800_cpu_device::sta_di,
&m6800_cpu_device::eora_di,&m6800_cpu_device::adca_di,&m6800_cpu_device::ora_di, &m6800_cpu_device::adda_di,&m6800_cpu_device::cmpx_di,&m6800_cpu_device::jsr_di, &m6800_cpu_device::lds_di, &m6800_cpu_device::sts_di,
&m6800_cpu_device::suba_ix,&m6800_cpu_device::cmpa_ix,&m6800_cpu_device::sbca_ix,&m6800_cpu_device::illegl2,&m6800_cpu_device::anda_ix,&m6800_cpu_device::bita_ix,&m6800_cpu_device::lda_ix, &m6800_cpu_device::sta_ix,
&m6800_cpu_device::eora_ix,&m6800_cpu_device::adca_ix,&m6800_cpu_device::ora_ix, &m6800_cpu_device::adda_ix,&m6800_cpu_device::cmpx_ix,&m6800_cpu_device::jsr_ix, &m6800_cpu_device::lds_ix, &m6800_cpu_device::sts_ix,
&m6800_cpu_device::suba_ex,&m6800_cpu_device::cmpa_ex,&m6800_cpu_device::sbca_ex,&m6800_cpu_device::illegl3,&m6800_cpu_device::anda_ex,&m6800_cpu_device::bita_ex,&m6800_cpu_device::lda_ex, &m6800_cpu_device::sta_ex,
&m6800_cpu_device::eora_ex,&m6800_cpu_device::adca_ex,&m6800_cpu_device::ora_ex, &m6800_cpu_device::adda_ex,&m6800_cpu_device::cmpx_ex,&m6800_cpu_device::jsr_ex, &m6800_cpu_device::lds_ex, &m6800_cpu_device::sts_ex,
&m6800_cpu_device::subb_im,&m6800_cpu_device::cmpb_im,&m6800_cpu_device::sbcb_im,&m6800_cpu_device::illegl2,&m6800_cpu_device::andb_im,&m6800_cpu_device::bitb_im,&m6800_cpu_device::ldb_im, &m6800_cpu_device::stb_im,
&m6800_cpu_device::eorb_im,&m6800_cpu_device::adcb_im,&m6800_cpu_device::orb_im, &m6800_cpu_device::addb_im,&m6800_cpu_device::illegl3,&m6800_cpu_device::illegl3,&m6800_cpu_device::ldx_im, &m6800_cpu_device::stx_im,
&m6800_cpu_device::subb_di,&m6800_cpu_device::cmpb_di,&m6800_cpu_device::sbcb_di,&m6800_cpu_device::illegl2,&m6800_cpu_device::andb_di,&m6800_cpu_device::bitb_di,&m6800_cpu_device::ldb_di, &m6800_cpu_device::stb_di,
&m6800_cpu_device::eorb_di,&m6800_cpu_device::adcb_di,&m6800_cpu_device::orb_di, &m6800_cpu_device::addb_di,&m6800_cpu_device::illegl2,&m6800_cpu_device::illegl2,&m6800_cpu_device::ldx_di, &m6800_cpu_device::stx_di,
&m6800_cpu_device::subb_ix,&m6800_cpu_device::cmpb_ix,&m6800_cpu_device::sbcb_ix,&m6800_cpu_device::illegl2,&m6800_cpu_device::andb_ix,&m6800_cpu_device::bitb_ix,&m6800_cpu_device::ldb_ix, &m6800_cpu_device::stb_ix,
&m6800_cpu_device::eorb_ix,&m6800_cpu_device::adcb_ix,&m6800_cpu_device::orb_ix, &m6800_cpu_device::addb_ix,&m6800_cpu_device::illegl2,&m6800_cpu_device::illegl2,&m6800_cpu_device::ldx_ix, &m6800_cpu_device::stx_ix,
&m6800_cpu_device::subb_ex,&m6800_cpu_device::cmpb_ex,&m6800_cpu_device::sbcb_ex,&m6800_cpu_device::illegl3,&m6800_cpu_device::andb_ex,&m6800_cpu_device::bitb_ex,&m6800_cpu_device::ldb_ex, &m6800_cpu_device::stb_ex,
&m6800_cpu_device::eorb_ex,&m6800_cpu_device::adcb_ex,&m6800_cpu_device::orb_ex, &m6800_cpu_device::addb_ex,&m6800_cpu_device::illegl3,&m6800_cpu_device::illegl3,&m6800_cpu_device::ldx_ex, &m6800_cpu_device::stx_ex
};

const m6800_cpu_device::op_func m6800_cpu_device::nsc8105_insn[0x100] = {
// 0
&m6800_cpu_device::illegl1,&m6800_cpu_device::illegl1,&m6800_cpu_device::nop,    &m6800_cpu_device::illegl1,&m6800_cpu_device::illegl1,&m6800_cpu_device::tap,    &m6800_cpu_device::illegl1,&m6800_cpu_device::tpa,
// 8
&m6800_cpu_device::inx,    &m6800_cpu_device::clv,    &m6800_cpu_device::dex,    &m6800_cpu_device::sev,    &m6800_cpu_device::clc,    &m6800_cpu_device::cli,    &m6800_cpu_device::sec,    &m6800_cpu_device::sei,
// 10
&m6800_cpu_device::sba,    &m6800_cpu_device::illegl1,&m6800_cpu_device::cba,    &m6800_cpu_device::illegl1,&m6800_cpu_device::illegl1,&m6800_cpu_device::tab,    &m6800_cpu_device::illegl1,&m6800_cpu_device::tba,
// 18
&m6800_cpu_device::illegl1,&m6800_cpu_device::illegl1,&m6800_cpu_device::daa,    &m6800_cpu_device::aba,    &m6800_cpu_device::illegl1,&m6800_cpu_device::illegl1,&m6800_cpu_device::illegl1,&m6800_cpu_device::illegl1,
// 20
&m6800_cpu_device::bra,    &m6800_cpu_device::bhi,    &m6800_cpu_device::brn,    &m6800_cpu_device::bls,    &m6800_cpu_device::bcc,    &m6800_cpu_device::bne,    &m6800_cpu_device::bcs,    &m6800_cpu_device::beq,
// 28
&m6800_cpu_device::bvc,    &m6800_cpu_device::bpl,    &m6800_cpu_device::bvs,    &m6800_cpu_device::bmi,    &m6800_cpu_device::bge,    &m6800_cpu_device::bgt,    &m6800_cpu_device::blt,    &m6800_cpu_device::ble,
// 30
&m6800_cpu_device::tsx,    &m6800_cpu_device::pula,   &m6800_cpu_device::ins,    &m6800_cpu_device::pulb,   &m6800_cpu_device::des,    &m6800_cpu_device::psha,   &m6800_cpu_device::txs,    &m6800_cpu_device::pshb,
// 38
&m6800_cpu_device::illegl1,&m6800_cpu_device::illegl1,&m6800_cpu_device::rts,    &m6800_cpu_device::rti,    &m6800_cpu_device::illegl1,&m6800_cpu_device::wai,    &m6800_cpu_device::illegl1,&m6800_cpu_device::swi,
// 40
&m6800_cpu_device::suba_im,&m6800_cpu_device::sbca_im,&m6800_cpu_device::cmpa_im,&m6800_cpu_device::illegl1,&m6800_cpu_device::anda_im,&m6800_cpu_device::lda_im, &m6800_cpu_device::bita_im,&m6800_cpu_device::sta_im,
// 48
&m6800_cpu_device::eora_im,&m6800_cpu_device::ora_im, &m6800_cpu_device::adca_im,&m6800_cpu_device::adda_im,&m6800_cpu_device::cmpx_im,&m6800_cpu_device::lds_im, &m6800_cpu_device::bsr,    &m6800_cpu_device::sts_im,
// 50
&m6800_cpu_device::suba_di,&m6800_cpu_device::sbca_di,&m6800_cpu_device::cmpa_di,&m6800_cpu_device::illegl1,&m6800_cpu_device::anda_di,&m6800_cpu_device::lda_di, &m6800_cpu_device::bita_di,&m6800_cpu_device::sta_di,
// 58
&m6800_cpu_device::eora_di,&m6800_cpu_device::ora_di, &m6800_cpu_device::adca_di,&m6800_cpu_device::adda_di,&m6800_cpu_device::cmpx_di,&m6800_cpu_device::lds_di, &m6800_cpu_device::jsr_di, &m6800_cpu_device::sts_di,
// 60
&m6800_cpu_device::suba_ix,&m6800_cpu_device::sbca_ix,&m6800_cpu_device::cmpa_ix,&m6800_cpu_device::illegl1,&m6800_cpu_device::anda_ix,&m6800_cpu_device::lda_ix, &m6800_cpu_device::bita_ix,&m6800_cpu_device::sta_ix,
// 68
&m6800_cpu_device::eora_ix,&m6800_cpu_device::ora_ix, &m6800_cpu_device::adca_ix,&m6800_cpu_device::adda_ix,&m6800_cpu_device::cmpx_ix,&m6800_cpu_device::lds_ix, &m6800_cpu_device::jsr_ix, &m6800_cpu_device::sts_ix,
// 70
&m6800_cpu_device::suba_ex,&m6800_cpu_device::sbca_ex,&m6800_cpu_device::cmpa_ex,&m6800_cpu_device::illegl1,&m6800_cpu_device::anda_ex,&m6800_cpu_device::lda_ex, &m6800_cpu_device::bita_ex,&m6800_cpu_device::sta_ex,
// 78
&m6800_cpu_device::eora_ex,&m6800_cpu_device::ora_ex, &m6800_cpu_device::adca_ex,&m6800_cpu_device::adda_ex,&m6800_cpu_device::cmpx_ex,&m6800_cpu_device::lds_ex, &m6800_cpu_device::jsr_ex, &m6800_cpu_device::sts_ex,
// 80
&m6800_cpu_device::nega,   &m6800_cpu_device::illegl1,&m6800_cpu_device::illegl1,&m6800_cpu_device::coma,   &m6800_cpu_device::lsra,   &m6800_cpu_device::rora,   &m6800_cpu_device::illegl1,&m6800_cpu_device::asra,
// 88
&m6800_cpu_device::asla,   &m6800_cpu_device::deca,   &m6800_cpu_device::rola,   &m6800_cpu_device::illegl1,&m6800_cpu_device::inca,   &m6800_cpu_device::illegl1,&m6800_cpu_device::tsta,   &m6800_cpu_device::clra,
// 90
&m6800_cpu_device::negb,   &m6800_cpu_device::illegl1,&m6800_cpu_device::illegl1,&m6800_cpu_device::comb,   &m6800_cpu_device::lsrb,   &m6800_cpu_device::rorb,   &m6800_cpu_device::illegl1,&m6800_cpu_device::asrb,
// 98
&m6800_cpu_device::aslb,   &m6800_cpu_device::decb,   &m6800_cpu_device::rolb,   &m6800_cpu_device::illegl1,&m6800_cpu_device::incb,   &m6800_cpu_device::illegl1,&m6800_cpu_device::tstb,   &m6800_cpu_device::clrb,
// a0
&m6800_cpu_device::neg_ix, &m6800_cpu_device::illegl1,&m6800_cpu_device::illegl1,&m6800_cpu_device::com_ix, &m6800_cpu_device::lsr_ix, &m6800_cpu_device::ror_ix, &m6800_cpu_device::illegl1,&m6800_cpu_device::asr_ix,
// a8
&m6800_cpu_device::asl_ix, &m6800_cpu_device::dec_ix, &m6800_cpu_device::rol_ix, &m6800_cpu_device::illegl1,&m6800_cpu_device::inc_ix, &m6800_cpu_device::jmp_ix, &m6800_cpu_device::tst_ix, &m6800_cpu_device::clr_ix,
// b0
&m6800_cpu_device::neg_ex, &m6800_cpu_device::illegl1,&m6800_cpu_device::stx_nsc,&m6800_cpu_device::com_ex, &m6800_cpu_device::lsr_ex, &m6800_cpu_device::ror_ex, &m6800_cpu_device::illegl1,&m6800_cpu_device::asr_ex,
// b8
&m6800_cpu_device::asl_ex, &m6800_cpu_device::dec_ex, &m6800_cpu_device::rol_ex, &m6800_cpu_device::btst_ix,&m6800_cpu_device::inc_ex, &m6800_cpu_device::jmp_ex, &m6800_cpu_device::tst_ex, &m6800_cpu_device::clr_ex,
&m6800_cpu_device::subb_im,&m6800_cpu_device::sbcb_im,&m6800_cpu_device::cmpb_im,&m6800_cpu_device::illegl1,&m6800_cpu_device::andb_im,&m6800_cpu_device::ldb_im, &m6800_cpu_device::bitb_im,&m6800_cpu_device::stb_im,
&m6800_cpu_device::eorb_im,&m6800_cpu_device::orb_im, &m6800_cpu_device::adcb_im,&m6800_cpu_device::addb_im,&m6800_cpu_device::illegl1,&m6800_cpu_device::ldx_im, &m6800_cpu_device::illegl1,&m6800_cpu_device::stx_im,
&m6800_cpu_device::subb_di,&m6800_cpu_device::sbcb_di,&m6800_cpu_device::cmpb_di,&m6800_cpu_device::illegl1,&m6800_cpu_device::andb_di,&m6800_cpu_device::ldb_di, &m6800_cpu_device::bitb_di,&m6800_cpu_device::stb_di,
&m6800_cpu_device::eorb_di,&m6800_cpu_device::orb_di, &m6800_cpu_device::adcb_di,&m6800_cpu_device::addb_di,&m6800_cpu_device::illegl1,&m6800_cpu_device::ldx_di, &m6800_cpu_device::illegl1,&m6800_cpu_device::stx_di,
&m6800_cpu_device::subb_ix,&m6800_cpu_device::sbcb_ix,&m6800_cpu_device::cmpb_ix,&m6800_cpu_device::illegl1,&m6800_cpu_device::andb_ix,&m6800_cpu_device::ldb_ix, &m6800_cpu_device::bitb_ix,&m6800_cpu_device::stb_ix,
&m6800_cpu_device::eorb_ix,&m6800_cpu_device::orb_ix, &m6800_cpu_device::adcb_ix,&m6800_cpu_device::addb_ix,&m6800_cpu_device::adcx_im,&m6800_cpu_device::ldx_ix, &m6800_cpu_device::illegl1,&m6800_cpu_device::stx_ix,
&m6800_cpu_device::subb_ex,&m6800_cpu_device::sbcb_ex,&m6800_cpu_device::cmpb_ex,&m6800_cpu_device::illegl1,&m6800_cpu_device::andb_ex,&m6800_cpu_device::ldb_ex, &m6800_cpu_device::bitb_ex,&m6800_cpu_device::stb_ex,
&m6800_cpu_device::eorb_ex,&m6800_cpu_device::orb_ex, &m6800_cpu_device::adcb_ex,&m6800_cpu_device::addb_ex,&m6800_cpu_device::addx_ex,&m6800_cpu_device::ldx_ex, &m6800_cpu_device::illegl1,&m6800_cpu_device::stx_ex
};
