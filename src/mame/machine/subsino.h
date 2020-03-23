// license:BSD-3-Clause
// copyright-holders:Luca Elia, David Haywood, Angelo Salese, Roberto Fresca
#ifndef __MACHINE_SUBSINO__
#define __MACHINE_SUBSINO__


extern const uint8_t crsbingo_xors[8];
extern const uint8_t sharkpy_xors [8];
extern const uint8_t victor5_xors [8];
extern const uint8_t victor21_xors[8];

void crsbingo_bitswaps(uint8_t *decrypt, int i);
void sharkpy_bitswaps (uint8_t *decrypt, int i);
void victor5_bitswaps (uint8_t *decrypt, int i);
void victor21_bitswaps(uint8_t *decrypt, int i);

void subsino_decrypt(running_machine& machine, void (*bitswaps)(uint8_t *decrypt, int i), const uint8_t *xors, int size);

#endif
