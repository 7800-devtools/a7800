// license:BSD-3-Clause
// copyright-holders:Luca Elia, David Haywood,Stephane Humbert
/* Kaneko Hit protection */
#ifndef MAME_MACHINE_KANEKO_HIT_H
#define MAME_MACHINE_KANEKO_HIT_H

#pragma once

#include "machine/watchdog.h"


class kaneko_hit_device : public device_t
{
public:
	kaneko_hit_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	static void set_type(device_t &device, int hittype);

	int m_hittype;

	DECLARE_READ16_MEMBER(kaneko_hit_r);
	DECLARE_WRITE16_MEMBER(kaneko_hit_w);

protected:
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	struct calc1_hit_t
	{
		uint16_t x1p, y1p, x1s, y1s;
		uint16_t x2p, y2p, x2s, y2s;

		int16_t x12, y12, x21, y21;

		uint16_t mult_a, mult_b;
	};

	struct calc3_hit_t
	{
		int x1p, y1p, z1p, x1s, y1s, z1s;
		int x2p, y2p, z2p, x2s, y2s, z2s;

		int x1po, y1po, z1po, x1so, y1so, z1so;
		int x2po, y2po, z2po, x2so, y2so, z2so;

		int x12, y12, z12, x21, y21, z21;

		int x_coll, y_coll, z_coll;

		int x1tox2, y1toy2, z1toz2;

		uint16_t mult_a, mult_b;

		uint16_t flags;
		uint16_t mode;
	};



	required_device<watchdog_timer_device> m_watchdog;

	calc1_hit_t m_hit;
	calc3_hit_t m_hit3;

	DECLARE_READ16_MEMBER(kaneko_hit_type0_r);
	DECLARE_WRITE16_MEMBER(kaneko_hit_type0_w);

	DECLARE_READ16_MEMBER(kaneko_hit_type1_r);
	DECLARE_WRITE16_MEMBER(kaneko_hit_type1_w);
	int16_t calc_compute_x(calc1_hit_t &hit);
	int16_t calc_compute_y(calc1_hit_t &hit);

	DECLARE_WRITE16_MEMBER(kaneko_hit_type2_w);
	DECLARE_READ16_MEMBER(kaneko_hit_type2_r);
	int type2_calc_compute(int x1, int w1, int x2, int w2);
	void type2_calc_org(int mode, int x0, int s0,  int* x1, int* s1);
	void type2_recalc_collisions(calc3_hit_t &hit3);
};


DECLARE_DEVICE_TYPE(KANEKO_HIT, kaneko_hit_device)

#endif // MAME_MACHINE_KANEKO_HIT_H
