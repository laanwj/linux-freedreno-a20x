/*
 * Marvell 88E6xxx Switch Global 3 Registers (TCAM) support
 *
 * Copyright (c) 2016 Andrew Lunn <andrew@lunn.ch>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _MV88E6XXX_GLOBAL3_H
#define _MV88E6XXX_GLOBAL3_H

//#include "chip.h"

#define MV88E6XXX_TCAM_PARAM_DISABLED	INT_MIN

struct mv88e6xxx_tcam_data {
	u16 page0[32];
	u16 page1[32];
	u16 page2[32];
};

enum mv88e6xxx_tcam_param {
	MV88E6XXX_P0_KEY1_FRAME_TYPE,
	MV88E6XXX_P0_KEY2_SRC_PORT_VECTOR,
	MV88E6XXX_P0_KEY3_PPRI,
	MV88E6XXX_P0_KEY4_PVID,
	MV88E6XXX_P2_ACTION1_INTERRUPT,
	MV88E6XXX_P2_ACTION1_INC_TCAM_COUNTER,
	MV88E6XXX_P2_ACTION1_VID,
	MV88E6XXX_P2_ACTION2_FLOW_ID,
	MV88E6XXX_P2_ACTION2_QPRI,
	MV88E6XXX_P2_ACTION2_FPRI,
	MV88E6XXX_P2_ACTION3_DST_PORT_VECTOR,
	MV88E6XXX_P2_ACTION4_FRAME_ACTION,
	MV88E6XXX_P2_ACTION4_LOAD_BALANCE,
	MV88E6XXX_P2_DEBUG_PORT,
	MV88E6XXX_P2_DEBUG_HIT,
};

#ifdef CONFIG_NET_DSA_MV88E6XXX_GLOBAL3

int mv88e6xxx_g3_tcam_get_next(struct mv88e6xxx_chip *chip, u32 *entry,
			       struct mv88e6xxx_tcam_data *data);
int mv88e6xxx_g3_tcam_read(struct mv88e6xxx_chip *chip, u32 entry,
			   struct mv88e6xxx_tcam_data *data);
int mv88e6xxx_g3_tcam_load_entry(struct mv88e6xxx_chip *chip, u32 entry,
				 struct mv88e6xxx_tcam_data *data);
int mv88e6xxx_g3_tcam_flush_entry(struct mv88e6xxx_chip *chip, u32 entry);
int mv88e6xxx_g3_tcam_flush_all(struct mv88e6xxx_chip *chip);
int mv88e6xxx_g3_tcam_set(struct mv88e6xxx_chip *chip,
			  struct mv88e6xxx_tcam_data *data,
			  enum mv88e6xxx_tcam_param param, u16 value);
int mv88e6xxx_g3_tcam_get(struct mv88e6xxx_chip *chip,
			  struct mv88e6xxx_tcam_data *data,
			  enum mv88e6xxx_tcam_param param, int *value);
int mv88e6xxx_g3_tcam_get_match(struct mv88e6xxx_chip *chip,
				struct mv88e6xxx_tcam_data *data,
				unsigned int offset, u8 *octet, u8 *mask);
int mv88e6xxx_g3_tcam_set_match(struct mv88e6xxx_chip *chip,
				struct mv88e6xxx_tcam_data *data,
				unsigned int offset, u8 octet, u8 mask);

#else /* !CONFIG_NET_DSA_MV88E6XXX_GLOBAL3 */

int mv88e6xxx_g3_tcam_get_next(struct mv88e6xxx_chip *chip, u32 *entry,
			       struct mv88e6xxx_tcam_data *data)
{
	return -EOPNOTSUPP;
}

int mv88e6xxx_g3_tcam_read(struct mv88e6xxx_chip *chip, u32 entry,
			   struct mv88e6xxx_tcam_data *data)
{
	return -EOPNOTSUPP;
}

int mv88e6xxx_g3_tcam_load_entry(struct mv88e6xxx_chip *chip, u32 entry,
				 struct mv88e6xxx_tcam_data *data)
{
	return -EOPNOTSUPP;
}

int mv88e6xxx_g3_tcam_flush_entry(struct mv88e6xxx_chip *chip, u32 entry)
{
	return -EOPNOTSUPP;
}

int mv88e6xxx_g3_tcam_flush_all(struct mv88e6xxx_chip *chip)
{
	return -EOPNOTSUPP;
}

int mv88e6xxx_g3_tcam_set(struct mv88e6xxx_chip *chip,
			  struct mv88e6xxx_tcam_data *data,
			  enum mv88e6xxx_tcam_param param, u16 value)
{
	return -EOPNOTSUPP;
}

int mv88e6xxx_g3_tcam_get(struct mv88e6xxx_chip *chip,
			  struct mv88e6xxx_tcam_data *data,
			  enum mv88e6xxx_tcam_param param, int *value)
{
	return -EOPNOTSUPP;
}

int mv88e6xxx_g3_tcam_get_match(struct mv88e6xxx_chip *chip,
				struct mv88e6xxx_tcam_data *data,
				unsigned int offset, u8 *octet, u8 *mask)
{
	return -EOPNOTSUPP;
}

int mv88e6xxx_g3_tcam_set_match(struct mv88e6xxx_chip *chip,
				struct mv88e6xxx_tcam_data *data,
				unsigned int offset, u8 octet, u8 mask)
{
	return -EOPNOTSUPP;
}

#endif /* CONFIG_NET_DSA_MV88E6XXX_GLOBAL3 */

#endif /* _MV88E6XXX_GLOBAL3_H */
