/*
 * Copyright (C) 2018 Elnico s.r.o.
 *
 * Author: Petr Kubiznak <kubiznak.petr@elnico.cz>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __SQM4SX6_H
#define __SQM4SX6_H

int sqm4_dram_init(void);
int sqm4_board_init(void);

#ifdef CONFIG_SPL_BUILD
void sqm4_board_init_f(ulong dummy);
#endif /* CONFIG_SPL_BUILD */

#endif /* __SQM4SX6_H */
