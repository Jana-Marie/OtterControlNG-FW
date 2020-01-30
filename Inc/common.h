/*
* This file is part of the stmbl project.
*
* Copyright (C) 2013-2017 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2013-2017 Nico Stute <crinq@crinq.de>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#include <stdint.h>

#if __GNUC__ < 5
#error gcc to old (< 5.0)
#endif

typedef enum{
  VOLT_MODE = 0,
  CURRENT_MODE,
} packet_to_hv_cmd_type_t;

typedef enum{
  PHASE_90_3PH = 0,
  PHASE_90_4PH,
  PHASE_120_3PH,
  PHASE_180_2PH,
  PHASE_180_3PH,
} packet_to_hv_phase_type_t;

//fault state
typedef enum {
  DISABLED = 0,
  ENABLED,
  PHASING,
  SOFT_FAULT,
  HARD_FAULT,
  LED_TEST,
} state_t;