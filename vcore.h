#ifndef __VCORE_H
#define __VCORE_H

//Some of this came from TI with the following note:
/* --COPYRIGHT--,BSD
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

#include <stdint.h>


//*****************************************************************************
//
// The following are values that can be passed to the level parameter for
// functions: PMM_setVCoreUp(), PMM_setVCoreDown(), and PMM_setVCore().
//
//*****************************************************************************
#define PMM_CORE_LEVEL_0                                             PMMCOREV_0
#define PMM_CORE_LEVEL_1                                             PMMCOREV_1
#define PMM_CORE_LEVEL_2                                             PMMCOREV_2
#define PMM_CORE_LEVEL_3                                             PMMCOREV_3

//*****************************************************************************
//
//! \brief Increase Vcore by one level
//!
//! \param level level to which Vcore needs to be increased
//!        Valid values are:
//!        - \b PMM_CORE_LEVEL_0 [Default]
//!        - \b PMM_CORE_LEVEL_1
//!        - \b PMM_CORE_LEVEL_2
//!        - \b PMM_CORE_LEVEL_3
//!
//! Modified bits of \b PMMCTL0 register, bits of \b PMMIFG register, bits of
//! \b PMMRIE register, bits of \b SVSMHCTL register and bits of \b SVSMLCTL
//! register.
//!
//! \return STATUS_SUCCESS or STATUS_FAIL
//
//*****************************************************************************
extern uint16_t PMM_setVCoreUp(uint8_t level);

//*****************************************************************************
//
//! \brief Decrease Vcore by one level
//!
//! \param level level to which Vcore needs to be decreased
//!        Valid values are:
//!        - \b PMM_CORE_LEVEL_0 [Default]
//!        - \b PMM_CORE_LEVEL_1
//!        - \b PMM_CORE_LEVEL_2
//!        - \b PMM_CORE_LEVEL_3
//!
//! Modified bits of \b PMMCTL0 register, bits of \b PMMIFG register, bits of
//! \b PMMRIE register, bits of \b SVSMHCTL register and bits of \b SVSMLCTL
//! register.
//!
//! \return STATUS_SUCCESS
//
//*****************************************************************************
extern uint16_t PMM_setVCoreDown(uint8_t level);

//*****************************************************************************
//
//! \brief Set Vcore to expected level
//!
//! \param level level to which Vcore needs to be decreased/increased
//!        Valid values are:
//!        - \b PMM_CORE_LEVEL_0 [Default]
//!        - \b PMM_CORE_LEVEL_1
//!        - \b PMM_CORE_LEVEL_2
//!        - \b PMM_CORE_LEVEL_3
//!
//! Modified bits of \b PMMCTL0 register, bits of \b PMMIFG register, bits of
//! \b PMMRIE register, bits of \b SVSMHCTL register and bits of \b SVSMLCTL
//! register.
//!
//! \return STATUS_SUCCESS or STATUS_FAIL
//
//*****************************************************************************
extern int PMM_setVCore(uint8_t level);

#endif
