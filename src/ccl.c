/**
 * \file
 *
 * \brief CCL related functionality implementation.
 *
 (c) 2020 Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms,you may use this software and
    any derivatives exclusively with Microchip products.It is your responsibility
    to comply with third party license terms applicable to your use of third party
    software (including open source software) that may accompany Microchip software.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 */

/**
 * \addtogroup doc_driver_ccl
 *
 * \section doc_driver_ccl_rev Revision History
 * - v0.0.0.1 Initial Commit
 *
 *@{
 */
#include <ccl.h>

/**
 * \brief Initialize CCL peripheral
 * \return Return value 0 if success
 */
int8_t DIGITAL_GLUE_LOGIC_0_init()
{

	CCL.TRUTH1 = 0xE0; /* Truth 1: 224 */ // In 0 and 2 is reversed compared to original

	CCL.LUT1CTRLC = CCL_INSEL2_SPI0_gc /* SPI0 SCK source (was TCA0 WO2 input source) */;

	CCL.LUT1CTRLB = CCL_INSEL0_TCB0_gc /* TCA0 WO0 input source (was SPI0 SCK source) */
	                | CCL_INSEL1_SPI0_gc /* SPI0 MOSI input source */;

	CCL.LUT1CTRLA =   CCL_CLKSRC_CLKPER_gc       /* Clock Source Selection: disabled */
	                | CCL_EDGEDET_DIS_gc     /* Edge detector is disabled */
	                | CCL_FILTSEL_DISABLE_gc /* Filter disabled */
	                | 1 << CCL_ENABLE_bp     /* LUT Enable: enabled */
	                | 1 << CCL_OUTEN_bp;     /* Output Enable: enabled */

	CCL.CTRLA = 1 << CCL_ENABLE_bp      /* Enable: enabled */
	            | 1 << CCL_RUNSTDBY_bp; /* Run in Standby: enabled */

	return 0;
}
