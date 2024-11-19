/*
 * s2lp_hw.c
 *
 *  Created on: 17 nov. 2024
 *      Author: Ludo
 */

#include "s2lp_hw.h"

#ifndef S2LP_DRIVER_DISABLE_FLAGS_FILE
#include "s2lp_driver_flags.h"
#endif
#include "s2lp.h"
#include "types.h"

#ifndef S2LP_DRIVER_DISABLE

/*** S2LP HW functions ***/

/*******************************************************************/
S2LP_status_t __attribute__((weak)) S2LP_HW_init(void) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    /* To be implemented */
    return status;
}

/*******************************************************************/
S2LP_status_t __attribute__((weak)) S2LP_HW_de_init(void) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    /* To be implemented */
    return status;
}

/*******************************************************************/
S2LP_status_t __attribute__((weak)) S2LP_HW_spi_write_read_8(uint8_t* tx_data, uint8_t* rx_data, uint8_t transfer_size) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    /* To be implemented */
    UNUSED(tx_data);
    UNUSED(rx_data);
    UNUSED(transfer_size);
    return status;
}

/*******************************************************************/
S2LP_status_t __attribute__((weak)) S2LP_HW_set_sdn_gpio(uint8_t state) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    /* To be implemented */
    UNUSED(state);
    return status;
}

/*******************************************************************/
S2LP_status_t __attribute__((weak)) S2LP_HW_delay_milliseconds(uint32_t delay_ms) {
    // Local variables.
    S2LP_status_t status = S2LP_SUCCESS;
    /* To be implemented */
    UNUSED(delay_ms);
    return status;
}

#endif /* S2LP_DRIVER_DISABLE */
