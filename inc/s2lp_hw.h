/*
 * s2lp_hw.h
 *
 *  Created on: 17 nov. 2024
 *      Author: Ludo
 */

#ifndef __S2LP_HW_H__
#define __S2LP_HW_H__

#ifndef S2LP_DRIVER_DISABLE_FLAGS_FILE
#include "s2lp_driver_flags.h"
#endif
#include "s2lp.h"
#include "types.h"

#ifndef S2LP_DRIVER_DISABLE

/*** S2LP HW functions ***/

/*!******************************************************************
 * \fn S2LP_status_t S2LP_HW_init(void)
 * \brief Init S2LP hardware interface.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_HW_init(void);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_HW_de_init(void)
 * \brief Release S2LP hardware interface.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_HW_de_init(void);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_HW_spi_write_read_8(uint8_t* tx_data, uint8_t* rx_data, uint8_t transfer_size)
 * \brief Write data to transceiver over SPI interface.
 * \param[in]   tx_data: Byte array to send.
 * \param[in]   transfer_size: Number of bytes to send and receive.
 * \param[out]  rx_data: Pointer to the received bytes.
 * \retval      Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_HW_spi_write_read_8(uint8_t* tx_data, uint8_t* rx_data, uint8_t transfer_size);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_HW_set_sdn_gpio(uint8_t state)
 * \brief Set S2LP shutdown pin state.
 * \param[in]   state: SDN pin state to apply.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_HW_set_sdn_gpio(uint8_t state);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_HW_delay_milliseconds(uint32_t delay_ms)
 * \brief Delay function.
 * \param[in]   delay_ms: Delay to wait in ms.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_HW_delay_milliseconds(uint32_t delay_ms);

#endif /* S2LP_DRIVER_DISABLE */

#endif /* __S2LP_HW_H__ */
