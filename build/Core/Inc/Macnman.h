/*
 * Macnman.h
 *
 *  Created on: Mar 15, 2024
 *      Author: LENOVO
 */

#ifndef INC_MACNMAN_H_
#define INC_MACNMAN_H_


#define HARDWARE_VERSION_MAIN   (0x01U) /*!< [31:24] main version */
#define HARDWARE_VERSION_SUB1   (0x02U) /*!< [23:16] sub1 version */
#define HARDWARE_VERSION_SUB2   (0x01U) /*!< [15:8]  sub2 version */
#define HARDWARE_TYPE           (0x01U) /*!< [7:0] type version */
#define HARDWARE_VERSION        ((HARDWARE_VERSION_MAIN  << 24) \
                                   |(HARDWARE_VERSION_SUB1 << 16) \
                                   |(HARDWARE_VERSION_SUB2 << 8)  \
                                   |(HARDWARE_TYPE))

#define SOFTWARE_VERSION_MAIN   (0x01U) /*!< [31:24] main version */
#define SOFTWARE_VERSION_SUB1   (0x03U) /*!< [23:16] sub1 version */
#define SOFTWARE_VERSION_SUB2   (0x00U) /*!< [15:8]  sub2 version */
#define SOFTWARE_TYPE           (0x01U) /*!< [7:0] type version */
#define SOFTWARE_VERSION        ((SOFTWARE_VERSION_MAIN  << 24) \
                                   |(SOFTWARE_VERSION_SUB1 << 16) \
                                   |(SOFTWARE_VERSION_SUB2 << 8)  \
                                   |(SOFTWARE_TYPE))


#endif /* INC_MACNMAN_H_ */
