/**
 * @author Adrian Cinal
 * @file wcu_defs.h
 * @brief Header file containing WCU definitions and macros
 */

#ifndef __WCU_DEFS_H_
#define __WCU_DEFS_H_

#include <stddef.h>
#include <stdint.h>

/* Exported typedefs -------------------------------------------------------------------------- */
typedef float float32_t;
typedef double float64_t;

/**
 * @brief WCU functions return value
 */
typedef enum EWcuRet {
    EWcuRet_Ok = 0,
    EWcuRet_InvalidParams,
    EWcuRet_Error
} EWcuRet;

#endif /* __WCU_DEFS_H_ */
