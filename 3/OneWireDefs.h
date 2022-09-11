/*
 * Defs.h
 *
 *  Created on: 3 џэт. 2021 у.
 *      Author: Brylov Boris
 */

#ifndef ONEWIREDEFS_H_
#define ONEWIREDEFS_H_
namespace Mcucpp {
#define TimeoutTransfer 10000
typedef enum {
    Success = 1,
    NotDetectReset = -1,
    Not1WireDevises = -2,
    Error1Wire = -3,
} Returning;


}   // namespace Mcucpp
#endif /* ONEWIREDEFS_H_ */
