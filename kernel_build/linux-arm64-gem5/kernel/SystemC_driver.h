/*
 * Copyright (c) 2018 The Regents of The Technical University of Crete
 * All rights reserved.
 * 
 * ----------------------------------------------------------------------------
 * Accelerator Kernel Driver
 * Copyright (c) 2018, H2020 ECOSCALE.
 * Copyright (c) 2018, Telecommunications Systems Institute.
 * ----------------------------------------------------------------------------
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Tampouratzis Nikolaos, ntampouratzis@isc.tuc.gr
 */

/* @file
 * Accelerator Kernel Driver. You can referee in this paper 
 * https://ieeexplore.ieee.org/document/7927071/
 */
#ifndef QUERY_IOCTL_H
#define QUERY_IOCTL_H
#include <linux/ioctl.h>
 


typedef struct
{
    size_t size;
    uint64_t SWAddr;
} parameters_t;

typedef struct
{
    size_t size;
    char DevMemName;
    uint64_t AllocAddr;
} MallocParams_t;
 
#define QUERY_GET_DATA _IOR('q', 1, void *)
#define QUERY_CALL_DEVICE 	_IOR('q', 2, uint8_t *)
#define QUERY_SET_DATA 		_IOW('q', 3, void *)
#define QUERY_SET_PARAMETERS 	_IOW('p', 4, parameters_t *)
#define QUERY_WAIT _IOR('q', 5, uint8_t *)
#define DMA_FROM_DEVICE_START _IO('q', 6)
#define DMA_FROM_DEVICE_WAIT _IOR('q', 7, uint8_t *)
#define DMA_TO_DEVICE_WAIT _IOR('q', 8, uint8_t *)
#define QUERY_MALLOC 	_IOWR('p', 9, MallocParams_t *)
#define QUERY_FREE 	_IOW('p', 10, uint64_t *)

#endif
