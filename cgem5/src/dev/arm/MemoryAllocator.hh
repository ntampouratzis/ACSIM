/*
 * Copyright (c) 2018 The Regents of The Technical University of Crete
 * All rights reserved.
 * 
 * ----------------------------------------------------------------------------
 * Buddy Dynamic Memory Allocator
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
 * The Buddy dynamic memory allocation algorithm scheme is implemented 
 * in the Accelerator Wrapper to allocate and free Device Memory segments 
 * through the cGEM5 operating system; You can referee in this paper 
 * https://ieeexplore.ieee.org/document/7927071/
 */

#define BASE_ADDRESS 0
#define M 31               /* 2^12 is the size in bytes in total memory */
#define ELEMENTS_NUM 4096  /* MAX_ELEMENTS is the maximum number of messages that the allocator is able to handle*/

//! Structure for the List entries
typedef struct List{
  
  //! Head field
  unsigned long head;

  //! Tail field
  unsigned long tail;

  //! The Number of elements in the list
  unsigned long num_elements;

} List;


typedef struct allocatorSegment{

  //! Size of the segment
  unsigned long size;

  //! Tag of the segment
  int tag; // 0: Free | 1: Allocated

  //! Address of the Segment
  unsigned long address;

}allocatorSegment;



//! Structure for the SRAM entries
typedef struct SRAMEntry{
  
  //! Header Field
  allocatorSegment element;

  //! Next entry field
  unsigned long next;

}SRAMEntry;


class MemoryAllocator {
  public:
    MemoryAllocator();
    //virtual ~MemoryAllocator();
    unsigned long alloc_thread(unsigned long size);
    void free_thread(unsigned long size, unsigned long address_free);
  protected:
    //! The list with the allocated SRAM Entries
    List allocated[M+1];

    //! The list with the free Entries
    List unallocated[1];

    //!The SRAM
    SRAMEntry SRAM[ELEMENTS_NUM];
    
    
    void initFastList();
    void insertFastListItem(allocatorSegment element, int listId);
    unsigned char searchFastList(allocatorSegment element, int listId, int * previous, int * current);
    void removeFastListItem(allocatorSegment element, int listId);
    int match(allocatorSegment s1, allocatorSegment s2);
    unsigned long higher_power_of_two(unsigned long num);
    void init_thread();
    
};