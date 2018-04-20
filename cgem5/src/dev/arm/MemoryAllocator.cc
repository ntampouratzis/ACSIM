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

#include <iostream>
#include "MemoryAllocator.hh"
#include <stdio.h>
#include <assert.h>

using namespace std;


MemoryAllocator::MemoryAllocator(){
  init_thread();
}




void MemoryAllocator::initFastList(){
    int j = 0;
    //NULL Pointer
    SRAM[0].next = 0;


    //!Every SRAM Entry initially points to its succeeding Entry
    for(j=1;j<ELEMENTS_NUM-1;j++){
      SRAM[j].next   = j+1;
    }

    SRAM[j].next = 0;


    for(j=0;j<M+1;j++){
      allocated[j].head = 0;
      allocated[j].tail = 0;
      allocated[j].num_elements = 0;

    }

    unallocated[0].head = 1;
    unallocated[0].tail = ELEMENTS_NUM - 1;
    unallocated[0].num_elements = ELEMENTS_NUM - 1;

}



int MemoryAllocator::match(allocatorSegment s1, allocatorSegment s2){
	//printf("s1.tag: %d | s2.tag: %d\n",s1.tag,s2.tag);
	//printf("s1.address: %ld | s2.address: %ld\n",s1.address,s2.address);
	if(s2.address != 0){
		return ((s1.tag == s2.tag) && (s1.address == s2.address));
	}
	else{
		return (s1.tag == s2.tag);
    }
}

unsigned char MemoryAllocator::searchFastList(allocatorSegment element, int listId, int * previous, int * current){

   unsigned long iterator = allocated[listId].head;
    *previous = 0;
    for(;iterator;){

      if(match(SRAM[iterator].element,element)){
		*current = iterator;
		return 1;
      }
      *previous = iterator;
      iterator=SRAM[iterator].next;
    }
    return 0;
 }


void MemoryAllocator::insertFastListItem(allocatorSegment element, int listId){
   //If the allocated list is empty the tail points to NULL
    //The inserted item will be inserted at the element being
    //pointed by unlocated list head
    if(allocated[listId].tail == 0){
      allocated[listId].tail = unallocated[0].head;
      allocated[listId].head = unallocated[0].head;
    }
    //Else the next member of the element pointed by allocated[listId] list's tail
    //should point to unallocated[listId] list's head, the allocated[listId] list's tail
    //should be updated to point to the element pointed by unallocated[listId] list's
    //head
    else{
      //! To keep the allocated[listId].tail stable
      SRAM[allocated[listId].tail].next = unallocated[0].head;
      allocated[listId].tail = unallocated[0].head;
    }

    //Data should be copied to the payload member of the element pointed
    //by allocated[listId] list's tail pointer
    SRAM[allocated[listId].tail].element = element;

    //Unallocated[listId] list's head should point to the address of the next
    //pointer of the element is points to
    unallocated[0].head = SRAM[unallocated[0].head].next;

    //The new element's next member should be updated to NULL
    SRAM[allocated[listId].tail].next = 0;

    //Set the new element numbers of the two underlying lists
    allocated[listId].num_elements++;
    unallocated[0].num_elements--;


 }

void MemoryAllocator::removeFastListItem(allocatorSegment element, int listId){
   //A list search should have been performed setting the current and
    //previous pointers.
    //If there exists a previous element to the one found it should be
    //updated so that its next member point to element's next member

    int previous, current;

    int Found = searchFastList(element,listId, &previous, &current);

    if(!Found){
     return;
    }

    if(previous != 0){
      SRAM[previous].next = SRAM[current].next;
    }
    else{ //First element
      allocated[listId].head = SRAM[current].next;
    }


    //Last element
    if(allocated[listId].tail == current){
      allocated[listId].tail = previous;
    }
    //Removed elements next pointer is updated to point to unallocated
    //list's head pointer.
    SRAM[current].next = unallocated[0].head;

    //The unallocated list's head is updated to point to the removed
    //element.
    unallocated[0].head = current;

    //Last element
    if(unallocated[0].tail == 0){
      unallocated[0].tail = current;
    }

    //Set the new element numbers of the two underlying lists
    unallocated[0].num_elements++;
    allocated[listId].num_elements--;


 }


unsigned long MemoryAllocator::higher_power_of_two(unsigned long num){

  char leadZeroPos = 32;

  unsigned long mask0 = 1 << (leadZeroPos-1);

  for(;leadZeroPos>=0;leadZeroPos--){
    if(num & mask0){
      break;
    }
    mask0 >>= 1;
  }

  char leadOnePos = leadZeroPos-2;

  unsigned long mask1 = 1 << (leadZeroPos-2);

  for(;leadOnePos>=0;leadOnePos--){
    if(num & mask1){
      break;
    }
    mask1 >>= 1;
  }

  return (leadOnePos>=0)?leadZeroPos:leadZeroPos-1;

}


unsigned long MemoryAllocator::alloc_thread(unsigned long size){
    
    //printf("MA Alloc: Allocating Element : ");
    //printf("Size: %ld, ",size);

    unsigned long k = higher_power_of_two(size);
    
    //printf("Class: %ld\n",k);
    
    for(unsigned long j = k;j <= M;){

      //!Search in list j for an available item i.e. with tag=0
      allocatorSegment segment;
      segment.tag = 0;
      segment.address = 0;
      unsigned char itemFound;
      int previous, current;
      itemFound = searchFastList(segment, j,&previous,&current);
      
      //!In the case that an element with tag==0 was found
      if(itemFound == 1){
        /*! 
         * If the element was found in the list with size 2^k,
         * allocate it add the BASE_ADDRESS value to it and return.
         */
        if(j == k){

          SRAM[current].element.tag = 1;
          SRAM[current].element.size = size;

          //printf("MA Alloc: Available Chunk Found at List %ld\n",k);
          //printf("MA Alloc: Allocating it and Returning...\n");
	  return (SRAM[current].element.address + BASE_ADDRESS);

        }
        /*!
         * If an element was found at a list j such that j != k,
         * remove it from list j and insert two items at list j-1
         * with half the size of the removed item each
         */ 
        else{
          //Initiate a delete command at the item at list j. 
	  removeFastListItem(segment, j);
	  //printf("MA Alloc: Removed an available element from List %ld\n",j);

          /*!
           *Insert two buddy blocks with sizes equal to 2^(j-1), i.e.
           *half the size of the removed element
           *The address of the first buddy matches the address of the
           *original item stemming from the higher class
           */
          allocatorSegment seg;
          seg.size = 1 << (j-1);
          seg.tag = 0;
          seg.address = SRAM[current].element.address;
          insertFastListItem(seg, j-1);

          //The address of the second buddy equals the original address
          //increased by the size of the lower class. The address
          //difference of the two budies is thus equal to the size of
          //their list class.
          segment.size = 1 << (j-1);
          segment.tag = 0;
          segment.address = SRAM[current].element.address + (1 << (j-1));
          insertFastListItem(segment, j-1);
          
          //printf("MA Alloc: Inserted two buddies (%ld , %ld ) in List %ld\n",SRAM[current].element.address,SRAM[current].element.address+ (1 << (j-1)),j-1);


          j = j - 1;

        }
      }
      else{
        /*!
         * No element with tag=0 found, so move to the higher level classes
         *of lists.
         */
        //cout << "MA Alloc: No available chunk at Class "<<j<<"\n";

        j = j + 1;
      }
    }
  printf("ERROR: Memory Fully Utilized\n");
  assert(0);
  return(0);
}

void MemoryAllocator::free_thread(unsigned long size, unsigned long address_free){

    /*!
     * The free thread begins with identifying the class in which the
     * element is stored. The above class is stored in k
     */
    unsigned long k = higher_power_of_two(size);

    unsigned char itemFound;
    int previous = 0, current = 0;

    /*
     * A list search operation is initiated at list class k with tag
     * value = 1 as we are looking for an already allocated segment.
     * The value of the address being freed is also stored in the
     * address field and should match an element at least k.
     */
    allocatorSegment segment;
    segment.tag = 1;
    segment.address = address_free - BASE_ADDRESS;

    if(segment.address == 0){
            printf("MA Free (FATAL ERROR): Address %ld is the NULL word and should never be freed\n",segment.address);
    }

    itemFound = searchFastList(segment, k,&previous,&current);

    if(itemFound == 1){
    	//printf("MA Free: Item Found at List %ld. Set its tag to 0\n",k);
    }
    else{
    	printf("MA Free (ERROR): Address %ld not found in List %ld\n",address_free,k);
    }

    //! Free the element by setting its tag to 0.
    SRAM[current].element.tag = 0;

    unsigned long address = address_free;

    /*!Begin traversing the higher order classes starting from k. If the
     * just freed element and its buddy are both freed they are both
     * removed from their corresponding class j, they are merged in a
     * single entry of double their size (2^(j+1)) and this element is
     * inserted at the class j+1. The above process terminates as soon
     * as the buddy of the freed element is allocated.
     */
    for(unsigned long j = k;j<=M;){
   
      unsigned long buddy_address = (address & ((1<<(j+1))-1))?
                                    address - (1<<j):
                                    address + (1<<j);

      /*!
       * A special case is when the buddy is the word NULL in which case
       * the free also stops.
       */
      if(buddy_address == 0){
    	  //printf("Buddy with address 0 cannot be merged.\n");
        break;
      }

      /*!
       * Initiate a LIST SEARCH for element's buddy. The tag should be 0
       * as it should be free and its address should be as calculated
       * above.
       */
      segment.tag = 0;
      segment.address = buddy_address;

      itemFound = searchFastList(segment, j,&previous,&current);

      /*!
       * If either the element is not found or it is found but it is not
       * freed, the classes traversal should stop.
       */
      if(itemFound == 0){
    	  //printf("MA Free: Item's Buddy (Address= %ld) was either not Found in List %ld, or it was "
    	  //                "found but it was Allocated\n",buddy_address,j);
        break;
      }

      //printf("MA Free: Item's Buddy (Address=%ld) Found in List %ld and it is Free\n",buddy_address,j);

      /*!
       * Elsewhere, if the element is found and it is already freed,
       * it should be deleted from list-class j and the same should
       * happen with its buddy.
       */

      //!Initiate two List Delete operations, one for the freed element
      //and one for its buddy.
      segment.tag = 0;
      segment.address = buddy_address;
      removeFastListItem(segment, j);

      //printf("MA Free: Deleted Item with address %ld\n",buddy_address);
      segment.tag = 0;
      segment.address = address;
      removeFastListItem(segment, j);
      //printf("MA Free: Deleted Item with address %ld\n",address);

      /*!
       * Create an element merging the former two buddies and insert in
       * list class j + 1.
       */
      segment.tag = 0;
      /*!
       * The address is the AND of the two buddy addresses which only
       * differ in a single bit, which is set to 0 for the new address.
       */
      segment.address = address & buddy_address;
      segment.size = 1 << (j+1);

      insertFastListItem(segment, j+1);
      //printf("MA Free: Inserted Item with address %ld at List %ld\n",address,j+1);

      /*
       *!Update the address which is just inserted in j+1 and also
       *freed.
       */
      address = address & buddy_address;

      //!Go to a higher list class.
      j = j + 1;

    }

}


void MemoryAllocator::init_thread(){

    initFastList();

    /*!
     * In the initial state of the memory allocator, address=0 which
     * represents the NULL has allocated. It is probihibited for any
     * free command to free this location. In order to set this initial
     * scenario the list of class M has its single element allocated,
     * all the lists from j=M-1 to 0 have a free element for further use
     * at address j except from list of class 0 which has its NULL
     * element allocated.
     */

    allocatorSegment segment;
    for(int j = 0;j <= M-1;j++){
      segment.size = 1 << j;
      segment.tag = 0;
      segment.address = 1 << j;
      insertFastListItem(segment, j);
    }

    //Allocate the single chunk which represents the whole memory space
    //in list of class M
    segment.size = 1 << M;
    segment.tag = 1;
    segment.address = 0;
    insertFastListItem(segment, M);

    //! Allocate the NULL address
    segment.size = 1;
    segment.tag = 1;
    segment.address = 0;
    insertFastListItem(segment, 0);

}
