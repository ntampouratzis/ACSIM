#include "AccelDriver.h"


uint64_t CPUNonBlockingCode(){
  uint64_t sum = 0;
  uint64_t i;
  
  for(i=0;i<100000;i++){
      sum+= i;
    }
    return sum;
}

#define DATA_SIZE 18

int main(int argc, char *argv[])
{
    
    AccelInitialization();
    uint64_t i;
    DevMemAddr ArrayA = AccelMalloc(1000, 'A');
    DevMemAddr ArrayB = AccelMalloc(2048, 'B');
    DevMemAddr ArrayC = AccelMalloc(512, 'C');
    DevMemAddr ArrayD = AccelMalloc(2048, 'D');
    DevMemAddr ArrayE = AccelMalloc(8, 'E');
    
    printf("ArrayA: %d\n",(int)ArrayA);
    printf("ArrayB: %d\n",(int)ArrayB);
    printf("ArrayC: %d\n",(int)ArrayC);
    printf("ArrayD: %d\n",(int)ArrayD);
    printf("ArrayE: %d\n",(int)ArrayE);
    
    
    
    
    int * SetData = (int*) malloc (DATA_SIZE*sizeof(int));
    for(i = 0;i<DATA_SIZE;i++){
      SetData[i] = 1000+i;
      printf("%d ",SetData[i]);
    }
    printf("\n");
    
    AccelMemcpy(ArrayA, SetData, DATA_SIZE*sizeof(int), SystemCMemcpyHostToDevice);
    
    
    AccelCallDevice();
    
    printf("NONBLOCING CALL: %ld\n",(unsigned long int)CPUNonBlockingCode());
    
    int * GetData = (int*) malloc (DATA_SIZE*sizeof(int));
    AccelMemcpy(ArrayA, GetData,DATA_SIZE*sizeof(int), SystemCMemcpyDeviceToHost);
    
    for(i = 0;i<DATA_SIZE;i++){
      printf("%d ",GetData[i]);
    }
    printf("\n");
    
    
    AccelFree(ArrayA);
    AccelFree(ArrayB);
    AccelFree(ArrayC);
    AccelFree(ArrayD);
    AccelFree(ArrayE);
    
    AccelFinalization();
 
    return 0;
}