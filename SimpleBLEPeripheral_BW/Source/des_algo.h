#ifndef _DES_ALGO_H
#define _DES_ALGO_H

#ifdef __cplusplus
extern "C"
{
#endif




typedef unsigned char ElemType;   


int DES_Encrypt(ElemType plainBlock[8], 
				ElemType keyBlock[8], 
				ElemType cipherBlock[8],
        ElemType subKeys[16][48]);



#ifdef __cplusplus
}
#endif

#endif /* _DES_ALGO_H */
