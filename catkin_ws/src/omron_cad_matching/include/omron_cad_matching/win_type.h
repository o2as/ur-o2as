
#ifndef	WIN_TYPE_H_
#define	WIN_TYPE_H_

#if defined( __cplusplus )
extern "C"{
#endif

typedef char            INT8;
typedef unsigned char   UINT8;
typedef short           INT16;
typedef unsigned short  UINT16;
typedef int             INT32;
typedef unsigned int    UINT32;
typedef int             BOOL;
typedef long            LONG;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

typedef struct tagRECT
{
    LONG    left;
    LONG    top;
    LONG    right;
    LONG    bottom;
} RECT;


#if defined( __cplusplus )
}
#endif

#endif	/* WIN_TYPE_H_ */