/* Host harness stand-in for the kernel header: only what bench_main.c uses.
 * Layouts as include/tk/syscall.h with USE_OBJECT_NAME 0. The sim never
 * starts the tasks (bench_loop): every create returns 0 there, so the
 * bench code keeps its direct paths.
 */
#ifndef HARNESS_TKERNEL_H
#define HARNESS_TKERNEL_H
#include <stdint.h>
typedef unsigned char UB;
typedef int INT;
typedef unsigned int UINT;
typedef unsigned long UW;

typedef long SZ;
typedef INT PRI;
typedef int ER;
typedef int32_t TMO;
typedef uint32_t RELTIM;
typedef int ID;
typedef uint32_t ATR;
typedef void (*FP)();
typedef struct
{
    void * exinf;
    ATR    cycatr;
    FP     cychdr;
    RELTIM cyctim;
    RELTIM cycphs;
} T_CCYC;
typedef struct
{
    void * exinf;
    ATR    tskatr;
    FP     task;
    PRI    itskpri;
    SZ     stksz;
    void * bufptr;
    UW     assprc;
} T_CTSK;
typedef struct
{
    void * exinf;
    ATR    flgatr;
    UINT   iflgptn;
} T_CFLG;
typedef struct
{
    void * exinf;
    ID     wtsk;
    UINT   flgptn;
} T_RFLG;
typedef struct
{
    void * exinf;
    ATR    mbfatr;
    SZ     bufsz;
    INT    maxmsz;
    void * bufptr;
} T_CMBF;
typedef struct
{
    void * exinf;
    ID     wtsk;
    ID     stsk;
    INT    msgsz;
    SZ     frbufsz;
    INT    maxmsz;
} T_RMBF;
typedef struct
{
    void * exinf;
    ATR    mtxatr;
    PRI    ceilpri;
} T_CMTX;
typedef struct
{
    void * exinf;
    ATR    sematr;
    INT    isemcnt;
    INT    maxsem;
} T_CSEM;
#define TA_HLNG (0x00000001UL)
#define TA_USERBUF (0x00000020UL)
#define TA_RNG0 (0x00000000UL)
#define TA_TFIFO (0x00000000UL)
#define TA_WSGL (0x00000000UL)
#define TA_INHERIT (0x00000002UL)
#define TWF_ORW (0x00000001U)
#define TWF_BITCLR (0x00000020U)
#define TMO_POL (0)
#define TMO_FEVR (-1)
#define TSK_SELF (0)
#define E_OK (0)
#define E_TMOUT (-50)
ER tk_dly_tsk (RELTIM dlytim);
ID tk_cre_cyc (const T_CCYC * pk_ccyc);
ER tk_sta_cyc (ID cycid);
ER tk_stp_cyc (ID cycid);
ID tk_get_tid (void);
ER tk_wup_tsk (ID tskid);
ER tk_slp_tsk (TMO tmout);
INT tk_can_wup (ID tskid);
ID tk_cre_tsk (const T_CTSK * pk_ctsk);
ER tk_sta_tsk (ID tskid, INT stacd);
ID tk_cre_flg (const T_CFLG * pk_cflg);
ER tk_set_flg (ID flgid, UINT setptn);
ER tk_wai_flg (ID flgid, UINT waiptn, UINT wfmode, UINT * p_flgptn,
               TMO tmout);
ER tk_ref_flg (ID flgid, T_RFLG * pk_rflg);
ID tk_cre_mbf (const T_CMBF * pk_cmbf);
ER tk_snd_mbf (ID mbfid, const void * msg, INT msgsz, TMO tmout);
INT tk_rcv_mbf (ID mbfid, void * msg, TMO tmout);
ER tk_ref_mbf (ID mbfid, T_RMBF * pk_rmbf);
ID tk_cre_mtx (const T_CMTX * pk_cmtx);
ER tk_loc_mtx (ID mtxid, TMO tmout);
ER tk_unl_mtx (ID mtxid);
ID tk_cre_sem (const T_CSEM * pk_csem);
ER tk_sig_sem (ID semid, INT cnt);
ER tk_wai_sem (ID semid, INT cnt, TMO tmout);
#endif
