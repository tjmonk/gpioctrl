/*===========================================================================
    Mock varserver header for unit testing gpioctrl
===========================================================================*/
#ifndef MOCK_VARSERVER_H
#define MOCK_VARSERVER_H

#include <stdint.h>
#include <stddef.h>
#include <signal.h>

#ifndef SIGRTMIN
#define SIGRTMIN 34
#endif

#ifndef EOK
#define EOK 0
#endif

/*============================================================================
        Type Definitions (mirroring varobject.h / var.h / varserver.h)
============================================================================*/

typedef enum _VarType
{
    VARTYPE_INVALID = 0,
    VARTYPE_UINT16,
    VARTYPE_UINT32,
    VARTYPE_FLOAT,
    VARTYPE_STR,
    VARTYPE_END_MARKER
} VarType;

typedef union _VarData
{
    uint16_t ui;
    uint32_t ul;
    float f;
    char *str;
} VarData;

typedef struct _VarObject
{
    VarType type;
    size_t len;
    VarData val;
} VarObject;

#define VAR_INVALID     ( 0 )

typedef uint32_t VAR_HANDLE;

typedef void *VARSERVER_HANDLE;

typedef enum _NotificationType
{
    NOTIFY_NONE = 0,
    NOTIFY_MODIFIED = 1,
    NOTIFY_CALC = 2,
    NOTIFY_VALIDATE = 3,
    NOTIFY_PRINT = 4
} NotificationType;

#define SIG_VAR_MODIFIED ( SIGRTMIN + 6 )
#define SIG_VAR_CALC     ( SIGRTMIN + 7 )
#define SIG_VAR_VALIDATE ( SIGRTMIN + 8 )
#define SIG_VAR_PRINT    ( SIGRTMIN + 9 )

/*============================================================================
        Mock control
============================================================================*/

typedef struct
{
    VARSERVER_HANDLE open_returns;
    VAR_HANDLE find_by_name_returns;
    int var_get_result;
    VarObject var_get_object;
    int var_set_result;
    int var_notify_result;
} MockVarserverState;

extern MockVarserverState mock_varserver;

void mock_varserver_reset( void );

/*============================================================================
        Stubbed varserver API
============================================================================*/

VARSERVER_HANDLE VARSERVER_Open( void );
int VARSERVER_Close( VARSERVER_HANDLE hVarServer );
int VARSERVER_WaitSignal( int *sigval );

VAR_HANDLE VAR_FindByName( VARSERVER_HANDLE hVarServer, char *pName );
int VAR_Get( VARSERVER_HANDLE hVarServer,
             VAR_HANDLE hVar,
             VarObject *pVarObject );
int VAR_Set( VARSERVER_HANDLE hVarServer,
             VAR_HANDLE hVar,
             VarObject *pVarObject );
int VAR_Notify( VARSERVER_HANDLE hVarServer,
                VAR_HANDLE hVar,
                NotificationType notificationType );

int VAR_OpenPrintSession( VARSERVER_HANDLE hVarServer,
                          uint32_t id,
                          VAR_HANDLE *hVar,
                          int *fd );
int VAR_ClosePrintSession( VARSERVER_HANDLE hVarServer,
                           uint32_t id,
                           int fd );

#endif /* MOCK_VARSERVER_H */
