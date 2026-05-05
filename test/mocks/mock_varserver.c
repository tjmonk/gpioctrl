/*===========================================================================
    Mock varserver implementation for unit testing gpioctrl
===========================================================================*/
#include "mock_varserver.h"
#include <string.h>
#include <stdlib.h>

/*============================================================================
        Mock state
============================================================================*/

MockVarserverState mock_varserver;

static int fake_varserver_handle_data;

void mock_varserver_reset( void )
{
    memset( &mock_varserver, 0, sizeof( mock_varserver ) );
    mock_varserver.open_returns = (VARSERVER_HANDLE)&fake_varserver_handle_data;
    mock_varserver.find_by_name_returns = 1;
    mock_varserver.var_get_result = EOK;
    mock_varserver.var_get_object.type = VARTYPE_UINT16;
    mock_varserver.var_get_object.len = sizeof( uint16_t );
    mock_varserver.var_get_object.val.ui = 0;
    mock_varserver.var_set_result = EOK;
    mock_varserver.var_notify_result = EOK;
}

/*============================================================================
        Stub implementations
============================================================================*/

VARSERVER_HANDLE VARSERVER_Open( void )
{
    return mock_varserver.open_returns;
}

int VARSERVER_Close( VARSERVER_HANDLE hVarServer )
{
    (void)hVarServer;
    return EOK;
}

int VARSERVER_WaitSignal( int *sigval )
{
    (void)sigval;
    return 0;
}

VAR_HANDLE VAR_FindByName( VARSERVER_HANDLE hVarServer, char *pName )
{
    (void)hVarServer;
    (void)pName;
    return mock_varserver.find_by_name_returns;
}

int VAR_Get( VARSERVER_HANDLE hVarServer,
             VAR_HANDLE hVar,
             VarObject *pVarObject )
{
    (void)hVarServer;
    (void)hVar;
    if ( pVarObject != NULL )
    {
        *pVarObject = mock_varserver.var_get_object;
    }
    return mock_varserver.var_get_result;
}

int VAR_Set( VARSERVER_HANDLE hVarServer,
             VAR_HANDLE hVar,
             VarObject *pVarObject )
{
    (void)hVarServer;
    (void)hVar;
    (void)pVarObject;
    return mock_varserver.var_set_result;
}

int VAR_Notify( VARSERVER_HANDLE hVarServer,
                VAR_HANDLE hVar,
                NotificationType notificationType )
{
    (void)hVarServer;
    (void)hVar;
    (void)notificationType;
    return mock_varserver.var_notify_result;
}

int VAR_OpenPrintSession( VARSERVER_HANDLE hVarServer,
                          uint32_t id,
                          VAR_HANDLE *hVar,
                          int *fd )
{
    (void)hVarServer;
    (void)id;
    (void)hVar;
    (void)fd;
    return EOK;
}

int VAR_ClosePrintSession( VARSERVER_HANDLE hVarServer,
                           uint32_t id,
                           int fd )
{
    (void)hVarServer;
    (void)id;
    (void)fd;
    return EOK;
}
