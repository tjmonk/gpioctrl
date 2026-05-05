/*===========================================================================
    Mock tjson (libjson) implementation for unit testing gpioctrl
===========================================================================*/
#include "mock_tjson.h"
#include <string.h>
#include <stdlib.h>

/*============================================================================
        Mock state
============================================================================*/

MockTjsonState mock_tjson;

void mock_tjson_reset( void )
{
    memset( &mock_tjson, 0, sizeof( mock_tjson ) );
    mock_tjson.iterate_result = EOK;
}

/*============================================================================
        Stub implementations
============================================================================*/

JNode *JSON_Process( char *inputFile )
{
    (void)inputFile;
    return mock_tjson.process_returns;
}

JNode *JSON_Find( JNode *json, char *key )
{
    JObject *pObj;
    JNode *pChild;

    if ( ( json == NULL ) || ( key == NULL ) )
    {
        return NULL;
    }

    if ( mock_tjson.find_returns != NULL )
    {
        return mock_tjson.find_returns;
    }

    if ( json->type == JSON_OBJECT )
    {
        pObj = (JObject *)json;
        for ( pChild = pObj->pFirst; pChild != NULL; pChild = pChild->pNext )
        {
            if ( ( pChild->name != NULL ) &&
                 ( strcmp( pChild->name, key ) == 0 ) )
            {
                return pChild;
            }
        }
    }

    return NULL;
}

char *JSON_GetStr( JNode *pNode, char *name )
{
    JObject *pObj;
    JNode *pChild;

    if ( mock_tjson.getstr_returns != NULL )
    {
        return mock_tjson.getstr_returns;
    }

    if ( ( pNode == NULL ) || ( name == NULL ) )
    {
        return NULL;
    }

    if ( pNode->type == JSON_OBJECT )
    {
        pObj = (JObject *)pNode;
        for ( pChild = pObj->pFirst; pChild != NULL; pChild = pChild->pNext )
        {
            if ( ( pChild->name != NULL ) &&
                 ( strcmp( pChild->name, name ) == 0 ) &&
                 ( pChild->type == JSON_VAR ) )
            {
                return pChild->name;
            }
        }
    }

    return NULL;
}

int JSON_Iterate( JArray *pArray,
                  int (*fn)( JNode *pNode, void *arg ),
                  void *arg )
{
    JNode *pNode;
    int result = EOK;

    if ( ( pArray == NULL ) || ( fn == NULL ) )
    {
        return EOK;
    }

    for ( pNode = pArray->pFirst; pNode != NULL; pNode = pNode->pNext )
    {
        result = fn( pNode, arg );
    }

    return result;
}

void JSON_Print( JNode *json, FILE *fp, bool comma )
{
    (void)json;
    (void)fp;
    (void)comma;
}
