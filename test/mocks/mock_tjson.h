/*===========================================================================
    Mock tjson (libjson) header for unit testing gpioctrl
===========================================================================*/
#ifndef MOCK_TJSON_H
#define MOCK_TJSON_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#ifndef EOK
#define EOK 0
#endif

/*============================================================================
        JSON Type Definitions (mirroring libjson/inc/json.h)
============================================================================*/

typedef enum _JType
{
    JSON_INVALID = 0,
    JSON_ARRAY = 1,
    JSON_OBJECT = 2,
    JSON_VAR = 3,
    JSON_BOOL = 4
} JType;

typedef struct _JNode
{
    JType type;
    char *name;
    struct _JNode *pNext;
} JNode;

typedef struct _JArray
{
    JNode node;
    size_t n;
    JNode *pFirst;
    JNode *pLast;
} JArray;

typedef struct _JObject
{
    JNode node;
    size_t n;
    JNode *pFirst;
    JNode *pLast;
} JObject;

/*============================================================================
        Mock control
============================================================================*/

typedef struct
{
    JNode *process_returns;
    JNode *find_returns;
    char  *getstr_returns;
    int    iterate_result;
} MockTjsonState;

extern MockTjsonState mock_tjson;

void mock_tjson_reset( void );

/*============================================================================
        Stubbed tjson API
============================================================================*/

JNode *JSON_Process( char *inputFile );
JNode *JSON_Find( JNode *json, char *key );
char *JSON_GetStr( JNode *pNode, char *name );
int JSON_Iterate( JArray *pArray,
                  int (*fn)( JNode *pNode, void *arg ),
                  void *arg );
void JSON_Print( JNode *json, FILE *fp, bool comma );

#endif /* MOCK_TJSON_H */
