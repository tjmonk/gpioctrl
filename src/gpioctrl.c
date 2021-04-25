/*============================================================================
   Copyright (C) Trevor Monk - All Rights Reserved
   Unauthorized copying of this file, via any medium is strictly prohibited
   Proprietary and confidential
   Written by Trevor Monk <tjmonk@gmail.com>, Apr 2021
 ===========================================================================*/

/*!
 * @defgroup gpioctrl gpioctrl
 * @brief Map GPIO pins to variables
 * @{
 */

/*==========================================================================*/
/*!
@file gpioctrl.c

   GPIO Controller

    The gpioctrl Application maps variables to General Purpose Digital
    Input/Output pins on the device using a JSON object definition to
    describe the mapping


    Variables and their GPIO mappings are defined in
    a JSON array as follows:

    { "gpiodef" : [
            { "chip" : "gpio0",
              "lines" : [
                {
                  "line" : "0",
                  "var" : "/HW/GPIO/0",
                  "active_state" : "low",
                  "direction" : "output",
                  "drive" : "open_drain",
                  "bias" : "pull-up" },
                {
                  "line" : "1",
                  "var" : "/HW/GPIO/1",
                  "direction" : "input",
                  "drive" : "push-pull",
                  "bias" : "pull-up" },
                {
                  "line" : "2",
                  "var" : "/HW/GPIO/2",
                  "direction" : "input",
                  "drive" : "push-pull",
                  "bias" : "pull-up" }
                ]
            }
        ]
    }

    When the value of a variable associated with a hardware output pin is
    changed, that value (0 or 1) is written to the output pin.

    Input pins can be monitored using a waiting task and when the input
    pin changes state, the variable value is updated.

*/
/*==========================================================================*/

/*============================================================================
        Includes
============================================================================*/

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <syslog.h>
#include <varserver/varserver.h>
#include <tjson/json.h>
#include <gpiod.h>

/*============================================================================
        Private definitions
============================================================================*/

/*! the _gpio structure manages the mapping between a gpiod_chip/gpiod_line
 *  and its associated variable */
typedef struct _gpio
{
    /*! pointer to the gpiod_line object associated with the variable */
	struct gpiod_line *pLine;

    /*! handle to the variable */
	VAR_HANDLE hVar;

    /*! name of the variable */
	char *name;

    /*! value of the variable */
    int value;

    /*! direction of the GPIO
        GPIOD_LINE_DIRECTION_INPUT or GPIOD_LINE_DIRECTION_OUTPUT */
    int direction;

    /*! pointer to the next GPIO variable */
	struct _gpio *pNext;

} GPIO;

/*! the _gpio_chip structure maintains a link between each GPIO chip
 * and its lines.  It is used to construct a linked list of
 * GPIO chips under the control of the gpioctrl service */
typedef struct _gpio_chip
{
    /*! name of the chip as was used to instantiate it */
    char *name;

    /*! pointer to the libgpiod gpiod_chip structure */
    struct gpiod_chip *pChip;

    /*! pointer to the first line in the GPIO chip */
    GPIO *pFirstLine;

    /*! pointer to the last line in the GPIO chip */
    GPIO *pLastLine;

    /*! pointer to the next GPIO chip in the list */
    struct _gpio_chip *pNext;
} GPIOChip;

/*! GPIO controller state */
typedef struct _gpioctrl_state
{
    /*! variable server handle */
    VARSERVER_HANDLE hVarServer;

    /*! verbose flag */
    bool verbose;

    /*! name of the GPIO definition file */
    char *pFileName;

    /*! flag to indicate the GPIO controller is running */
    bool running;

    /*! pointer to the first GPIO chip managed by the gpioctrl service */
    GPIOChip *pFirstGPIOChip;

    /*! pointer to the last GPIO chip managed by the gpioctrl service */
    GPIOChip *pLastGPIOChip;

    /*! pointer to the current gpiochip we are parsing */
    struct gpiod_chip *pChip;

} GPIOCtrlState;

/*============================================================================
        Private file scoped variables
============================================================================*/

/*! ExecVars State object */
GPIOCtrlState state;

/*============================================================================
        Private function declarations
============================================================================*/

void main(int argc, char **argv);
static int ProcessOptions( int argC, char *argV[], GPIOCtrlState *pState );
static void usage( char *cmdname );
static int ParseChip( JNode *pNode, void *arg );
static void SetupTerminationHandler( void );
static void TerminationHandler( int signum, siginfo_t *info, void *ptr );
static int GetVarValue( VARSERVER_HANDLE hVarServer, GPIO *pGPIO );
static int GetLineOutputValue( VARSERVER_HANDLE hVarServer, GPIO *pGPIO );
static GPIOChip *CreateChip( JNode *pNode, GPIOCtrlState *pState );
static int CreateLines( JNode *pNode, GPIOCtrlState *pState );
static int ParseLine( JNode *pNode, void *arg );
static GPIO *CreateLine( JNode *pNode, GPIOCtrlState *pState );
static VAR_HANDLE GetVarHandle( VARSERVER_HANDLE hVarServer,
                                JNode *pNode,
                                char **ppName );
static int ParseLineDirection( GPIO *pGPIO, JNode *pNode );
static GPIO *FindGPIO( GPIOCtrlState *pState, VAR_HANDLE hVar );
static int UpdateOutput( VAR_HANDLE hVar, GPIOCtrlState *pState );
static int UpdateInput( VAR_HANDLE hVar, GPIOCtrlState *pState );
static int run( GPIOCtrlState *pState );

/*============================================================================
        Private function definitions
============================================================================*/

/*==========================================================================*/
/*  main                                                                    */
/*!
    Main entry point for the gpioctrl application

    The main function starts the gpioctrl application

    @param[in]
        argc
            number of arguments on the command line
            (including the command itself)

    @param[in]
        argv
            array of pointers to the command line arguments

    @return none

*//*
    REVISION HISTORY:

    Version: 1.0    23-Apr-2021     By: Trevor Monk
        - created

============================================================================*/
void main(int argc, char **argv)
{
    JNode *config;
    JArray *gpiodef;

    /* clear the gpioctrl state object */
    memset( &state, 0, sizeof( state ) );

    if( argc < 2 )
    {
        usage( argv[0] );
        exit( 1 );
    }

    /* set up an abnormal termination handler */
    SetupTerminationHandler();

    /* process the command line options */
    ProcessOptions( argc, argv, &state );

    /* process the input file */
    config = JSON_Process( state.pFileName );

    /* get the configuration array */
    gpiodef = (JArray *)JSON_Find( config, "gpiodef" );

    /* get a handle to the VAR server */
    state.hVarServer = VARSERVER_Open();
    if( state.hVarServer != NULL )
    {
        /* set up the file vars by iterating through the configuration array */
        JSON_Iterate( gpiodef, ParseChip, (void *)&state );

        /* run the GPIO controller */
        run( &state );

        /* close the variable server */
        VARSERVER_Close( state.hVarServer );
    }
}

/*==========================================================================*/
/*  run                                                                     */
/*!
    Run the GPIO controller

    The run function loops forever waiting for signals from the
    variable server and acting on them.

    @param[in]
        pState
            pointer to the GPIO controller state object

    @retval EOK the GPIO controller completed successfully
    @retval EINVAL invalid arguments

*//*
    REVISION HISTORY:

    Version: 1.0    25-Apr-2021     By: Trevor Monk
        - created

============================================================================*/
static int run( GPIOCtrlState *pState )
{
    int result = EINVAL;
    int sig;
    int sigval;
    VAR_HANDLE hVar;

    if ( pState != NULL )
    {
        result = EOK;

        pState->running = true;

        while( pState->running == true )
        {
            /* wait for a signal from the variable server */
            sig = VARSERVER_WaitSignal( &sigval );
            if( sig == SIG_VAR_MODIFIED )
            {
                /* get the handle of the variable which has changed */
                hVar = (VAR_HANDLE)sig;
                UpdateOutput( hVar, &state );
            }
            else if( sig == SIG_VAR_CALC )
            {
                hVar = (VAR_HANDLE)sig;
                UpdateInput( hVar, &state );
            }
        }
    }

    return result;
}

/*============================================================================*/
/*  ParseChip                                                                 */
/*!

  Parse a GPIO chip definition

    The ParseChip function is a callback function for the JSON_Iterate
    function which parses a GPIO chip definition object.
    The chip definition object is expected to look as follows:

    { "chip": "chipname", "lines": [<array of line objects>] }

    @param[in]
       pNode
            pointer to the chip node

    @param[in]
        arg
            opaque pointer argument used for the gpioctrl state object

    @retval EOK - the chip object was parsed successfully
    @retval EINVAL - the chipe object could not be parsed

*//*
    REVISION HISTORY:

    Version: 1.0    23-Apr-2021     By: Trevor Monk
        - created

==============================================================================*/
static int ParseChip( JNode *pNode, void *arg )
{
    int result = EINVAL;
    GPIOCtrlState *pState = (GPIOCtrlState *)arg;
    char *chipName;

    /* get the name of the chip we are creating */
    chipName = JSON_GetStr( pNode, "chip" );
    if( chipName != NULL )
    {
        /* create the GPIOChip object */
        if( CreateChip( pNode, pState ) == EOK )
        {
            /* create the GPIO lines in the GPIOChip object */
            result = CreateLines( pNode, pState );
        }
    }

    return result;
}

/*============================================================================*/
/*  CreateChip                                                                */
/*!

    Create a GPIO chip

    The CreateChip function opens a libgpiod chip specified in the JSON
    node.  The JSON Node is expected to contain a chip name specified
    using the "chip" key.  A GPIOChip object is allocated on the heap to
    manage the newly created chip.  The CreateChip function calls the
    gpiod_chip_open_by_name function in the libgpiod library.
    If the chip is opened and created successfully, it will be appended
    to the list of GPIO chips in the GPIOCtrlState object.

    @param[in]
       pNode
            pointer to the chip node containing the "chip" name key

    @param[in]
        pState
            pointer to the gpioctrl state object containing the chip list
            where the newly created GPIOChip object will be appended.

    @retval pointer to the newly created GPIOChip
    @retval NULL the GPIOChip could not be created

*//*
    REVISION HISTORY:

    Version: 1.0    25-Apr-2021     By: Trevor Monk
        - created

==============================================================================*/
static GPIOChip *CreateChip( JNode *pNode, GPIOCtrlState *pState )
{
    char *chipName;
    struct gpiod_chip *pChip;
    GPIOChip *pGPIOChip = NULL;

    if ( ( pNode != NULL ) &&
         ( pState != NULL ) )
    {
        /* get the chip name */
        chipName = JSON_GetStr( pNode, "chip" );
        if ( chipName != NULL )
        {
            /* try to open the chip */
            pChip = gpiod_chip_open_by_name( chipName );
            if ( pChip != NULL )
            {
                /* allocate memory for the GPIOChip object */
                pGPIOChip = calloc( 1, sizeof( GPIOChip ) );
                if( pGPIOChip != NULL )
                {
                    pGPIOChip->name = chipName;
                    pGPIOChip->pChip = pChip;

                    /* insert the chip at the tail of the list */
                    if( pState->pLastGPIOChip == NULL )
                    {
                        /* first chip found, point both first and last
                         * pointers at it */
                        pState->pLastGPIOChip = pGPIOChip;
                        pState->pFirstGPIOChip = pGPIOChip;
                    }
                    else
                    {
                        pState->pLastGPIOChip->pNext = pGPIOChip;
                        pState->pLastGPIOChip = pGPIOChip;
                    }
                }
                else
                {
                    /* cloud not allocate memory for the GPIO Chip */
                    /* clean up resources used by the chip */
                    gpiod_chip_close( pChip );
                }

            }
        }
    }

    return pGPIOChip;
}

/*============================================================================*/
/*  CreateLines                                                               */
/*!

    Create all the GPIO lines referenced in the JSON definition object

    The CreateLines function iterates through all the lines specified
    in the GPIO definition object for the current chip being processed.
    For each line, a GPIO line object is created that associates a
    variable handle with a gpiod_line object.

    The JSON object is expected to contain a "line" attribute containing
    an array of line definition objects.

    @param[in]
       pNode
            pointer to the chip node containing the "lines" name key

    @param[in]
        pState
            pointer to the gpioctrl state object containing a reference
            to the GPIOChip object containing the GPIO line list
            where the newly created GPIO line objects will be appended.

    @retval EOK all the lines in the chip were successfully created
    @retval ENOTSUP invalid JSON object specified in pNode
    @retval EINVAL invalid arguments
    @retval other error returned by JSON_Iterate

*//*
    REVISION HISTORY:

    Version: 1.0    25-Apr-2021     By: Trevor Monk
        - created

==============================================================================*/
static int CreateLines( JNode *pNode, GPIOCtrlState *pState )
{
    int result = EINVAL;
    JArray *pLines;

    if ( ( pNode != NULL ) &&
         ( pState != NULL ) )
    {
        /* find the lines */
        pNode = JSON_Find( pNode, "lines" );
        if( pNode != NULL )
        {
            /* confirm it is an array type */
            if( pNode->type == JSON_ARRAY )
            {
                pLines = (JArray *)pNode;

                /* iterate through the lines */
                result = JSON_Iterate( pLines, ParseLine, (void *)pState );
            }
            else
            {
                /* JSON type is not supported */
                result = ENOTSUP;
            }
        }
    }
}

/*============================================================================*/
/*  ParseLine                                                                 */
/*!
    Parse a GPIO line definition

    The ParseLine function is a callback function for the JSON_Iterate
    function which parses a GPIO line definition object.
    The line definition object is expected to look as follows:

    { "line": "<line number>",
      "var": "<variable name>",
      "active_state" : "<active state>",
      "direction": "<direction>",
      "drive", "<drive type>",
      "bias", "<bias type>"
      }

    @param[in]
       pNode
            pointer to the line node

    @param[in]
        arg
            opaque pointer argument used for the gpioctrl state object

    @retval EOK - the chip object was parsed successfully
    @retval EINVAL - the chipe object could not be parsed

*//*
    REVISION HISTORY:

    Version: 1.0    23-Apr-2021     By: Trevor Monk
        - created

==============================================================================*/
static int ParseLine( JNode *pNode, void *arg )
{
    int result = EINVAL;
    GPIOCtrlState *pState = (GPIOCtrlState *)arg;
    GPIO *pGPIO;
    VAR_HANDLE hVar;

    JNode *pAttribute;

    JVar *pLineNumber;

    if ( ( pNode != NULL ) &&
         ( pState != NULL ) )
    {
        /* create a GPIO line */
        pGPIO = CreateLine( pNode, pState );
        if( pGPIO != NULL )
        {
            /* set the line direction */
            ParseLineDirection( pGPIO, pNode );


            result = EOK;
        }
    }

    return EOK;
}

/*============================================================================*/
/*  CreateLine                                                                */
/*!
    Create a GPIO line definition

    The CreateLine function creates a GPIO line object which links
    a varaible handle with a gpiod_line object from the libgpiod
    library

    The pNode is expected to point to a JSON object which contains
    the line number and variable name as follows:

    "line": "<line number>",
    "var": "<variable name>",

    @param[in]
        pNode
            pointer to the line node

    @param[in]
        pState
            pointer to the gpioctrl state object which contains
            a reference to the variable server, and the gpiod_chip
            object which specifies the chip that the owns this line.

    @retval pointer to the GPIO line object that was created and added to the
            GPIO line list for the chip we are currently processing
    @retval NULL the GPIO line could not be created

*//*
    REVISION HISTORY:

    Version: 1.0    25-Apr-2021     By: Trevor Monk
        - created

==============================================================================*/
static GPIO *CreateLine( JNode *pNode, GPIOCtrlState *pState )
{
    GPIO *pGPIOLine = NULL;
    VAR_HANDLE hVar = VAR_INVALID;
    char *line_str;
    unsigned int line_num;
    char *varname;
    GPIOChip *pGPIOChip;
    struct gpiod_chip *pChip;
    struct gpiod_line *pLine;

    if ( ( pNode != NULL ) &&
         ( pState != NULL ) &&
         ( pState->pLastGPIOChip != NULL ) )
    {
        /* get a pointer to the GPIOChip object we are currently processing */
        pGPIOChip = pState->pLastGPIOChip;

        /* get a pointer to the gpiod_chip that is the parent of this line */
        pChip = pGPIOChip->pChip;

        /* get a handle to the variable associated with the GPIO line */
        hVar = GetVarHandle( pState->hVarServer, pNode, &varname );
        if( hVar != VAR_INVALID )
        {
            /* get the "line" attribute from the GPIO line definition object */
            line_str = JSON_GetStr( pNode, "line" );
            if ( line_str != NULL )
            {
                /* convert the line number to an integer */
                line_num = strtoul( line_str, NULL, 0 );

                /* get a handle to the GPIO line from the GPIOD library */
                pLine = gpiod_chip_get_line( pChip, line_num );
                if( pLine != NULL )
                {
                    /* allocate memory for the GPIO line object */
                    pGPIOLine = calloc( 1, sizeof( GPIO ) );
                    if( pGPIOLine != NULL )
                    {
                        /* store the variable handle */
                        pGPIOLine->hVar = hVar;

                        /* store the variable name */
                        pGPIOLine->name = varname;

                        /* store a pointer to the gpiod_line */
                        pGPIOLine->pLine = pLine;

                        /* add the GPIO line to the line list */
                        if ( pGPIOChip->pLastLine == NULL )
                        {
                            pGPIOChip->pFirstLine = pGPIOLine;
                            pGPIOChip->pLastLine = pGPIOLine;
                        }
                        else
                        {
                            pGPIOChip->pLastLine->pNext = pGPIOLine;
                            pGPIOChip->pLastLine = pGPIOLine;
                        }
                    }
                    else
                    {
                        /* memory allocation failed */
                        /* clean up the libgpiod resources */
                        gpiod_line_release( pLine );
                    }
                }
            }
        }
    }

    return pGPIOLine;
}

/*============================================================================*/
/*  GetVarHandle                                                              */
/*!
    Get the variable handle for the GPIO line

    The GetVarHandle function looks up the variable handle for the variable
    specified by the "var" attribute in the GPIO line definition.

    @param[in]
        hVarServer
            handle to the variable server to query

    @param[in]
        pNode
            pointer to the line node to search for the "var" attribute

    @param[out]
        ppName
            pointer to the location to store a pointer to the variable name

    @retval handle of the specified variable
    @retval VAR_INVALID if the variable was not found

*//*
    REVISION HISTORY:

    Version: 1.0    24-Apr-2021     By: Trevor Monk
        - created

==============================================================================*/
static VAR_HANDLE GetVarHandle( VARSERVER_HANDLE hVarServer,
                                JNode *pNode,
                                char **ppName )
{
    VAR_HANDLE hVar = VAR_INVALID;
    char *varname;

    if ( ( hVarServer != NULL ) &&
         ( pNode != NULL ) &&
         ( ppName != NULL ) )
    {
        varname = JSON_GetStr( pNode, "var" );
        if( varname != NULL )
        {
            hVar = VAR_FindByName( hVarServer, varname );
            if( hVar != VAR_INVALID )
            {
                /* store a pointer to the variable name */
                *ppName = varname;
            }
        }
    }

    return hVar;
}

/*============================================================================*/
/*  ParseLineDirection                                                        */
/*!
    Parse the GPIO definition to set the direction for the specified GPIO line

    The ParseLineDirection function sets the line direction for the
    GPIO line object.  It is assumed the GPIO line handle has already
    been assigned to the GPIO object.

    Two valid directions values are supported:  "input" and "output"

    If the direction is not specified, it is assumed to be an "input"

    @param[in]
        pGPIO
            pointer to the GPIO object to store the line information in

    @param[in]
        pNode
            pointer to the line node to search for the "direction" attribute

    @retval EOK the line information object was obtained
    @retval ENOENT the line information was not found
    @retval EINVAL invalid arguments or incorrect JSON format

*//*
    REVISION HISTORY:

    Version: 1.0    24-Apr-2021     By: Trevor Monk
        - created

==============================================================================*/
static int ParseLineDirection( GPIO *pGPIO, JNode *pNode )
{
    int result = EINVAL;
    char *direction;
    const char *consumer = "gpioctrl";

    if ( ( pGPIO != NULL ) &&
         ( pGPIO->pLine != NULL ) &&
         ( pNode != NULL ) )
    {
        /* get the "direction" attribute from the GPIO line definition */
        direction = JSON_GetStr( pNode, "direction" );
        if ( direction == NULL )
        {
            direction = "input";
        }

        if ( strcmp( direction, "input" ) == 0 )
        {
            /* set the line to input */
            result = gpiod_line_request_input( pGPIO->pLine, consumer );
        }
        else if( strcmp( direction, "output" ) == 0 )
        {
            /* set the line to "output" and set the default value */
            result = gpiod_line_request_output( pGPIO->pLine,
                                                consumer,
                                                pGPIO->value );
        }
        else
        {
            result = ENOTSUP;
        }
    }

    return result;
}

/*============================================================================*/
/*  GetLineOutputValue                                                        */
/*!
    Get the value from the variable associated with the GPIO output line

    The GetLineOutputValue function gets the value from the variable associated
    with the specified GPIO line but only if the line direction is "output".

    The type of the variable must be VARTYPE_UINT16

    The line direction must be "output"

    @param[in]
        hVarServer
            handle to the variable server to query the variable value

    @param[in]
        pGPIO
            pointer to the GPIO object to store the variable value

    @retval EOK the variable value was successfully retrieved
    @retval ENOENT the variable was not found
    @retval ENOTSUP invalid variable type or incorrect direction
    @retval EINVAL invalid arguments

*//*
    REVISION HISTORY:

    Version: 1.0    24-Apr-2021     By: Trevor Monk
        - created

==============================================================================*/
static int GetLineOutputValue( VARSERVER_HANDLE hVarServer, GPIO *pGPIO )
{
    int result = EINVAL;
    VarObject var;
    int direction;

    if ( ( hVarServer != NULL ) &&
         ( pGPIO != NULL ) &&
         ( pGPIO->hVar != VAR_INVALID ) &&
         ( pGPIO->pLine != NULL ) )
    {
        /* get the line direction */
        direction = gpiod_line_direction( pGPIO->pLine );
        if ( direction == GPIOD_LINE_DIRECTION_OUTPUT )
        {
            /* get the variable value */
            if ( VAR_Get( hVarServer, pGPIO->hVar, &var ) == EOK )
            {
                if( var.type == VARTYPE_UINT16 )
                {
                    /* get the requested GPIO output value and store it
                     * in the GPIO object */
                    pGPIO->value = var.val.ui;

                    /* indicate success */
                    result = EOK;
                }
                else
                {
                    result = ENOTSUP;
                }
            }
            else
            {
                /* cannot get variable */
                result = ENOENT;
            }
        }
        else
        {
            /* unsupported operation */
            result = ENOTSUP;
        }
    }

    return result;
}

/*============================================================================*/
/*  FindGPIO                                                                  */
/*!
    Find a GPIO given a handle to its associated variable

    The FindGPIO function iterates through all of the GPIO chips looking
    for the GPIO line associated with the specified variable handle.

    @param[in]
        pState
            pointer to the GPIOCtrl state which contains the list of
            GPIO chips to search

    @param[in]
        hVar
            handle of the variable to search for

    @retval pointer to the GPIO line object associated with the specified var
    @retval NULL if the GPIO line object could not be found

*//*
    REVISION HISTORY:

    Version: 1.0    25-Apr-2021     By: Trevor Monk
        - created

==============================================================================*/
static GPIO *FindGPIO( GPIOCtrlState *pState, VAR_HANDLE hVar )
{
    GPIOChip *pGPIOChip;
    GPIO *pGPIO;
    GPIO *foundGPIO = NULL;
    bool found = false;

    if ( ( pState != NULL ) &&
         ( hVar != VAR_INVALID ) )
    {
        /* start looking in the first GPIO chip */
        pGPIOChip = pState->pFirstGPIOChip;
        while ( ( pGPIOChip != NULL ) && ( found == false ) )
        {
            /* start looking in the first line of the chip */
            pGPIO = pGPIOChip->pFirstLine;
            while( ( pGPIO != NULL ) && ( found == false ) )
            {
                /* check for a variable handle match */
                if( pGPIO->hVar == hVar )
                {
                    /* save the found GPIO line */
                    foundGPIO = pGPIO;

                    /* abort the search */
                    found = true;
                }

                /* move on to the next GPIO line */
                pGPIO = pGPIO->pNext;
            }

            /* move to the next GPIO chip */
            pGPIOChip = pGPIOChip->pNext;
        }
    }

    /* return the found GPIO, or NULL if it is not found */
    return foundGPIO;
}

/*==========================================================================*/
/*  usage                                                                   */
/*!
    Display the application usage

    The usage function dumps the application usage message
    to stderr.

    @param[in]
       cmdname
            pointer to the invoked command name

    @return none

*//*
    REVISION HISTORY:

    Version: 1.0    19-Mar-2021     By: Trevor Monk
        - created

============================================================================*/
static void usage( char *cmdname )
{
    if( cmdname != NULL )
    {
        fprintf(stderr,
                "usage: %s [-v] [-h] "
                " [-h] : display this help"
                " [-v] : verbose output"
                " -f <filename> : configuration file",
                cmdname );
    }
}

/*==========================================================================*/
/*  ProcessOptions                                                          */
/*!
    Process the command line options

    The ProcessOptions function processes the command line options and
    populates the GPIOCtrlState object

    @param[in]
        argC
            number of arguments
            (including the command itself)

    @param[in]
        argv
            array of pointers to the command line arguments

    @param[in]
        pState
            pointer to the GPIOCtrl state object

    @return none

*//*
    REVISION HISTORY:

    Version: 1.0    19-Mar-2021     By: Trevor Monk
        - created

============================================================================*/
static int ProcessOptions( int argC, char *argV[], GPIOCtrlState *pState )
{
    int c;
    int result = EINVAL;
    const char *options = "hvf:";

    if( ( pState != NULL ) &&
        ( argV != NULL ) )
    {
        while( ( c = getopt( argC, argV, options ) ) != -1 )
        {
            switch( c )
            {
                case 'v':
                    pState->verbose = true;
                    break;

                case 'h':
                    usage( argV[0] );
                    break;

                case 'f':
                    pState->pFileName = strdup(optarg);
                    break;

                default:
                    break;

            }
        }
    }

    return 0;
}

/*============================================================================*/
/*  SetupTerminationHandler                                                   */
/*!
    Set up an abnormal termination handler

    The SetupTerminationHandler function registers a termination handler
    function with the kernel in case of an abnormal termination of this
    process.

*//*
    REVISION HISTORY:

    Version: 1.0    12-Apr-2021     By: Trevor Monk
        - created

==============================================================================*/
static void SetupTerminationHandler( void )
{
    static struct sigaction sigact;

    memset( &sigact, 0, sizeof(sigact) );

    sigact.sa_sigaction = TerminationHandler;
    sigact.sa_flags = SA_SIGINFO;

    sigaction( SIGTERM, &sigact, NULL );
    sigaction( SIGINT, &sigact, NULL );

}

/*============================================================================*/
/*  TerminationHandler                                                        */
/*!
    Abnormal termination handler

    The TerminationHandler function will be invoked in case of an abnormal
    termination of this process.  The termination handler closes
    the connection with the variable server and cleans up its VARFP shared
    memory.

@param[in]
    signum
        The signal which caused the abnormal termination (unused)

@param[in]
    info
        pointer to a siginfo_t object (unused)

@param[in]
    ptr
        signal context information (ucontext_t) (unused)

*//*
    REVISION HISTORY:

    Version: 1.0    12-Apr-2021     By: Trevor Monk
        - created

==============================================================================*/
static void TerminationHandler( int signum, siginfo_t *info, void *ptr )
{
    syslog( LOG_ERR, "Abnormal termination of gpioctrl\n" );
    VARSERVER_Close( state.hVarServer );
    exit( 1 );
}

/*============================================================================*/
/*  UpdateOutput                                                              */
/*!
    Update a GPIO output

    The UpdateOutput function will be find the variable given by it's handle,
    get the variable value, and write either a 1 (variable value is non-zero),
    or a 0 (variable value is zero) to the GPIO line associated with the
    variable handle.

@param[in]
    hVar
        Handle for the variable associated with the GPIO output

@param[in]
    pState
        pointer to a GPIOCtrlState object which contains the GPIO lines,
        and a handle to the variable server

@retval EOK the GPIO line was updated correctly
@retval ENOENT the variable was not found
@retval ENOTSUP the variable type was invalid, or the GPIO is not an output
@retval EINVAL invalid arguments

*//*
    REVISION HISTORY:

    Version: 1.0    25-Apr-2021     By: Trevor Monk
        - created

==============================================================================*/
static int UpdateOutput( VAR_HANDLE hVar, GPIOCtrlState *pState )
{
    int result = EINVAL;
    GPIO *pGPIO;
    VarObject var;
    int direction;
    int rc;

    if ( ( pState != NULL ) &&
         ( hVar != VAR_INVALID ) )
    {
        /* find the GPIO associated with the specified variable */
        pGPIO = FindGPIO( pState, hVar );
        if( pGPIO != NULL )
        {
            /* get the direction of this GPIO */
            direction = pGPIO->direction;

            /* confirm it is an output */
            if ( direction == GPIOD_LINE_DIRECTION_OUTPUT )
            {
                /* get the requested output value */
                if ( VAR_Get( pState->hVarServer,
                              hVar,
                              &var ) == EOK )
                {
                    if ( var.type == VARTYPE_UINT16 )
                    {
                        /* get the value to write to the output */
                        pGPIO->value = ( var.val.ui > 0 ) ? 1 : 0;

                        /* set the output value to the hardware */
                        rc = gpiod_line_set_value( pGPIO->pLine,
                                                   pGPIO->value );

                        /* check the result */
                        result = ( rc == EOK ) ? EOK : errno;
                    }
                    else
                    {
                        /* invalid data type */
                        result = ENOTSUP;
                    }
                }
                else
                {
                    /* unable to get the value */
                    result = ENOENT;
                }
            }
            else
            {
                /* invalid line direction */
                result = ENOTSUP;
            }
        }
        else
        {
            /* variable not found */
            result = ENOENT;
        }
    }

    return result;
}

/*============================================================================*/
/*  UpdateInput                                                               */
/*!
    Update a GPIO input

    The UpdateInput function will be find the variable given by it's handle,
    get the current state of the associated GPIO input and update the
    variable value with the appropriate input value.

@param[in]
    hVar
        Handle for the variable associated with the GPIO output

@param[in]
    pState
        pointer to a GPIOCtrlState object which contains the GPIO lines,
        and a handle to the variable server

@retval EOK the GPIO line was read correctly and the variable was updated
@retval ENOENT the variable was not found
@retval ENOTSUP the GPIO is not an input
@retval EIO input error
@retval EINVAL invalid arguments

*//*
    REVISION HISTORY:

    Version: 1.0    25-Apr-2021     By: Trevor Monk
        - created

==============================================================================*/
static int UpdateInput( VAR_HANDLE hVar, GPIOCtrlState *pState )
{
    int result = EINVAL;
    GPIO *pGPIO;
    VarObject var;
    int direction;
    int rc;

    if ( ( pState != NULL ) &&
         ( hVar != VAR_INVALID ) )
    {
        /* find the GPIO associated with the specified variable */
        pGPIO = FindGPIO( pState, hVar );
        if( pGPIO != NULL )
        {
            /* get the direction of this GPIO */
            direction = pGPIO->direction;

            /* confirm it is an output */
            if ( direction == GPIOD_LINE_DIRECTION_INPUT )
            {
                /* read the GPIO line */
                rc = gpiod_line_get_value( pGPIO->pLine );
                if ( rc != -1 )
                {
                    /* set the value of the variable */
                    var.val.ui = ( rc > 0 ) ? 1 : 0;
                    var.type == VARTYPE_UINT16;

                    /* write to the variable */
                    result = VAR_Set( pState->hVarServer,
                                      hVar,
                                      &var );
                }
                else
                {
                    /* input error when reading from the GPIO line */
                    result = EIO;
                }
            }
            else
            {
                /* unsupported action on this GPIO line */
                result = ENOTSUP;
            }
        }
        else
        {
            /* variable not found */
            result = ENOENT;
        }
    }

    return result;
}

/*! @}
 * end of gpioctrl group */
