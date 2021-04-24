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
    /*! pointer to the gpiod_chip object associated with the variable */
	struct gpiod_chip *pChip;

    /*! pointer to the gpiod_line object associated with the variable */
	struct gpiod_line *pLine;

    /*! handle to the variable */
	VAR_HANDLE hVar;

    /*! name of the variable */
	char *name;

    /*! pointer to the next GPIO variable */
	struct _gpio *pNext;

} GPIO;


/*! GPIO controller state */
typedef struct _gpioctrl_state
{
    /*! variable server handle */
    VARSERVER_HANDLE hVarServer;

    /*! verbose flag */
    bool verbose;

    /*! name of the GPIO definition file */
    char *pFileName;

    /*! pointer to the file vars list */
    GPIO *pGPIOVars;
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
static int ParseLine( JNode *pNode, void *arg );
static void SetupTerminationHandler( void );
static void TerminationHandler( int signum, siginfo_t *info, void *ptr );

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
    VARSERVER_HANDLE hVarServer = NULL;
    VAR_HANDLE hVar;
    int result;
    JNode *config;
    JArray *gpiodef;
    int sigval;
    int fd;
    int sig;

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

    /* set up the file vars by iterating through the configuration array */
    JSON_Iterate( gpiodef, ParseChip, (void *)&state );

    /* get a handle to the VAR server */
    state.hVarServer = VARSERVER_Open();
    if( state.hVarServer != NULL )
    {
        /* set up the file vars by iterating through the configuration array */
        JSON_Iterate( gpiodef, ParseChip, (void *)&state );

        /* close the variable server */
        VARSERVER_Close( state.hVarServer );
    }
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
    JVar *pChip;
    JArray *pLines;

    pChip = (JVar *)JSON_Find( pNode, "chip" );

    pLines = (JArray *)JSON_Find( pNode, "lines" );

    JSON_Iterate( pLines, ParseLine, arg );

    return EOK;
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
    JVar *pLineNumber;
    JVar *pVarName;

    pLineNumber = (JVar *)JSON_Find( pNode, "line" );

    pVarName = (JVar *)JSON_Find( pNode, "var" );

    printf("%s : GPIO pin %s\n",
            pVarName->var.val.str,
            pLineNumber->var.val.str );

    return EOK;
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

/*! @}
 * end of gpioctrl group */
