/*==============================================================================
MIT License

Copyright (c) 2023 Trevor Monk

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
==============================================================================*/

/*!
 * @defgroup gpioctrl gpioctrl
 * @brief Map GPIO pins to variables
 * @{
 */

/*============================================================================*/
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
                  "drive" : "open-drain",
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
                  "drive" : "open-source",
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
/*============================================================================*/

/*==============================================================================
        Includes
==============================================================================*/

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <syslog.h>
#include <pthread.h>
#include <varserver/varserver.h>
#include <tjson/json.h>
#include <gpiod.h>

/*==============================================================================
        Private definitions
==============================================================================*/

/*! the _gpio structure manages the mapping between a gpiod_chip/gpiod_line
 *  and its associated variable */
typedef struct _gpio
{
    /*! pointer to the gpiod_line object associated with the variable */
	struct gpiod_line *pLine;

    /*! handle to the variable */
	VAR_HANDLE hVar;

    /*! line number */
    int line_num;

    /*! name of the variable */
	char *name;

    /*! value of the variable */
    int value;

    /*! direction of the GPIO
        GPIOD_LINE_DIRECTION_INPUT or GPIOD_LINE_DIRECTION_OUTPUT */
    int direction;

    /*! software PWM output */
    bool PWM;

    /*! event type.  one of:
        0
        GPIOD_LINE_REQUEST_EVENT_FALLING_EDGE
        GPIOD_LINE_REQUEST_EVENT_RISING_EDGE
        GPIOD_LINE_REQUEST_EVENT_BOTH_EDGES */
    int event_type;

    /*! line request */
    struct gpiod_line_request_config request;

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
    /*! service name */
    char *service;

    /*! operating mode */
    bool gpiowatch;

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

    /*! bulk lines array for event monitoring */
    struct gpiod_line_bulk monitoredLines;

} GPIOCtrlState;

/*==============================================================================
        Private file scoped variables
==============================================================================*/

/*! GPIO Controller State object */
GPIOCtrlState state;

/*==============================================================================
        Private function declarations
==============================================================================*/

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
static int ParseLineDirection( GPIO *pGPIO,
                               JNode *pNode,
                               GPIOCtrlState *pState );
static int ParseLineActiveState( GPIO *pGPIO, JNode *pNode );
static int ParseLineBias( GPIO *pGPIO, JNode *pNode );
static int ParseLineDrive( GPIO *pGPIO, JNode *pNode );
static int ParseLineEvent( GPIO *pGPIO, JNode *pNode );
static GPIO *FindGPIO( GPIOCtrlState *pState, VAR_HANDLE hVar );
static VAR_HANDLE FindVar( GPIOCtrlState *pState, struct gpiod_line *pLine );
static int UpdateOutput( VAR_HANDLE hVar, GPIOCtrlState *pState );
static int UpdateInput( VAR_HANDLE hVar, GPIOCtrlState *pState );
static int run( GPIOCtrlState *pState );
static int WaitVarSignal( GPIOCtrlState *pState );
static int WaitGPIOEvent( GPIOCtrlState *pState );
static int HandleGPIOEvent( GPIOCtrlState *pState, struct gpiod_line *pLine );
static int RequestLine( GPIO *pGPIO, GPIOCtrlState *pState );
static int SetupNotification( GPIO *pGPIO, GPIOCtrlState *pState );
static int SetupPrintNotifications( GPIOCtrlState *pState );
static int PrintStatus( GPIOCtrlState *pState, int fd );
static int PrintLineInfo( GPIOCtrlState *pState, GPIO *pGPIO, int fd );
static void Shutdown( GPIOCtrlState *pState );
static int CreatePWM( GPIO *pGPIO );
static void *PWMThread( void *arg );

/*==============================================================================
        Private function definitions
==============================================================================*/

/*============================================================================*/
/*  main                                                                      */
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

==============================================================================*/
void main(int argc, char **argv)
{
    JNode *config;
    JArray *gpiodef;

    printf("Starting %s\n", argv[0]);

    /* clear the gpioctrl state object */
    memset( &state, 0, sizeof( state ) );

    if( argc < 2 )
    {
        usage( argv[0] );
        exit( 1 );
    }

    state.service = strdup( argv[0] );

    if (strcmp( state.service, "gpiowatch" ) == 0 )
    {
        state.gpiowatch = true;
    }

    /* set up an abnormal termination handler */
    SetupTerminationHandler();

    /* process the command line options */
    ProcessOptions( argc, argv, &state );

    /* process the input file */
    config = JSON_Process( state.pFileName );

    if( state.verbose == true )
    {
	    JSON_Print(config, stdout, false );
        printf("\n");
    }

    /* get the configuration array */
    gpiodef = (JArray *)JSON_Find( config, "gpiodef" );

    /* get a handle to the VAR server */
    state.hVarServer = VARSERVER_Open();
    if( state.hVarServer != NULL )
    {
        /* set up the print notifications */
        SetupPrintNotifications( &state );

        /* set up the file vars by iterating through the configuration array */
        JSON_Iterate( gpiodef, ParseChip, (void *)&state );

        /* run the GPIO controller */
        run( &state );

        /* shut down the reserved GPIO lines */
        Shutdown( &state );

        /* close the variable server */
        VARSERVER_Close( state.hVarServer );
    }
}

/*============================================================================*/
/*  run                                                                       */
/*!
    Run the GPIO controller

    The run function loops forever waiting for signals from the
    variable server or events from the GPIO library and acting on them.
    The operating mode (signals or events) is determined based on the
    value of the gpiowatch variable:
        true => wait gpio events
        false => wait varserver signals

    @param[in]
        pState
            pointer to the GPIO controller state object

    @retval EOK the GPIO controller completed successfully
    @retval EINVAL invalid arguments

==============================================================================*/
static int run( GPIOCtrlState *pState )
{
    int result = EINVAL;

    if ( pState != NULL )
    {
        result = EOK;

        pState->running = true;

        while( pState->running == true )
        {
            if( pState->gpiowatch == true )
            {
                WaitGPIOEvent( pState );
            }
            else
            {
                WaitVarSignal( pState );
            }
        }
    }

    return result;
}

/*==========================================================================*/
/*  WaitGPIOEvent                                                           */
/*!
    Wait for GPIO events

    The WaitGPIOEvent function waits for a GPIO rising or falling
    edge event.

    @param[in]
        pState
            pointer to the GPIO controller state object

    @retval EOK the event was handled successfully
    @retval EINVAL invalid arguments

==============================================================================*/
static int WaitGPIOEvent( GPIOCtrlState *pState )
{
    int result = EINVAL;
    int rc;
    int i;
    struct gpiod_line_bulk events;

    if ( pState != NULL )
    {
        rc = gpiod_line_event_wait_bulk( &pState->monitoredLines,
                                         NULL,
                                         &events );
        if ( rc < 0 )
        {
            result = errno;
        }
        else if ( rc > 0 )
        {
            for( i = 0; i < events.num_lines; i++ )
            {
                /* handle the line state update */
                HandleGPIOEvent( pState, events.lines[i] );
            }
        }
    }

    return result;
}

/*============================================================================*/
/*  HandleGPIOEvent                                                           */
/*!
    Handle a GPIO input event

    The HandleGPIOEvent function processes a single gpio event
    ( low to high, or high to low transition on an input pin )
    The function searches for system variable that the line is
    associated with, and sets its value to 0 or 1 depending on
    if the transition was high to low, or low to high.

    @param[in]
        pState
            pointer to the GPIO controller state object

    @param[in]
        pLine
            pointer to the gpiod_line object associated with the event

    @retval EOK the event was handled successfully
    @retval other error reported by VAR_Set()
    @retval EINVAL invalid arguments

==============================================================================*/
static int HandleGPIOEvent( GPIOCtrlState *pState, struct gpiod_line *pLine )
{
    int result = EINVAL;
    struct gpiod_line_event event;
    VAR_HANDLE hVar;
    VarObject var;
    uint16_t val;

    if ( ( pState != NULL ) &&
         ( pLine != NULL ) )
    {
        /* determine the type of event that occurred */
        if ( gpiod_line_event_read( pLine, &event ) == 0 )
        {
            val = ( event.event_type == GPIOD_LINE_EVENT_RISING_EDGE ) ? 1 : 0;

            /* find the variable associated with the gpiod line */
            hVar = FindVar( pState, pLine );
            if ( hVar != VAR_INVALID )
            {
                /* set the value of the variable */
                var.val.ui = val;
                var.type = VARTYPE_UINT16;
                var.len = sizeof(uint16_t);

                /* write to the variable */
                result = VAR_Set( pState->hVarServer,
                                  hVar,
                                  &var );
            }
            else
            {
                result = ENOENT;
            }
        }
        else
        {
            result = EIO;
        }
    }

    return result;
}

/*============================================================================*/
/*  WaitVarSignal                                                             */
/*!
    Wait for signals from the variable server

    The WaitVarSignal function waits for a signal from the variable server
    such as one of the following:
        - SIG_VAR_MODIFIED
        - SIG_VAR_CALC
        - SIG_VAR_PRINT

    @param[in]
        pState
            pointer to the GPIO controller state object

    @retval EOK the signal was handled successfully
    @retval ENOTSUP the signal was not supported
    @retval EINVAL invalid arguments

==============================================================================*/
static int WaitVarSignal( GPIOCtrlState *pState )
{
    int sig;
    int sigval;
    VAR_HANDLE hVar;
    int fd = -1;
    int result = EINVAL;

    if ( pState != NULL )
    {
        /* wait for a signal from the variable server */
        sig = VARSERVER_WaitSignal( &sigval );
        if( sig == SIG_VAR_MODIFIED )
        {
            /* get the handle of the variable which has changed */
            hVar = (VAR_HANDLE)sigval;
            UpdateOutput( hVar, pState );
            result = EOK;
        }
        else if( sig == SIG_VAR_CALC )
        {
            hVar = (VAR_HANDLE)sigval;
            UpdateInput( hVar, pState);
            result = EOK;
        }
        else if ( sig == SIG_VAR_PRINT )
        {
            /* open a print session */
            VAR_OpenPrintSession( state.hVarServer,
                                  sigval,
                                  &hVar,
                                  &fd );

            /* print the file variable */
            PrintStatus( pState, fd );

            /* Close the print session */
            VAR_ClosePrintSession( state.hVarServer,
                                   sigval,
                                   fd );

            result = EOK;
        }
        else
        {
            result = ENOTSUP;
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

==============================================================================*/
static int ParseChip( JNode *pNode, void *arg )
{
    int result = EINVAL;
    GPIOCtrlState *pState = (GPIOCtrlState *)arg;

    /* create the GPIOChip object */
    if( CreateChip( pNode, pState ) != NULL )
    {
        /* create the GPIO lines in the GPIOChip object */
        result = CreateLines( pNode, pState );
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

==============================================================================*/
static GPIOChip *CreateChip( JNode *pNode, GPIOCtrlState *pState )
{
    char *chipName;
    struct gpiod_chip *pChip;
    GPIOChip *pGPIOChip = NULL;
    char buf[BUFSIZ];

    if ( ( pNode != NULL ) &&
         ( pState != NULL ) )
    {
        /* get the chip name */
        chipName = JSON_GetStr( pNode, "chip" );
        if ( chipName != NULL )
        {
            /* build the chip name */
            sprintf(buf, "/dev/%s", chipName );

            /* try to open the chip */
            pChip = gpiod_chip_open( buf );
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
            else
            {
                printf("unable to open chip: %s\n", buf );
            }
        }
        else
        {
            printf("chip name is NULL\n");
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

    return result;
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
      "event": "<event_type>",
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
    @retval EINVAL - the chip object could not be parsed

==============================================================================*/
static int ParseLine( JNode *pNode, void *arg )
{
    int result = EINVAL;
    GPIOCtrlState *pState = (GPIOCtrlState *)arg;
    GPIO *pGPIO;
    int n;

    if ( ( pNode != NULL ) &&
         ( pState != NULL ) )
    {
        /* create a GPIO line */
        pGPIO = CreateLine( pNode, pState );
        if( pGPIO != NULL )
        {
            /* set the line direction */
            ParseLineDirection( pGPIO, pNode, pState );

            /* set the line active state */
            ParseLineActiveState( pGPIO, pNode );

            /* get the line event enable status */
            ParseLineEvent( pGPIO, pNode );

            /* set the line bias */
            ParseLineBias( pGPIO, pNode );

            /* set the line drive mode */
            ParseLineDrive( pGPIO, pNode );

            /* request (reserve) the line */
            RequestLine( pGPIO, pState );

            /* track monitored events */
            if ( pGPIO->event_type != 0 )
            {
                n = pState->monitoredLines.num_lines;
                if ( n < GPIOD_LINE_BULK_MAX_LINES )
                {
                    pState->monitoredLines.lines[n] = pGPIO->pLine;
                    pState->monitoredLines.num_lines++;
                }
            }

            /* set up the variable notification on the GPIO line */
            SetupNotification( pGPIO, pState );

            /* create a software PWM if applicable */
            if ( ( pState->gpiowatch == false ) &&
                 ( pGPIO->PWM == true ) )
            {
                CreatePWM( pGPIO );
            }
        }
    }

    return EOK;
}

/*============================================================================*/
/*  RequestLine                                                               */
/*!
    Request the line from the gpiod library

    The RequestLine function requests access to the line from the gpiod library
    It sets up the gpio line direction, active state, bias, and drive mode,
    as well as setting the value of the line if it is an output.

    @param[in]
       pGPIO
            pointer to the GPIO line to request

    @param[in]
        pState
            pointer to the gpioctrl state object

    @retval EOK the line was successfully requested
    @retval EINVAL invalid arguments

==============================================================================*/
static int RequestLine( GPIO *pGPIO, GPIOCtrlState *pState )
{
    int result = EINVAL;
    int rc;
    bool request = false;
    int value;

    if ( ( pGPIO != NULL ) &&
         ( pState != NULL ) &&
         ( pState->service != NULL ) )
    {
        /* set the consumer name */
        pGPIO->request.consumer = pState->service;
        pGPIO->request.flags = 0;

        if( pGPIO->event_type != 0 )
        {
            /* if an event type is specified, we auomatically configure
            the line as an input and set the request type to the specified
            edge trigger */
            pGPIO->request.request_type = pGPIO->event_type;
        }

        /* perform the line request */
        request = (((pState->gpiowatch == true) && (pGPIO->event_type != 0)) ||
                   ((pState->gpiowatch == false) && (pGPIO->event_type == 0)));

        if ( request == true )
        {
            value = pGPIO->value;

            if( pGPIO->PWM == true )
            {
                /* initial value for sofware PWM lines is 0 */
                value = 0;
            }

            rc = gpiod_line_request( pGPIO->pLine,
                                     &pGPIO->request,
                                     value );

            result = ( rc == -1 ) ? errno : EOK;
        }
        else
        {
            result = EOK;
        }
    }

    return result;
}

/*============================================================================*/
/*  SetupPrintNotifications                                                   */
/*!
    Set up a render notifications for the GPIO controller

    The SetupPrintNotifications function sets up the render notifications
    for the GPIO controller.

    @param[in]
        pState
            pointer to the GPIO controller state which contains a handle
            to the variable server for requesting the notifications.

    @retval EOK the notification was successfully requested
    @retval ENOENT the requested variable was not found
    @retval EINVAL invalid arguments

==============================================================================*/
static int SetupPrintNotifications( GPIOCtrlState *pState )
{
    int result = EINVAL;
    VAR_HANDLE hVar;

    if ( ( pState != NULL ) &&
         ( pState->gpiowatch == false ) )
    {
        hVar = VAR_FindByName( pState->hVarServer, "/SYS/GPIOCTRL/INFO" );
        if( hVar != VAR_INVALID )
        {
            result = VAR_Notify( pState->hVarServer,
                                 hVar,
                                 NOTIFY_PRINT );
        }
        else
        {
            result = ENOENT;
        }
    }

    return result;
}

/*============================================================================*/
/*  SetupNotification                                                         */
/*!
    Set up a variable server notification for the GPIO line

    The SetupNotification function requests a notification from the variable
    server for the variable associated with the specified GPIO line.

    GPIO lines in INPUT mode will set up a CALC notification to force a query
    of the input line when the variable is requested.

    GPIO lines in OUTPUT mode will set up a MODIFIED notification to force
    the output to be changed when the variable is modified.

    It is assumed that the line direction has already been initialized when
    this function is called.

    @param[in]
       pGPIO
            pointer to the GPIO line to set up a notification for

    @param[in]
        pState
            pointer to the GPIO controller state which contains a handle
            to the variable server for requesting the notification.

    @retval EOK the notification was successfully requested
    @retval EINVAL invalid arguments

==============================================================================*/
static int SetupNotification( GPIO *pGPIO, GPIOCtrlState *pState )
{
    int result = EINVAL;

    if ( ( pGPIO != NULL ) &&
         ( pState != NULL ) &&
         ( pState->gpiowatch == false ) )
    {
        if ( ( pGPIO->direction == GPIOD_LINE_DIRECTION_INPUT ) &&
             ( pGPIO->event_type == 0 ) )
        {
            result = VAR_Notify( pState->hVarServer,
                                 pGPIO->hVar,
                                 NOTIFY_CALC );
        }
        else if ( pGPIO->direction == GPIOD_LINE_DIRECTION_OUTPUT )
        {
            result = VAR_Notify( pState->hVarServer,
                                 pGPIO->hVar,
                                 NOTIFY_MODIFIED );
        }
    }

    return result;
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

                        /* store the line number */
                        pGPIOLine->line_num = line_num;

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
                else
                {
                    printf("failed to create line %d\n", line_num );
                }
            }
            else
            {
                printf("cannot get line\n");
            }
        }
        else
        {
            printf("Unable to Get var handle\n");
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

    @param[in]
        pGPIOCtrlState
            pointer to the GPIO controller state containing the handle to
            the variable server to get the output value

    @retval EOK the line information object was obtained
    @retval ENOENT the line information was not found
    @retval EINVAL invalid arguments or incorrect JSON format

==============================================================================*/
static int ParseLineDirection( GPIO *pGPIO,
                               JNode *pNode,
                               GPIOCtrlState *pState )
{
    int result = EINVAL;
    char *direction;

    if ( ( pGPIO != NULL ) &&
         ( pGPIO->pLine != NULL ) &&
         ( pState != NULL ) &&
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
            pGPIO->direction = GPIOD_LINE_DIRECTION_INPUT;
            pGPIO->request.request_type = GPIOD_LINE_REQUEST_DIRECTION_INPUT;
            result = EOK;
        }
        else if( strcmp( direction, "output" ) == 0 )
        {
            /* set the line to "output" and set the default value */
            pGPIO->direction = GPIOD_LINE_DIRECTION_OUTPUT;
            pGPIO->request.request_type = GPIOD_LINE_REQUEST_DIRECTION_OUTPUT;
            GetLineOutputValue( pState->hVarServer, pGPIO );
            result = EOK;
        }
        else if( strcmp( direction, "pwm" ) == 0 )
        {
            /* set the line to "output" and set the default value */
            pGPIO->PWM = true;
            pGPIO->direction = GPIOD_LINE_DIRECTION_OUTPUT;
            pGPIO->request.request_type = GPIOD_LINE_REQUEST_DIRECTION_OUTPUT;
            GetLineOutputValue( pState->hVarServer, pGPIO );
            result = EOK;
        }
        else
        {
            result = ENOTSUP;
        }
    }

    return result;
}

/*============================================================================*/
/*  ParseLineActiveState                                                      */
/*!
    Parse the GPIO definition to set the active state for the GPIO line

    The ParseLineActiveState function sets the active state for the
    GPIO line object.

    Two valid active state values are supported:  "low" and "high"

    If the active state is not specified, it is assumed to be "high"

    If an active state is specified, the JSON object is expected to have an
    attribute key "active_state"

    @param[in]
        pGPIO
            pointer to the GPIO object for the specified line

    @param[in]
        pNode
            pointer to the line node to search for the "active_state" attribute

    @retval EOK the line active state was set up
    @retval ENOTSUP the specified line active state was not supported
    @retval EINVAL invalid arguments

==============================================================================*/
static int ParseLineActiveState( GPIO *pGPIO, JNode *pNode )
{
    int result = EINVAL;
    char *active_state;
    const char *consumer = "gpioctrl";

    if ( ( pGPIO != NULL ) &&
         ( pNode != NULL ) )
    {
        /* indicate success */
        result = EOK;

        /* get the "active_state" attribute from the GPIO line definition */
        active_state = JSON_GetStr( pNode, "active_state" );
        if ( active_state != NULL )
        {
            if ( strcmp( active_state, "low" ) == 0 )
            {
                /* set the line to active low */
                pGPIO->request.flags |= GPIOD_LINE_REQUEST_FLAG_ACTIVE_LOW;
            }
            else if ( strcmp( active_state, "high" ) == 0 )
            {
                /* set the line to active low */
                pGPIO->request.flags &= ~GPIOD_LINE_REQUEST_FLAG_ACTIVE_LOW;
            }
            else
            {
                /* unsupported line active state */
                result = ENOTSUP;
            }
        }
    }

    return result;
}

/*============================================================================*/
/*  ParseLineEvent                                                            */
/*!
    Parse the GPIO definition to see if the GPIO input generates an event

    The ParseLineEvent function checks the event attribute to determine
    if the GPIO input triggers an event on transition.

    Two valid event state values are supported:  true and false

    If the event state is not specified, it is assumed to be false

    @param[in]
        pGPIO
            pointer to the GPIO object for the specified line

    @param[in]
        pNode
            pointer to the line node to search for the "event" attribute

    @retval EOK the line event state was set up
    @retval ENOTSUP the specified line event state was not supported
    @retval EINVAL invalid arguments

==============================================================================*/
static int ParseLineEvent( GPIO *pGPIO, JNode *pNode )
{
    int result = EINVAL;
    char *event_state;
    int event_type;
    const char *consumer = "gpioctrl";

    if ( ( pGPIO != NULL ) &&
         ( pNode != NULL ) )
    {
        /* indicate success */
        result = EOK;

        /* get the "active_state" attribute from the GPIO line definition */
        event_state = JSON_GetStr( pNode, "event" );
        if ( event_state != NULL )
        {
            if ( strcmp( event_state, "RISING_EDGE" ) == 0 )
            {
                /* event monitoring for rising edges */
                pGPIO->event_type = GPIOD_LINE_REQUEST_EVENT_RISING_EDGE;
            }
            else if ( strcmp( event_state, "FALLING_EDGE" ) == 0 )
            {
                /* event monitoring for falling edges */
                pGPIO->event_type = GPIOD_LINE_REQUEST_EVENT_FALLING_EDGE;
            }
            else if ( strcmp( event_state, "BOTH_EDGES") == 0 )
            {
                /* event monitoring for both edges */
                pGPIO->event_type = GPIOD_LINE_REQUEST_EVENT_BOTH_EDGES;
            }
            else
            {
                /* unsupported line active state */
                pGPIO->event_type = 0;
                result = ENOTSUP;
            }
        }
        else
        {
            pGPIO->event_type = 0;
        }
    }

    return result;
}

/*============================================================================*/
/*  ParseLineBias                                                             */
/*!
    Parse the GPIO definition to set the bias for the GPIO line

    The ParseLineBias function sets the bias for the
    GPIO line object.

    Three valid bias state values are supported:
        "disabled", "pull-down", "pull-up"

    If the bias is not specified, it is assumed to be "disabled"

    If a bias is specified, the JSON object is expected to have an
    attribute key "bias"

    @param[in]
        pGPIO
            pointer to the GPIO object for the specified line

    @param[in]
        pNode
            pointer to the line node to search for the "bias" attribute

    @retval EOK the line bias was set up
    @retval ENOTSUP the specified line bias was not supported
    @retval EINVAL invalid arguments or incorrect JSON format

==============================================================================*/
static int ParseLineBias( GPIO *pGPIO, JNode *pNode )
{
    int result = EINVAL;
    char *bias;

    if ( ( pGPIO != NULL ) &&
         ( pNode != NULL ) )
    {
        /* indicate success */
        result = EOK;

        /* get the "bias" attribute from the GPIO line definition */
        bias = JSON_GetStr( pNode, "bias" );
        if ( bias != NULL )
        {
            if ( strcmp( bias, "disabled" ) == 0 )
            {
                /* set the bias to disabled */
                pGPIO->request.flags |= GPIOD_LINE_REQUEST_FLAG_BIAS_DISABLE;
            }
            else if ( strcmp( bias, "pull-down" ) == 0 )
            {
                /* set the bias to pull-down */
                pGPIO->request.flags |= GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN;
            }
            else if ( strcmp( bias, "pull-up" ) == 0 )
            {
                /* set the bias to pull-up */
                pGPIO->request.flags |= GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP;
            }
            else
            {
                result = ENOTSUP;
            }
        }
    }

    return result;
}

/*============================================================================*/
/*  ParseLineDrive                                                            */
/*!
    Parse the GPIO definition to set the drive mode for the GPIO line

    The ParseLineDrive function sets the drive mode for the GPIO line object.

    Three valid drive mode values are supported:
        "push-pull", "open-drain", "open-source"

    If the drive mode is not specified, it is assumed to be "push-pull"

    If a drive mode is specified, the JSON object is expected to have an
    attribute key "drive"

    @param[in]
        pGPIO
            pointer to the GPIO object for the specified line

    @param[in]
        pNode
            pointer to the line node to search for the "drive" attribute

    @retval EOK the line bias was set up
    @retval ENOTSUP the specified line bias was not supported
    @retval EINVAL invalid arguments or incorrect JSON format

==============================================================================*/
static int ParseLineDrive( GPIO *pGPIO, JNode *pNode )
{
    int result = EINVAL;
    char *drive;
    int pushpull = ~( GPIOD_LINE_REQUEST_FLAG_OPEN_DRAIN |
                      GPIOD_LINE_REQUEST_FLAG_OPEN_SOURCE );

    if ( ( pGPIO != NULL ) &&
         ( pNode != NULL ) )
    {
        /* indicate success */
        result = EOK;

        /* get the "drive" attribute from the GPIO line definition */
        drive = JSON_GetStr( pNode, "drive" );
        if ( drive != NULL )
        {
            if ( strcmp( drive, "push-pull" ) == 0 )
            {
                /* set the drive to push-pull by clearing the open-source
                 * and open-drain bits */
                pGPIO->request.flags &= pushpull;
            }
            else if ( strcmp( drive, "open-source" ) == 0 )
            {
                /* set the drive mode to open-source */
                pGPIO->request.flags |= GPIOD_LINE_REQUEST_FLAG_OPEN_SOURCE;
            }
            else if ( strcmp( drive, "open-drain" ) == 0 )
            {
                /* set the drive mode to open-drain */
                pGPIO->request.flags |= GPIOD_LINE_REQUEST_FLAG_OPEN_DRAIN;
            }
            else
            {
                result = ENOTSUP;
            }
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
        direction = pGPIO->direction;
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

/*============================================================================*/
/*  FindVar                                                                  */
/*!
    Find a Variable given a handle to its associated gpiod line

    The FindVar function iterates through all of the GPIO chips looking
    for the variable handle associated with the specified gpiod line

    @param[in]
        pState
            pointer to the GPIOCtrl state which contains the list of
            GPIO chips to search

    @param[in]
        pLine
            pointer to the gpiod_line object to search for

    @retval handle to the variable associated with the gpiod line
    @retval VAR_INVALID if the variable could not be found

==============================================================================*/
static VAR_HANDLE FindVar( GPIOCtrlState *pState, struct gpiod_line *pLine )
{
    VAR_HANDLE hVar = VAR_INVALID;
    GPIOChip *pGPIOChip;
    GPIO *pGPIO;
    bool found = false;

    if ( ( pState != NULL ) &&
         ( pLine != NULL ) )
    {
        /* start looking in the first GPIO chip */
        pGPIOChip = pState->pFirstGPIOChip;
        while ( ( pGPIOChip != NULL ) && ( found == false ) )
        {
            /* start looking in the first line of the chip */
            pGPIO = pGPIOChip->pFirstLine;
            while( ( pGPIO != NULL ) && ( found == false ) )
            {
                /* check for a gpiod line match */
                if( pGPIO->pLine == pLine )
                {
                    /* save the found GPIO variable */
                    hVar = pGPIO->hVar;

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

    return hVar;
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

==============================================================================*/
static void TerminationHandler( int signum, siginfo_t *info, void *ptr )
{
    syslog( LOG_ERR, "termination of gpioctrl\n" );
    state.running = false;
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
                        if( pGPIO->PWM == true )
                        {
                            pGPIO->value = ( var.val.ui <= 255  ) ? var.val.ui
                                                                  : 255;
                        }
                        else
                        {
                            /* get the value to write to the output */
                            pGPIO->value = ( var.val.ui > 0 ) ? 1 : 0;

                            /* set the output value to the hardware */
                            rc = gpiod_line_set_value( pGPIO->pLine,
                                                       pGPIO->value );

                            /* check the result */
                            result = ( rc == EOK ) ? EOK : errno;
                            if( result != EOK )
                            {
                                syslog( LOG_ERR, "UpdateOutput: %d %s",
                                        result,
                                        strerror(result) );
                            }
                            result = EOK;
                        }
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
                    var.type = VARTYPE_UINT16;
                    var.len = sizeof(uint16_t);

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

/*============================================================================*/
/*  PrintStatus                                                               */
/*!
    Print the GPIO controller status

    The PrintStatus function iterates through the GPIO lines and
    outputs a JSON object which lists the GPIO lines.

@param[in]
    pState
        pointer to the GPIO controller state

@param[in]
    fd
        output file descriptor

@retval EOK the GPIO status was printed
@retval EINVAL invalid arguments

==============================================================================*/
static int PrintStatus( GPIOCtrlState *pState, int fd )
{
    GPIOChip *pGPIOChip;
    GPIO *pGPIO;
    int result = EINVAL;

    if ( ( pState != NULL ) &&
         ( fd != -1 ) )
    {
        (void)write( fd, "[", 1 );

        /* start looking in the first GPIO chip */
        pGPIOChip = pState->pFirstGPIOChip;
        while ( pGPIOChip != NULL )
        {
            if( pGPIOChip != pState->pFirstGPIOChip )
            {
                (void)write( fd, ",", 1 );
            }

            dprintf(fd, "{ \"chip\" : \"%s\", \"lines\" : [", pGPIOChip->name);

            /* start looking in the first line of the chip */
            pGPIO = pGPIOChip->pFirstLine;
            while ( pGPIO != NULL )
            {
                if( pGPIO != pGPIOChip->pFirstLine )
                {
                    (void)write( fd, ",", 1 );
                }

                /* print the line information */
                PrintLineInfo( pState, pGPIO, fd );

                /* move on to the next GPIO line */
                pGPIO = pGPIO->pNext;
            }

            /* close the chip */
            (void)write( fd, "]}", 2 );

            /* move to the next GPIO chip */
            pGPIOChip = pGPIOChip->pNext;
        }

        (void)write( fd, "]", 1 );

        result = EOK;
    }

    return result;
}

/*============================================================================*/
/*  PrintLineInfo                                                             */
/*!
    Print the GPIO line information

    The PrintLineInfo function prints a JSON object containing the
    GPIO line information

@param[in]
    pState
        pointer to the GPIO controller state

@param[in]
    pGPIO
        pointer to the GPIO object to print

@param[in]
    fd
        output file descriptor

@retval EOK the GPIO line information was printed
@retval EINVAL invalid arguments

==============================================================================*/
static int PrintLineInfo( GPIOCtrlState *pState, GPIO *pGPIO, int fd )
{
    GPIOChip *pGPIOChip;
    int result = EINVAL;
    const char *line_name;

    if ( ( pState != NULL ) &&
         ( pGPIO != NULL ) &&
         ( fd != -1 ) )
    {
        if ( pGPIO->pLine != NULL )
        {
            /* get the line name */
            line_name = gpiod_line_name( pGPIO->pLine );

            dprintf( fd,
                     "{ \"line\" : %d, "
                     "\"name\" : \"%s\", "
                     "\"var\" : \"%s\"}",
                     pGPIO->line_num,
                     ( line_name != NULL ) ? line_name : "unknown",
                     pGPIO->name);
        }

        result = EOK;
    }

    return result;
}

/*============================================================================*/
/* Shutdown                                                                   */
/*!
    Shutdown the GPIO control service

    The Shutdown service iterates through all of the GPIO resources
    and closes and deallocates them.

@param[in]
    pState
        pointer to the GPIO controller state

==============================================================================*/
static void Shutdown( GPIOCtrlState *pState )
{
    GPIOChip *pGPIOChip;
    GPIOChip *pTempGPIOChip;
    GPIO *pGPIO;
    GPIO *pTempGPIO;

    if ( pState != NULL )
    {
        /* start looking in the first GPIO chip */
        pGPIOChip = pState->pFirstGPIOChip;
        while ( pGPIOChip != NULL )
        {
            /* start looking in the first line of the chip */
            pGPIO = pGPIOChip->pFirstLine;
            while ( pGPIO != NULL )
            {
                pTempGPIO = pGPIO;

                /* move on to the next GPIO line */
                pGPIO = pGPIO->pNext;

                if ( pTempGPIO->pLine != NULL )
                {
                    gpiod_line_release( pTempGPIO->pLine );
                }

                /* free the GPIO line object */
                free( pTempGPIO );
            }

            pTempGPIOChip = pGPIOChip;

            if( pTempGPIOChip->pChip != NULL )
            {
                gpiod_chip_close( pTempGPIOChip->pChip );
            }

            /* move to the next GPIO chip */
            pGPIOChip = pGPIOChip->pNext;
        }
    }

    pState->pFirstGPIOChip = NULL;
    pState->pLastGPIOChip = NULL;
}

/*============================================================================*/
/*  CreatePWM                                                                 */
/*!
    Create an output PWM thread

    The CreatePWM thread creates a thread for controlling a software PWM
    pin.  This is highly inefficient and not recommended for a large
    number of GPIO pins, but may be used in a pinch if you have CPU
    cycles to burn.

@param[in]
    pGPIO
        pointer to the GPIO pin to create a PWM thread for

==============================================================================*/
static int CreatePWM( GPIO *pGPIO )
{
    int result = EINVAL;
    pthread_attr_t attr;
    pthread_t thread;

    if( pGPIO != NULL )
    {
        result = pthread_attr_init( &attr );

        result = pthread_create( &thread, NULL, PWMThread, (void *)pGPIO );
        pthread_attr_destroy( &attr );
    }
}

/*============================================================================*/
/*  PWM Thread                                                                */
/*!
    PWM Thread

    The PWM thread is associated with a single GPIO pin.  It toggles
    the GPIO pin on and off with ~ 100Hz frequency.  The value assigned
    to the PWM pin controls the duty cycle within the range [0.255]
    For example, setting the pin's value to 128 will set ~50% duty
    cycle.

@param[in]
    pGPIO
        pointer to the GPIO pin to control as a PWM output

==============================================================================*/
static void *PWMThread( void *arg )
{
    GPIO *pGPIO = (GPIO *)arg;
    int t;
    sigset_t mask;

    /* block real time signals on this thread */
    sigemptyset( &mask );
    sigaddset( &mask, SIG_VAR_MODIFIED );
    sigaddset( &mask, SIG_VAR_CALC );
    sigaddset( &mask, SIG_VAR_PRINT );
    sigaddset( &mask, SIG_VAR_VALIDATE );
    sigprocmask( SIG_BLOCK, &mask, NULL );

    if( pGPIO != NULL )
    {
        /* repeat forever */
        while ( 1 )
        {

            /* limit the value to [0.255] */
            if ( pGPIO->value < 0 )
            {
                pGPIO->value = 0;
            }

            if ( pGPIO->value > 255 )
            {
                pGPIO->value = 255;
            }

            /* sleep until it is tim to turn the output off */
            t = ( pGPIO->value * 40 );
            if ( t > 0 )
            {
                /* set the output value to the hardware */
                gpiod_line_set_value( pGPIO->pLine, 1 );
                usleep( t );
            }


            /* sleep until it is time to turn the output on */
            t = ( ( 255 - pGPIO->value ) * 40 );
            if ( t > 0 )
            {
                /* set the output value to the hardware */
                gpiod_line_set_value( pGPIO->pLine, 0 );
                usleep ( t );
            }
        }
    }
}

/*! @}
 * end of gpioctrl group */
