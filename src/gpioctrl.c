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
/*==========================================================================*/

/*============================================================================
        Includes
============================================================================*/

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <poll.h>
#include <syslog.h>
#include <pthread.h>
#include <varserver/varserver.h>
#include <tjson/json.h>
#include <gpiod.h>

/*============================================================================
        Private definitions
============================================================================*/

struct _gpio_chip;

/*! the _gpio structure maps a chip line offset to its associated variable */
typedef struct _gpio
{
    /*! chip containing this line (set when the line is created) */
    struct _gpio_chip *pParentChip;

    /*! handle to the variable */
	VAR_HANDLE hVar;

    /*! line offset on the parent chip */
    int line_num;

    /*! name of the variable */
	char *name;

    /*! value of the variable */
    int value;

    /*! direction (libgpiod v2 ::gpiod_line_direction) */
    enum gpiod_line_direction direction;

    /*! software PWM output */
    bool PWM;

    /*! true if JSON "event" requests edge monitoring */
    bool edge_configured;

    /*! edge detection when edge_configured (libgpiod v2 ::gpiod_line_edge) */
    enum gpiod_line_edge line_edge;

    bool active_low;

    enum gpiod_line_bias line_bias;

    enum gpiod_line_drive line_drive;

    /*! line included in gpiod_chip_request_lines for this chip */
    bool in_kernel_request;

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

    /*! single request for all kernel-requested offsets on this chip (libgpiod v2) */
    struct gpiod_line_request *pLineRequest;

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

    /*! reusable buffer for gpiod_line_request_read_edge_events */
    struct gpiod_edge_event_buffer *pEdgeBuf;

} GPIOCtrlState;

/*============================================================================
        Private file scoped variables
============================================================================*/

/*! GPIO Controller State object */
GPIOCtrlState state;

/*============================================================================
        Private function declarations
============================================================================*/

int main(int argc, char **argv);
static int ProcessOptions( int argC, char *argV[], GPIOCtrlState *pState );
static void usage( char *cmdname );
static bool GPIOShouldRequest( const GPIO *pGPIO, const GPIOCtrlState *pState );
static int FinalizeChipGPIORequest( GPIOChip *pGPIOChip, GPIOCtrlState *pState );
static bool ChipParticipatesInEdgePoll( const GPIOChip *pChip,
					const GPIOCtrlState *pState );
static int ParseChip( JNode *pNode, void *arg );
static void SetupTerminationHandler( void );
static void TerminationHandler( int signum, siginfo_t *info, void *ptr );
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
static VAR_HANDLE FindVarByLine( const GPIOCtrlState *pState,
				 const GPIOChip *pForChip,
				 unsigned int offset );
static int UpdateOutput( VAR_HANDLE hVar, GPIOCtrlState *pState );
static int UpdateInput( VAR_HANDLE hVar, GPIOCtrlState *pState );
static int run( GPIOCtrlState *pState );
static int WaitVarSignal( GPIOCtrlState *pState );
static int WaitGPIOEvent( GPIOCtrlState *pState );
static int HandleGPIOEdgeEvent( GPIOCtrlState *pState,
				GPIOChip *pChip,
				struct gpiod_edge_event *ev );
static int SetupNotification( GPIO *pGPIO, GPIOCtrlState *pState );
static int SetupPrintNotifications( GPIOCtrlState *pState );
static int PrintStatus( GPIOCtrlState *pState, int fd );
static int PrintLineInfo( GPIOCtrlState *pState, GPIO *pGPIO, int fd );
static void Shutdown( GPIOCtrlState *pState );
static int CreatePWM( GPIO *pGPIO );
static void *PWMThread( void *arg );

/*============================================================================
        Private function definitions
============================================================================*/

static bool GPIOShouldRequest( const GPIO *pGPIO, const GPIOCtrlState *pState )
{
	if ( pGPIO == NULL || pState == NULL )
	{
		return false;
	}

	return ( ( pState->gpiowatch == true ) && ( pGPIO->edge_configured == true ) ) ||
	       ( ( pState->gpiowatch == false ) && ( pGPIO->edge_configured == false ) );
}

static bool ChipParticipatesInEdgePoll( const GPIOChip *pChip,
					const GPIOCtrlState *pState )
{
	const GPIO *pGPIO;

	if ( pChip == NULL || pState == NULL || pState->gpiowatch == false ||
	     pChip->pLineRequest == NULL )
	{
		return false;
	}

	for ( pGPIO = pChip->pFirstLine; pGPIO != NULL; pGPIO = pGPIO->pNext )
	{
		if ( ( pGPIO->in_kernel_request == true ) &&
		     ( pGPIO->edge_configured == true ) )
		{
			return true;
		}
	}

	return false;
}

static int FinalizeChipGPIORequest( GPIOChip *pGPIOChip, GPIOCtrlState *pState )
{
	struct gpiod_line_config *line_cfg;
	struct gpiod_request_config *req_cfg;
	GPIO *pGPIO;
	int rc;
	unsigned int offset;
	int result = EOK;
	int added = 0;

	if ( ( pGPIOChip == NULL ) || ( pGPIOChip->pChip == NULL ) ||
	     ( pState == NULL ) )
	{
		return EINVAL;
	}

	line_cfg = gpiod_line_config_new( );
	req_cfg = gpiod_request_config_new( );
	if ( ( line_cfg == NULL ) || ( req_cfg == NULL ) )
	{
		if ( line_cfg != NULL )
		{
			gpiod_line_config_free( line_cfg );
		}
		if ( req_cfg != NULL )
		{
			gpiod_request_config_free( req_cfg );
		}
		return ENOMEM;
	}

	gpiod_request_config_set_consumer( req_cfg, pState->service );

	for ( pGPIO = pGPIOChip->pFirstLine; pGPIO != NULL; pGPIO = pGPIO->pNext )
	{
		struct gpiod_line_settings *settings;

		pGPIO->in_kernel_request = false;
		if ( GPIOShouldRequest( pGPIO, pState ) == false )
		{
			continue;
		}

		settings = gpiod_line_settings_new( );
		if ( settings == NULL )
		{
			continue;
		}

		gpiod_line_settings_set_direction( settings, pGPIO->direction );
		if ( pGPIO->direction == GPIOD_LINE_DIRECTION_OUTPUT )
		{
			int out_val = pGPIO->value;

			if ( pGPIO->PWM == true )
			{
				out_val = 0;
			}
			(void)gpiod_line_settings_set_output_value(
				settings,
				( out_val != 0 ) ? GPIOD_LINE_VALUE_ACTIVE
					       : GPIOD_LINE_VALUE_INACTIVE );
		}

		gpiod_line_settings_set_active_low( settings, pGPIO->active_low );

		if ( pGPIO->line_bias != GPIOD_LINE_BIAS_AS_IS )
		{
			(void)gpiod_line_settings_set_bias( settings, pGPIO->line_bias );
		}

		if ( pGPIO->line_drive != GPIOD_LINE_DRIVE_PUSH_PULL )
		{
			(void)gpiod_line_settings_set_drive( settings, pGPIO->line_drive );
		}

		if ( ( pGPIO->direction == GPIOD_LINE_DIRECTION_INPUT ) &&
		     ( pGPIO->edge_configured == true ) )
		{
			(void)gpiod_line_settings_set_edge_detection( settings,
								    pGPIO->line_edge );
		}

		offset = (unsigned int)pGPIO->line_num;
		rc = gpiod_line_config_add_line_settings( line_cfg, &offset, 1,
							  settings );
		gpiod_line_settings_free( settings );
		if ( rc == 0 )
		{
			pGPIO->in_kernel_request = true;
			added++;
		}
	}

	if ( added == 0 )
	{
		gpiod_line_config_free( line_cfg );
		gpiod_request_config_free( req_cfg );
		return EOK;
	}

	pGPIOChip->pLineRequest =
		gpiod_chip_request_lines( pGPIOChip->pChip, req_cfg, line_cfg );
	gpiod_request_config_free( req_cfg );
	gpiod_line_config_free( line_cfg );

	if ( pGPIOChip->pLineRequest == NULL )
	{
		syslog( LOG_ERR, "FinalizeChipGPIORequest: %s", strerror( errno ) );
		result = EIO;
	}

	return result;
}

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
int main(int argc, char **argv)
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

    return 0;
}

/*==========================================================================*/
/*  run                                                                     */
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

*//*
    REVISION HISTORY:

    Version: 1.00    25-Apr-2021     By: Trevor Monk
        - created

    Version: 1.01    17-May-2021     By: Trevor Monk
        - Replaced global state with local pState

    Version: 1.02    28-Feb-2022     By: Trevor Monk
        - Add support for handling gpio events

============================================================================*/
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

*//*
    REVISION HISTORY:

    Version: 1.00    28-Feb-2022     By: Trevor Monk
        - created

============================================================================*/
static int WaitGPIOEvent( GPIOCtrlState *pState )
{
	GPIOChip *pChip;
	struct pollfd pfds[64];
	GPIOChip *chips[64];
	int n;
	int i;
	int pr;
	size_t cap;
	int num_ev;
	size_t ev_idx;

	if ( pState == NULL )
	{
		return EINVAL;
	}

	if ( pState->pEdgeBuf == NULL )
	{
		pState->pEdgeBuf = gpiod_edge_event_buffer_new( 32 );
		if ( pState->pEdgeBuf == NULL )
		{
			return ENOMEM;
		}
	}

	n = 0;
	for ( pChip = pState->pFirstGPIOChip; pChip != NULL; pChip = pChip->pNext )
	{
		if ( ChipParticipatesInEdgePoll( pChip, pState ) == false )
		{
			continue;
		}
		if ( n >= (int)( sizeof( pfds ) / sizeof( pfds[0] ) ) )
		{
			break;
		}
		pfds[n].fd = gpiod_line_request_get_fd( pChip->pLineRequest );
		pfds[n].events = POLLIN;
		pfds[n].revents = 0;
		chips[n] = pChip;
		n++;
	}

	if ( n == 0 )
	{
		return EOK;
	}

	pr = poll( pfds, (nfds_t)n, -1 );
	if ( pr < 0 )
	{
		return errno;
	}

	cap = gpiod_edge_event_buffer_get_capacity( pState->pEdgeBuf );
	for ( i = 0; i < n; i++ )
	{
		if ( ( pfds[i].revents & POLLIN ) == 0 )
		{
			continue;
		}
		num_ev = gpiod_line_request_read_edge_events(
			chips[i]->pLineRequest, pState->pEdgeBuf, cap );
		if ( num_ev < 0 )
		{
			continue;
		}
		num_ev = (int)gpiod_edge_event_buffer_get_num_events(
			pState->pEdgeBuf );
		for ( ev_idx = 0; ev_idx < (size_t)num_ev; ev_idx++ )
		{
			struct gpiod_edge_event *ev = gpiod_edge_event_buffer_get_event(
				pState->pEdgeBuf, ev_idx );

			(void)HandleGPIOEdgeEvent( pState, chips[i], ev );
		}
	}

	return EOK;
}

/*==========================================================================*/
/*  HandleGPIOEdgeEvent                                                     */
/*!
    Handle a GPIO input edge event (libgpiod v2)

============================================================================*/
static int HandleGPIOEdgeEvent( GPIOCtrlState *pState,
				GPIOChip *pChip,
				struct gpiod_edge_event *ev )
{
	int result = EINVAL;
	VAR_HANDLE hVar;
	VarObject var;
	uint16_t val;

	if ( ( pState != NULL ) && ( pChip != NULL ) && ( ev != NULL ) )
	{
		val = ( gpiod_edge_event_get_event_type( ev ) ==
			GPIOD_EDGE_EVENT_RISING_EDGE )
			  ? 1
			  : 0;

		hVar = FindVarByLine( pState, pChip,
				      gpiod_edge_event_get_line_offset( ev ) );
		if ( hVar != VAR_INVALID )
		{
			var.val.ui = val;
			var.type = VARTYPE_UINT16;
			var.len = sizeof( uint16_t );

			result = VAR_Set( pState->hVarServer, hVar, &var );
		}
		else
		{
			result = ENOENT;
		}
	}

	return result;
}

/*==========================================================================*/
/*  WaitVarSignal                                                           */
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

*//*
    REVISION HISTORY:

    Version: 1.00    28-Feb-2022     By: Trevor Monk
        - created

============================================================================*/
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

*//*
    REVISION HISTORY:

    Version: 1.0    23-Apr-2021     By: Trevor Monk
        - created

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
        if ( result == EOK )
        {
            result = FinalizeChipGPIORequest( pState->pLastGPIOChip, pState );
        }
        if ( ( result == EOK ) && ( pState->gpiowatch == false ) )
        {
            GPIO *pPWM;

            for ( pPWM = pState->pLastGPIOChip->pFirstLine; pPWM != NULL;
                  pPWM = pPWM->pNext )
            {
                if ( pPWM->PWM == true )
                {
                    (void)CreatePWM( pPWM );
                }
            }
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

*//*
    REVISION HISTORY:

    Version: 1.00    25-Apr-2021     By: Trevor Monk
        - created

    Version: 1.01    17-May-2021     By: Trevor Monk
        - Added missing return value

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

*//*
    REVISION HISTORY:

    Version: 1.0    23-Apr-2021     By: Trevor Monk
        - created

    Version: 1.01   28-Feb-2022     By: Trevor Monk
        - track monitored lines using a gpiod_lines_bulk object

    Version: 1.02   16-Mar-2022     By: Trevor Monk
        - Add software PWM

==============================================================================*/
static int ParseLine( JNode *pNode, void *arg )
{
    GPIOCtrlState *pState = (GPIOCtrlState *)arg;
    GPIO *pGPIO;

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

            /* set up the variable notification on the GPIO line */
            SetupNotification( pGPIO, pState );
        }
    }

    return EOK;
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

*//*
    REVISION HISTORY:

    Version: 1.0    25-Apr-2021     By: Trevor Monk
        - created

    Version: 1.01   28-Feb-2022     By: Trevor Monk
        - only set up notification if we are not in gpiowatch mode

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

*//*
    REVISION HISTORY:

    Version: 1.0    25-Apr-2021     By: Trevor Monk
        - created

    Version: 1.01   28-Feb-2022     By: Trevor Monk
        - only set up notification if we are not in gpiowatch mode
        - only set up calc notifications for inputs that have no event type

==============================================================================*/
static int SetupNotification( GPIO *pGPIO, GPIOCtrlState *pState )
{
    int result = EINVAL;

    if ( ( pGPIO != NULL ) &&
         ( pState != NULL ) &&
         ( pState->gpiowatch == false ) )
    {
        if ( ( pGPIO->direction == GPIOD_LINE_DIRECTION_INPUT ) &&
             ( pGPIO->edge_configured == false ) )
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

    if ( ( pNode != NULL ) &&
         ( pState != NULL ) &&
         ( pState->pLastGPIOChip != NULL ) )
    {
        /* get a pointer to the GPIOChip object we are currently processing */
        pGPIOChip = pState->pLastGPIOChip;

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

                /* allocate memory for the GPIO line object */
                pGPIOLine = calloc( 1, sizeof( GPIO ) );
                if( pGPIOLine != NULL )
                {
                    /* store the variable handle */
                    pGPIOLine->hVar = hVar;

                    /* store the variable name */
                    pGPIOLine->name = varname;

                    /* store the line offset */
                    pGPIOLine->line_num = (int)line_num;

                    pGPIOLine->pParentChip = pGPIOChip;
                    pGPIOLine->line_bias = GPIOD_LINE_BIAS_AS_IS;
                    pGPIOLine->line_drive = GPIOD_LINE_DRIVE_PUSH_PULL;
                    pGPIOLine->line_edge = GPIOD_LINE_EDGE_NONE;
                    pGPIOLine->edge_configured = false;
                    pGPIOLine->active_low = false;

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

    @param[in]
        pGPIOCtrlState
            pointer to the GPIO controller state containing the handle to
            the variable server to get the output value

    @retval EOK the line information object was obtained
    @retval ENOENT the line information was not found
    @retval EINVAL invalid arguments or incorrect JSON format

*//*
    REVISION HISTORY:

    Version: 1.0    24-Apr-2021     By: Trevor Monk
        - created

==============================================================================*/
static int ParseLineDirection( GPIO *pGPIO,
                               JNode *pNode,
                               GPIOCtrlState *pState )
{
    int result = EINVAL;
    char *direction;

    if ( ( pGPIO != NULL ) &&
         ( pGPIO->pParentChip != NULL ) &&
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
            result = EOK;
        }
        else if( strcmp( direction, "output" ) == 0 )
        {
            /* set the line to "output" and set the default value */
            pGPIO->direction = GPIOD_LINE_DIRECTION_OUTPUT;
            GetLineOutputValue( pState->hVarServer, pGPIO );
            result = EOK;
        }
        else if( strcmp( direction, "pwm" ) == 0 )
        {
            /* set the line to "output" and set the default value */
            pGPIO->PWM = true;
            pGPIO->direction = GPIOD_LINE_DIRECTION_OUTPUT;
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

*//*
    REVISION HISTORY:

    Version: 1.0    25-Apr-2021     By: Trevor Monk
        - created

==============================================================================*/
static int ParseLineActiveState( GPIO *pGPIO, JNode *pNode )
{
    int result = EINVAL;
    char *active_state;

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
                pGPIO->active_low = true;
            }
            else if ( strcmp( active_state, "high" ) == 0 )
            {
                pGPIO->active_low = false;
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

*//*
    REVISION HISTORY:

    Version: 1.0    26-May-2021     By: Trevor Monk
        - created

==============================================================================*/
static int ParseLineEvent( GPIO *pGPIO, JNode *pNode )
{
    int result = EINVAL;
    char *event_state;

    if ( ( pGPIO != NULL ) &&
         ( pNode != NULL ) )
    {
        /* indicate success */
        result = EOK;

        pGPIO->edge_configured = false;
        pGPIO->line_edge = GPIOD_LINE_EDGE_NONE;

        event_state = JSON_GetStr( pNode, "event" );
        if ( event_state != NULL )
        {
            if ( strcmp( event_state, "RISING_EDGE" ) == 0 )
            {
                pGPIO->line_edge = GPIOD_LINE_EDGE_RISING;
                pGPIO->edge_configured = true;
            }
            else if ( strcmp( event_state, "FALLING_EDGE" ) == 0 )
            {
                pGPIO->line_edge = GPIOD_LINE_EDGE_FALLING;
                pGPIO->edge_configured = true;
            }
            else if ( strcmp( event_state, "BOTH_EDGES") == 0 )
            {
                pGPIO->line_edge = GPIOD_LINE_EDGE_BOTH;
                pGPIO->edge_configured = true;
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

*//*
    REVISION HISTORY:

    Version: 1.0    25-Apr-2021     By: Trevor Monk
        - created

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
                pGPIO->line_bias = GPIOD_LINE_BIAS_DISABLED;
            }
            else if ( strcmp( bias, "pull-down" ) == 0 )
            {
                pGPIO->line_bias = GPIOD_LINE_BIAS_PULL_DOWN;
            }
            else if ( strcmp( bias, "pull-up" ) == 0 )
            {
                pGPIO->line_bias = GPIOD_LINE_BIAS_PULL_UP;
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

*//*
    REVISION HISTORY:

    Version: 1.0    25-Apr-2021     By: Trevor Monk
        - created

==============================================================================*/
static int ParseLineDrive( GPIO *pGPIO, JNode *pNode )
{
    int result = EINVAL;
    char *drive;

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
                pGPIO->line_drive = GPIOD_LINE_DRIVE_PUSH_PULL;
            }
            else if ( strcmp( drive, "open-source" ) == 0 )
            {
                pGPIO->line_drive = GPIOD_LINE_DRIVE_OPEN_SOURCE;
            }
            else if ( strcmp( drive, "open-drain" ) == 0 )
            {
                pGPIO->line_drive = GPIOD_LINE_DRIVE_OPEN_DRAIN;
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
         ( pGPIO->pParentChip != NULL ) )
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

/*============================================================================*/
/*  FindVarByLine                                                            */
/*!
    Find a variable associated with a chip line offset (libgpiod v2)

==============================================================================*/
static VAR_HANDLE FindVarByLine( const GPIOCtrlState *pState,
				 const GPIOChip *pForChip,
				 unsigned int offset )
{
	VAR_HANDLE hVar = VAR_INVALID;
	const GPIOChip *pGPIOChip;
	const GPIO *pGPIO;

	if ( ( pState == NULL ) || ( pForChip == NULL ) )
	{
		return VAR_INVALID;
	}

	for ( pGPIOChip = pState->pFirstGPIOChip; pGPIOChip != NULL;
	      pGPIOChip = pGPIOChip->pNext )
	{
		if ( pGPIOChip != pForChip )
		{
			continue;
		}
		for ( pGPIO = pGPIOChip->pFirstLine; pGPIO != NULL;
		      pGPIO = pGPIO->pNext )
		{
			if ( (unsigned int)pGPIO->line_num == offset )
			{
				return pGPIO->hVar;
			}
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

    Version: 1.00    12-Apr-2021     By: Trevor Monk
        - created

    Version: 1.01    28-Apr-2021     By: Trevor Monk
        - Termination handler sets flag to abort execution.

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

*//*
    REVISION HISTORY:

    Version: 1.0    25-Apr-2021     By: Trevor Monk
        - created

    Version: 1.01   16-Mar-2022     By: Trevor Monk
        - Add support for PWM outputs

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

                            if ( ( pGPIO->in_kernel_request == true ) &&
                                 ( pGPIO->pParentChip != NULL ) &&
                                 ( pGPIO->pParentChip->pLineRequest !=
                                   NULL ) )
                            {
                                rc = gpiod_line_request_set_value(
                                    pGPIO->pParentChip->pLineRequest,
                                    (unsigned int)pGPIO->line_num,
                                    ( pGPIO->value != 0 )
                                        ? GPIOD_LINE_VALUE_ACTIVE
                                        : GPIOD_LINE_VALUE_INACTIVE );
                                if ( rc != 0 )
                                {
                                    syslog( LOG_ERR, "UpdateOutput: %s",
                                            strerror( errno ) );
                                }
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

*//*
    REVISION HISTORY:

    Version: 1.00    25-Apr-2021     By: Trevor Monk
        - created

    Version: 1.01    17-May-2021     By: Trevor Monk
        - Fixed incorrect type assignment and added missing length field

==============================================================================*/
static int UpdateInput( VAR_HANDLE hVar, GPIOCtrlState *pState )
{
    int result = EINVAL;
    GPIO *pGPIO;
    VarObject var;
    int direction;

    if ( ( pState != NULL ) &&
         ( hVar != VAR_INVALID ) )
    {
        /* find the GPIO associated with the specified variable */
        pGPIO = FindGPIO( pState, hVar );
        if( pGPIO != NULL )
        {
            /* get the direction of this GPIO */
            direction = pGPIO->direction;

            /* confirm it is an input */
            if ( direction == GPIOD_LINE_DIRECTION_INPUT )
            {
                enum gpiod_line_value v;

                if ( ( pGPIO->in_kernel_request == false ) ||
                     ( pGPIO->pParentChip == NULL ) ||
                     ( pGPIO->pParentChip->pLineRequest == NULL ) )
                {
                    result = ENOTSUP;
                }
                else
                {
                    v = gpiod_line_request_get_value(
                        pGPIO->pParentChip->pLineRequest,
                        (unsigned int)pGPIO->line_num );
                    if ( v != GPIOD_LINE_VALUE_ERROR )
                    {
                        var.val.ui = ( v == GPIOD_LINE_VALUE_ACTIVE ) ? 1
                                                                      : 0;
                        var.type = VARTYPE_UINT16;
                        var.len = sizeof( uint16_t );

                        result = VAR_Set( pState->hVarServer, hVar, &var );
                    }
                    else
                    {
                        result = EIO;
                    }
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

*//*
    REVISION HISTORY:

    Version: 1.0    26-Apr-2021     By: Trevor Monk
        - created

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

*//*
    REVISION HISTORY:

    Version: 1.0    26-Apr-2021     By: Trevor Monk
        - created

==============================================================================*/
static int PrintLineInfo( GPIOCtrlState *pState, GPIO *pGPIO, int fd )
{
    int result = EINVAL;
    char namebuf[128] = "unknown";
    struct gpiod_line_info *li;
    const char *tmp;

    if ( ( pState != NULL ) &&
         ( pGPIO != NULL ) &&
         ( fd != -1 ) )
    {
        if ( ( pGPIO->pParentChip != NULL ) &&
             ( pGPIO->pParentChip->pChip != NULL ) )
        {
            li = gpiod_chip_get_line_info( pGPIO->pParentChip->pChip,
						   (unsigned int)pGPIO->line_num );
            if ( li != NULL )
            {
                tmp = gpiod_line_info_get_name( li );
                if ( tmp != NULL )
                {
                    (void)snprintf( namebuf, sizeof( namebuf ), "%s", tmp );
                }
                gpiod_line_info_free( li );
            }
        }

        dprintf( fd,
                 "{ \"line\" : %d, "
                 "\"name\" : \"%s\", "
                 "\"var\" : \"%s\"}",
                 pGPIO->line_num,
                 namebuf,
                 pGPIO->name );

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

*//*
    REVISION HISTORY:

    Version: 1.0    27-Apr-2021     By: Trevor Monk
        - created

==============================================================================*/
static void Shutdown( GPIOCtrlState *pState )
{
    GPIOChip *pGPIOChip;
    GPIOChip *pTempGPIOChip;
    GPIO *pGPIO;
    GPIO *pTempGPIO;

    if ( pState != NULL )
    {
        if ( pState->pEdgeBuf != NULL )
        {
            gpiod_edge_event_buffer_free( pState->pEdgeBuf );
            pState->pEdgeBuf = NULL;
        }

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

                /* free the GPIO line object */
                free( pTempGPIO );
            }

            pTempGPIOChip = pGPIOChip;

            if ( pTempGPIOChip->pLineRequest != NULL )
            {
                gpiod_line_request_release( pTempGPIOChip->pLineRequest );
                pTempGPIOChip->pLineRequest = NULL;
            }

            if( pTempGPIOChip->pChip != NULL )
            {
                gpiod_chip_close( pTempGPIOChip->pChip );
                pTempGPIOChip->pChip = NULL;
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

*//*
    REVISION HISTORY:

    Version: 1.0    16-Mar-2022     By: Trevor Monk
        - created

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

    return result;
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

*//*
    REVISION HISTORY:

    Version: 1.0    16-Mar-2022     By: Trevor Monk
        - created

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
                if ( ( pGPIO->pParentChip != NULL ) &&
                     ( pGPIO->pParentChip->pLineRequest != NULL ) )
                {
                    (void)gpiod_line_request_set_value(
                        pGPIO->pParentChip->pLineRequest,
                        (unsigned int)pGPIO->line_num,
                        GPIOD_LINE_VALUE_ACTIVE );
                }
                usleep( t );
            }


            /* sleep until it is time to turn the output on */
            t = ( ( 255 - pGPIO->value ) * 40 );
            if ( t > 0 )
            {
                if ( ( pGPIO->pParentChip != NULL ) &&
                     ( pGPIO->pParentChip->pLineRequest != NULL ) )
                {
                    (void)gpiod_line_request_set_value(
                        pGPIO->pParentChip->pLineRequest,
                        (unsigned int)pGPIO->line_num,
                        GPIOD_LINE_VALUE_INACTIVE );
                }
                usleep ( t );
            }
        }
    }
}



/*! @}
 * end of gpioctrl group */
