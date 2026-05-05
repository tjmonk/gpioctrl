/*===========================================================================
    Unit tests for gpioctrl runtime logic

    Tests exercise the runtime functions (search, update, edge
    event handling, line request finalization) and valid parsing
    paths to verify correct behaviour under normal conditions.
===========================================================================*/

/* Suppress the real main so the test runner can provide its own */
#define main gpioctrl_main

/* Pull in the production source (shim headers intercept includes) */
#include "../src/gpioctrl.c"

#undef main

/*============================================================================
        Test infrastructure includes
============================================================================*/

#include "unity/unity.h"

/*============================================================================
        Test helpers
============================================================================*/

/*==========================================================================*/
/*! Reset all mock state and the global gpioctrl state object */
/*==========================================================================*/
static void reset_all_mocks( void )
{
    mock_gpiod_reset();
    mock_varserver_reset();
    mock_tjson_reset();
    memset( &state, 0, sizeof( state ) );
    state.service = "test_gpioctrl";
}

/*==========================================================================*/
/*!
    Build a minimal chip + line fixture

    Creates a GPIOChip with one GPIO line attached and wires
    it into the global state.  The caller provides storage for
    the chip and line structs.

    @param[in]
        pChip
            pointer to the GPIOChip to initialise

    @param[in]
        pLine
            pointer to the GPIO line to initialise

    @param[in]
        hVar
            variable handle to assign to the line

    @param[in]
        line_num
            line offset on the chip

    @param[in]
        dir
            line direction

============================================================================*/
static void build_fixture( GPIOChip *pChip,
                           GPIO *pLine,
                           VAR_HANDLE hVar,
                           int line_num,
                           enum gpiod_line_direction dir )
{
    if ( ( pChip == NULL ) || ( pLine == NULL ) )
    {
        return;
    }

    memset( pChip, 0, sizeof( *pChip ) );
    memset( pLine, 0, sizeof( *pLine ) );

    pChip->name = "gpiochip0";
    pChip->pChip = gpiod_chip_open( "/dev/gpiochip0" );
    pChip->pLineRequest = gpiod_chip_request_lines(
        pChip->pChip, NULL, NULL );
    pChip->pFirstLine = pLine;
    pChip->pLastLine = pLine;

    pLine->pParentChip = pChip;
    pLine->hVar = hVar;
    pLine->line_num = line_num;
    pLine->name = "/HW/GPIO/TEST";
    pLine->direction = dir;
    pLine->in_kernel_request = true;
    pLine->line_bias = GPIOD_LINE_BIAS_AS_IS;
    pLine->line_drive = GPIOD_LINE_DRIVE_PUSH_PULL;

    state.pFirstGPIOChip = pChip;
    state.pLastGPIOChip = pChip;
    state.hVarServer = VARSERVER_Open();
}

/*============================================================================
        GPIOShouldRequest tests
============================================================================*/

/*==========================================================================*/
/*!
    Test GPIOShouldRequest: gpiowatch + edge line -> true
============================================================================*/
void test_should_request_gpiowatch_edge_line( void )
{
    GPIO gpio;
    bool result;

    reset_all_mocks();
    memset( &gpio, 0, sizeof( gpio ) );
    state.gpiowatch = true;
    gpio.edge_configured = true;

    result = GPIOShouldRequest( &gpio, &state );
    TEST_ASSERT_TRUE( result );
}

/*==========================================================================*/
/*!
    Test GPIOShouldRequest: gpiowatch + non-edge line -> false
============================================================================*/
void test_should_request_gpiowatch_non_edge( void )
{
    GPIO gpio;
    bool result;

    reset_all_mocks();
    memset( &gpio, 0, sizeof( gpio ) );
    state.gpiowatch = true;
    gpio.edge_configured = false;

    result = GPIOShouldRequest( &gpio, &state );
    TEST_ASSERT_FALSE( result );
}

/*==========================================================================*/
/*!
    Test GPIOShouldRequest: normal mode + non-edge -> true
============================================================================*/
void test_should_request_normal_non_edge( void )
{
    GPIO gpio;
    bool result;

    reset_all_mocks();
    memset( &gpio, 0, sizeof( gpio ) );
    state.gpiowatch = false;
    gpio.edge_configured = false;

    result = GPIOShouldRequest( &gpio, &state );
    TEST_ASSERT_TRUE( result );
}

/*==========================================================================*/
/*!
    Test GPIOShouldRequest: NULL arguments -> false
============================================================================*/
void test_should_request_null_args( void )
{
    bool result;

    reset_all_mocks();

    result = GPIOShouldRequest( NULL, &state );
    TEST_ASSERT_FALSE( result );

    result = GPIOShouldRequest( NULL, NULL );
    TEST_ASSERT_FALSE( result );
}

/*============================================================================
        ChipParticipatesInEdgePoll tests
============================================================================*/

/*==========================================================================*/
/*!
    Test ChipParticipatesInEdgePoll: chip with edge line -> true
============================================================================*/
void test_chip_poll_has_edge_lines( void )
{
    GPIOChip chip;
    GPIO line;
    bool result;

    reset_all_mocks();
    build_fixture( &chip, &line, 1, 4,
                   GPIOD_LINE_DIRECTION_INPUT );
    line.edge_configured = true;
    line.in_kernel_request = true;
    state.gpiowatch = true;

    result = ChipParticipatesInEdgePoll( &chip, &state );
    TEST_ASSERT_TRUE( result );
}

/*==========================================================================*/
/*!
    Test ChipParticipatesInEdgePoll: no edge lines -> false
============================================================================*/
void test_chip_poll_no_edge_lines( void )
{
    GPIOChip chip;
    GPIO line;
    bool result;

    reset_all_mocks();
    build_fixture( &chip, &line, 1, 4,
                   GPIOD_LINE_DIRECTION_INPUT );
    line.edge_configured = false;
    state.gpiowatch = true;

    result = ChipParticipatesInEdgePoll( &chip, &state );
    TEST_ASSERT_FALSE( result );
}

/*==========================================================================*/
/*!
    Test ChipParticipatesInEdgePoll: gpiowatch=false -> false
============================================================================*/
void test_chip_poll_not_gpiowatch( void )
{
    GPIOChip chip;
    GPIO line;
    bool result;

    reset_all_mocks();
    build_fixture( &chip, &line, 1, 4,
                   GPIOD_LINE_DIRECTION_INPUT );
    line.edge_configured = true;
    state.gpiowatch = false;

    result = ChipParticipatesInEdgePoll( &chip, &state );
    TEST_ASSERT_FALSE( result );
}

/*==========================================================================*/
/*!
    Test ChipParticipatesInEdgePoll: NULL line request -> false
============================================================================*/
void test_chip_poll_null_request( void )
{
    GPIOChip chip;
    GPIO line;
    bool result;

    reset_all_mocks();
    build_fixture( &chip, &line, 1, 4,
                   GPIOD_LINE_DIRECTION_INPUT );
    chip.pLineRequest = NULL;
    line.edge_configured = true;
    state.gpiowatch = true;

    result = ChipParticipatesInEdgePoll( &chip, &state );
    TEST_ASSERT_FALSE( result );
}

/*============================================================================
        FinalizeChipGPIORequest tests
============================================================================*/

/*==========================================================================*/
/*!
    Test FinalizeChipGPIORequest: one output line -> EOK
============================================================================*/
void test_finalize_with_output_line( void )
{
    GPIOChip chip;
    GPIO line;
    int result;

    reset_all_mocks();
    memset( &chip, 0, sizeof( chip ) );
    memset( &line, 0, sizeof( line ) );

    chip.name = "gpiochip0";
    chip.pChip = gpiod_chip_open( "/dev/gpiochip0" );
    chip.pFirstLine = &line;
    chip.pLastLine = &line;

    line.pParentChip = &chip;
    line.direction = GPIOD_LINE_DIRECTION_OUTPUT;
    line.line_num = 4;
    line.line_bias = GPIOD_LINE_BIAS_AS_IS;
    line.line_drive = GPIOD_LINE_DRIVE_PUSH_PULL;

    state.gpiowatch = false;

    result = FinalizeChipGPIORequest( &chip, &state );
    TEST_ASSERT_EQUAL_INT( EOK, result );
    TEST_ASSERT_NOT_NULL( chip.pLineRequest );
    TEST_ASSERT_TRUE( line.in_kernel_request );
}

/*==========================================================================*/
/*!
    Test FinalizeChipGPIORequest: no eligible lines -> EOK
============================================================================*/
void test_finalize_no_eligible_lines( void )
{
    GPIOChip chip;
    GPIO line;
    int result;

    reset_all_mocks();
    memset( &chip, 0, sizeof( chip ) );
    memset( &line, 0, sizeof( line ) );

    chip.name = "gpiochip0";
    chip.pChip = gpiod_chip_open( "/dev/gpiochip0" );
    chip.pFirstLine = &line;
    chip.pLastLine = &line;

    line.pParentChip = &chip;
    line.direction = GPIOD_LINE_DIRECTION_OUTPUT;
    line.edge_configured = false;

    /* gpiowatch mode only requests edge lines; this one is not */
    state.gpiowatch = true;

    result = FinalizeChipGPIORequest( &chip, &state );
    TEST_ASSERT_EQUAL_INT( EOK, result );
    TEST_ASSERT_NULL( chip.pLineRequest );
}

/*==========================================================================*/
/*!
    Test FinalizeChipGPIORequest: NULL args -> EINVAL
============================================================================*/
void test_finalize_null_args( void )
{
    int result;

    reset_all_mocks();

    result = FinalizeChipGPIORequest( NULL, &state );
    TEST_ASSERT_EQUAL_INT( EINVAL, result );

    result = FinalizeChipGPIORequest( NULL, NULL );
    TEST_ASSERT_EQUAL_INT( EINVAL, result );
}

/*============================================================================
        FindGPIO tests
============================================================================*/

/*==========================================================================*/
/*!
    Test FindGPIO: existing handle -> pointer returned
============================================================================*/
void test_find_gpio_found( void )
{
    GPIOChip chip;
    GPIO line;
    GPIO *found;

    reset_all_mocks();
    build_fixture( &chip, &line, 42, 4,
                   GPIOD_LINE_DIRECTION_OUTPUT );

    found = FindGPIO( &state, 42 );
    TEST_ASSERT_EQUAL_PTR( &line, found );
}

/*==========================================================================*/
/*!
    Test FindGPIO: non-existent handle -> NULL
============================================================================*/
void test_find_gpio_not_found( void )
{
    GPIOChip chip;
    GPIO line;
    GPIO *found;

    reset_all_mocks();
    build_fixture( &chip, &line, 42, 4,
                   GPIOD_LINE_DIRECTION_OUTPUT );

    found = FindGPIO( &state, 99 );
    TEST_ASSERT_NULL( found );
}

/*==========================================================================*/
/*!
    Test FindGPIO: NULL state -> NULL
============================================================================*/
void test_find_gpio_null_state( void )
{
    GPIO *found;

    reset_all_mocks();

    found = FindGPIO( NULL, 42 );
    TEST_ASSERT_NULL( found );
}

/*==========================================================================*/
/*!
    Test FindGPIO: VAR_INVALID handle -> NULL
============================================================================*/
void test_find_gpio_invalid_handle( void )
{
    GPIOChip chip;
    GPIO line;
    GPIO *found;

    reset_all_mocks();
    build_fixture( &chip, &line, 42, 4,
                   GPIOD_LINE_DIRECTION_OUTPUT );

    found = FindGPIO( &state, VAR_INVALID );
    TEST_ASSERT_NULL( found );
}

/*============================================================================
        FindVarByLine tests
============================================================================*/

/*==========================================================================*/
/*!
    Test FindVarByLine: matching chip + offset -> handle
============================================================================*/
void test_find_var_found( void )
{
    GPIOChip chip;
    GPIO line;
    VAR_HANDLE hVar;

    reset_all_mocks();
    build_fixture( &chip, &line, 77, 10,
                   GPIOD_LINE_DIRECTION_INPUT );

    hVar = FindVarByLine( &state, &chip, 10 );
    TEST_ASSERT_EQUAL_UINT32( 77, hVar );
}

/*==========================================================================*/
/*!
    Test FindVarByLine: wrong offset -> VAR_INVALID
============================================================================*/
void test_find_var_not_found( void )
{
    GPIOChip chip;
    GPIO line;
    VAR_HANDLE hVar;

    reset_all_mocks();
    build_fixture( &chip, &line, 77, 10,
                   GPIOD_LINE_DIRECTION_INPUT );

    hVar = FindVarByLine( &state, &chip, 99 );
    TEST_ASSERT_EQUAL_UINT32( VAR_INVALID, hVar );
}

/*==========================================================================*/
/*!
    Test FindVarByLine: wrong chip -> VAR_INVALID
============================================================================*/
void test_find_var_wrong_chip( void )
{
    GPIOChip chip;
    GPIOChip other_chip;
    GPIO line;
    VAR_HANDLE hVar;

    reset_all_mocks();
    build_fixture( &chip, &line, 77, 10,
                   GPIOD_LINE_DIRECTION_INPUT );
    memset( &other_chip, 0, sizeof( other_chip ) );

    hVar = FindVarByLine( &state, &other_chip, 10 );
    TEST_ASSERT_EQUAL_UINT32( VAR_INVALID, hVar );
}

/*==========================================================================*/
/*!
    Test FindVarByLine: NULL args -> VAR_INVALID
============================================================================*/
void test_find_var_null_args( void )
{
    GPIOChip chip;
    VAR_HANDLE hVar;

    reset_all_mocks();
    memset( &chip, 0, sizeof( chip ) );

    hVar = FindVarByLine( NULL, &chip, 0 );
    TEST_ASSERT_EQUAL_UINT32( VAR_INVALID, hVar );

    hVar = FindVarByLine( &state, NULL, 0 );
    TEST_ASSERT_EQUAL_UINT32( VAR_INVALID, hVar );
}

/*============================================================================
        UpdateOutput tests
============================================================================*/

/*==========================================================================*/
/*!
    Test UpdateOutput: valid output GPIO -> EOK
============================================================================*/
void test_update_output_valid( void )
{
    GPIOChip chip;
    GPIO line;
    int result;

    reset_all_mocks();
    build_fixture( &chip, &line, 50, 7,
                   GPIOD_LINE_DIRECTION_OUTPUT );

    mock_varserver.var_get_object.type = VARTYPE_UINT16;
    mock_varserver.var_get_object.val.ui = 1;
    mock_varserver.var_get_result = EOK;

    result = UpdateOutput( 50, &state );
    TEST_ASSERT_EQUAL_INT( EOK, result );
    TEST_ASSERT_EQUAL_INT( 1, line.value );
}

/*==========================================================================*/
/*!
    Test UpdateOutput: unknown handle -> ENOENT
============================================================================*/
void test_update_output_not_found( void )
{
    GPIOChip chip;
    GPIO line;
    int result;

    reset_all_mocks();
    build_fixture( &chip, &line, 50, 7,
                   GPIOD_LINE_DIRECTION_OUTPUT );

    result = UpdateOutput( 99, &state );
    TEST_ASSERT_EQUAL_INT( ENOENT, result );
}

/*==========================================================================*/
/*!
    Test UpdateOutput: input GPIO -> ENOTSUP
============================================================================*/
void test_update_output_wrong_direction( void )
{
    GPIOChip chip;
    GPIO line;
    int result;

    reset_all_mocks();
    build_fixture( &chip, &line, 50, 7,
                   GPIOD_LINE_DIRECTION_INPUT );

    mock_varserver.var_get_result = EOK;

    result = UpdateOutput( 50, &state );
    TEST_ASSERT_EQUAL_INT( ENOTSUP, result );
}

/*==========================================================================*/
/*!
    Test UpdateOutput: wrong variable type -> ENOTSUP
============================================================================*/
void test_update_output_wrong_type( void )
{
    GPIOChip chip;
    GPIO line;
    int result;

    reset_all_mocks();
    build_fixture( &chip, &line, 50, 7,
                   GPIOD_LINE_DIRECTION_OUTPUT );

    mock_varserver.var_get_object.type = VARTYPE_FLOAT;
    mock_varserver.var_get_result = EOK;

    result = UpdateOutput( 50, &state );
    TEST_ASSERT_EQUAL_INT( ENOTSUP, result );
}

/*==========================================================================*/
/*!
    Test UpdateOutput: NULL state -> EINVAL
============================================================================*/
void test_update_output_null_state( void )
{
    int result;

    reset_all_mocks();

    result = UpdateOutput( 50, NULL );
    TEST_ASSERT_EQUAL_INT( EINVAL, result );
}

/*============================================================================
        UpdateInput tests
============================================================================*/

/*==========================================================================*/
/*!
    Test UpdateInput: valid input GPIO -> EOK
============================================================================*/
void test_update_input_valid( void )
{
    GPIOChip chip;
    GPIO line;
    int result;

    reset_all_mocks();
    build_fixture( &chip, &line, 50, 7,
                   GPIOD_LINE_DIRECTION_INPUT );

    mock_gpiod.get_value_result = GPIOD_LINE_VALUE_ACTIVE;

    result = UpdateInput( 50, &state );
    TEST_ASSERT_EQUAL_INT( EOK, result );
}

/*==========================================================================*/
/*!
    Test UpdateInput: unknown handle -> ENOENT
============================================================================*/
void test_update_input_not_found( void )
{
    GPIOChip chip;
    GPIO line;
    int result;

    reset_all_mocks();
    build_fixture( &chip, &line, 50, 7,
                   GPIOD_LINE_DIRECTION_INPUT );

    result = UpdateInput( 99, &state );
    TEST_ASSERT_EQUAL_INT( ENOENT, result );
}

/*==========================================================================*/
/*!
    Test UpdateInput: output GPIO -> ENOTSUP
============================================================================*/
void test_update_input_wrong_direction( void )
{
    GPIOChip chip;
    GPIO line;
    int result;

    reset_all_mocks();
    build_fixture( &chip, &line, 50, 7,
                   GPIOD_LINE_DIRECTION_OUTPUT );

    result = UpdateInput( 50, &state );
    TEST_ASSERT_EQUAL_INT( ENOTSUP, result );
}

/*==========================================================================*/
/*!
    Test UpdateInput: get_value returns ERROR -> EIO
============================================================================*/
void test_update_input_read_error( void )
{
    GPIOChip chip;
    GPIO line;
    int result;

    reset_all_mocks();
    build_fixture( &chip, &line, 50, 7,
                   GPIOD_LINE_DIRECTION_INPUT );

    mock_gpiod.get_value_result = GPIOD_LINE_VALUE_ERROR;

    result = UpdateInput( 50, &state );
    TEST_ASSERT_EQUAL_INT( EIO, result );
}

/*============================================================================
        HandleGPIOEdgeEvent tests
============================================================================*/

/*==========================================================================*/
/*!
    Test HandleGPIOEdgeEvent: rising edge -> var set to 1
============================================================================*/
void test_edge_event_rising( void )
{
    GPIOChip chip;
    GPIO line;
    struct gpiod_edge_event *pEv;
    int result;

    reset_all_mocks();
    build_fixture( &chip, &line, 50, 7,
                   GPIOD_LINE_DIRECTION_INPUT );
    line.edge_configured = true;

    mock_gpiod.edge_event_type = GPIOD_EDGE_EVENT_RISING_EDGE;
    mock_gpiod.edge_event_line_offset = 7;

    pEv = gpiod_edge_event_buffer_get_event( NULL, 0 );
    result = HandleGPIOEdgeEvent( &state, &chip, pEv );
    TEST_ASSERT_EQUAL_INT( EOK, result );
}

/*==========================================================================*/
/*!
    Test HandleGPIOEdgeEvent: falling edge -> var set to 0
============================================================================*/
void test_edge_event_falling( void )
{
    GPIOChip chip;
    GPIO line;
    struct gpiod_edge_event *pEv;
    int result;

    reset_all_mocks();
    build_fixture( &chip, &line, 50, 7,
                   GPIOD_LINE_DIRECTION_INPUT );
    line.edge_configured = true;

    mock_gpiod.edge_event_type = GPIOD_EDGE_EVENT_FALLING_EDGE;
    mock_gpiod.edge_event_line_offset = 7;

    pEv = gpiod_edge_event_buffer_get_event( NULL, 0 );
    result = HandleGPIOEdgeEvent( &state, &chip, pEv );
    TEST_ASSERT_EQUAL_INT( EOK, result );
}

/*==========================================================================*/
/*!
    Test HandleGPIOEdgeEvent: unknown line offset -> ENOENT
============================================================================*/
void test_edge_event_unknown_line( void )
{
    GPIOChip chip;
    GPIO line;
    struct gpiod_edge_event *pEv;
    int result;

    reset_all_mocks();
    build_fixture( &chip, &line, 50, 7,
                   GPIOD_LINE_DIRECTION_INPUT );

    mock_gpiod.edge_event_line_offset = 99;

    pEv = gpiod_edge_event_buffer_get_event( NULL, 0 );
    result = HandleGPIOEdgeEvent( &state, &chip, pEv );
    TEST_ASSERT_EQUAL_INT( ENOENT, result );
}

/*============================================================================
        Valid parsing happy-path tests
============================================================================*/

/*==========================================================================*/
/*!
    Test ParseLineDirection: "input" -> EOK
============================================================================*/
void test_direction_input_valid( void )
{
    GPIO gpio;
    GPIOChip chip;
    JObject line_obj;
    int result;

    reset_all_mocks();
    memset( &gpio, 0, sizeof( gpio ) );
    memset( &chip, 0, sizeof( chip ) );
    memset( &line_obj, 0, sizeof( line_obj ) );
    line_obj.node.type = JSON_OBJECT;
    gpio.pParentChip = &chip;

    state.hVarServer = VARSERVER_Open();
    mock_tjson.getstr_returns = "input";

    result = ParseLineDirection(
        &gpio, (JNode *)&line_obj, &state );
    TEST_ASSERT_EQUAL_INT( EOK, result );
    TEST_ASSERT_EQUAL_INT(
        GPIOD_LINE_DIRECTION_INPUT, gpio.direction );
}

/*==========================================================================*/
/*!
    Test ParseLineDirection: "output" -> EOK
============================================================================*/
void test_direction_output_valid( void )
{
    GPIO gpio;
    GPIOChip chip;
    JObject line_obj;
    int result;

    reset_all_mocks();
    memset( &gpio, 0, sizeof( gpio ) );
    memset( &chip, 0, sizeof( chip ) );
    memset( &line_obj, 0, sizeof( line_obj ) );
    line_obj.node.type = JSON_OBJECT;
    gpio.pParentChip = &chip;
    gpio.direction = GPIOD_LINE_DIRECTION_OUTPUT;
    gpio.hVar = 1;

    state.hVarServer = VARSERVER_Open();
    mock_tjson.getstr_returns = "output";
    mock_varserver.var_get_object.type = VARTYPE_UINT16;
    mock_varserver.var_get_object.val.ui = 0;

    result = ParseLineDirection(
        &gpio, (JNode *)&line_obj, &state );
    TEST_ASSERT_EQUAL_INT( EOK, result );
    TEST_ASSERT_EQUAL_INT(
        GPIOD_LINE_DIRECTION_OUTPUT, gpio.direction );
}

/*==========================================================================*/
/*!
    Test ParseLineDirection: "pwm" -> EOK, PWM=true
============================================================================*/
void test_direction_pwm_valid( void )
{
    GPIO gpio;
    GPIOChip chip;
    JObject line_obj;
    int result;

    reset_all_mocks();
    memset( &gpio, 0, sizeof( gpio ) );
    memset( &chip, 0, sizeof( chip ) );
    memset( &line_obj, 0, sizeof( line_obj ) );
    line_obj.node.type = JSON_OBJECT;
    gpio.pParentChip = &chip;
    gpio.hVar = 1;
    gpio.direction = GPIOD_LINE_DIRECTION_OUTPUT;

    state.hVarServer = VARSERVER_Open();
    mock_tjson.getstr_returns = "pwm";
    mock_varserver.var_get_object.type = VARTYPE_UINT16;

    result = ParseLineDirection(
        &gpio, (JNode *)&line_obj, &state );
    TEST_ASSERT_EQUAL_INT( EOK, result );
    TEST_ASSERT_TRUE( gpio.PWM );
    TEST_ASSERT_EQUAL_INT(
        GPIOD_LINE_DIRECTION_OUTPUT, gpio.direction );
}

/*==========================================================================*/
/*!
    Test ParseLineEvent: "RISING_EDGE" -> EOK, edge set
============================================================================*/
void test_event_rising_valid( void )
{
    GPIO gpio;
    JObject line_obj;
    int result;

    reset_all_mocks();
    memset( &gpio, 0, sizeof( gpio ) );
    memset( &line_obj, 0, sizeof( line_obj ) );
    line_obj.node.type = JSON_OBJECT;

    mock_tjson.getstr_returns = "RISING_EDGE";

    result = ParseLineEvent( &gpio, (JNode *)&line_obj );
    TEST_ASSERT_EQUAL_INT( EOK, result );
    TEST_ASSERT_TRUE( gpio.edge_configured );
    TEST_ASSERT_EQUAL_INT(
        GPIOD_LINE_EDGE_RISING, gpio.line_edge );
}

/*==========================================================================*/
/*!
    Test ParseLineBias: "pull-down" -> EOK
============================================================================*/
void test_bias_pulldown_valid( void )
{
    GPIO gpio;
    JObject line_obj;
    int result;

    reset_all_mocks();
    memset( &gpio, 0, sizeof( gpio ) );
    memset( &line_obj, 0, sizeof( line_obj ) );
    line_obj.node.type = JSON_OBJECT;

    mock_tjson.getstr_returns = "pull-down";

    result = ParseLineBias( &gpio, (JNode *)&line_obj );
    TEST_ASSERT_EQUAL_INT( EOK, result );
    TEST_ASSERT_EQUAL_INT(
        GPIOD_LINE_BIAS_PULL_DOWN, gpio.line_bias );
}

/*==========================================================================*/
/*!
    Test ParseLineDrive: "open-drain" -> EOK
============================================================================*/
void test_drive_open_drain_valid( void )
{
    GPIO gpio;
    JObject line_obj;
    int result;

    reset_all_mocks();
    memset( &gpio, 0, sizeof( gpio ) );
    memset( &line_obj, 0, sizeof( line_obj ) );
    line_obj.node.type = JSON_OBJECT;

    mock_tjson.getstr_returns = "open-drain";

    result = ParseLineDrive( &gpio, (JNode *)&line_obj );
    TEST_ASSERT_EQUAL_INT( EOK, result );
    TEST_ASSERT_EQUAL_INT(
        GPIOD_LINE_DRIVE_OPEN_DRAIN, gpio.line_drive );
}

/*============================================================================
        Unity setUp / tearDown
============================================================================*/

void setUp( void )
{
    reset_all_mocks();
}

void tearDown( void )
{
    memset( &state, 0, sizeof( state ) );
}

/*============================================================================
        Test runner
============================================================================*/

int main( void )
{
    UNITY_BEGIN();

    /* GPIOShouldRequest */
    RUN_TEST( test_should_request_gpiowatch_edge_line );
    RUN_TEST( test_should_request_gpiowatch_non_edge );
    RUN_TEST( test_should_request_normal_non_edge );
    RUN_TEST( test_should_request_null_args );

    /* ChipParticipatesInEdgePoll */
    RUN_TEST( test_chip_poll_has_edge_lines );
    RUN_TEST( test_chip_poll_no_edge_lines );
    RUN_TEST( test_chip_poll_not_gpiowatch );
    RUN_TEST( test_chip_poll_null_request );

    /* FinalizeChipGPIORequest */
    RUN_TEST( test_finalize_with_output_line );
    RUN_TEST( test_finalize_no_eligible_lines );
    RUN_TEST( test_finalize_null_args );

    /* FindGPIO */
    RUN_TEST( test_find_gpio_found );
    RUN_TEST( test_find_gpio_not_found );
    RUN_TEST( test_find_gpio_null_state );
    RUN_TEST( test_find_gpio_invalid_handle );

    /* FindVarByLine */
    RUN_TEST( test_find_var_found );
    RUN_TEST( test_find_var_not_found );
    RUN_TEST( test_find_var_wrong_chip );
    RUN_TEST( test_find_var_null_args );

    /* UpdateOutput */
    RUN_TEST( test_update_output_valid );
    RUN_TEST( test_update_output_not_found );
    RUN_TEST( test_update_output_wrong_direction );
    RUN_TEST( test_update_output_wrong_type );
    RUN_TEST( test_update_output_null_state );

    /* UpdateInput */
    RUN_TEST( test_update_input_valid );
    RUN_TEST( test_update_input_not_found );
    RUN_TEST( test_update_input_wrong_direction );
    RUN_TEST( test_update_input_read_error );

    /* HandleGPIOEdgeEvent */
    RUN_TEST( test_edge_event_rising );
    RUN_TEST( test_edge_event_falling );
    RUN_TEST( test_edge_event_unknown_line );

    /* Valid parsing happy paths */
    RUN_TEST( test_direction_input_valid );
    RUN_TEST( test_direction_output_valid );
    RUN_TEST( test_direction_pwm_valid );
    RUN_TEST( test_event_rising_valid );
    RUN_TEST( test_bias_pulldown_valid );
    RUN_TEST( test_drive_open_drain_valid );

    return UNITY_END();
}
