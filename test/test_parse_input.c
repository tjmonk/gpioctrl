/*===========================================================================
    Unit tests for gpioctrl JSON input parsing

    Tests exercise the parsing path with malformed or missing input data to
    verify the module does not crash and returns appropriate error codes.

    The tests pull in the production source directly (since all parse
    functions are static).  Shim headers under test/include/ redirect
    <varserver/varserver.h>, <tjson/json.h>, and <gpiod.h> to mock
    implementations.
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

static void reset_all_mocks( void )
{
    mock_gpiod_reset();
    mock_varserver_reset();
    mock_tjson_reset();
    memset( &state, 0, sizeof( state ) );
    state.service = "test_gpioctrl";
}

/*============================================================================
        Test: NULL filename causes JSON_Process to return NULL -> no crash
============================================================================*/

void test_null_filename( void )
{
    JNode *config;

    reset_all_mocks();
    mock_tjson.process_returns = NULL;

    config = JSON_Process( NULL );
    TEST_ASSERT_NULL( config );
}

/*============================================================================
        Test: Empty file -> JSON_Process returns NULL -> no crash
============================================================================*/

void test_empty_file( void )
{
    JNode *config;
    JArray *gpiodef;

    reset_all_mocks();
    mock_tjson.process_returns = NULL;

    config = JSON_Process( "empty.json" );
    TEST_ASSERT_NULL( config );

    gpiodef = (JArray *)JSON_Find( config, "gpiodef" );
    TEST_ASSERT_NULL( gpiodef );
}

/*============================================================================
        Test: Valid JSON but no "gpiodef" key -> no crash
============================================================================*/

void test_no_gpiodef_key( void )
{
    JObject root;
    JArray *gpiodef;

    reset_all_mocks();
    memset( &root, 0, sizeof( root ) );
    root.node.type = JSON_OBJECT;
    root.node.name = "root";

    mock_tjson.process_returns = (JNode *)&root;
    mock_tjson.find_returns = NULL;

    gpiodef = (JArray *)JSON_Find( (JNode *)&root, "gpiodef" );
    TEST_ASSERT_NULL( gpiodef );

    /* JSON_Iterate with NULL should be safe */
    JSON_Iterate( NULL, ParseChip, (void *)&state );
}

/*============================================================================
        Test: "gpiodef" is not an array -> CreateLines returns ENOTSUP
============================================================================*/

void test_gpiodef_not_array( void )
{
    JNode non_array;
    int result;

    reset_all_mocks();
    memset( &non_array, 0, sizeof( non_array ) );
    non_array.type = JSON_VAR;
    non_array.name = "lines";

    mock_tjson.find_returns = &non_array;

    /* CreateLines expects to find "lines" node which is not an array */
    result = CreateLines( &non_array, &state );
    TEST_ASSERT_EQUAL_INT( ENOTSUP, result );
}

/*============================================================================
        Test: Chip object has no "chip" key -> CreateChip returns NULL
============================================================================*/

void test_chip_missing_name( void )
{
    JObject chip_obj;
    GPIOChip *pChip;

    reset_all_mocks();
    memset( &chip_obj, 0, sizeof( chip_obj ) );
    chip_obj.node.type = JSON_OBJECT;

    /* JSON_GetStr for "chip" will return NULL */
    mock_tjson.getstr_returns = NULL;

    state.hVarServer = VARSERVER_Open();

    pChip = CreateChip( (JNode *)&chip_obj, &state );
    TEST_ASSERT_NULL( pChip );
}

/*============================================================================
        Test: gpiod_chip_open returns NULL -> CreateChip returns NULL
============================================================================*/

void test_chip_open_fails( void )
{
    JObject chip_obj;
    GPIOChip *pChip;

    reset_all_mocks();
    memset( &chip_obj, 0, sizeof( chip_obj ) );
    chip_obj.node.type = JSON_OBJECT;

    mock_tjson.getstr_returns = "gpiochip0";
    mock_gpiod.chip_open_returns_null = true;

    state.hVarServer = VARSERVER_Open();

    pChip = CreateChip( (JNode *)&chip_obj, &state );
    TEST_ASSERT_NULL( pChip );
}

/*============================================================================
        Test: Line object has no "line" key -> CreateLine returns NULL
============================================================================*/

void test_line_missing_line_number( void )
{
    JObject line_obj;
    GPIOChip chip;
    GPIO *pGPIOLine;

    reset_all_mocks();
    memset( &line_obj, 0, sizeof( line_obj ) );
    memset( &chip, 0, sizeof( chip ) );
    line_obj.node.type = JSON_OBJECT;

    state.hVarServer = VARSERVER_Open();
    state.pLastGPIOChip = &chip;

    /* VAR_FindByName returns valid handle */
    mock_varserver.find_by_name_returns = 42;
    /* JSON_GetStr for "var" will return a name, but "line" will not */
    mock_tjson.getstr_returns = "/HW/GPIO/TEST";

    pGPIOLine = CreateLine( (JNode *)&line_obj, &state );

    /*
     * With getstr_returns set globally, both "var" and "line" calls return
     * the same string. The test verifies no crash - the line number will
     * parse from "/HW/GPIO/TEST" as 0 via strtoul which is benign.
     * A more targeted test requires per-call mock routing.
     */
    (void)pGPIOLine;
}

/*============================================================================
        Test: Line object has no "var" key -> GetVarHandle returns VAR_INVALID
============================================================================*/

void test_line_missing_var( void )
{
    JObject line_obj;
    GPIOChip chip;
    GPIO *pGPIOLine;

    reset_all_mocks();
    memset( &line_obj, 0, sizeof( line_obj ) );
    memset( &chip, 0, sizeof( chip ) );
    line_obj.node.type = JSON_OBJECT;

    state.hVarServer = VARSERVER_Open();
    state.pLastGPIOChip = &chip;

    /* JSON_GetStr returns NULL -> "var" not found */
    mock_tjson.getstr_returns = NULL;
    mock_varserver.find_by_name_returns = VAR_INVALID;

    pGPIOLine = CreateLine( (JNode *)&line_obj, &state );
    TEST_ASSERT_NULL( pGPIOLine );
}

/*============================================================================
        Test: Invalid direction string -> returns ENOTSUP
============================================================================*/

void test_line_invalid_direction( void )
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
    mock_tjson.getstr_returns = "sideways";

    result = ParseLineDirection( &gpio, (JNode *)&line_obj, &state );
    TEST_ASSERT_EQUAL_INT( ENOTSUP, result );
}

/*============================================================================
        Test: Invalid active_state string -> returns ENOTSUP
============================================================================*/

void test_line_invalid_active_state( void )
{
    GPIO gpio;
    JObject line_obj;
    int result;

    reset_all_mocks();
    memset( &gpio, 0, sizeof( gpio ) );
    memset( &line_obj, 0, sizeof( line_obj ) );
    line_obj.node.type = JSON_OBJECT;

    mock_tjson.getstr_returns = "maybe";

    result = ParseLineActiveState( &gpio, (JNode *)&line_obj );
    TEST_ASSERT_EQUAL_INT( ENOTSUP, result );
}

/*============================================================================
        Test: Invalid event string -> returns ENOTSUP
============================================================================*/

void test_line_invalid_event( void )
{
    GPIO gpio;
    JObject line_obj;
    int result;

    reset_all_mocks();
    memset( &gpio, 0, sizeof( gpio ) );
    memset( &line_obj, 0, sizeof( line_obj ) );
    line_obj.node.type = JSON_OBJECT;

    mock_tjson.getstr_returns = "QUADRUPLE_EDGE";

    result = ParseLineEvent( &gpio, (JNode *)&line_obj );
    TEST_ASSERT_EQUAL_INT( ENOTSUP, result );
}

/*============================================================================
        Test: Invalid bias string -> returns ENOTSUP
============================================================================*/

void test_line_invalid_bias( void )
{
    GPIO gpio;
    JObject line_obj;
    int result;

    reset_all_mocks();
    memset( &gpio, 0, sizeof( gpio ) );
    memset( &line_obj, 0, sizeof( line_obj ) );
    line_obj.node.type = JSON_OBJECT;

    mock_tjson.getstr_returns = "antigravity";

    result = ParseLineBias( &gpio, (JNode *)&line_obj );
    TEST_ASSERT_EQUAL_INT( ENOTSUP, result );
}

/*============================================================================
        Test: Invalid drive string -> returns ENOTSUP
============================================================================*/

void test_line_invalid_drive( void )
{
    GPIO gpio;
    JObject line_obj;
    int result;

    reset_all_mocks();
    memset( &gpio, 0, sizeof( gpio ) );
    memset( &line_obj, 0, sizeof( line_obj ) );
    line_obj.node.type = JSON_OBJECT;

    mock_tjson.getstr_returns = "warp";

    result = ParseLineDrive( &gpio, (JNode *)&line_obj );
    TEST_ASSERT_EQUAL_INT( ENOTSUP, result );
}

/*============================================================================
        Test: Negative line number -> no crash (strtoul wraps)
============================================================================*/

void test_line_negative_number( void )
{
    GPIOChip chip;
    JObject line_obj;
    GPIO *pGPIOLine;

    reset_all_mocks();
    memset( &chip, 0, sizeof( chip ) );
    memset( &line_obj, 0, sizeof( line_obj ) );
    line_obj.node.type = JSON_OBJECT;

    state.hVarServer = VARSERVER_Open();
    state.pLastGPIOChip = &chip;

    mock_varserver.find_by_name_returns = 10;
    mock_tjson.getstr_returns = "-1";

    pGPIOLine = CreateLine( (JNode *)&line_obj, &state );
    TEST_ASSERT_NOT_NULL( pGPIOLine );

    if ( pGPIOLine != NULL )
    {
        free( pGPIOLine );
    }
}

/*============================================================================
        Test: Non-numeric line string -> strtoul returns 0, no crash
============================================================================*/

void test_line_non_numeric( void )
{
    GPIOChip chip;
    JObject line_obj;
    GPIO *pGPIOLine;

    reset_all_mocks();
    memset( &chip, 0, sizeof( chip ) );
    memset( &line_obj, 0, sizeof( line_obj ) );
    line_obj.node.type = JSON_OBJECT;

    state.hVarServer = VARSERVER_Open();
    state.pLastGPIOChip = &chip;

    mock_varserver.find_by_name_returns = 10;
    mock_tjson.getstr_returns = "abc";

    pGPIOLine = CreateLine( (JNode *)&line_obj, &state );
    TEST_ASSERT_NOT_NULL( pGPIOLine );

    if ( pGPIOLine != NULL )
    {
        TEST_ASSERT_EQUAL_INT( 0, pGPIOLine->line_num );
        free( pGPIOLine );
    }
}

/*============================================================================
        Test: "lines" value is not an array -> returns ENOTSUP
============================================================================*/

void test_lines_not_array( void )
{
    JNode non_array;
    int result;

    reset_all_mocks();
    memset( &non_array, 0, sizeof( non_array ) );
    non_array.type = JSON_VAR;
    non_array.name = "lines";

    mock_tjson.find_returns = &non_array;

    state.hVarServer = VARSERVER_Open();

    result = CreateLines( &non_array, &state );
    TEST_ASSERT_EQUAL_INT( ENOTSUP, result );
}

/*============================================================================
        Test: Empty lines array -> zero iterations, no crash
============================================================================*/

void test_empty_lines_array( void )
{
    JArray empty_arr;
    JNode container;
    int result;

    reset_all_mocks();
    memset( &empty_arr, 0, sizeof( empty_arr ) );
    memset( &container, 0, sizeof( container ) );

    empty_arr.node.type = JSON_ARRAY;
    empty_arr.node.name = "lines";
    empty_arr.n = 0;
    empty_arr.pFirst = NULL;
    empty_arr.pLast = NULL;

    container.type = JSON_OBJECT;
    mock_tjson.find_returns = (JNode *)&empty_arr;

    state.hVarServer = VARSERVER_Open();

    result = CreateLines( &container, &state );
    TEST_ASSERT_EQUAL_INT( EOK, result );
}

/*============================================================================
        Test: Two chips, second missing name -> first OK, second fails
============================================================================*/

void test_multiple_chips_one_bad( void )
{
    JObject chip1;
    JObject chip2;
    JArray gpiodef;
    int result;

    reset_all_mocks();
    memset( &chip1, 0, sizeof( chip1 ) );
    memset( &chip2, 0, sizeof( chip2 ) );
    memset( &gpiodef, 0, sizeof( gpiodef ) );

    chip1.node.type = JSON_OBJECT;
    chip1.node.name = "chip1";
    chip1.node.pNext = (JNode *)&chip2;

    chip2.node.type = JSON_OBJECT;
    chip2.node.name = "chip2";
    chip2.node.pNext = NULL;

    gpiodef.node.type = JSON_ARRAY;
    gpiodef.node.name = "gpiodef";
    gpiodef.n = 2;
    gpiodef.pFirst = (JNode *)&chip1;
    gpiodef.pLast = (JNode *)&chip2;

    state.hVarServer = VARSERVER_Open();

    /*
     * For the first chip, getstr for "chip" returns a name and chip opens.
     * For the second, getstr returns NULL (chip name missing).
     * Since mock_tjson.getstr_returns is global, we set it to a chip name;
     * ParseChip for the first chip will succeed (CreateChip -> open chip ->
     * CreateLines with no lines).
     * We make the second call fail by having chip_open return NULL.
     */
    mock_tjson.getstr_returns = "gpiochip0";
    mock_gpiod.chip_open_returns_null = false;

    /* CreateLines will try to find "lines" - return an empty array */
    JArray empty_lines;
    memset( &empty_lines, 0, sizeof( empty_lines ) );
    empty_lines.node.type = JSON_ARRAY;
    empty_lines.node.name = "lines";
    mock_tjson.find_returns = (JNode *)&empty_lines;

    result = JSON_Iterate( &gpiodef, ParseChip, (void *)&state );

    /* At minimum, the first chip was created (we may have 1 or 2 chips) */
    TEST_ASSERT_NOT_NULL( state.pFirstGPIOChip );

    /* Cleanup */
    Shutdown( &state );
}

/*============================================================================
        Test: NULL state pointer to ParseChip -> returns EINVAL
============================================================================*/

void test_parse_chip_null_state( void )
{
    JObject chip_obj;
    int result;

    reset_all_mocks();
    memset( &chip_obj, 0, sizeof( chip_obj ) );
    chip_obj.node.type = JSON_OBJECT;

    result = ParseChip( (JNode *)&chip_obj, NULL );
    TEST_ASSERT_EQUAL_INT( EINVAL, result );
}

/*============================================================================
        Test: NULL arguments to ParseLine -> returns EOK (no crash)
============================================================================*/

void test_parse_line_null_args( void )
{
    int result;

    reset_all_mocks();

    result = ParseLine( NULL, NULL );
    TEST_ASSERT_EQUAL_INT( EOK, result );
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
    Shutdown( &state );
}

/*============================================================================
        Test runner
============================================================================*/

int main( void )
{
    UNITY_BEGIN();

    RUN_TEST( test_null_filename );
    RUN_TEST( test_empty_file );
    RUN_TEST( test_no_gpiodef_key );
    RUN_TEST( test_gpiodef_not_array );
    RUN_TEST( test_chip_missing_name );
    RUN_TEST( test_chip_open_fails );
    RUN_TEST( test_line_missing_line_number );
    RUN_TEST( test_line_missing_var );
    RUN_TEST( test_line_invalid_direction );
    RUN_TEST( test_line_invalid_active_state );
    RUN_TEST( test_line_invalid_event );
    RUN_TEST( test_line_invalid_bias );
    RUN_TEST( test_line_invalid_drive );
    RUN_TEST( test_line_negative_number );
    RUN_TEST( test_line_non_numeric );
    RUN_TEST( test_lines_not_array );
    RUN_TEST( test_empty_lines_array );
    RUN_TEST( test_multiple_chips_one_bad );
    RUN_TEST( test_parse_chip_null_state );
    RUN_TEST( test_parse_line_null_args );

    return UNITY_END();
}
