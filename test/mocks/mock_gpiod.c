/*===========================================================================
    Mock libgpiod v2 implementation for unit testing gpioctrl
===========================================================================*/
#include "mock_gpiod.h"
#include <stdlib.h>
#include <string.h>

/*============================================================================
        Concrete opaque types (test-visible only)
============================================================================*/

struct gpiod_chip            { int dummy; };
struct gpiod_line_request    { int dummy; };
struct gpiod_line_settings   { int dummy; };
struct gpiod_line_config     { int dummy; };
struct gpiod_request_config  { int dummy; };
struct gpiod_edge_event_buffer { size_t capacity; };
struct gpiod_edge_event      { int dummy; };
struct gpiod_line_info       { int dummy; };

/*============================================================================
        Mock state
============================================================================*/

MockGpiodState mock_gpiod;

static struct gpiod_chip            fake_chip;
static struct gpiod_line_request    fake_line_request;
static struct gpiod_line_settings   fake_settings;
static struct gpiod_line_config     fake_line_config;
static struct gpiod_request_config  fake_request_config;
static struct gpiod_edge_event_buffer fake_event_buffer;
static struct gpiod_edge_event      fake_edge_event;
static struct gpiod_line_info       fake_line_info;

void mock_gpiod_reset( void )
{
    memset( &mock_gpiod, 0, sizeof( mock_gpiod ) );
    mock_gpiod.get_value_result = GPIOD_LINE_VALUE_INACTIVE;
    mock_gpiod.line_request_get_fd_value = 3;
    mock_gpiod.line_request_read_edge_events_value = 0;
    mock_gpiod.edge_event_type = GPIOD_EDGE_EVENT_RISING_EDGE;
    mock_gpiod.edge_event_line_offset = 0;
    fake_event_buffer.capacity = 32;
}

/*============================================================================
        Stub implementations
============================================================================*/

struct gpiod_chip *gpiod_chip_open( const char *path )
{
    (void)path;
    if ( mock_gpiod.chip_open_returns_null )
    {
        return NULL;
    }
    return &fake_chip;
}

void gpiod_chip_close( struct gpiod_chip *chip )
{
    (void)chip;
}

struct gpiod_line_config *gpiod_line_config_new( void )
{
    return &fake_line_config;
}

void gpiod_line_config_free( struct gpiod_line_config *config )
{
    (void)config;
}

int gpiod_line_config_add_line_settings( struct gpiod_line_config *config,
                                         const unsigned int *offsets,
                                         size_t num_offsets,
                                         struct gpiod_line_settings *settings )
{
    (void)config;
    (void)offsets;
    (void)num_offsets;
    (void)settings;
    return 0;
}

struct gpiod_request_config *gpiod_request_config_new( void )
{
    return &fake_request_config;
}

void gpiod_request_config_free( struct gpiod_request_config *config )
{
    (void)config;
}

void gpiod_request_config_set_consumer( struct gpiod_request_config *config,
                                        const char *consumer )
{
    (void)config;
    (void)consumer;
}

struct gpiod_line_settings *gpiod_line_settings_new( void )
{
    return &fake_settings;
}

void gpiod_line_settings_free( struct gpiod_line_settings *settings )
{
    (void)settings;
}

void gpiod_line_settings_set_direction( struct gpiod_line_settings *settings,
                                        enum gpiod_line_direction direction )
{
    (void)settings;
    (void)direction;
}

int gpiod_line_settings_set_output_value( struct gpiod_line_settings *settings,
                                          enum gpiod_line_value value )
{
    (void)settings;
    (void)value;
    return 0;
}

void gpiod_line_settings_set_active_low( struct gpiod_line_settings *settings,
                                         bool active_low )
{
    (void)settings;
    (void)active_low;
}

int gpiod_line_settings_set_bias( struct gpiod_line_settings *settings,
                                  enum gpiod_line_bias bias )
{
    (void)settings;
    (void)bias;
    return 0;
}

int gpiod_line_settings_set_drive( struct gpiod_line_settings *settings,
                                   enum gpiod_line_drive drive )
{
    (void)settings;
    (void)drive;
    return 0;
}

int gpiod_line_settings_set_edge_detection(
    struct gpiod_line_settings *settings,
    enum gpiod_line_edge edge )
{
    (void)settings;
    (void)edge;
    return 0;
}

struct gpiod_line_request *gpiod_chip_request_lines(
    struct gpiod_chip *chip,
    struct gpiod_request_config *req_cfg,
    struct gpiod_line_config *line_cfg )
{
    (void)chip;
    (void)req_cfg;
    (void)line_cfg;
    return &fake_line_request;
}

void gpiod_line_request_release( struct gpiod_line_request *request )
{
    (void)request;
}

int gpiod_line_request_get_fd( struct gpiod_line_request *request )
{
    (void)request;
    return mock_gpiod.line_request_get_fd_value;
}

enum gpiod_line_value gpiod_line_request_get_value(
    struct gpiod_line_request *request,
    unsigned int offset )
{
    (void)request;
    (void)offset;
    return mock_gpiod.get_value_result;
}

int gpiod_line_request_set_value( struct gpiod_line_request *request,
                                  unsigned int offset,
                                  enum gpiod_line_value value )
{
    (void)request;
    (void)offset;
    (void)value;
    return mock_gpiod.set_value_result;
}

int gpiod_line_request_read_edge_events( struct gpiod_line_request *request,
                                         struct gpiod_edge_event_buffer *buffer,
                                         size_t max_events )
{
    (void)request;
    (void)buffer;
    (void)max_events;
    return mock_gpiod.line_request_read_edge_events_value;
}

int gpiod_line_request_wait_edge_events( struct gpiod_line_request *request,
                                         long timeout_ns )
{
    (void)request;
    (void)timeout_ns;
    return 0;
}

struct gpiod_edge_event_buffer *gpiod_edge_event_buffer_new( size_t capacity )
{
    fake_event_buffer.capacity = capacity;
    return &fake_event_buffer;
}

void gpiod_edge_event_buffer_free( struct gpiod_edge_event_buffer *buffer )
{
    (void)buffer;
}

size_t gpiod_edge_event_buffer_get_capacity(
    struct gpiod_edge_event_buffer *buffer )
{
    if ( buffer != NULL )
    {
        return buffer->capacity;
    }
    return 0;
}

size_t gpiod_edge_event_buffer_get_num_events(
    struct gpiod_edge_event_buffer *buffer )
{
    (void)buffer;
    return (size_t)mock_gpiod.line_request_read_edge_events_value;
}

struct gpiod_edge_event *gpiod_edge_event_buffer_get_event(
    struct gpiod_edge_event_buffer *buffer,
    unsigned long index )
{
    (void)buffer;
    (void)index;
    return &fake_edge_event;
}

enum gpiod_edge_event_type gpiod_edge_event_get_event_type(
    struct gpiod_edge_event *event )
{
    (void)event;
    return mock_gpiod.edge_event_type;
}

unsigned int gpiod_edge_event_get_line_offset(
    struct gpiod_edge_event *event )
{
    (void)event;
    return mock_gpiod.edge_event_line_offset;
}

struct gpiod_line_info *gpiod_chip_get_line_info( struct gpiod_chip *chip,
                                                  unsigned int offset )
{
    (void)chip;
    (void)offset;
    return &fake_line_info;
}

const char *gpiod_line_info_get_name( struct gpiod_line_info *info )
{
    (void)info;
    return "mock_line";
}

void gpiod_line_info_free( struct gpiod_line_info *info )
{
    (void)info;
}
