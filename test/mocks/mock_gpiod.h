/*===========================================================================
    Mock libgpiod v2 header for unit testing gpioctrl
===========================================================================*/
#ifndef MOCK_GPIOD_H
#define MOCK_GPIOD_H

#include <stdbool.h>
#include <stddef.h>

/*============================================================================
        Enumerations
============================================================================*/

enum gpiod_line_direction
{
    GPIOD_LINE_DIRECTION_AS_IS = 1,
    GPIOD_LINE_DIRECTION_INPUT = 2,
    GPIOD_LINE_DIRECTION_OUTPUT = 3
};

enum gpiod_line_edge
{
    GPIOD_LINE_EDGE_NONE = 1,
    GPIOD_LINE_EDGE_RISING = 2,
    GPIOD_LINE_EDGE_FALLING = 3,
    GPIOD_LINE_EDGE_BOTH = 4
};

enum gpiod_line_bias
{
    GPIOD_LINE_BIAS_AS_IS = 1,
    GPIOD_LINE_BIAS_UNKNOWN = 2,
    GPIOD_LINE_BIAS_DISABLED = 3,
    GPIOD_LINE_BIAS_PULL_UP = 4,
    GPIOD_LINE_BIAS_PULL_DOWN = 5
};

enum gpiod_line_drive
{
    GPIOD_LINE_DRIVE_PUSH_PULL = 1,
    GPIOD_LINE_DRIVE_OPEN_DRAIN = 2,
    GPIOD_LINE_DRIVE_OPEN_SOURCE = 3
};

enum gpiod_line_value
{
    GPIOD_LINE_VALUE_ERROR = -1,
    GPIOD_LINE_VALUE_INACTIVE = 0,
    GPIOD_LINE_VALUE_ACTIVE = 1
};

enum gpiod_edge_event_type
{
    GPIOD_EDGE_EVENT_RISING_EDGE = 1,
    GPIOD_EDGE_EVENT_FALLING_EDGE = 2
};

/*============================================================================
        Opaque types
============================================================================*/

struct gpiod_chip;
struct gpiod_line_request;
struct gpiod_line_settings;
struct gpiod_line_config;
struct gpiod_request_config;
struct gpiod_edge_event_buffer;
struct gpiod_edge_event;
struct gpiod_line_info;

/*============================================================================
        Mock control
============================================================================*/

typedef struct
{
    bool chip_open_returns_null;
    int  line_request_get_fd_value;
    int  line_request_read_edge_events_value;
    enum gpiod_line_value get_value_result;
    int  set_value_result;
    enum gpiod_edge_event_type edge_event_type;
    unsigned int edge_event_line_offset;
} MockGpiodState;

extern MockGpiodState mock_gpiod;

void mock_gpiod_reset( void );

/*============================================================================
        Stubbed libgpiod v2 API
============================================================================*/

struct gpiod_chip *gpiod_chip_open( const char *path );
void gpiod_chip_close( struct gpiod_chip *chip );

struct gpiod_line_config *gpiod_line_config_new( void );
void gpiod_line_config_free( struct gpiod_line_config *config );
int gpiod_line_config_add_line_settings( struct gpiod_line_config *config,
                                         const unsigned int *offsets,
                                         size_t num_offsets,
                                         struct gpiod_line_settings *settings );

struct gpiod_request_config *gpiod_request_config_new( void );
void gpiod_request_config_free( struct gpiod_request_config *config );
void gpiod_request_config_set_consumer( struct gpiod_request_config *config,
                                        const char *consumer );

struct gpiod_line_settings *gpiod_line_settings_new( void );
void gpiod_line_settings_free( struct gpiod_line_settings *settings );
void gpiod_line_settings_set_direction( struct gpiod_line_settings *settings,
                                        enum gpiod_line_direction direction );
int gpiod_line_settings_set_output_value( struct gpiod_line_settings *settings,
                                          enum gpiod_line_value value );
void gpiod_line_settings_set_active_low( struct gpiod_line_settings *settings,
                                         bool active_low );
int gpiod_line_settings_set_bias( struct gpiod_line_settings *settings,
                                  enum gpiod_line_bias bias );
int gpiod_line_settings_set_drive( struct gpiod_line_settings *settings,
                                   enum gpiod_line_drive drive );
int gpiod_line_settings_set_edge_detection(
    struct gpiod_line_settings *settings,
    enum gpiod_line_edge edge );

struct gpiod_line_request *gpiod_chip_request_lines(
    struct gpiod_chip *chip,
    struct gpiod_request_config *req_cfg,
    struct gpiod_line_config *line_cfg );

void gpiod_line_request_release( struct gpiod_line_request *request );
int gpiod_line_request_get_fd( struct gpiod_line_request *request );
enum gpiod_line_value gpiod_line_request_get_value(
    struct gpiod_line_request *request,
    unsigned int offset );
int gpiod_line_request_set_value( struct gpiod_line_request *request,
                                  unsigned int offset,
                                  enum gpiod_line_value value );
int gpiod_line_request_read_edge_events( struct gpiod_line_request *request,
                                         struct gpiod_edge_event_buffer *buffer,
                                         size_t max_events );
int gpiod_line_request_wait_edge_events( struct gpiod_line_request *request,
                                         long timeout_ns );

struct gpiod_edge_event_buffer *gpiod_edge_event_buffer_new( size_t capacity );
void gpiod_edge_event_buffer_free( struct gpiod_edge_event_buffer *buffer );
size_t gpiod_edge_event_buffer_get_capacity(
    struct gpiod_edge_event_buffer *buffer );
size_t gpiod_edge_event_buffer_get_num_events(
    struct gpiod_edge_event_buffer *buffer );
struct gpiod_edge_event *gpiod_edge_event_buffer_get_event(
    struct gpiod_edge_event_buffer *buffer,
    unsigned long index );
enum gpiod_edge_event_type gpiod_edge_event_get_event_type(
    struct gpiod_edge_event *event );
unsigned int gpiod_edge_event_get_line_offset(
    struct gpiod_edge_event *event );

struct gpiod_line_info *gpiod_chip_get_line_info( struct gpiod_chip *chip,
                                                  unsigned int offset );
const char *gpiod_line_info_get_name( struct gpiod_line_info *info );
void gpiod_line_info_free( struct gpiod_line_info *info );

#endif /* MOCK_GPIOD_H */
