#ifndef _GPIO_H
#define _GPIO_H


#if 0
typedef enum {
  e_gpio_input,
  e_gpio_output,
} gpio_direction_e;

typedef enum {
  e_gpio_edge_none,
  e_gpio_edge_riding,
  e_gpio_edge_falling,
  e_gpio_edge_both,
  e_gpio_num_edges
} gpio_edge_e;


extern int gpio_export( unsigned int gpio);
extern int gpio_unexport( unsigned int gpio);
extern int gpio_set_direction( unsigned int gpio, const char* out_flag);
extern int gpio_set_value( unsigned int gpio, unsigned int value);
extern int gpio_get_value( unsigned int gpio, unsigned int *value);
extern int gpio_set_edge( unsigned int gpio, const char* edge);

#else

extern int gpio_write_value_to_pin_file( unsigned int gpio, const char* file, const char* value);
extern int gpio_write_value_to_file( const char* file, const char* value);
extern int gpio_write_int_value_to_file( const char* file, int value);
extern int gpio_set_pin( unsigned int gpio, const char* file, const char* value);
extern int gpio_open_file( unsigned int gpio, const char* file);

#endif


#endif
