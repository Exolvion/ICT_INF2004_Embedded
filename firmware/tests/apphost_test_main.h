/** @file apphost_test_main.h
 * @brief Application integration fixture; it never compiles board.c or RTOS.
 * Test doubles use the real boundary ABI. Their names are externally fixed.
 */
#ifndef APPHOST_TEST_MAIN_H
#define APPHOST_TEST_MAIN_H

int main (int argument_count, char * p_arguments[]);

#endif /* APPHOST_TEST_MAIN_H */

/* End of file. */

