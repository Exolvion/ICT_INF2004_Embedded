/** @file bench.h
 * @brief BENCH IMAGE ONLY: entry point used by firmware/bench/build.sh.
 * Fixed-width types only, so kernel and SDK translation units can include it.
 */
#ifndef BENCH_H
#define BENCH_H

/** Kernel side: run the bench measurements; never returns. */
void bench_loop (void) __attribute__((noreturn));

#endif /* BENCH_H */

/* End of file. */
