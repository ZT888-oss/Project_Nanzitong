/* parallel/optimized life.c
 *
 * Replace the game_of_life(...) function in the parallel life.c with this.
 *
 * This implementation preserves exact semantics of the reference
 * sequential implementation (wrap-around torus).
 */

#include <omp.h>    /* requires -fopenmp */
#include <stdint.h> /* for intptr_t if needed */

#include "life.h"
#include "util.h"

char* game_of_life(char* outboard, char* inboard, const int nrows,
                   const int ncols, const int num_generations) {
  /* local alias with restrict to help optimizer */
  char* restrict cur = inboard;
  char* restrict next = outboard;

  /* LDA = leading dimension for column-major layout used in the project:
     indexing: board[row + LDA*col]  (same as BOARD macro in sequential) */
  const int LDA = nrows;

  for (int gen = 0; gen < num_generations; gen++) {
/* Parallelize by rows (geometric decomposition).
 * Each thread writes distinct rows of `next` and only reads from `cur`.
 * We use schedule(static) for low overhead; chunk size default is fine.
 */
#pragma omp parallel for schedule(static)
    for (int i = 0; i < nrows; i++) {
      /* compute north/south (wrap-around) once per row */
      const int inorth = (i == 0) ? (nrows - 1) : (i - 1);
      const int isouth = (i == nrows - 1) ? 0 : (i + 1);

      /* for speed, cache LDA locally */
      const int lda = LDA;

      /* iterate columns */
      for (int j = 0; j < ncols; j++) {
        /* compute west/east with branch wraparound (cheaper than mod) */
        const int jwest = (j == 0) ? (ncols - 1) : (j - 1);
        const int jeast = (j == ncols - 1) ? 0 : (j + 1);

        /* neighbor count: use indexing row + lda * col */
        const unsigned char nc =
            (unsigned char)((unsigned char)cur[inorth + lda * jwest] +
                            (unsigned char)cur[inorth + lda * j] +
                            (unsigned char)cur[inorth + lda * jeast] +
                            (unsigned char)cur[i + lda * jwest] +
                            (unsigned char)cur[i + lda * jeast] +
                            (unsigned char)cur[isouth + lda * jwest] +
                            (unsigned char)cur[isouth + lda * j] +
                            (unsigned char)cur[isouth + lda * jeast]);

        next[i + lda * j] = alivep(nc, cur[i + lda * j]);
      }
    } /* end parallel for */

    /* swap boards for next generation */
    char* tmp = cur;
    cur = next;
    next = tmp;
  }

  /* cur now points to the buffer containing final state.
     Return pointer consistent with original API contract. */
  return cur;
}
