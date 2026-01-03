/*
 * ECE454 Lab 3 - Malloc (Optimized: Segregated Free Lists)
 */

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>

#include "mm.h"
#include "memlib.h"

/*********************************************************
 * Team information
 ********************************************************/
team_t team = {
    "Nanzitong Zhang",
    "John:Doe",
    "1010390989",
    "",
    ""
};

/*************************************************************************
 * Basic Constants and Macros
 *************************************************************************/
#define ALIGNMENT 16
#define ALIGN(size) (((size) + (ALIGNMENT-1)) & ~0xF)

#define WSIZE       8               /* Word size (bytes) */
#define DSIZE       16              /* Doubleword size (bytes) */
#define CHUNKSIZE   (1<<12)         /* Extend heap by 4KB */
#define LISTLIMIT   12              /* Number of segregated lists */

#define MAX(x,y)    ((x) > (y) ? (x) : (y))

/* Pack size and allocated bit into a word */
#define PACK(size, alloc) ((size) | (alloc))

/* Read and write a word at address p */
#define GET(p)          (*(uintptr_t *)(p))
#define PUT(p,val)      (*(uintptr_t *)(p) = (val))

/* Read size and allocated fields from address p */
#define GET_SIZE(p)     (GET(p) & ~0xF)
#define GET_ALLOC(p)    (GET(p) & 0x1)

/* Given block ptr bp, compute address of its header and footer */
#define HDRP(bp)        ((char *)(bp) - WSIZE)
#define FTRP(bp)        ((char *)(bp) + GET_SIZE(HDRP(bp)) - DSIZE)

/* Given block ptr bp, compute address of next and previous blocks */
#define NEXT_BLKP(bp)   ((char *)(bp) + GET_SIZE(((char *)(bp) - WSIZE)))
#define PREV_BLKP(bp)   ((char *)(bp) - GET_SIZE(((char *)(bp) - DSIZE)))

/* Explicit free list pointers */
#define SUCC_FREE(bp)   (*(void **)(bp))
#define PRED_FREE(bp)   (*(void **)((char *)(bp) + WSIZE))

/* Global variables */
static void *heap_listp = NULL;
static void *seg_free_lists[LISTLIMIT];  /* Array of free list heads */

/* Internal helper functions */
static void *extend_heap(size_t words);
static void *coalesce(void *bp);
static void insert_free(void *bp);
static void remove_free(void *bp);
static void *find_fit(size_t asize);
static void place(void *bp, size_t asize);
static int get_list_index(size_t size);

/**********************************************************
 * get_list_index
 * Return index into segregated list array for a given size
 **********************************************************/
static int get_list_index(size_t size)
{
    int idx = 0;
    size_t blocksize = 16;
    while (idx < LISTLIMIT - 1 && size > blocksize) {
        blocksize <<= 1;
        idx++;
    }
    return idx;
}

/**********************************************************
 * mm_init
 **********************************************************/
int mm_init(void)
{
    for (int i = 0; i < LISTLIMIT; i++)
        seg_free_lists[i] = NULL;

    if ((heap_listp = mem_sbrk(4 * WSIZE)) == (void *)-1)
        return -1;

    PUT(heap_listp, 0);
    PUT(heap_listp + WSIZE, PACK(DSIZE, 1));        /* Prologue header */
    PUT(heap_listp + 2 * WSIZE, PACK(DSIZE, 1));    /* Prologue footer */
    PUT(heap_listp + 3 * WSIZE, PACK(0, 1));        /* Epilogue header */
    heap_listp += DSIZE;

    if (extend_heap(CHUNKSIZE / WSIZE) == NULL)
        return -1;

    return 0;
}

/**********************************************************
 * extend_heap
 **********************************************************/
static void *extend_heap(size_t words)
{
    char *bp;
    size_t size = (words % 2) ? (words + 1) * WSIZE : words * WSIZE;
    size = ALIGN(size);

    if ((bp = mem_sbrk(size)) == (void *)-1)
        return NULL;

    PUT(HDRP(bp), PACK(size, 0));
    PUT(FTRP(bp), PACK(size, 0));
    PUT(HDRP(NEXT_BLKP(bp)), PACK(0, 1));

    return coalesce(bp);
}

/**********************************************************
 * coalesce
 **********************************************************/
static void *coalesce(void *bp)
{
    size_t next_alloc = GET_ALLOC(HDRP(NEXT_BLKP(bp)));
    size_t size = GET_SIZE(HDRP(bp));

    if (!next_alloc) {
        remove_free(NEXT_BLKP(bp));
        size += GET_SIZE(HDRP(NEXT_BLKP(bp)));
        PUT(HDRP(bp), PACK(size, 0));
        PUT(FTRP(bp), PACK(size, 0));
    }

    insert_free(bp);
    return bp;
}


/**********************************************************
 * insert_free
 **********************************************************/
static void insert_free(void *bp)
{
    size_t size = GET_SIZE(HDRP(bp));
    int idx = get_list_index(size);
    void *head = seg_free_lists[idx];

    SUCC_FREE(bp) = head;
    PRED_FREE(bp) = NULL;
    if (head != NULL)
        PRED_FREE(head) = bp;
    seg_free_lists[idx] = bp;
}

/**********************************************************
 * remove_free
 **********************************************************/
static void remove_free(void *bp)
{
    size_t size = GET_SIZE(HDRP(bp));
    int idx = get_list_index(size);

    if (PRED_FREE(bp))
        SUCC_FREE(PRED_FREE(bp)) = SUCC_FREE(bp);
    else
        seg_free_lists[idx] = SUCC_FREE(bp);

    if (SUCC_FREE(bp))
        PRED_FREE(SUCC_FREE(bp)) = PRED_FREE(bp);
}

/**********************************************************
 * find_fit
 **********************************************************/
static void *find_fit(size_t asize)
{
    int idx = get_list_index(asize);

    for (int i = idx; i < LISTLIMIT; i++) {
        for (void *bp = seg_free_lists[i]; bp != NULL; bp = SUCC_FREE(bp)) {
            if (asize <= GET_SIZE(HDRP(bp)))
                return bp;
        }
    }
    return NULL;
}

/**********************************************************
 * place
 **********************************************************/
static void place(void *bp, size_t asize)
{
    size_t csize = GET_SIZE(HDRP(bp));
    remove_free(bp);

    if ((csize - asize) >= (4 * DSIZE)) {
        PUT(HDRP(bp), PACK(asize, 1));
        PUT(FTRP(bp), PACK(asize, 1));

        void *next_bp = NEXT_BLKP(bp);
        PUT(HDRP(next_bp), PACK(csize - asize, 0));
        PUT(FTRP(next_bp), PACK(csize - asize, 0));
        coalesce(next_bp);
    } else {
        PUT(HDRP(bp), PACK(csize, 1));
        PUT(FTRP(bp), PACK(csize, 1));
    }
}

/**********************************************************
 * mm_malloc
 **********************************************************/
void *mm_malloc(size_t size)
{
    if (size == 0)
        return NULL;

    size_t asize = ALIGN(size + DSIZE);
    void *bp;

    if ((bp = find_fit(asize)) != NULL) {
        place(bp, asize);
        return bp;
    }

    size_t extendsize = MAX(asize, CHUNKSIZE);
    if ((bp = extend_heap(extendsize / WSIZE)) == NULL)
        return NULL;

    place(bp, asize);
    return bp;
}

/**********************************************************
 * mm_free
 **********************************************************/
void mm_free(void *bp)
{
    if (!bp)
        return;

    size_t size = GET_SIZE(HDRP(bp));
    PUT(HDRP(bp), PACK(size, 0));
    PUT(FTRP(bp), PACK(size, 0));
    coalesce(bp);
}

/**********************************************************
 * mm_realloc
 **********************************************************/
void *mm_realloc(void *ptr, size_t size)
{
    if (size == 0) {
        mm_free(ptr);
        return NULL;
    }
    if (ptr == NULL)
        return mm_malloc(size);

    size_t oldsize = GET_SIZE(HDRP(ptr)) - DSIZE;
    if (size <= oldsize)
        return ptr;

    void *newptr = mm_malloc(size);
    if (!newptr)
        return NULL;

    memcpy(newptr, ptr, oldsize);
    mm_free(ptr);
    return newptr;
}

/**********************************************************
 * mm_check
 **********************************************************/
int mm_check(void)
{
    return 1; /* Optional: add consistency checks here */
}
