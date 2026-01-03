#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

#include "defs.h"
#include "hash.h"

#define SAMPLES_PER_STREAM   10000000
#define RAND_NUM_UPPER_BOUND 100000
#define NUM_SEED_STREAMS     8
#define HASH_SIZE            16384   // 2^14, same as h.setup(14)

/* Team info */
team_t team = {
    "NO NAME", 
    "NANZITONG ZHANG",
    "1010390989",
    "nzt.zhang@mail.utoronto.ca"
};

/* Sample class */
class sample {
    unsigned my_key;
public:
    sample *next;
    unsigned count;

    sample(unsigned the_key) { my_key = the_key; count = 0; next = NULL; }
    unsigned key() { return my_key; }
    void print(FILE *f) { printf("%d %d\n", my_key, count); }
};

/* Hash table instance */
hash<sample, unsigned> h;

/* List-level mutexes */
pthread_mutex_t list_locks[HASH_SIZE];

/* Command-line globals */
static int num_threads;
static int sampling_interval;

/* Usage function */
void usage(char *prog) {
    printf("Usage: %s <num_threads> <sampling_interval>\n", prog);
    printf("       num_threads should be 1, 2, 4 or 8\n");
    printf("       sampling_interval should be > 0\n");
    exit(1);
}

/* Thread worker function */
void* process_streams(void* arg) {
    int thread_id = *((int*)arg);
    unsigned int rnum;

    for (int i = thread_id; i < NUM_SEED_STREAMS; i += num_threads) {
        rnum = i;

        for (int j = 0; j < SAMPLES_PER_STREAM; j++) {
            unsigned int num = 0;

            // Sample random numbers
            for (int k = 0; k < sampling_interval; k++) {
                num = rand_r(&rnum);
            }

            unsigned key = num % RAND_NUM_UPPER_BOUND;
            sample *s;

            // Compute list index in hash table
            int slot_index = key % HASH_SIZE;

            // Lock the list for this slot
            pthread_mutex_lock(&list_locks[slot_index]);

            s = h.lookup(key);
            if (!s) {
                s = new sample(key);
                h.insert(s); // insert into hash table
            }
            s->count++;

            pthread_mutex_unlock(&list_locks[slot_index]);
        }
    }
    return NULL;
}

int main(int argc, char *argv[]) {
    if (argc != 3) usage(argv[0]);
    sscanf(argv[1], "%d", &num_threads);
    sscanf(argv[2], "%d", &sampling_interval);

    if (num_threads != 1 && num_threads != 2 && num_threads != 4 && num_threads != 8)
        usage(argv[0]);
    if (sampling_interval <= 0) usage(argv[0]);

    printf("Team Name: %s\n", team.team);
    printf("Student Name: %s\n", team.name);
    printf("Student Number: %s\n", team.number);
    printf("Student Email: %s\n\n", team.email);

    // Initialize hash table and list mutexes
    h.setup(14);
    for (int i = 0; i < HASH_SIZE; i++) {
        pthread_mutex_init(&list_locks[i], NULL);
    }

    // Create threads
    pthread_t threads[num_threads];
    int thread_ids[num_threads];
    for (int t = 0; t < num_threads; t++) {
        thread_ids[t] = t;
        pthread_create(&threads[t], NULL, process_streams, &thread_ids[t]);
    }

    // Join threads
    for (int t = 0; t < num_threads; t++) {
        pthread_join(threads[t], NULL);
    }

    // Print results
    h.print();

    return 0;
}
