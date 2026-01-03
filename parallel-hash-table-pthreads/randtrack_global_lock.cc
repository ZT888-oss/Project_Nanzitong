#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#include "defs.h"
#include "hash.h"

#define SAMPLES_PER_STREAM 10000000
#define RAND_NUM_UPPER_BOUND 100000
#define NUM_SEED_STREAMS 8

/* Team information */
team_t team = {
    "NO NAME",                    // Team name
    "NANZITONG ZHANG",            // Member full name
    "1010390989",                 // Student number
    "nzt.zhang@mail.utoronto.ca"  // Email
};

/* Sample class with element-level lock */
class sample {
  unsigned my_key;

 public:
  sample* next;
  unsigned count;
  pthread_mutex_t lock;  // mutex for element-level lock

  sample(unsigned the_key) {
    my_key = the_key;
    count = 0;
    pthread_mutex_init(&lock, NULL);
    next = NULL;
  }

  unsigned key() { return my_key; }

  void print(FILE* f) { printf("%d %d\n", my_key, count); }
};

/* Hash table: element type = sample, key type = unsigned */
hash<sample, unsigned> h;

/* Global variables for thread count and sampling interval */
static int num_threads;
static int sampling_interval;

/* Function to print usage */
void usage(char* prog) {
  printf("Usage: %s <num_threads> <sampling_interval>\n", prog);
  printf("       num_threads should be 1, 2, 4 or 8\n");
  printf("       sampling_interval should be > 0\n");
  exit(1);
}

/* Thread worker function */
void* process_streams(void* arg) {
  int thread_id = *((int*)arg);
  unsigned int rnum;

  // Each thread handles a subset of seed streams
  for (int i = thread_id; i < NUM_SEED_STREAMS; i += num_threads) {
    rnum = i;

    for (int j = 0; j < SAMPLES_PER_STREAM; j++) {
      unsigned int num = 0;

      // Sample the random numbers
      for (int k = 0; k < sampling_interval; k++) {
        num = rand_r(&rnum);
      }

      unsigned key = num % RAND_NUM_UPPER_BOUND;

      sample* s;

      // Lookup in hash table
      if (!(s = h.lookup(key))) {
        // Insert new sample if not found
        s = new sample(key);
        h.insert(
            s);  // Make sure hash insert is thread-safe (hash may need locking)
      }

      // Element-level lock to increment count
      pthread_mutex_lock(&s->lock);
      s->count++;
      pthread_mutex_unlock(&s->lock);
    }
  }

  return NULL;
}

/* Main function */
int main(int argc, char* argv[]) {
  if (argc != 3) {
    usage(argv[0]);
  }

  sscanf(argv[1], " %d", &num_threads);
  sscanf(argv[2], " %d", &sampling_interval);

  if (num_threads != 1 && num_threads != 2 && num_threads != 4 &&
      num_threads != 8) {
    usage(argv[0]);
  }
  if (sampling_interval <= 0) {
    usage(argv[0]);
  }

  // Print team information
  printf("Team Name: %s\n", team.team);
  printf("Student Name: %s\n", team.name);
  printf("Student Student Number: %s\n", team.number);
  printf("Student Email: %s\n", team.email);
  printf("\n");

  // Initialize hash table with 16K entries
  h.setup(14);

  // Create threads
  pthread_t threads[num_threads];
  int thread_ids[num_threads];

  for (int t = 0; t < num_threads; t++) {
    thread_ids[t] = t;
    pthread_create(&threads[t], NULL, process_streams, &thread_ids[t]);
  }

  // Wait for threads to finish
  for (int t = 0; t < num_threads; t++) {
    pthread_join(threads[t], NULL);
  }

  // Print all samples and counts
  h.print();

  return 0;
}
