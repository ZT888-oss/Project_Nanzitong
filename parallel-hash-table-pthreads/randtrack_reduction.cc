#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#include <sstream>
#include <string>
#include <vector>

#include "defs.h"
#include "hash.h"

#define SAMPLES_PER_STREAM 10000000
#define RAND_NUM_UPPER_BOUND 100000
#define NUM_SEED_STREAMS 8

/* Team info */
team_t team = {"NO NAME", "NANZITONG ZHANG", "1010390989",
               "nzt.zhang@mail.utoronto.ca"};

/* Sample class */
class sample {
  unsigned my_key;

 public:
  sample* next;
  unsigned count;

  sample(unsigned k) {
    my_key = k;
    count = 0;
    next = NULL;
  }
  unsigned key() { return my_key; }
  void print(FILE* f) { fprintf(f, "%u %u\n", my_key, count); }
};

/* Global hash table */
hash<sample, unsigned> global_hash;

/* Per-thread hash tables */
hash<sample, unsigned>* thread_hashes;

static int num_threads;
static int sampling_interval;

void usage(char* p) {
  printf("Usage: %s <num_threads> <sampling_interval>\n", p);
  exit(1);
}

void* process_streams(void* arg) {
  int tid = *((int*)arg);
  unsigned int rnum;

  for (int i = tid; i < NUM_SEED_STREAMS; i += num_threads) {
    rnum = i;
    for (int j = 0; j < SAMPLES_PER_STREAM; j++) {
      unsigned int num = 0;
      for (int k = 0; k < sampling_interval; k++) num = rand_r(&rnum);

      unsigned key = num % RAND_NUM_UPPER_BOUND;

      sample* s = thread_hashes[tid].lookup(key);
      if (!s) {
        s = new sample(key);
        thread_hashes[tid].insert(s);
      }
      s->count++;
    }
  }
  return NULL;
}

/*
 * Merge by capturing print() output into a buffer, parsing lines.
 * This avoids accessing private hash internals.
 */
void merge_hashes() {
  for (int t = 0; t < num_threads; t++) {
    // Capture print output into a temporary file
    FILE* tmp = tmpfile();
    thread_hashes[t].print(tmp);
    rewind(tmp);

    unsigned key, count;

    // Parse each line "key count"
    while (fscanf(tmp, "%u %u", &key, &count) == 2) {
      // Insert into global hash
      sample* gs = global_hash.lookup(key);
      if (!gs) {
        gs = new sample(key);
        global_hash.insert(gs);
      }
      gs->count += count;
    }

    fclose(tmp);
  }
}

int main(int argc, char* argv[]) {
  if (argc != 3) usage(argv[0]);
  sscanf(argv[1], "%d", &num_threads);
  sscanf(argv[2], "%d", &sampling_interval);

  printf("Team Name: %s\n", team.team);
  printf("Student Name: %s\n", team.name);
  printf("Student Number: %s\n", team.number);
  printf("Student Email: %s\n\n", team.email);

  global_hash.setup(14);

  thread_hashes = new hash<sample, unsigned>[num_threads];
  for (int t = 0; t < num_threads; t++) thread_hashes[t].setup(14);

  pthread_t threads[num_threads];
  int ids[num_threads];

  for (int t = 0; t < num_threads; t++) {
    ids[t] = t;
    pthread_create(&threads[t], NULL, process_streams, &ids[t]);
  }

  for (int t = 0; t < num_threads; t++) pthread_join(threads[t], NULL);

  merge_hashes();

  global_hash.print();

  delete[] thread_hashes;
  return 0;
}
