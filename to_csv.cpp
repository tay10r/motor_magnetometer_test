#include <stdio.h>
#include <stdlib.h>

#include "common.h"

int main() {
  FILE* infile = fopen("out.bin", "rb");
  if (!infile) {
    return EXIT_FAILURE;
  }

  FILE* outfile = fopen("out.csv", "w");

  while (!feof(infile)) {
    packet p;

    size_t read_size = fread(&p, sizeof(p), 1, infile);
    if (read_size != 1) {
      break;
    }

    fprintf(outfile, "%d,%u,%u,%u,%u\n", p.motor_state ? 1 : 0, p.xyz[0],
            p.xyz[1], p.xyz[2], p.time);
  }

  return 0;
}
