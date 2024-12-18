#include <stdio.h>
#include <stdlib.h>

#include <vector>

#include "common.h"

int main(int argc, char** argv) {
  const char* path = "out.bin";

  if (argc == 2) {
    path = argv[1];
  } else if (argc != 1) {
    printf("unsupported number of arguments.\n");
    return EXIT_FAILURE;
  }

  FILE* infile = fopen(path, "rb");
  if (!infile) {
    printf("failed to open file '%s'\n", path);
    return EXIT_FAILURE;
  }

  std::vector<packet> packets;

  int packet_index = 0;

  while (!feof(infile)) {
    packet p;

    size_t read_size = fread(&p, sizeof(p), 1, infile);
    if (read_size != 1) {
      break;
    }

    if (p.compute_checksum() != p.checksum) {
      printf("found bad packet (i=%d) due to checksum failure\n", packet_index);
      return EXIT_FAILURE;
    }

    if ((p.magic[0] != 0xff) || (p.magic[1] != 0xfd)) {
      printf("found bad packet due to missing magic number.\n");
      return EXIT_FAILURE;
    }

    packets.emplace_back(p);

    packet_index++;
  }

  auto max_time_error = -100000;
  auto min_time_error = 100000;
  auto total_time_error = 0;

  for (size_t i = 1; i < packets.size(); i++) {
    const auto t0 = packets[i - 1].time;
    const auto t1 = packets[i].time;
    const auto dt = t1 - t0;
    const auto expected = 10u;
    const auto delta = static_cast<int>(expected) - static_cast<int>(dt);
    max_time_error = (delta > max_time_error) ? delta : max_time_error;
    min_time_error = (delta < min_time_error) ? delta : min_time_error;
    total_time_error += delta * delta;
    if (delta > 500) {
      printf("t0 = %u, t1 = %u\n", t0, t1);
    }
  }

  printf("total time error squared: %d ms\n", total_time_error);
  printf("max time error: %d ms\n", max_time_error);

  return 0;
}
