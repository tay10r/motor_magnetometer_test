#include <errno.h>
#include <fcntl.h>
#include <motor_test/common.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

int
main()
{
  FILE* outfile = fopen("out.bin", "wb");

  const char* portname = "/dev/ttyACM0";

  // Open the serial port
  int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd == -1) {
    perror("Failed to open serial port");
    return 1;
  }

  // Configure the serial port
  struct termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    perror("Failed to get terminal attributes");
    close(fd);
    return 1;
  }

  // Set up the baud rate
  cfsetospeed(&tty, B9600); // Output speed
  cfsetispeed(&tty, B9600); // Input speed

  // Configure 8N1 mode (8 data bits, no parity, 1 stop bit)
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8 data bits
  tty.c_cflag &= ~PARENB;                     // No parity
  tty.c_cflag &= ~CSTOPB;                     // 1 stop bit
  tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines

  // Configure raw input mode
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_oflag &= ~OPOST;

  // Apply the configuration
  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    perror("Failed to set terminal attributes");
    close(fd);
    return 1;
  }

  // Read data from the serial port
  int num_samples = 0;
  const int max_samples = 4096;
  mt::packet p;

  while (num_samples < max_samples) {
    const ssize_t bytes_read = read(fd, &p, sizeof(p));

    if (bytes_read == sizeof(p)) {
      if (p.compute_checksum() != p.checksum) {
        printf("checksum failed\n");
        continue;
      }
      num_samples++;
      printf(
        "[%d/%d]: Read %zd bytes.\n", num_samples, max_samples, bytes_read);
      fwrite(&p, sizeof(p), 1, outfile);
    } else if (bytes_read < 0) {
      perror("Error reading from serial port");
      break;
    } else {
      printf("No data available\n");
    }
  }

  // Close the serial port
  close(fd);
  fclose(outfile);

  return 0;
}
