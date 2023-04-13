#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#define DM510_IOC_MAGIC 255
#define DM510_IOCSBUFFER _IOW(DM510_IOC_MAGIC, 1, int)
#define DM510_IOCSREADERS _IOW(DM510_IOC_MAGIC, 2, int)


int main() {
  int fd, result;
  char buf[10];

  fd = open("/dev/dm510-0", O_WRONLY);
  printf("Opened dm510-0...\n");
  sleep(1);
  write(fd, "123456789", 9);
  printf("Wrote \"123456789\"\n");
  sleep(1);
  close(fd);
  printf("Closed dm510-0...\n");
  sleep(1);

  fd = open("/dev/dm510-1", O_RDONLY);
  printf("Opened dm510-1...\n");
  sleep(1);
  result = read (fd, &buf, sizeof(buf));
  buf[result] = '\0';
  fprintf (stdout, "read: \"%s\"\n", buf);
  sleep(1);
  close(fd);
  printf("Closed dm510-1...\n");
  sleep(1);

  fd = open("/dev/dm510-1", O_RDWR);
  printf("Opened dm510-1...\n");
  sleep(1);
  write(fd, "1234", 4);
  printf("Wrote \"1234\"...\n");
  sleep(1);
  close(fd);
  printf("Closed dm510-1...\n");
  sleep(1);


  fd = open("/dev/dm510-0", O_RDWR);
  printf("Opened dm510-0...\n");
  sleep(1);
  result = read(fd, &buf, sizeof(buf));
  buf[result] = '\0';
  fprintf (stdout, "read: %s\n", buf);
  sleep(1);
  close(fd);
  printf("Closed dm510-0...\n");
  sleep(1);
  return 0;
}
