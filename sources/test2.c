#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#define DM510_IOC_MAGIC 255
#define DM510_IOCSBUFFER _IOW(DM510_IOC_MAGIC, 1, int)
#define DM510_IOCSREADERS _IOW(DM510_IOC_MAGIC, 2, int)
#define DM510_IOCTBUFFERSIZE _IO(DM510_IOC_MAGIC, 3)
#define DM510_IOCTMAXREADERS _IO(DM510_IOC_MAGIC, 4)


int main() {
  int fd, result;
  char buf[40];

  fd = open("/dev/dm510-0", O_WRONLY);
  printf("Opened dm510-0...\n");
  sleep(1);
  result = ioctl(fd, DM510_IOCTBUFFERSIZE);
  printf("Buffersize is: %d\n", result);
  ioctl(fd, DM510_IOCSBUFFER, 40);
  sleep(1);
  result = ioctl(fd, DM510_IOCTBUFFERSIZE);
  printf("Buffersize is now: %d\n", result);
  sleep(1);
  result = ioctl(fd, DM510_IOCTMAXREADERS);
  printf("Max readers are: %d\n", result);
  sleep(1);
  ioctl(fd, DM510_IOCSREADERS, 4);
  result = ioctl(fd, DM510_IOCTMAXREADERS);
  printf("Max readers are now: %d\n", result);
  sleep(1);
  close(fd);
  printf("Closed dm510-0...\n");
  sleep(1);

  return 0;
}
