#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>
#include <termio.h>

#include "zen.h"

char * read_readme(const char *filename) {
  struct stat statbuf;
  int i = stat(filename, &statbuf);
  if ( i == 0 ) {
    char *message = calloc(statbuf.st_size+1, sizeof(char));
    if (message) {
      FILE* file = fopen(filename, "r");
      size_t nread = 0;
      while (!feof(file) && !ferror(file)) {
	nread = fread(message+nread, sizeof(char), statbuf.st_size-nread, file);
      }
      return message;
    }
  }
  return NULL;
}

inline void clear_screen(void) {
  printf("\033[H\033[J");
}

int getch(void) {
  char ch;
  int fd = fileno(stdin);
  struct termio old_tty, new_tty;
  ioctl(fd, TCGETA, &old_tty);
  new_tty = old_tty;
  new_tty.c_lflag &= ~(ICANON | ECHO | ISIG);
  ioctl(fd, TCSETA, &new_tty);
  fread(&ch, 1, sizeof(ch), stdin);
  ioctl(fd, TCSETA, &old_tty);
  return ch;
}
