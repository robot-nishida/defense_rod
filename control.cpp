#include <stdio.h>
#include "const.h"

/*** 制御式 ***/
int control(int src)
{
  printf("src = %d\n",src);
  printf("const = %d\n",param[0]);
  printf("const = %d\n",param[1]);
  return 0;
}

