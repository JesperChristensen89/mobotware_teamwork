#include "commands.h"
#include <stdio.h>
#include <stdlib.h>

using namespace std;

void Commands::angle(char* angleStr, float angle)
{
  char str[50];
  snprintf(str,50, "A=%f\n", angle);
  
  for (int i = 0; i < 50; i++)
    angleStr[i] = str[i];
}

void Commands::distance(char* distanceStr, float distance)
{
  char str[50];
  snprintf(str, 50, "D=%f\n", distance);
  for (int i = 0; i < 50; i++)
    distanceStr[i] = str[i];
}
