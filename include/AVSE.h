/*******************************************************************************
*
* PROJECT: AUTONOMOUS VEHICULE IN A STRUCTURED ENVIRONMENT
*
* SECTION: General
*
* AUTHORS: Jean-Sebastien Fiset and Alexandre Fawcett
*
* DESCRIPTION:
*
*       
*
* NOTES:
*
*
*
********************************************************************************/
#ifndef AVSE
#define AVSE

#include "Grid.h"
#include "NavigationModel.h"
#include "protocol.h"
#include "computervision.h"
#include <stdio.h>
#include <time.h>

static const string GRID_FILE_PATH = "data/Grid25x5.txt";
static const string CAM_PARAM="data/camparam.yml";

FILE *file;

static const int CAMANGLE_START = -90;
static const int CAMANGLE_END = 90;
static const int CAMANGLE_INCREMENT = 45;

static const int NB_TRIES_CAMANGLE = 10;
static const int DELAY_CAMANGLE = 1000000;

static const int NB_TRIES_NEXT = 20;
static const int DELAY_NEXT = 1000000;

#endif /* AVSE */
