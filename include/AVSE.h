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

static const string GRID_FILE_PATH = "data/Grid.txt";
static const string CAM_PARAM="data/camparam.yml";

static const int CAMANGLE_START = -90;
static const int CAMANGLE_END = 90;
static const int CAMANGLE_INCREMENT = 45;

static const int NB_TRIES_CAMANGLE = 10;
static const int DELAY_CAMANGLE = 2;

static const int NB_TRIES_NEXT = 10;
static const int DELAY_NEXT = 5;

#endif /* AVSE */
