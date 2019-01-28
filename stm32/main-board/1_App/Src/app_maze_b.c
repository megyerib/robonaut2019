////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//!
//!  \file      app_maze.c
//!  \brief
//!  \details
//!
//! 	Gr�f pontjai:
//! 		N:    N-edik keresztez�d�s, el�re
//! 		N+16: N-edik keresztez�d�s, h�tra
//!
//!     Ha h�tra megy�nk, a bal-jobb ir�nyt akkor is az el�re �ll� aut�hoz k�pest vessz�k.
//!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Includes ------------------------------------------------------------------------------------------------------------

#include "navigation.h"

// Defines -------------------------------------------------------------------------------------------------------------

#define DONTCARE          (0)
#define JUNCTION_NUM_MAX  (16u)
#define JUNCTION_DIR_MASK (0x10u)
#define EDGE_NUM_MAX      (128u)
#define PATH_MAX_LEN      (20u)

// Typedefs ------------------------------------------------------------------------------------------------------------

typedef enum
{
	carDirForward,
	carDirBackward,

	carDirNum
}
CAR_DIR;

typedef enum
{
	exitFront,
	exitRear,
	exitSide,

	exitTypeNum
}
EXIT_TYPE;

typedef enum
{
	dirRight,
	dirLeft,

	dirNum
}
DIR;

typedef struct
{
	uint8_t     exitVisited[exitTypeNum];
	DIR         direction;
	cNAVI_STATE position;
}
JUNCTION;

typedef enum
{
	discoveryMode, // Megy�nk a vil�gba' egy ismeretlen �ton
	travelMode     // El akarunk jutni egy ismert helyre egy ismert �ton
}
MAZE_MODE;

// Local (static) & extern variables -----------------------------------------------------------------------------------

static JUNCTION junctions[JUNCTION_NUM_MAX];

static int junction_num = 0;

static int junctionFound = 0;
static int exitFound     = 0;

static int currentVertex = 0;

static CAR_DIR carDirection = carDirForward;
static MAZE_MODE mode = discoveryMode;

// Local (static) function prototypes ----------------------------------------------------------------------------------

static int isMazeComplete();
static void checkRoadSignal();
static void followLine();
static void exitTrack();
static void handleJunction();
static int isJunctionVisited();
static void registerPath();
static int isPathVisited();
static void addJunction();
static void chooseExit(EXIT_TYPE exitType, DIR exitDir);
static void switchLine(DIR d);

// Global function definitions -----------------------------------------------------------------------------------------

void maze()
{
	while (1)
	{
		if (isMazeComplete())
		{
			exitTrack();
		}
		else
		{
			followLine();

			checkRoadSignal();

			if (junctionFound)
			{
				handleJunction();
				junctionFound = 0;
			}

			if (exitFound)
			{

				exitFound = 0;
			}
		}
	}
}

// Local (static) function definitions ---------------------------------------------------------------------------------

static int isMazeComplete()
{
	int exitsUncplt = 0;

	// Ha m�g nincs keresztez�d�s (frissen rajtoltunk)
	if (junction_num == 0)
	{
		return 0;
	}

	for (int i = 0; i < junction_num; i++)
	{
		exitsUncplt += exitTypeNum;

		for (int j = 0; j < exitTypeNum; j++)
		{
			exitsUncplt -= junctions[i].exitVisited[j];
		}
	}

	if (exitsUncplt == 0)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

static void checkRoadSignal()
{
	// TODO
	// Set junctionFound, exitFound
}

static void followLine()
{
	// TODO
}

static void exitTrack()
{

}

static void handleJunction()
{
	int recentlyVisited = junction_num;

	// M�g nem voltunk itt
	if (!isJunctionVisited())
	{
		addJunction();
	}

	// �t regisztr�l�sa,
	// Ha m�g nem voltunk itt �s m�r nem a rajt ut�n vagyunk
	if (!isPathVisited() && recentlyVisited > 0)
	{
		registerPath();
	}

	if (mode == discoveryMode)
	{
		// TODO �tvonaltervez�s

		mode = travelMode;
	}

	// TODO kij�rat kiv�laszt�sa a k�vetkez� szakasz alapj�n
}

static int isJunctionVisited()
{
	// TODO
	return 0;
}

static void registerPath()
{
	// Ha rajt ut�n vagyunk, nem vesz�nk fel �lt
	// TODO


}

static int isPathVisited()
{
	// TODO
	return 0;
}

// > ------> >
// < <------ <
static void addForwardEdges()
{

}

// > ------> >
// > <------ >
// < <------ <
// < ------> <
/*static void addEdgesWithReverse()
{

}*/

static void addJunction()
{
	// TODO
}

// TODO biztos meg lehet oldani �gyesebben is
// Ha megh�v�dik a f�ggv�ny, a keresztez�d�s k�zepe a k�t szenzor k�zt van.
//
// ----------------------
// -------------
//             ^
//   A keresztez�d�s k�zepe
//
static void chooseExit(EXIT_TYPE exitType, DIR exitDir)
{
	// El�re �llunk
	if ((currentVertex & JUNCTION_DIR_MASK) == 0)
	{
		switch (exitType)
		{
			case exitFront:
			{
				if (exitDir == dirRight)
				{
					switchLine(dirLeft);
				}
				else
				{
					switchLine(dirRight);
				}

				carDirection = carDirForward;

				break;
			}
			case exitRear:
			{
				carDirection = carDirBackward;

				break;
			}
			case exitSide:
			{
				if (exitDir == dirRight)
				{
					switchLine(dirRight);
				}
				else
				{
					switchLine(dirLeft);
				}

				carDirection = carDirForward;

				break;
			}
			default:
			{
				break;
			}
		}
	}
	// H�tra �llunk

	/*                   |
	 *                 H�tra
	 *                   |
	 *                   ^
	 *                  /|\
	 *          Balra  / | \  Jobbra
	 *  (Jobb kanyar) /  |  \ (Bal kanyar)
	 *
	 *                 El�re
	 */

	else
	{
		switch (exitType)
		{
			case exitFront:
			{
				if (exitDir == dirRight)
				{
					switchLine(dirRight);
				}
				else
				{
					switchLine(dirLeft);
				}

				carDirection = carDirBackward;

				break;
			}
			case exitRear:
			{
				carDirection = carDirForward;

				break;
			}
			case exitSide:
			{
				if (exitDir == dirRight)
				{
					switchLine(dirLeft);
				}
				else
				{
					switchLine(dirRight);
				}

				carDirection = carDirBackward;

				break;
			}
			default:
			{
				break;
			}
		}
	}
}

static void switchLine(DIR d)
{
	// TODO
	// TODO arra is figyelni kell, ha �ppen nem �rz�keli mindk�t vonalat a szenzor
}

// END -----------------------------------------------------------------------------------------------------------------
