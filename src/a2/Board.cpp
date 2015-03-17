#include "Board.hpp"

std::string COLOR_NAMES[] = {"GREEN", "RED", "YELLOW"};

Board::Board(ImageProcessor *ip_, bool red) : ip(ip_)
{
    playerColor = (red ? "RED" : "GREEN");
}

Board::~Board()
{
}

void Board::getCalibration(coord *data)
{
    // TODO: fix this
    std::ifstream input("calibration_test2.txt");
    double x, y;
    for(int i = 0; i < 3; i++)
    {
        input >> x >> y;
        data[i] = {x, y};
    }
}

bool Board::withinBounds(int p1, int p2)
{
    // TODO: fix so that it adds and subtracts 60
    return p1 < p2+30 && p1 > p2-30;
}

int Board::getCornersIndex(int x, int y, coord *calibrationData)
{
    for(int i = 0; i < 3; i++)
    {
        if(withinBounds(x, calibrationData[i].x) && withinBounds(y, calibrationData[i].y))
        {
            return i;
        }
    }
    return 3;
}

void Board::getInput(coord *calibrationData, coord *corners, coord *greenBalls, coord *redBalls, std::string filename)
{
    std::ifstream input(filename.c_str());
    std::string elt;
    gsl_vector *world;
    world = gsl_vector_alloc(3);
    gsl_vector *point;
    point = gsl_vector_alloc(3);
    int mode_switch = 0;
    int index = 0;
    
    input >> elt; // get first '#'
    while(input >> elt)
    {
        if(elt == "#")
        {
            mode_switch++;
            index = 0;
            continue;
        }
        double x = atoi(elt.c_str());
        input >> elt;
        double y = atoi(elt.c_str());
    
        gsl_vector_set(point, 0, x);
        gsl_vector_set(point, 1, y);
        gsl_vector_set(point, 2, 1);
        ip->calculate_arm_coords(point, world);
        if(mode_switch == 0) // corners
        {
            std::cout << "corner: " << x << ", " << y << std::endl;
            int cornersIndex = getCornersIndex(x, y, calibrationData);
            std::cout << "corners index: " << cornersIndex << std::endl;
            //corners[cornersIndex] = {gsl_vector_get(world, 0), gsl_vector_get(world, 1)};
            corners[cornersIndex] = {x, y};
        }
        else if(mode_switch == 1) // green balls
        {
            //greenBalls[index] = {gsl_vector_get(world, 0), gsl_vector_get(world, 1)};
            greenBalls[index] = {x, y};
            index++;
        }
        else // mode_switch == 2 : red balls
        {
            //redBalls[index] = {gsl_vector_get(world, 0), gsl_vector_get(world, 1)};
            redBalls[index] = {x, y};
            index++;
        }
    }
}

//void Board::boardInit(coord balls_[5], coord pos1, coord pos2, coord pos3, coord pos4, std::string color)
void Board::boardInit(std::string filename)
{
    // grab calibration data
    coord calibrationData[3];
    getCalibration(calibrationData);
    
    // get positions from file
    coord corners[4];
    coord greenBalls[5];
    coord redBalls[5];
    getInput(calibrationData, corners, greenBalls, redBalls, filename);

    // figure out x and y coords of each square
    double boardx2 = (corners[1].x + corners[0].x)/2;
    double boardx1 = (corners[1].x + boardx2)/2;
    double boardx3 = (corners[0].x + boardx2)/2;

    double boardy2 = (corners[1].y + corners[2].y)/2;
    double boardy1 = (corners[1].y + boardy2)/2;
    double boardy3 = (corners[2].y + boardy2)/2;

    coord min0 = {boardx1-15, boardy1-15};
    coord max0 = {boardx1+15, boardy1+15};
    board[0] = {YELLOW, min0, max0};

    coord min1 = {boardx1-15, boardy2-15};
    coord max1 = {boardx1+15, boardy2+15};
    board[1] = {YELLOW, min1, max1};

    coord min2 = {boardx1-15, boardy3-15};
    coord max2 = {boardx1+15, boardy3+15};
    board[2] = {YELLOW, min2, max2};

    coord min3 = {boardx2-15, boardy1-15};
    coord max3 = {boardx2+15, boardy1+15};
    board[3] = {YELLOW, min3, max3};

    coord min4 = {boardx2-15, boardy2-15};
    coord max4 = {boardx2+15, boardy2+15};
    board[4] = {YELLOW, min4, max4};

    coord min5 = {boardx2-15, boardy3-15};
    coord max5 = {boardx2+15, boardy3+15};
    board[5] = {YELLOW, min5, max5};

    coord min6 = {boardx3-15, boardy1-15};
    coord max6 = {boardx3+15, boardy1+15};
    board[6] = {YELLOW, min6, max6};

    coord min7 = {boardx3-15, boardy2-15};
    coord max7 = {boardx3+15, boardy2+15};
    board[7] = {YELLOW, min7, max7};

    coord min8 = {boardx3-15, boardy3-15};
    coord max8 = {boardx3+15, boardy3+15};
    board[8] = {YELLOW, min8, max8};

    if(playerColor == "GREEN")
    {
        for(int i = 0; i < 5; i++)
        {
            freeBalls[i].position = greenBalls[i];
            freeBalls[i].color = GREEN;
        }
    }
    else // if playerColor == "RED"
    {
        for(int i = 0; i < 5; i++)
        {
            freeBalls[i].position = redBalls[i];
            freeBalls[i].color = RED;
        }
    }

    numFreeBalls = 5;
}

void Board::updateBoard(std::string filename)
{
    // clear board
    clearBoard();
    
    // get positions from file
    coord greenBalls[5];
    coord redBalls[5];
    getBalls(greenBalls, redBalls, filename);
    
    // update red balls
    int j = 0;
    for(int i = 0; i < 5; i++)
    {
        int board_index = findBoardIndex(redBalls[i]);
        if(board_index != -1)
        {
            board[board_index].ball = RED;
        }
        else
        {
            if(playerColor == "RED")
            {
                freeBalls[j].position = redBalls[i];
                freeBalls[j].color = RED;
                j++;
            }
        }
    }

    // update green balls
    for(int i = 0; i < 5; i++)
    {
        int board_index = findBoardIndex(greenBalls[i]);
        if(board_index != -1)
        {
            board[board_index].ball = GREEN;
        }
        else
        {
            if(playerColor == "GREEN")
            {
                freeBalls[j].position = greenBalls[i];
                freeBalls[j].color = GREEN;
                j++;
            }
        }
    }
    numFreeBalls = j;
}

void Board::getBalls(coord *greenBalls, coord *redBalls, std::string filename)
{
    std::ifstream input(filename.c_str());
    std::string elt;
    gsl_vector *world;
    world = gsl_vector_alloc(3);
    gsl_vector *point;
    point = gsl_vector_alloc(3);
    int mode_switch = 0;
    int index = 0;
    
    input >> elt; // get first '#'
    while(input >> elt)
    {
        if(elt == "#")
        {
            mode_switch++;
            index = 0;
            continue;
        }
        if(mode_switch == 0) // corners
        {
            continue;
        }
        double x = atoi(elt.c_str());
        input >> elt;
        double y = atoi(elt.c_str());
    
        gsl_vector_set(point, 0, x);
        gsl_vector_set(point, 1, y);
        gsl_vector_set(point, 2, 1);
        ip->calculate_arm_coords(point, world);
        if(mode_switch == 1) // green balls
        {
            //greenBalls[index] = {gsl_vector_get(world, 0), gsl_vector_get(world, 1)};
            greenBalls[index] = {x, y};
            index++;
        }
        else // mode_switch == 2 : red balls
        {
            //redBalls[index] = {gsl_vector_get(world, 0), gsl_vector_get(world, 1)};
            redBalls[index] = {x, y};
            index++;
        }
    }
}

bool Board::gameOver()
{
    return false;
}

void Board::clearBoard()
{
    for(int i = 0; i < 9; i++)
    {
        board[i].ball = YELLOW;
    }
}

int Board::findBoardIndex(coord ball)
{
    for(int i = 0; i < 9; i++)
    {
        if(ball.x > board[i].min_coord.x &&
           ball.x < board[i].max_coord.x &&
           ball.y > board[i].min_coord.y &&
           ball.y < board[i].max_coord.y)
        {
            return i;
        }
    }
    return -1;
}

Ball *Board::getFreeBalls()
{
    // note to self: RETURNING NULL IS USEFUL FOR DETERMINING WHEN THE GAME IS OVER
    if(numFreeBalls > 0)
    {
        return freeBalls;
    }
    else
    {
        return NULL;
    }
}

void Board::printInit()
{
    for(int i = 0; i < 9; i++)
    {
        std::cout << "(" << board[i].min_coord.x << ", " << board[i].min_coord.y << ") -> ("
                  << board[i].max_coord.x << ", " << board[i].max_coord.y << ")    ";
        if(i%3 == 2)
        {
            std::cout << std::endl;
        }
    }
}

void Board::print()
{
    // print board
    for(int i = 0, j = 0; i < 9; i++)
    {
        if(board[i].ball == GREEN) std::cout << "G ";
        else if(board[i].ball == RED) std::cout << "R ";
        else std::cout << "  ";
        if(i%3 != 2) std::cout << "| ";
        if(i%3 == 2){
            j++;
            std::cout << std::endl;
        }
        if(j < 3 && i%3 == 2) std::cout << "--+---+--" << std::endl;
    }

    // print free balls
    for(int i = 0; i < numFreeBalls; i++)
    {
        std::cout << "freeBalls[" << i << "]: (" << freeBalls[i].position.x << ", " << freeBalls[i].position.y << ")" << std::endl;
        std::cout << "                        " << COLOR_NAMES[freeBalls[i].color] << std::endl;
    }
}
