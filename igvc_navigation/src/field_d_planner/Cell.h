/**
Represents a cell on the graph

Author: Alejandro Escontrela <aescontrela3@gatech.edu>
Date Created: December 16, 2018
*/

#ifndef CELL_H
#define CELL_H

class Cell
{
public:
    /**
    Default cell
    */
    Cell(int x, int y, int c, int b);
    /**
    Returns true if this is a boundary cell. Boundary cells neighbor nodes that
    lie on the corners of the graph or along its edge.

    @return whether this cell is or is not a boundary cell
    */
    bool isBoundary();

private:

};

#endif // CELL_H
