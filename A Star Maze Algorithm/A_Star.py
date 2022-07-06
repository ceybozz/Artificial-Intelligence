from pyamaze import maze, agent, textLabel
from queue import PriorityQueue

# heuristic function
# distance between 2 cells
def heuristic(cell1, cell2):
    x1, y1 = cell1
    x2, y2 = cell2

    return abs(x1-x2) + abs(y1-y2)

# A* Star Algorithm function input a maze (myMaze)
def aStarAlgorithm(myMaze):
    start = (myMaze.rows, myMaze.cols)
    g_score = {cell:float('inf') for cell in myMaze.grid}
    g_score[start] = 0
    f_score = {cell:float('inf') for cell in myMaze.grid}
    # f score of start cell is the heuristic cost of the start point
    f_score[start] = heuristic(start, (1,1))

    # PriorityQueue as open
    open = PriorityQueue()
    # PriorityQueue order {1} = start cell {2} = heuristic cost {3} = cell value
    open.put((heuristic(start,(1,1)), heuristic(start,(1,1)), start))

    # A* Star Algorithm pathfinding, reverse path
    aStarPath = {}

    # while loop, PriorityQueue is not empty
    while not open.empty():
        # goal condition
        # third element in tuple is the current cell [2]
        current_Cell = open.get()[2]
        # if current cell is the goal end the process
        if current_Cell == (1,1):
            break
        for g_destination in 'NESW':
            if myMaze.maze_map[current_Cell][g_destination] == True:
                # neighbor cell is different in each directions
                # child cell will use for 4 if statements for each directions 'NESW' North, East, South and West
                if g_destination == 'N':
                    child_Cell = (current_Cell[0] - 1, current_Cell[1])
                if g_destination == 'E':
                    child_Cell = (current_Cell[0], current_Cell[1] + 1)
                if g_destination == 'S':
                    child_Cell = (current_Cell[0] + 1, current_Cell[1])
                if g_destination == 'W':
                    child_Cell = (current_Cell[0], current_Cell[1] - 1)

                # Once the child cell is found, calculate new g score
                # witch is simply the g score of the current cell + 1
                new_g_score = g_score[current_Cell] + 1
                new_f_score = new_g_score + heuristic(child_Cell, (1,1))

                # check if the f score is less than the previous f score
                # update the g score and the f score of the child cell
                if new_f_score < f_score[child_Cell]:
                    g_score[child_Cell] = new_g_score
                    f_score[child_Cell] = new_f_score
                    # adds the new childcell values to the PriorityQueue (open)
                    open.put((new_f_score, heuristic(child_Cell, (1,1)), child_Cell))
                    # key value as the child cell and value as the current cell
                    aStarPath[child_Cell] = current_Cell

    fwdPath = {}
    cell = (1,1)
    while cell!=start:
        fwdPath[aStarPath[cell]] = cell
        cell = aStarPath[cell]
    return fwdPath


if __name__ == '__main__':
    # size of maze
    myMaze = maze(10,15)
    myMaze.CreateMaze()
    path=aStarAlgorithm(myMaze)

    a = agent(myMaze, footprints=True)
    myMaze.tracePath({a:path})
    text = textLabel(myMaze, 'A Star Path Steps', len(path) + 1)
    
    # prints the directions 
    # #print(m.maze_map)
    
    # prints all cells
    # print(m.grid)
    
    myMaze.run()