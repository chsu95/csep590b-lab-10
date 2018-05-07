
#author1:
#author2:

from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo
import asyncio
from cozmo.util import degrees, Pose
import time

def astar(grid, heuristic):
    """Perform the A* search algorithm on a defined grid

        Arguments:
        grid -- CozGrid instance to perform search on
        heuristic -- supplied heuristic function
    """
    start = grid.getStart()
    goal = grid.getGoals()[0]

    curOpenQ = PriorityQueue()
    curOpenQ.put((0, start))
    curOpenSet = {start}
    gValues = {start: 0}
    parents = {}

    while not curOpenQ.empty():
        curNode = curOpenQ.get()[1]
        if curNode == goal:
            # Get path
            pathNode = curNode
            path = [pathNode]
            while pathNode in parents:
                pathNode = parents[pathNode]
                path.append(pathNode)
            grid.setPath(list(reversed(path)))
            break
        grid.addVisited(curNode)
        curOpenSet.remove(curNode)
        neighbors = grid.getNeighbors(curNode)
        for neighbor in neighbors:
            nbNode = neighbor[0]
            weight = neighbor[1]
            if nbNode in grid.getVisited():
                continue
            nbGValue = gValues[curNode] + weight
            if nbNode not in gValues or nbGValue < gValues[nbNode]:
                gValues[nbNode] =  nbGValue
                parents[nbNode] = curNode
            nbFValue = gValues[nbNode] + heuristic(nbNode, goal)
            if nbNode not in curOpenSet:
                curOpenSet.add(nbNode)
                curOpenQ.put((nbFValue, nbNode))

    pass


def heuristic(current, goal):
    """Heuristic function for A* algorithm

        Arguments:
        current -- current cell
        goal -- desired goal cell
    """
    return math.sqrt((current[0] - goal[0]) ** 2 + (current[1] - goal[1]) ** 2)


def cozmoBehavior(robot: cozmo.robot.Robot):
    """Cozmo search behavior. See assignment description for details

        Has global access to grid, a CozGrid instance created by the main thread, and
        stopevent, a threading.Event instance used to signal when the main thread has stopped.
        You can use stopevent.is_set() to check its status or stopevent.wait() to wait for the
        main thread to finish.

        Arguments:
        robot -- cozmo.robot.Robot instance, supplied by cozmo.run_program
    """
        
    global grid, stopevent

    # Start robot at start location
    x = grid.getStart()[0] * grid.scale
    y = grid.getStart()[1] * grid.scale
    robot.go_to_pose(Pose(x, y, 0, angle_z=degrees(0))).wait_for_completed()

    # Add dummy goal
    x = int(grid.width / 2)
    y = int(grid.height / 2)
    grid.addGoal((x, y))

    foundCubes = set()
    goalRotation = 0
    while not stopevent.is_set():
        # Clear visited
        grid.clearVisited()

        # Clear path
        grid.clearPath()

        # Spin to find cubes until all are found
        if len(foundCubes) < 3:
            for _ in range(4):
                robot.turn_in_place(degrees(90), speed=degrees(180)).wait_for_completed()
                if len(foundCubes) < 3:
                    start = time.time()
                    while time.time() - start < 1:
                        try:
                            cube = robot.world.wait_for_observed_light_cube(timeout=0.5, include_existing=False)
                            if cube and cube.cube_id not in foundCubes:
                                x, y, _ = cube.pose.position.x_y_z
                                x = int(round(x / grid.scale))
                                y = int(round(y / grid.scale))
                                foundCubes.add(cube.cube_id)
                                if cube.cube_id == 1:
                                    goalRotation = (round(cube.pose.rotation.angle_z.degrees / 90) * 90) % 360
                                    print(x, y)
                                    if goalRotation == 0:
                                        x -= 3
                                    elif goalRotation == 90:
                                        y -= 3
                                    elif goalRotation == 180:
                                        x += 3
                                    else:
                                        y += 3
                                    print(x, y)
                                    grid.clearGoals()
                                    grid.addGoal((x, y))
                                else:
                                    # Add obstacle in 3x3 around cube
                                    for i in range(-1, 2, 1):
                                        for j in range(-1, 2, 1):
                                            grid.addObstacle((x + i, y + j))
                        except asyncio.TimeoutError:
                            continue

        # Take next step
        astar(grid, heuristic)
        if len(grid.getPath()) < 2:
            if 1 in foundCubes:
                robot.turn_in_place(degrees(goalRotation), speed=degrees(180)).wait_for_completed()
                print("COMPLETE")
                stopevent.wait()
            else:
                print("CUBE 1 NOT VISIBLE")
                continue # Keep spinning until cube 1 found
        if len(foundCubes) == 3:
            index = len(grid.getPath()) - 1
        else:
            index = 1
        x = grid.getPath()[index][0] * grid.scale
        y = grid.getPath()[index][1] * grid.scale
        robot.go_to_pose(Pose(x, y, 0, angle_z=degrees(0))).wait_for_completed()

        # Update start
        x, y, _ = robot.pose.position.x_y_z
        x = int(round(x / grid.scale))
        y = int(round(y / grid.scale))
        grid.setStart((x, y))
        pass

######################## DO NOT MODIFY CODE BELOW THIS LINE ####################################


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """
        
    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        cozmo.run_program(cozmoBehavior)


# If run as executable, start RobotThread and launch visualizer with empty grid file
if __name__ == "__main__":
    global grid, stopevent
    stopevent = threading.Event()
    grid = CozGrid("emptygrid.json")
    visualizer = Visualizer(grid)
    updater = UpdateThread(visualizer)
    updater.start()
    robot = RobotThread()
    robot.start()
    visualizer.start()
    stopevent.set()

