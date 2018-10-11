from p1_support import load_level, show_level, save_level_costs
from math import inf, sqrt
from heapq import heappop, heappush


def dijkstras_shortest_path(initial_position, destination, graph, adj):
    """ Searches for a minimal cost path through a graph using Dijkstra's algorithm.

    Args:
        initial_position: The initial cell from which the path extends.
        destination: The end location for the path.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        If a path exits, return a list containing all cells from initial_position to destination.
        Otherwise, return None.

    """

    queue = []                                      # essentially handles visited and unvisited positions
    heappush(queue, (0, initial_position))          # add the starting position to the queue
    shortest_distance = {}                          # updated list of shortest paths to position
    previous_position = {}                          # updated list of parent position of position

    shortest_distance[initial_position] = 0         # initial position is cost 0 from itself
    previous_position[initial_position] = None      # initial position is the root

    while queue:
        # fetch the position with the shortest distance
        current_distance, current_position = heappop(queue)

        # exit loop if destination found
        if current_position == destination:
            shortest_path = []
            index_path = destination
            # build the found shortest_path
            while index_path != None:
                shortest_path.insert(0, index_path) # initial position should be at beginning of list
                index_path = previous_position[index_path]
            return shortest_path

        # iterate through visitable cells
        for next_distance, next_direction in navigation_edges(graph, current_position):
            new_distance = shortest_distance[current_position] + next_distance  # caluculate new potentially shortest distance
            # enter when next position has not been visited or new distance is less than current position's distance 
            if next_direction not in shortest_distance or new_distance < shortest_distance[next_direction]:
                shortest_distance[next_direction] = new_distance        # update the distance
                previous_position[next_direction] = current_position    # update the trail
                heappush(queue, (new_distance, next_direction))         # add updated waypoint to queue

    return None


def dijkstras_shortest_path_to_all(initial_position, graph, adj):
    """ Calculates the minimum cost to every reachable cell in a graph from the initial_position.

    Args:
        initial_position: The initial cell from which the path extends.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        A dictionary, mapping destination cells to the cost of a path from the initial_position.
    """

    queue = []
    heappush(queue, (0, initial_position))
    shortest_distance = {}
    previous_position = {}

    shortest_distance[initial_position] = 0
    previous_position[initial_position] = None

    while queue:
        current_distance, current_position = heappop(queue)

        for next_distance, next_position in navigation_edges(graph, current_position):
            new_distance = shortest_distance[current_position] + next_distance
            if next_position not in shortest_distance or new_distance < shortest_distance[next_position]:
                shortest_distance[next_position] = new_distance
                previous_position[next_position] = current_position
                heappush(queue, (new_distance, next_position))
    return shortest_distance

def navigation_edges(level, cell):
    """ Provides a list of adjacent cells and their respective costs from the given cell.

    Args:
        level: A loaded level, containing walls, spaces, and waypoints.
        cell: A target location.

    Returns:
        A list of tuples containing an adjacent cell's coordinates and the cost of the edge joining it and the
        originating cell.

        E.g. from (0,0):
            [((0,1), 1),
             ((1,0), 1),
             ((1,1), 1.4142135623730951),
             ... ]
    """

    adjacentCells = []

    x = cell[0]
    y = cell[1]

    # default cell value to 1 for waypoints
    cellValue = 1
    
    if (x,y) in level['spaces']:
        # give proper values to space cells
        cellValue = level['spaces'][(x,y)]

    # vertical and horizontal directions
    normal_directions = [(x, y-1), (x+1, y), (x, y+1), (x-1, y)]
    # diagonal directions
    diagonal_directions = [(x+1, y-1), (x+1, y+1), (x-1, y+1), (x-1, y-1)]

    # make proper calculations for valid directions
    for direction in normal_directions:
        # accounts for walls and cells out of bounds
        if direction in level['spaces'] or direction in level['waypoints']:
            distance = 0.5*cellValue + 0.5*level['spaces'][direction]
            adjacentCells.append((distance, direction))

    for direction in diagonal_directions:
        if direction in level['spaces'] or direction in level['waypoints']:
            distance = 0.5*sqrt(2)*cellValue + 0.5*sqrt(2)*level['spaces'][direction]
            adjacentCells.append((distance, direction))

    return adjacentCells

def test_route(filename, src_waypoint, dst_waypoint):
    """ Loads a level, searches for a path between the given waypoints, and displays the result.

    Args:
        filename: The name of the text file containing the level.
        src_waypoint: The character associated with the initial waypoint.
        dst_waypoint: The character associated with the destination waypoint.

    """

    # Load and display the level.
    level = load_level(filename)
    show_level(level)

    # Retrieve the source and destination coordinates from the level.
    src = level['waypoints'][src_waypoint]
    dst = level['waypoints'][dst_waypoint]

    # Search for and display the path from src to dst.
    path = dijkstras_shortest_path(src, dst, level, navigation_edges)
    if path:
        show_level(level, path)
    else:
        print("No path possible!")


def cost_to_all_cells(filename, src_waypoint, output_filename):
    """ Loads a level, calculates the cost to all reachable cells from 
    src_waypoint, then saves the result in a csv file with name output_filename.

    Args:
        filename: The name of the text file containing the level.
        src_waypoint: The character associated with the initial waypoint.
        output_filename: The filename for the output csv file.

    """
    
    # Load and display the level.
    level = load_level(filename)
    show_level(level)

    # Retrieve the source coordinates from the level.
    src = level['waypoints'][src_waypoint]
    
    # Calculate the cost to all reachable cells from src and save to a csv file.
    costs_to_all_cells = dijkstras_shortest_path_to_all(src, level, navigation_edges)
    save_level_costs(level, costs_to_all_cells, output_filename)


if __name__ == '__main__':
    filename, src_waypoint, dst_waypoint = 'my_maze.txt', 'a','d'

    # Use this function call to find the route between two waypoints.
    test_route(filename, src_waypoint, dst_waypoint)

    # Use this function to calculate the cost to all reachable cells from an origin point.
    cost_to_all_cells(filename, src_waypoint, 'my_costs.csv')
