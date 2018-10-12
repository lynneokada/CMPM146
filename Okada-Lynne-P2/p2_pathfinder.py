from math import inf, sqrtfrom heapq import heappop, heappushdef find_path (source_point, destination_point, mesh):    """    Searches for a path from source_point to destination_point through the mesh    Args:        source_point: starting point of the pathfinder        destination_point: the ultimate goal the pathfinder must reach        mesh: pathway constraints the path adheres to    Returns:        A path (list of points) from source_point to destination_point if exists        A list of boxes explored by the algorithm    """    all_boxes = mesh['boxes']    print("source_point ",source_point)    print("destination_point ", destination_point)    def inBox(point):        px = point[1]        py = point[0]        # find the box the source_point lies in        print(point)        return_box = (0,0,0,0)        for box in all_boxes:            if px >= box[0] and px <= box[1] and py >= box[2] and py <= box[3]:                print("px box[0]: ", px, box[0])                print("px box[1]: ", px, box[1])                print("py box[2]: ", py, box[2])                print("py box[3]: ", py, box[3])                return_box = box                # print("source_box: ", source_box)        return return_box    source_box = inBox(source_point)    destination_box = inBox(destination_point)    print("source_box ", source_box)    print("destination_box ", destination_box)    boxes = {}    if source_box == destination_box:        print("same box")        path = [(source_point,destination_point)]        boxes[source_box] = None        return path, boxes.keys()    # print("source_point: ", source_point)    # print("destination_point: ", destination_point)    # print("source_box: ", source_box)    # print("destination_box: ", destination_box)    path_points, boxes = a_star(source_box, destination_box, source_point, destination_point, mesh)    path =[]    for i in range(0,len(path_points)-1):        path.append((path_points[i],path_points[i+1]))        i = i + 1    print(path)    return path, boxes.keys()# Euclidean distance formuladef euclidean(point_a, point_b):    return sqrt((point_a[0] - point_b[0])**2 + (point_a[1] - point_b[1])**2)def a_star(initial_box, destination_box, source_point, destination_point, mesh):    """ Searches for a minimal cost path through a graph using Dijkstra's algorithm.    Args:        initial_position: The initial cell from which the path extends.        destination: The end location for the path.        graph: A loaded level, containing walls, spaces, and waypoints.        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.    Returns:        If a path exits, return a list containing all cells from initial_position to destination.        Otherwise, return None.    """    # The priority queue    queue = [(0, initial_box)]    detail_points = {}    detail_points[initial_box] = source_point    # The dictionary that will be returned with the costs    estimated_distances = {}    estimated_distances[initial_box] = euclidean(source_point, destination_point)    distances = {}    distances[initial_box] = 0    # The dictionary that will store the backpointers    backpointers = {}    backpointers[initial_box] = None    while queue:        current_dist, current_box = heappop(queue)        # Check if current node is the destination        if current_box == destination_box:            print("destination found")            # List containing all points from initial_position to destination            path = []            path.append(destination_point)            # Go backwards from destination until the source using backpointers            # and add all the nodes in the shortest path into a list            current_back_box = current_box            while current_back_box is not None:                path.append(detail_points[current_back_box])                current_back_box = backpointers[current_back_box]            # path.append(source_point)            for p in path:                print(p)            return (path[::-1], backpointers)        # Calculate cost from current box to all the adjacent ones        for adj_box, adj_point, adj_box_cost in navigation_edges(mesh, detail_points[current_box], current_box):            # print(detail_points[current_box])            pathcost = distances[current_box] + adj_box_cost            pathcost_estimated = pathcost + euclidean((adj_point),(destination_point))            # If the cost is new            if adj_box not in distances or pathcost_estimated < distances[adj_box]:                estimated_distances[adj_box] = pathcost_estimated                distances[adj_box] = pathcost                backpointers[adj_box] = current_box                detail_points[adj_box] = adj_point                heappush(queue, (pathcost_estimated, adj_box))                # print(backpointers)    return Nonedef navigation_edges(mesh, source_point, source_box):    """ Provides a list of adjacent cells and their respective costs from the given cell.    Args:        level: A loaded level, containing walls, spaces, and waypoints.        cell: A target location.    Returns:        A list of tuples containing an adjacent cell's coordinates and the cost of the edge joining it and the        originating cell.        E.g. from (0,0):            [((0,1), 1),             ((1,0), 1),             ((1,1), 1.4142135623730951),             ... ]    """    adj_result = []    all_boxes = mesh['boxes']    adj_boxes = mesh['adj']    px = source_point[1]    py = source_point[0]    # print("source_point: ", source_point)    # def inBox(source_point):    #     source_box = (0,0,0,0)    #     # find the box the source_point lies in    #     for box in all_boxes:    #         if px >= box[0] and px <= box[1] and py >= box[2] and py <= box[3]:    #             source_box = box    #             # print("source_box: ", source_box)    #     return source_box    # source_box = inBox(source_point)    for adj_box in adj_boxes[source_box]:        # valid x axis for source_box x and adj_box x        x_range = [max(source_box[0], adj_box[0]), min(source_box[1], adj_box[1])]        # valid y axis for source_box y and adj_box y        y_range = [max(source_box[2], adj_box[2]), min(source_box[3], adj_box[3])]        # print("current adj_box: ", adj_box)        # print("x_range: ", x_range)        # print("y_range: ", y_range)        if px > x_range[0] and px < x_range[1]:            # print("within x_range")            distance = min(euclidean((px,py),(px,y_range[0])),euclidean((px,py),(px,y_range[1])))            if(py < y_range[0]):                detail_point = (px, y_range[0])            else:                detail_point = (px, y_range[1])            adj_result.append((adj_box, detail_point, distance))        elif py > y_range[0] and py < y_range[1]:            # print("within y_range")            # print("py ", py)            distance = min(euclidean((px,py),(x_range[0],py)),euclidean((px,py),(x_range[1],py)))            if(px < x_range[0]):                detail_point = (x_range[0], py)            else:                detail_point = (x_range[1], py)            adj_result.append((adj_box, detail_point, distance))        else:            # print("outside of range")            distance = min(euclidean((px,py),(x_range[0],y_range[0])), euclidean((px,py),(x_range[1],y_range[1])))            # print("px,py -> x_range[0],y_range[0]:",euclidean((px,py),(x_range[0],y_range[0])))            # print("px,py -> x_range[1],y_range[1]:",euclidean((px,py),(x_range[1],y_range[1])))            if px < x_range[0]:                detail_x = x_range[0]            else:                detail_x = x_range[1]            if py < y_range[0]:                detail_y = y_range[0]            else:                detail_y = y_range[1]            detail_point = (detail_x, detail_y)            adj_result.append((adj_box, detail_point, distance))    # for i in adj_result:    #     print(i)    return adj_result