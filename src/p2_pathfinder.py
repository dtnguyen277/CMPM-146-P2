from heapq import heappop, heappush
def find_path(source_point, destination_point, mesh):
    print(source_point, destination_point)

    sourceBox = None
    sx = source_point[0]
    sy = source_point[1]
    for box in mesh['boxes']:
        x1,x2,y1,y2 = box[0],box[1],box[2],box[3]
        if sx >= x1 and sx < x2 and sy >= y1 and sy < y2:
            sourceBox = box
            #print(x1,x2,y1,y2, "{", sx,sy, "}")

    endBox = None
    sx = destination_point[0]
    sy = destination_point[1]
    for box in mesh['boxes']:
        x1,x2,y1,y2 = box[0],box[1],box[2],box[3]
        if sx >= x1 and sx < x2 and sy >= y1 and sy < y2:
            endBox = box
            #print(x1,x2,y1,y2, "{", sx,sy, "}")

    for b in mesh['boxes']:
        print(b)


    """
    Searches for a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:

        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """

    path = []
    boxes = {sourceBox: sourceBox, endBox: endBox}


    #breadth search
    queue = [(0, sourceBox)]
    visited = {sourceBox: None}
    cost_so_far = {sourceBox: 0}
    destFound = False

    while queue:
        current = heappop(queue)
        #print(str(current[1]))
        if current[1] == endBox:
            print("Destination Found!")
            destFound = True
            break
        for box in mesh['adj'][current[1]]:
            if box not in visited:
                visited[box] = current
                #temp line method
                line = ((box[0], box[2]), (box[1], box[3]))
                path.append(line)
                heappush(queue, (0, box))

    if not destFound:
        print("Destination was not found")

    return path, boxes.keys()
