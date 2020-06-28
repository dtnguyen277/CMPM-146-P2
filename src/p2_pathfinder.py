from heapq import heappop, heappush
def find_path(source_point, destination_point, mesh):
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

    # find which box the source_point is in
    sourceBox = None
    sx = source_point[0]
    sy = source_point[1]
    for box in mesh['boxes']:
        x1,x2,y1,y2 = box[0],box[1],box[2],box[3]
        if sx >= x1 and sx < x2 and sy >= y1 and sy < y2:
            sourceBox = box

    # find which box the destination_point is in
    endBox = None
    sx = destination_point[0]
    sy = destination_point[1]
    for box in mesh['boxes']:
        x1,x2,y1,y2 = box[0],box[1],box[2],box[3]
        if sx >= x1 and sx < x2 and sy >= y1 and sy < y2:
            endBox = box

    # initialize the variables that will be returned by this function
    path = []
    boxes = {sourceBox: None}

    # initialize stuff for a simple breadth search
    queue = [(0, sourceBox)]
    cost_so_far = {sourceBox: 0}
    destFound = False

    while queue:
        current = heappop(queue)

        # check if destination found
        if current[1] == endBox:
            print("Destination Found!")
            destFound = True
            break

        # check all neighbors and add them to the queue
        for box in mesh['adj'][current[1]]:
            if box not in boxes:
                # mark as visited
                boxes[box] = current

                # TODO: fix this stupid stuff
                # this is a simple middle point distance check, just as a first pass
                xMiddle, yMiddle = (box[0] + box[1])/2, (box[2] + box[3])/2
                xMiddleLast, yMiddleLast = (current[1][0] + current[1][1])/2, (current[1][2] + current[1][3])/2

                # make the line from the middle points of this rectangle and the next
                line = ((xMiddle, yMiddle), (xMiddleLast, yMiddleLast))
                path.append(line)

                # add this neighbor to the queue with its distance to destination as its priority
                distance = ((xMiddle - destination_point[0])**2 + (yMiddle - destination_point[1])**2)**0.5
                heappush(queue, (distance, box))

    if not destFound:
        print("Destination was not found")

    return path, boxes.keys()
