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
    # first point of the line is the end point so we set it here
    lastPoint = (destination_point[0], destination_point[1])

    while queue:
        current = heappop(queue)

        # check if destination found
        if current[1] == endBox:
            print("Destination Found!")
            destFound = True

            current_back_node = boxes[current[1]]

            # backtrack through the boxes visited to make a line path
            while current_back_node != sourceBox:
                # to get the point for this box, take the last point
                # and constrain it to the bounds of the next box
                next_node = boxes[current_back_node]
                thisPoint = (lastPoint[0], lastPoint[1])
                thisPoint = (max(thisPoint[0], next_node[0]), thisPoint[1])
                thisPoint = (min(thisPoint[0], next_node[1]), thisPoint[1])
                thisPoint = (thisPoint[0], max(thisPoint[1], next_node[2]))
                thisPoint = (thisPoint[0], min(thisPoint[1], next_node[3]))

                # draw a line from the previous point to this point
                line = ((lastPoint[0], lastPoint[1]), (thisPoint[0], thisPoint[1]))
                path.append(line)

                # go to the next box
                lastPoint = (thisPoint[0], thisPoint[1])
                current_back_node = next_node

            # links source point to the line
            line = ((lastPoint[0], lastPoint[1]), (source_point[0], source_point[1]))
            path.append(line)
            break

        # check all neighbors and add them to the queue
        for box in mesh['adj'][current[1]]:
            if box not in boxes:
                # mark as visited
                boxes[box] = current[1]

                # this is just a simple middle point distance check
                xMiddle, yMiddle = (box[0] + box[1])/2, (box[2] + box[3])/2

                # add this neighbor to the queue with its distance to destination as its priority
                distance = ((xMiddle - destination_point[0])**2 + (yMiddle - destination_point[1])**2)**0.5
                heappush(queue, (distance, box))

    if not destFound:
        print("Destination was not found")

    return path, boxes.keys()
