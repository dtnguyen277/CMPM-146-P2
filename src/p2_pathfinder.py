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
    # TODO: putting too many boxes in the boxes list, only put the ones we actually explore
    boxes = {sourceBox: None}
    startBoxes = {sourceBox: None}
    endBoxes = {endBox: None}

    # initialize stuff for a simple breadth search
    # TODO: we're told not to use two queues, but like come on
    startQueue = [(0, sourceBox)]
    endQueue = [(0, endBox)]
    cost_so_far = {sourceBox: 0}

    while True:
        startCurrent = heappop(startQueue)
        endCurrent = heappop(endQueue)

        if startCurrent[1] not in boxes:
            boxes[startCurrent[1]] = startCurrent
        if endCurrent[1] not in boxes:
            boxes[endCurrent[1]] = endCurrent

        # test if hit the same box with the start and end searches
        # if we do, then we have found a path
        matching = None
        if startCurrent[1] in endBoxes:
            matching = startCurrent[1]
        if endCurrent[1] in startBoxes:
            matching = endCurrent[1]

        if matching != None:
            # we have found a path, now time to make a good line
            # TODO: this line should meet in the middle, instead of being one-way
            print("Destination Found!")

            # populate the pathBoxes list with boxes from both sides of the search
            pathBoxes = []
            this = matching
            while this != sourceBox:
                pathBoxes.insert(0, this)
                this = startBoxes[this]
            pathBoxes.insert(0, sourceBox)
            this = matching
            while this != endBox:
                this = endBoxes[this]
                pathBoxes.append(this)

            # drawing the actual line
            lastPoint = (source_point[0], source_point[1])
            for i in range(0, len(pathBoxes)-1):
                thisBox = pathBoxes[i]
                nextBox = pathBoxes[i+1]

                thisPoint = (lastPoint[0], lastPoint[1])
                thisPoint = (max(thisPoint[0], nextBox[0]), thisPoint[1])
                thisPoint = (min(thisPoint[0], nextBox[1]), thisPoint[1])
                thisPoint = (thisPoint[0], max(thisPoint[1], nextBox[2]))
                thisPoint = (thisPoint[0], min(thisPoint[1], nextBox[3]))

                line = ((lastPoint[0], lastPoint[1]), (thisPoint[0], thisPoint[1]))
                path.append(line)

                lastPoint = thisPoint
            line = ((lastPoint[0], lastPoint[1]), (destination_point[0], destination_point[1]))
            path.append(line)

            # break because now we're done with the search loop
            break

        # check all neighbors and add them to the queue
        for box in mesh['adj'][startCurrent[1]]:

            # this is just a simple middle point distance check
            xMiddle, yMiddle = (box[0] + box[1])/2, (box[2] + box[3])/2

            # add this neighbor to the queue with its distance to destination as its priority
            distance_from_dest = ((xMiddle - destination_point[0])**2 + (yMiddle - destination_point[1])**2)**0.5
            distance_from_start = ((xMiddle - source_point[0])**2 + (yMiddle - source_point[1])**2)**0.5
            distance = distance_from_start + distance_from_dest
            if box not in startBoxes:
                # mark as visited
                startBoxes[box] = startCurrent[1]

                heappush(startQueue, (distance, box))

        # check all neighbors and add them to the queue
        for box in mesh['adj'][endCurrent[1]]:
            if box not in endBoxes:
                # mark as visited
                endBoxes[box] = endCurrent[1]

                # this is just a simple middle point distance check
                xMiddle, yMiddle = (box[0] + box[1])/2, (box[2] + box[3])/2

                # add this neighbor to the queue with its distance to destination as its priority
                distance = ((xMiddle - source_point[0])**2 + (yMiddle - source_point[1])**2)**0.5
                heappush(endQueue, (distance, box))


    return path, boxes.keys()
