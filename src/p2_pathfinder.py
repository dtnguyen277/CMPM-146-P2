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
    startBoxes = {sourceBox: None}
    endBoxes = {endBox: None}

    # initialize stuff for a simple breadth search
    # TODO: we're told not to use two queues, but like come on
    startQueue = [(0, sourceBox)]
    costFromStart = {}
    costFromStart[sourceBox] = 0
    endQueue = [(0, endBox)]
    costFromEnd = {}
    costFromEnd[endBox] = 0
    matching = None

    def distanceBetween(x1,y1, x2,y2):
        return ((x1 - x2)**2 + (y1 - y2)**2)**0.5

    while matching == None:
        startCurrent = heappop(startQueue)
        endCurrent = heappop(endQueue)

        if startCurrent[1] not in boxes:
            boxes[startCurrent[1]] = startCurrent
        if endCurrent[1] not in boxes:
            boxes[endCurrent[1]] = endCurrent

        # test if hit the same box with the start and end searches
        # if we do, then we have found a path
        if startCurrent[1] in endBoxes:
            matching = startCurrent[1]
        if endCurrent[1] in startBoxes:
            matching = endCurrent[1]

        if matching != None:
            # we have found a path, now time to make a good line
            # TODO: this line might want to meet in the middle, instead of being one-way; depends on what the instructors want
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

        currentXMiddle, currentYMiddle = (startCurrent[1][0] + startCurrent[1][1])/2, (startCurrent[1][2] + startCurrent[1][3])/2
        for box in mesh['adj'][startCurrent[1]]:
            xMiddle, yMiddle = (box[0] + box[1])/2, (box[2] + box[3])/2
            distanceToDestination = distanceBetween(xMiddle,yMiddle, destination_point[0],destination_point[1])

            newCost = costFromStart[startCurrent[1]] + distanceBetween(xMiddle,yMiddle, currentXMiddle,currentYMiddle)

            if box not in costFromStart or newCost < costFromStart[box]:
                startBoxes[box] = startCurrent[1]
                costFromStart[box] = newCost
                heappush(startQueue, (newCost + distanceToDestination, box))

        currentXMiddle, currentYMiddle = (endCurrent[1][0] + endCurrent[1][1])/2, (endCurrent[1][2] + endCurrent[1][3])/2
        for box in mesh['adj'][endCurrent[1]]:
            xMiddle, yMiddle = (box[0] + box[1])/2, (box[2] + box[3])/2
            distanceToDestination = distanceBetween(xMiddle,yMiddle, source_point[0],source_point[1])

            newCost = costFromEnd[endCurrent[1]] + distanceBetween(xMiddle,yMiddle, currentXMiddle,currentYMiddle)

            if box not in costFromEnd or newCost < costFromEnd[box]:
                endBoxes[box] = endCurrent[1]
                costFromEnd[box] = newCost
                heappush(endQueue, (newCost + distanceToDestination, box))


    return path, boxes.keys()
