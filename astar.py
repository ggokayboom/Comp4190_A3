from pathplanning import Rectangle
from queue import PriorityQueue

class AStar(object):
    def __init__(self,domain,start,goal):
        self.root = domain.root # entire board 
        self.free_nodes = domain.free_nodes  # list of free nodes
        self.start = self.getNode(self.root, start)  # get node that start rectangle is inside
        self.start_point = start    # start rectangle
        self.goal_point = goal  # goal rectangle
        self.goal = self.getNode(self.root, goal)    # get node that goal rectangle is inside   
        self.path_to_goal = self.runAStar(self.start)

    # return node target_rec belongs to
    def getNode(self, start_node, target_rec):
        for node in self.free_nodes:
            if node[0].CalculateOverlap(target_rec) > 0:
                return node
        return None
    
    # get adjacent nodes to node (from free list)
    def getNeighbors(self, node):
        neighbors = [] 
        for free_node in self.free_nodes:
            if node != free_node and node[0].IsNeighbor(free_node[0]):
                neighbors.append(free_node)
        return neighbors

    # calculate distance from start to dest (manhattan)
    def getDistance(self,start,dest):
        start_mid_x = (start[0].x + (start[0].x + start[0].width))/2
        start_mid_y = (start[0].y + (start[0].y + start[0].height))/2
        dest_mid_x = (dest[0].x + (dest[0].x + dest[0].width))/2
        dest_mid_y = (dest[0].y + (dest[0].y + dest[0].height))/2
        return abs(dest_mid_x - start_mid_x) + abs(dest_mid_y - start_mid_y)
    
    # returns list of coordinates from node to its root ancestor
    def getPath(self,node):
        path = []
        curr = node
        if curr:
            prev = ((curr[0].x + (curr[0].x + curr[0].width))/2,(curr[0].y + (curr[0].y + curr[0].height))/2)
            while curr:
                mid_x = (curr[0].x + (curr[0].x + curr[0].width))/2
                mid_y = (curr[0].y + (curr[0].y + curr[0].height))/2
                distance = abs(mid_x - prev[0]) + abs(mid_y - prev[1])
                path.append((mid_x,mid_y,distance))
                prev = (mid_x,mid_y)
                curr = curr[-1]
        
        # if len(path) > 1:
        #     path.pop(0)
        #     path.pop()

        path.insert(0,(self.goal_point.x,self.goal_point.y,0))
        path.append((self.start_point.x,self.start_point.y,0))
        return path

    # checks to see if target_node exists inside node_list
    def exists(self, target_node, node_list):
        for _,_,node in node_list:
            if target_node[0].x == node[0].x and target_node[0].y == node[0].y:
                return True
        return False

    # find least cost path from self.start to self.goal and returns it
    def runAStar(self,root):
        open = PriorityQueue()
        closed = []
        root += [0] + [0] + [0] # g, h, f 
        root.append(None)       # parent
        open.put((0,id(root),root)) 

        while open.queue:
            curr_f,node_id, curr_node = open.get()  # get lowest cost node from open list
            closed.append((curr_f,node_id,curr_node))   # add to closed list

            # if curr node is goal node 
            if curr_node[0].x == self.goal[0].x and curr_node[0].y == self.goal[0].y:
                return self.getPath(self.goal)
            
            neighbors = self.getNeighbors(curr_node)    # get curr_nodes adjacent nodes

            for neighbor in neighbors:
                
                # if neighbor exists in closed list
                if self.exists(neighbor,closed):
                    continue
                
                # calc g, h, f scores for curr neighbor
                g = curr_node[3] + self.getDistance(curr_node,neighbor)
                h = self.getDistance(neighbor, self.goal)
                f = g + h

                # if curr neighbor exists in open list and its g value is more then update
                if self.exists(neighbor,open.queue):
                    if neighbor[3] > g:
                        neighbor[3] = g
                        neighbor[4] = h
                        neighbor[5] = f
                        neighbor[6] = curr_node
                # add curr neighbor to open list if doesn't exist
                else:
                    neighbor += [g] + [h] + [f]
                    neighbor.append(curr_node)
                    open.put((f,id(neighbor),neighbor))
        
        # return empty list if no path found
        return []
