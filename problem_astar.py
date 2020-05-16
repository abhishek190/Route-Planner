import math

class AStarPathNode:
  
    def __init__(self, initial_costs, assumed_costs_to_destination, previous_node):
       
        self.total_costs = initial_costs
        self.assumed_costs_to_destination = assumed_costs_to_destination
        self.previous_node = previous_node

class AStarRouter:
   
    def __init__(self, map_data):        
       
        self.map_data = map_data
        self.tree = {}
        self.goal = -1
        self.frontier = set()
        
    def beeline_dist(self,start,dest):
       
        a = self.map_data.intersections[start]
        b = self.map_data.intersections[dest]
        x_diff = b[0]-a[0]
        y_diff = b[1]-a[1]
        return math.sqrt(x_diff**2 + y_diff**2)        
        
    def road_costs(self,start,dest):
      
        return self.beeline_dist(start,dest)
        
    def expand_intersection(self,start,costs):
        
        self.frontier.remove(start)
      
        for dest in self.map_data.roads[start]:
           
            road_distance = costs+self.road_costs(start,dest)
            total_assumed_distance = road_distance + self.beeline_dist(dest,self.goal)
            
            if not dest in self.tree or self.tree[dest].assumed_costs_to_destination>total_assumed_distance:
                self.tree[dest] = AStarPathNode(road_distance, total_assumed_distance, start) 
                self.frontier.add(dest)
                
    def cheapest_front_node(self):
        if len(self.frontier)==0: 
            return -1
        cheapest = next(iter(self.frontier))
        cheapest_costs = self.tree[cheapest].assumed_costs_to_destination
       
        for front_node in self.frontier:
            node = self.tree[front_node]
            if node.assumed_costs_to_destination<cheapest_costs:
                cheapest_costs = node.assumed_costs_to_destination
                cheapest = front_node
                
        return cheapest
                
    def show_front_status(self, cheapest):
        
        print("Current state:")
        for intersection in self.frontier:
            node = self.tree[intersection]
            print("{} {} {}".format(intersection, node.total_costs, node.assumed_costs_to_destination))
        print("New cheapest front node is {}".format(cheapest))
        
    def shortest_path(self, start, goal):
        if start==goal:
            return [goal]
        self.tree = {}
        self.goal = goal
        self.frontier = set([start])        
        self.tree[start] = AStarPathNode(0,self.beeline_dist(start,goal), -1)
        self.expand_intersection(start,0)
        target_reached = False
        cheapest_last = -1
        while True:
            cheapest_next = self.cheapest_front_node()
            if cheapest_next==goal:
                target_reached = True
                break
            if cheapest_next!=-1:
                node = self.tree[cheapest_next]
                self.expand_intersection(cheapest_next,node.total_costs)
            if cheapest_last==cheapest_next:
                return []
        result = []        
        if target_reached:
            cur_index = goal
            while cur_index!=-1:
                cur_node = self.tree[cur_index]
                result.append(cur_index)
                cur_index = cur_node.previous_node
            result.reverse()
        return result
def shortest_path(M,start,goal):
   
    router = AStarRouter(M)
    result = router.shortest_path(start,goal)
    return result
