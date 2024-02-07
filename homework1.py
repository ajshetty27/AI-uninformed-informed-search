from collections import deque, defaultdict
from queue import PriorityQueue
import heapq
import math


input = open("testcases//input7.txt", "r")
#read in function type
func_type  = str(input.readline()).strip()
#read in uphill energy
energy_lim  = float(input.readline())
#read in number of locations to be read into locations dictionary
loc_nums = input.readline()
#declare locations list
locations = {}
for x in range (int(loc_nums)):
    input_str = input.readline()
    split_values = input_str.split()
    loc_dic = {'location':split_values[0], 'x':float(split_values[1]), 'y':float(split_values[2]), 'z':float(split_values[3])}
    locations[split_values[0]] = loc_dic
#read in number of safe paths
path_nums = input.readline()
#declare path list
paths = []
for x in range(int(path_nums)):
    path_str = input.readline()
    split_path_str = path_str.split()
    path_dic = {'current':split_path_str[0], 'neighbour':split_path_str[1]}
    paths.append(path_dic)


#Formatting to see all neighbours for each node
graph = {}

#Create graph as adjacency matrix for neighbours
for path in paths:
    start, end = path['current'], path['neighbour']
    if start not in graph:
        graph[start] = [end]
    else:
        graph[start].append(end)
    if end not in graph:
        graph[end] = [start]
    else:
        graph[end].append(start)

#Create distance and evaluation arrays for quick distance lookup
def calculate_euclidean_distance(loc1, loc2):
    x1, y1, z1 = loc1['x'], loc1['y'], loc1['z']
    x2, y2, z2 = loc2['x'], loc2['y'], loc2['z']
    distance = abs(math.sqrt((x1 - x2)**2 + (y1 - y2)**2))
    h_distance = abs(math.sqrt((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2))
    return distance, h_distance

start = 'start'
goal = 'goal'

#BFS
def BFS(graph, start, goal, energy_lim, locations):
    # tuple with start and momentum at each state
    start_state = (start, 0)
    queue = deque([(start_state, [])])
    visited_states = set()

    while queue:
        #pop path
        curr_state, path = queue.popleft()
        #get current_node
        curr_state_dic = locations[curr_state[0]]
        z_1 = curr_state_dic['z']


        if curr_state[0] == goal:
            return path + [goal]

        if curr_state in visited_states:
            continue


        for neighbour in graph.get(curr_state[0], []):
                neighbour_loc = locations[neighbour]
                z_2 = neighbour_loc['z']
                incline = float(z_2) - float(z_1)
                if curr_state[1]+energy_lim >= incline:
                    momentum = abs(incline) if incline <=0 else 0
                    next_state = (neighbour_loc['location'], momentum)
                    queue.append((next_state, path+[curr_state[0]]))

        visited_states.add(curr_state)
    return ['FAIL']

#UCS
def UCS(graph, start, goal, energy_lim, locations):
     # node, momentum, distance
    start_state = (start, 0, 0)
    heap = [(0, start_state, [])]
    open = {}
    visited = {}
    distance = {}

    while heap:
        curr_path_len, curr_state, path = heapq.heappop(heap)

        curr_state_dic = locations[curr_state[0]]
        z_1 = curr_state_dic['z']

        if curr_state[0] == goal:
            return path + [goal], curr_path_len

        if (curr_state[0],curr_state[1]) in visited.keys():
            continue
        else:
            visited[(curr_state[0],curr_state[1])] = curr_path_len


        for neighbour in graph.get(curr_state[0], []):
            neighbour_loc = locations[neighbour]
            z_2 = neighbour_loc['z']
            incline = float(z_2) - float(z_1)
            #curr_distance = distance[(curr_state[0],neighbour)]
            if (curr_state[0],neighbour) in distance.keys():
                curr_distance = distance[(curr_state[0],neighbour)]
            else:
                curr_distance = abs(math.sqrt((curr_state_dic['x'] - neighbour_loc['x'])**2 + (curr_state_dic['y'] - neighbour_loc['y'])**2))
                distance[(curr_state[0],neighbour)] = curr_distance

            if curr_state[1] + energy_lim >= incline:
                momentum = abs(incline) if incline <= 0 else 0
                new_dist = curr_state[2] + curr_distance
                next_state = (neighbour_loc['location'], momentum, new_dist)
                next_state_check = (neighbour_loc['location'], momentum)
                #check if state is seen and if seen if it is shorter
                if next_state_check not in visited and next_state_check not in open:
                    heapq.heappush(heap, (new_dist, next_state, path+ [curr_state[0]]))
                    open[next_state_check] = next_state
                elif next_state_check in open:
                    if open[next_state_check] is not None:
                        existing_state = open[next_state_check]
                        if next_state[2] < existing_state[2] and next_state[1] < existing_state[1]:
                            del open[next_state_check]
                            heapq.heappop(heap)
                            heapq.heappush(heap, (new_dist, next_state, path + [curr_state[0]]))
                            open[next_state_check] = next_state
                        elif next_state[2] < existing_state[2]:
                            heapq.heappush(heap, (new_dist, next_state, path + [curr_state[0]]))
                            open[next_state_check] = next_state
                elif next_state_check in visited and visited[next_state_check] > new_dist:
                    del visited[next_state_check]
                    heapq.heappush(heap, (new_dist, next_state, path+ [curr_state[0]]))
                    open[next_state_check] = next_state

    return ['FAIL']

#A*
def A(graph, start, goal, energy_lim, locations):

 # node, momentum, distance
 goal_loc = locations[goal]
 start_loc = locations[start]
 start_total = abs(math.sqrt((start_loc['x'] - goal_loc['x'])**2 + (start_loc['y'] - goal_loc['y'])**2 + (start_loc['z'] - goal_loc['z'])**2))
 start_state = (start, 0, start_total, 0)
 heap = [(start_total, start_state, [])]
 open = {}
 visited = {}
 distance = {}
 h_distance = {}

 while heap:
     curr_path_len, curr_state, path = heapq.heappop(heap)

     curr_state_dic = locations[curr_state[0]]
     z_1 = curr_state_dic['z']



     if curr_state[0] == goal:

         return path + [goal], curr_state[3]

     if (curr_state[0],curr_state[1]) in visited.keys():
         continue
     else:
         visited[(curr_state[0],curr_state[1])] = curr_path_len

     for neighbour in graph.get(curr_state[0], []):
         neighbour_loc = locations[neighbour]
         z_2 = neighbour_loc['z']
         incline = float(z_2) - float(z_1)

         if (curr_state[0],neighbour) in distance.keys():
             curr_distance = distance[(curr_state[0],neighbour)]
         else:
             curr_distance = abs(math.sqrt((curr_state_dic['x'] - neighbour_loc['x'])**2 + (curr_state_dic['y'] - neighbour_loc['y'])**2 + (curr_state_dic['z'] - neighbour_loc['z'])**2))
             distance[(curr_state[0],neighbour)] = curr_distance

         if curr_state[1] + energy_lim >= incline:
             momentum = abs(incline) if incline <= 0 else 0
             new_dist = curr_state[3] + curr_distance
             g_cost = new_dist

             if (neighbour,goal) in h_distance.keys():
                 h_cost = h_distance[(neighbour,goal)]
             else:
                 h_cost = abs(math.sqrt((goal_loc['x'] - neighbour_loc['x'])**2 + (goal_loc['y'] - neighbour_loc['y'])**2 + (goal_loc['z'] - neighbour_loc['z'])**2))
                 h_distance[(neighbour,goal)] = h_cost

             f_cost = g_cost+h_cost
             next_state = (neighbour_loc['location'], momentum, f_cost, g_cost)
             next_state_check = (neighbour_loc['location'], momentum)

             #check if state is seen and if seen if it is shorter
             if next_state_check not in visited and next_state_check not in open:
                 heapq.heappush(heap, (f_cost, next_state, path+ [curr_state[0]]))
                 open[next_state_check] = next_state
             elif next_state_check in open:
                 if open[next_state_check] is not None:
                     existing_state = open[next_state_check]
                     if next_state[2] < existing_state[2] and next_state[1] < existing_state[1]:
                         del open[next_state_check]
                         heapq.heappop(heap)
                         heapq.heappush(heap, (f_cost, next_state, path + [curr_state[0]]))
                         open[next_state_check] = next_state

                     elif next_state[2] < existing_state[2]:
                         heapq.heappush(heap, (f_cost, next_state, path + [curr_state[0]]))
                         open[next_state_check] = next_state
             elif next_state_check in visited and visited[next_state_check] > f_cost:
                 del visited[next_state_check]
                 heapq.heappush(heap, (f_cost, next_state, path + [curr_state[0]]))
                 open[next_state_check] = next_state

 return ['FAIL']


output = open('output.txt','w')
if func_type == "BFS":
    safe_path = BFS(graph, start, goal, energy_lim, locations)
    print(len(safe_path)-1)
    safe_path_str = ' '.join(map(str, safe_path))
    print(safe_path_str)
    output.write(safe_path_str)
elif func_type == "UCS":
    safe_path, safe_path_len = UCS(graph, start, goal, energy_lim, locations)
    safe_path_str = ' '.join(map(str, safe_path))
    print(safe_path_str)
    print(safe_path_len)
    output.write(safe_path_str)
elif func_type =="A*":
    safe_path, safe_path_len = A(graph, start, goal, energy_lim, locations)
    safe_path_str = ' '.join(map(str, safe_path))
    print(safe_path_str)
    print(safe_path_len)
    output.write(safe_path_str)
else:
    output("Error with file to be read")
