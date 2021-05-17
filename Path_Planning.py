#GOAL: the goal of this project is to compare different path planning algorithms
#Two fo the algorithms are popular in robot navigation (1. Potential Field 2. Dynamic Window Approach)
#The other two are grid searches for most optimal path from two distances (1. A* Algorithm 2. Dijkstra Algorithm)

import pygame
import sys
import time
from queue import PriorityQueue

#Define the color constants
BLACK = (0, 0, 0)
GRAY = (127, 127, 127)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
CYAN = (0, 255, 255)
MAGENTA = (255, 0, 255)

#Create class to hold the node attributes
class Node():
    #Initialization function once a node class is created. Each node will be initialized with its own characteristics shown below
    def __init__(self, row, col, size):
        self.row = row
        self.col = col
        self.x = row*size
        self.y = col*size
        self.color = WHITE
        self.neighbor = []
        self.size = size
        self.source = False
        self.destination = False
    
    def get_pos(self): #Return the row and column of the node
        return self.row, self.col
    def set_source(self): #Set the node as the source/start color (ie color blue)
        self.color = BLUE
    def set_destination(self): #Set the node as the destination/target color (ie color Red)
        self.color = RED
    def set_barrier(self): #Set the node as a barrier color (ie color black)
        self.color = BLACK
    def set_path(self): #Set the node to a shortest path node (ie color Cyan)
        self.color = CYAN
    def is_source(self): #Set the node as the source/start node
        self.source = True
    def is_destination(self): #Set the node as the destination/target node
        self.destination = True
    def is_barrier(self): #Return true if the node is a barrier, the algorithm must not use this node as a means to a short path
        return self.color == BLACK
    def visited(self): #Set the node as visited color (ie yellow), means the node has been checked by the algorithm
        self.color = YELLOW
    def was_visited(self): #Return true if the node has been visited
        return self.color == YELLOW
    def draw(self, scn): #Draw the node on the grid with the appropriate color and size, send in the x and y position and then the width is the size
        pygame.draw.rect(scn, self.color, pygame.Rect(self.x, self.y, self.size, self.size))
        
# Set the size of our grid (just need width cause we want it to be square)
screen_width = 750

#Initialize the pygame window
pygame.init()
screen = pygame.display.set_mode((screen_width, screen_width))
pygame.display.set_caption('Path Grid')
screen.fill(WHITE)

#This is the main loop that continuously updates the grid pop up 
def grid_loop(rows, cols):
    #Need a while loop to keep the window open
    nodes = make_nodes(rows, cols, screen_width)
    #Get the size (ie height x width) of each square in the grid
    size = screen_width // rows
    #Initialize the number of mouse clicks
    mouseclick = 0
    while True:
        drawGrid(nodes, rows, cols) #Draw the grid and nodes in the grid
        for event in pygame.event.get(): #If the exit button is pressed end the program
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if pygame.mouse.get_pressed() == (1, 0, 0): #If the left mouse click is clicked have to decide if its source, destination, or barrier node
                mousepos = pygame.mouse.get_pos() #Get the posistion of the mouse on the grid
                i, j = mousepos
                #Return which square/node was clicked
                i = i // size  
                j = j // size
                node = nodes[i][j]
                if mouseclick == 0: #If its the first mouse click then we set the clicked node to source
                    node.is_source()
                    node.set_source()
                    start = node
                    mouseclick = 1
                elif mouseclick == 1: #If its the second mouse click then we set the clicked node to destination
                    node.is_destination()
                    node.set_destination()
                    end = node
                    mouseclick = 2
                else: #If its the third or more mouse click then we set the clicked node to barrier
                    node.set_barrier()
            elif pygame.mouse.get_pressed() == (0, 0, 1): #If the right mouse button is clicked start the algorithm
                #dijkstra(nodes, start, end)
                Astar(nodes, start, end)
                

#Function to draw the grid (ie horizontal and vertical lines)
def draw_gridlines(rows, cols):
    gap = screen_width // rows
    #Print horizontal lines
    for i in range(rows):
        pygame.draw.line(screen, BLACK, (0, i*gap), (screen_width, i*gap))
        #Print vertical lines
        for j in range(cols):
            pygame.draw.line(screen, BLACK, (j*gap, 0), (j*gap, screen_width))

#Function to create the nodes on the grid
def make_nodes(rows, cols, screenwidth):
    grid = [[]]
    gap = screenwidth // rows
    for i in range(rows):
        grid.append([])
        for j in range(cols):
            node = Node(i, j, gap)
            grid[i].append(node)
    return grid

#Function to actually draw and update the grid with the correct node colors
def drawGrid(grid, rows, cols):
    for row in grid:
        for node in row:
            node.draw(screen)
    
    draw_gridlines(rows, cols)
    pygame.display.update()

#What it needs to do: 
# - Start at the start node and initialize a distance of 0
# - Initialize all other nodes in the grid to have a distance of infinity
# - Take the neighbor of the start node and update the distance to current distance plus 1
# - Repeat this for the each neighbor node
def dijkstra(grid, start, end):
    #Get the row and col of the start node
    start_row, start_col = start.get_pos()
    print('Start Row: ', start_row, 'Start Col: ', start_col)
    # Get the row and col of the end node
    end_row, end_col = end.get_pos()
    #Set infinity to very high number
    inf = 1000000000
    #Make an array/list that has infinity for all the distances
    distance = []
    tempdist = [] #Temp dist is used to find the index of the minimum distance, this is done by setting the current row and column to inf because technically 
    #the current index is actually a minimum so we want to find the next lowest minimum
    
    #Initialize all the distances in the distance and tempdist lists to inf
    for i in range(len(grid) - 1):
        distance.append([])
        tempdist.append([])
        for j in range(len(grid) - 1):
            distance[i].append(inf)
            tempdist[i].append(inf)
    
    #Get the position of the start node and make the distance equal to 0
    distance[start_row][start_col] = 0
    tempdist[start_row][start_col] = 0

    #Create a list for the visited nodes (holds False for node index that hasnt been visited)
    visited = []
    for i in range(len(grid) - 1):
        visited.append([])
        for j in range(len(grid) - 1):
            visited[i].append((False))

    #Initialize the starting node to be visited
    visited[start_row][start_col] = True

    #Set the current node to the starting node
    current_node = start
    #Initialize a finish variable to False and feed into while loop until finished variable is set to True when the shortest path is found
    #Essentially every new node visited (whether above, below, or to the sides) will get a distance of the current distance at that node plus 1
    finished = False
    while not finished:
        #Get the current nodes row and column
        current_row, current_col = current_node.get_pos()
        if current_col > 0: #Check/update the distance of the up node
            if distance[current_row][current_col - 1] > distance[current_row][current_col] + 1 and not visited[current_row][current_col - 1] and not grid[current_row][current_col - 1].is_barrier():
                distance[current_row][current_col - 1] = distance[current_row][current_col] + 1
                tempdist[current_row][current_col - 1] = distance[current_row][current_col] + 1
                #Make the node color change to Yellow to indicate distance has been updated
                grid[current_row][current_col - 1].visited()
                #Redraw/update the grid to display updated distances
                drawGrid(grid, 25, 25)
        if current_col < 24: #Check/update the distance of the down node
            if distance[current_row][current_col + 1] > distance[current_row][current_col] + 1 and not visited[current_row][current_col + 1] and not grid[current_row][current_col + 1].is_barrier():
                distance[current_row][current_col + 1] = distance[current_row][current_col] + 1
                tempdist[current_row][current_col + 1] = distance[current_row][current_col] + 1
                grid[current_row][current_col + 1].visited()
                drawGrid(grid, 25, 25)
        if current_row < 24: #Check/update the distance of the right node
            if distance[current_row + 1][current_col] > distance[current_row][current_col] + 1 and not visited[current_row + 1][current_col] and not grid[current_row + 1][current_col].is_barrier():
                distance[current_row + 1][current_col] = distance[current_row][current_col] + 1
                tempdist[current_row + 1][current_col] = distance[current_row][current_col] + 1
                grid[current_row + 1][current_col].visited()
                drawGrid(grid, 25, 25)
        if current_row > 0: #Check/update the distance of the left node
            if distance[current_row - 1][current_col] > distance[current_row][current_col] + 1 and not visited[current_row - 1][current_col] and not grid[current_row - 1][current_col].is_barrier():
                distance[current_row - 1][current_col] = distance[current_row][current_col] + 1
                tempdist[current_row - 1][current_col] = distance[current_row][current_col] + 1
                grid[current_row - 1][current_col].visited()
                drawGrid(grid, 25, 25)

        #Update the visited list to True for the current node index
        visited[current_row][current_col] = True
        #Now have to check which node should be visited next. It should be the node with the smallest distance. Need a temporary distance list that will set the
        #current node location to infinity and then look in that list for the index with the smallest value
        tempdist[current_row][current_col] = inf
        #Initialize the min_val to inf so we know we wont be stuck in the loop or the loop outputs a min_val that was for a previous node analyzed
        min_val = inf
        for i in range(len(grid) - 1):
            for j in range(len(grid) - 1):
                if tempdist[i][j] < min_val:
                    min_val = tempdist[i][j]
                    current_row = i
                    current_col = j
        #Make the new current node equal to the node that has a min_val at the index obtained from the for loop above
        current_node = grid[current_row][current_col]

        #If the next node to be analyzed is actually the target/destination node we must proceed to finding and displaying the shortest path
        #To find the shortest path we have to back propagate through the distance array and check the distance for every top, bottom, right, and left node 
        # and see which one of the four has the lowest distance. That will be the node of the shortest path. Repeat this until we end up at our starting node
        if current_row == end_row and current_col == end_col:
            print('Displaying shortest path')
            #Set the target/destination node to the color of Cyan (ie the shortest path color)
            current_node.set_path()
            #Initialize the min_val to inf so we know once we start we'll get a min_val
            min_val = inf
            while distance[current_row][current_col] != 0:
                #node_switch holds the direction of which node should be the next shortest path. Initialized to be 'UP'
                node_switch = 'UP'
                if current_col > 0 and distance[current_row][current_col - 1] < min_val and not grid[current_row][current_col - 1].is_barrier(): #Check up node distance
                    node_switch = 'UP'
                elif current_col < 24 and distance[current_row][current_col + 1] < min_val and not grid[current_row][current_col + 1].is_barrier(): #Check down node distance
                    node_switch = 'DOWN'
                elif current_row < 24 and distance[current_row + 1][current_col] < min_val and not grid[current_row + 1][current_col].is_barrier(): #Check right node distance
                    node_switch = 'RIGHT'
                elif current_row > 0 and distance[current_row - 1][current_col] < min_val and not grid[current_row - 1][current_col].is_barrier(): #Check left node distance
                    node_switch = 'LEFT'
                #Based on which direction the next shortest path node should be, we will update the new current node indeces, update the min_val, and set the new current node to the next shortest path node
                #When we go back to the top of the while loop however, min_val will not be reset to inf but to the lowest previous value since at least one node surrounding
                #the current shortest path node will be one less the current nodes distance 
                if node_switch == 'UP':
                    current_row, current_col = current_row, current_col - 1
                    min_val = distance[current_row][current_col]
                    current_node = grid[current_row][current_col]
                elif node_switch == 'DOWN':
                    current_row, current_col = current_row, current_col + 1
                    min_val = distance[current_row][current_col]
                    current_node = grid[current_row][current_col]
                elif node_switch == 'RIGHT':
                    current_row, current_col = current_row + 1, current_col
                    min_val = distance[current_row][current_col]
                    current_node = grid[current_row][current_col]
                elif node_switch == 'LEFT':
                    current_row, current_col = current_row - 1, current_col
                    min_val = distance[current_row][current_col]
                    current_node = grid[current_row][current_col]

                #Make the color of the new shortest path node to Cyan
                current_node.set_path()
                #Update the grid with the colored nodes
                drawGrid(grid, 25, 25)
            #Once the shortest path node is at the starting node we exit the shortest path generating while loop and the algorithm while loop
            print('Is finished')
            finished = True
        #Time delay the algorithm loop to 0.02 seconds in order to better visualize each node being checked
        time.sleep(0.02)

#Now distance is equal to g(n) = cost to next node which will always just be 1, and h(n) = heuristic function which in this case will be the Manhatton 
#distance/cost from the current node to the destination node

#Function to calculate the hueristic function of the node
def heuristic(current, end):
    x1, y1 = current.get_pos()
    x2, y2 = end.get_pos()
    return abs(x2 - x1) + abs(y2 - y1)

def Astar(grid, start, end):
    #Initialize an open set which is a priority queue
    open_set = PriorityQueue()
    count = 0
    #Get the row and col of the start node
    start_row, start_col = start.get_pos()
    print('Start Row: ', start_row, 'Start Col: ', start_col)
    # Get the row and col of the end node
    end_row, end_col = end.get_pos()
    #Set infinity to very high number
    inf = 1000000000
    #Make an array/list that has infinity for all the distances
    g_dist = []
    f_dist = []
    tempg_dist = [] #Temp dist is used to find the index of the minimum distance, this is done by setting the current row and column to inf because technically 
    #the current index is actually a minimum so we want to find the next lowest minimum
    
    #Initialize all the distances in the distance and tempdist lists to inf
    for i in range(len(grid) - 1):
        g_dist.append([])
        tempg_dist.append([])
        f_dist.append([])
        for j in range(len(grid) - 1):
            g_dist[i].append(0)
            tempg_dist[i].append(inf)
            f_dist[i].append(inf)
    
    #Get the position of the start node and make the distance equal to 0
    g_dist[start_row][start_col] = 0
    tempg_dist[start_row][start_col] = 0
    initial_dist = heuristic(start, end)
    f_dist[start_row][start_col] = initial_dist
    open_set.put((f_dist[start_row][start_col], count, start))

    #Create a list for the visited nodes (holds False for node index that hasnt been visited)
    visited = []
    for i in range(len(grid) - 1):
        visited.append([])
        for j in range(len(grid) - 1):
            visited[i].append((False))

    #Initialize the starting node to be visited
    visited[start_row][start_col] = True

    #Set the current node to the starting node
    current_node = start
    #Initialize a finish variable to False and feed into while loop until finished variable is set to True when the shortest path is found
    #Essentially every new node visited (whether above, below, or to the sides) will get a distance of the current distance at that node plus 1
    finished = False
    
    while not finished:
        #Get the current nodes row and column
        current_row, current_col = current_node.get_pos()
        #Check to see if the current new node is actually the finished node
        print('New Node Row: ', current_row, 'New Node Col: ', current_col)
        if current_row == end_row and current_col == end_col:
            print('Displaying shortest path')
            #Set the target/destination node to the color of Cyan (ie the shortest path color)
            current_node.set_path()
            #Initialize the min_val to inf so we know once we start we'll get a min_val
            min_val = inf
            while f_dist[current_row][current_col] != initial_dist:
                print('Distance is: ', f_dist[current_row][current_col])
                #node_switch holds the direction of which node should be the next shortest path. Initialized to be 'UP'
                node_switch = 'UP'
                if current_col > 0 and f_dist[current_row][current_col - 1] < min_val and not grid[current_row][current_col - 1].is_barrier(): #Check up node distance
                    node_switch = 'UP'
                elif current_col < 24 and f_dist[current_row][current_col + 1] < min_val and not grid[current_row][current_col + 1].is_barrier(): #Check down node distance
                    node_switch = 'DOWN'
                elif current_row < 24 and f_dist[current_row + 1][current_col] < min_val and not grid[current_row + 1][current_col].is_barrier(): #Check right node distance
                    node_switch = 'RIGHT'
                elif current_row > 0 and f_dist[current_row - 1][current_col] < min_val and not grid[current_row - 1][current_col].is_barrier(): #Check left node distance
                    node_switch = 'LEFT'
                #Based on which direction the next shortest path node should be, we will update the new current node indeces, update the min_val, and set the new current node to the next shortest path node
                #When we go back to the top of the while loop however, min_val will not be reset to inf but to the lowest previous value since at least one node surrounding
                #the current shortest path node will be one less the current nodes distance 
                print('Node is: ', node_switch)
                if node_switch == 'UP':
                    current_row, current_col = current_row, current_col - 1
                    min_val = f_dist[current_row][current_col]
                    current_node = grid[current_row][current_col]
                elif node_switch == 'DOWN':
                    current_row, current_col = current_row, current_col + 1
                    min_val = f_dist[current_row][current_col]
                    current_node = grid[current_row][current_col]
                elif node_switch == 'RIGHT':
                    current_row, current_col = current_row + 1, current_col
                    min_val = f_dist[current_row][current_col]
                    current_node = grid[current_row][current_col]
                elif node_switch == 'LEFT':
                    current_row, current_col = current_row - 1, current_col
                    min_val = f_dist[current_row][current_col]
                    current_node = grid[current_row][current_col]
                print('Min Value is: ', min_val)

                #Make the color of the new shortest path node to Cyan
                current_node.set_path()
                #Update the grid with the colored nodes
                drawGrid(grid, 25, 25)
            #Once the shortest path node is at the starting node we exit the shortest path generating while loop and the algorithm while loop
            print('Is finished')
            finished = True
        
        #Update the f distance of the neighbor nodes of the current and pop them into the priority queue
        if current_col > 0 and not visited[current_row][current_col - 1] and not grid[current_row][current_col - 1].is_barrier(): #Check/update the distance of the up node
            f_dist[current_row][current_col - 1] = g_dist[current_row][current_col] + 1 + heuristic(current_node, end) #Up node
            count = count + 1
            #Put the up node in the priority queue
            open_set.put((f_dist[current_row][current_col - 1], count, grid[current_row][current_col - 1]))
            #Make the node color change to Yellow to indicate distance has been updated
            grid[current_row][current_col - 1].visited()
            #Redraw/update the grid to display updated distances
            drawGrid(grid, 25, 25)

        if current_col < 24 and not visited[current_row][current_col + 1] and not grid[current_row][current_col + 1].is_barrier(): #Check/update the distance of the down node
            f_dist[current_row][current_col + 1] = g_dist[current_row][current_col] + 1 + heuristic(current_node, end) #Down node
            count = count + 1
            #Put the up node in the priority queue
            open_set.put((f_dist[current_row][current_col + 1], count, grid[current_row][current_col + 1]))
            grid[current_row][current_col + 1].visited()
            #Redraw/update the grid to display updated distances
            drawGrid(grid, 25, 25)

        if current_row < 24 and not visited[current_row + 1][current_col] and not grid[current_row + 1][current_col].is_barrier():
            f_dist[current_row + 1][current_col] = g_dist[current_row][current_col] + 1 + heuristic(current_node, end) #Right node
            count = count + 1
            #Put the up node in the priority queue
            open_set.put((f_dist[current_row + 1][current_col], count, grid[current_row + 1][current_col]))
            grid[current_row + 1][current_col].visited()
            #Redraw/update the grid to display updated distances
            drawGrid(grid, 25, 25)

        if current_row > 0 and not visited[current_row - 1][current_col] and not grid[current_row - 1][current_col].is_barrier():
            f_dist[current_row - 1][current_col] = g_dist[current_row][current_col] + 1 + heuristic(current_node, end) #Left node
            count = count + 1
            #Put the up node in the priority queue
            open_set.put((f_dist[current_row - 1][current_col], count, grid[current_row - 1][current_col]))
            grid[current_row - 1][current_col].visited()
            #Redraw/update the grid to display updated distances
            drawGrid(grid, 25, 25)
        
        visited[current_row][current_col] = True
        current_node = open_set.get()[2]
        
        
#Make a grid of 25 rows and 25 columns and begin the path planning   
grid_loop(25, 25)