#GOAL: the goal of this project is to compare different path planning algorithms
#Two fo the algorithms are popular in robot navigation (1. Potential Field 2. Dynamic Window Approach)
#The other two are grid searches for most optimal path from two distances (1. A* Algorithm 2. Dijkstra Algorithm)

import pygame
import sys
import time
import numpy as np
import pandas as pd

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

#Create class of node
class Node():
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
    
    def get_pos(self):
        return self.row, self.col
    def set_source(self):
        self.color = BLUE
    def set_destination(self):
        self.color = RED
    def set_barrier(self):
        self.color = BLACK
    def set_path(self):
        self.color = CYAN
    def is_source(self):
        self.source = True
    def is_destination(self):
        self.destination = True
    def is_barrier(self):
        return self.color == BLACK
    def visited(self):
        self.color = YELLOW
    def was_visited(self):
        return self.color == YELLOW
    # This function can tell us which node to visit next 
    def update_neighbor(self, grid): #This function lets every node hold its neighbors for easier computation
        if self.col > 0: 
            self.neighbor.append(grid[self.row][self.col - 1]) #Up
        if self.col < 23:
            self.neighbor.append(grid[self.row][self.col + 1]) #Down
        if self.row < 23:
            self.neighbor.append(grid[self.row + 1][self.col]) #Right
        if self.row > 0:
            self.neighbor.append(grid[self.row - 1][self.col]) #Left
    def get_neighbor(self):
        return self.neighbor
    def neighbor_num(self):
        return len(self.neighbor)
    def draw(self, scn):
        pygame.draw.rect(scn, self.color, pygame.Rect(self.x, self.y, self.size, self.size))
        
        

# Set the size of our grid (just need width cause we want it to be square)
screen_width = 750

#Initialize the pygame window
pygame.init()
screen = pygame.display.set_mode((screen_width, screen_width))
pygame.display.set_caption('Path Grid')
screen.fill(WHITE)

def grid_loop(rows, cols):
    #Need a while loop to keep the window open
    nodes = make_nodes(rows, cols, screen_width)
    size = screen_width // rows
    mouseclick = 0
    while True:
        drawGrid(nodes, rows, cols) 
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if pygame.mouse.get_pressed() == (1, 0, 0): #makes sure that only the left mouse click works, well use the same syntax but right click to start the algorithm 
                mousepos = pygame.mouse.get_pos()
                i, j = mousepos
                i = i // size
                j = j // size
                node = nodes[i][j]
                if mouseclick == 0:
                    node.is_source()
                    node.set_source()
                    start = node
                    mouseclick = 1
                elif mouseclick == 1:
                    node.is_destination()
                    node.set_destination()
                    end = node
                    mouseclick = 2
                else:
                    node.set_barrier()
            elif pygame.mouse.get_pressed() == (0, 0, 1): #if the right mouse button is clicked start the algorithm
                dijkstra(nodes, start, end)
                
    
        
                

#Function to draw the grid (ie horizontal and vertical lines)
def draw_gridlines(rows, cols):
    gap = screen_width // rows
    #Print horizontal lines
    for i in range(rows):
        #print('Horizontal Line at: ', i*gap)
        pygame.draw.line(screen, BLACK, (0, i*gap), (screen_width, i*gap))
        #Print vertical lines
        for j in range(cols):
            #print('Vertical Line at: ', j*gap)
            pygame.draw.line(screen, BLACK, (j*gap, 0), (j*gap, screen_width))

def make_nodes(rows, cols, screenwidth):
    grid = [[]]
    gap = screenwidth // rows
    for i in range(rows):
        grid.append([])
        for j in range(cols):
            node = Node(i, j, gap)
            grid[i].append(node)
    return grid

def drawGrid(grid, rows, cols):
    #screen.fill(WHITE) #have to fill everything every fps

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
    #For now just set infinity to very high number
    inf = 1000000000
    #Make an array/list that has infinity for all the distances
    distance = []
    tempdist = []
    for i in range(len(grid) - 1):
        distance.append([])
        tempdist.append([])
        for j in range(len(grid) - 1):
            distance[i].append(inf)
            tempdist[i].append(inf)
    #Get the position of the start node and make the distance equal to 0
    distance[start_row][start_col] = 0
    tempdist[start_row][start_col] = 0

    #Create a list for the visited and unvisisted nodes (just holds the row and col of the nodes visited)
    visited = []
    for i in range(len(grid) - 1):
        visited.append([])
        for j in range(len(grid) - 1):
            visited[i].append((False))
    unvisited = []
    for i in range(len(grid) - 1):
        unvisited.append([])
        for j in range(len(grid) - 1):
            unvisited[i].append((i, j))

    #Append the starting node to the visited list and pop the start node from the unvisited list (For some reason Pygame assumed rows is top and columns is side)
    #visited.append(unvisited[start_row].pop(start_col))
    visited[start_row][start_col] = True

    #Make the distance of the starting nodes neighbors equal to 1 and change their color 
    for row in grid:
        for node in row:
            node.update_neighbor(grid)

    current_node = start
    #Were gonna go through each nodes neighbors and update the distance
    finished = False
    while not finished:
        current_row, current_col = current_node.get_pos() 
        #Move one up
        if current_col > 0: #Check the distance of the up node
            if distance[current_row][current_col - 1] > distance[current_row][current_col] + 1 and not visited[current_row][current_col - 1] and not grid[current_row][current_col - 1].is_barrier():
                distance[current_row][current_col - 1] = distance[current_row][current_col] + 1
                tempdist[current_row][current_col - 1] = distance[current_row][current_col] + 1
                #print('Top Node DIst: ', distance[current_row][current_col - 1])
                grid[current_row][current_col - 1].visited()
                drawGrid(grid, 25, 25)
        if current_col < 24: #Check the distance of the down node
            if distance[current_row][current_col + 1] > distance[current_row][current_col] + 1 and not visited[current_row][current_col + 1] and not grid[current_row][current_col + 1].is_barrier():
                distance[current_row][current_col + 1] = distance[current_row][current_col] + 1
                tempdist[current_row][current_col + 1] = distance[current_row][current_col] + 1
                #print('Down Node DIst: ', distance[current_row][current_col + 1])
                grid[current_row][current_col + 1].visited()
                drawGrid(grid, 25, 25)
        if current_row < 24: #Check the distance of the right node
            if distance[current_row + 1][current_col] > distance[current_row][current_col] + 1 and not visited[current_row + 1][current_col] and not grid[current_row + 1][current_col].is_barrier():
                distance[current_row + 1][current_col] = distance[current_row][current_col] + 1
                tempdist[current_row + 1][current_col] = distance[current_row][current_col] + 1
                #print('Right Node DIst: ', distance[current_row + 1][current_col])
                grid[current_row + 1][current_col].visited()
                drawGrid(grid, 25, 25)
        if current_row > 0: #Check the distance of the left node
            if distance[current_row - 1][current_col] > distance[current_row][current_col] + 1 and not visited[current_row - 1][current_col + 1] and not grid[current_row - 1][current_col].is_barrier():
                distance[current_row - 1][current_col] = distance[current_row][current_col] + 1
                tempdist[current_row - 1][current_col] = distance[current_row][current_col] + 1
                #print('Left Node DIst: ', distance[current_row - 1][current_col])
                grid[current_row - 1][current_col].visited()
                drawGrid(grid, 25, 25)

        visited[current_row][current_col] = True
        #Now to somehow have to check which node should be visited next. It should be the smallest distance. Need a temporary distance list that will set the
        #current node location to infinity and then look in that list for the index with the smallest value
        tempdist[current_row][current_col] = inf
        min_val = inf
        for i in range(len(grid) - 1):
            for j in range(len(grid) - 1):
                if tempdist[i][j] < min_val:
                    min_val = tempdist[i][j]
                    current_row = i
                    current_col = j
        current_node = grid[current_row][current_col]

        if current_row == end_row and current_col == end_col:
            print('In back propagat')
            current_node.set_path()
            #If were in this if statement that means we got the path, now we need to back propagate and set the path to different color
            min_val = inf
            while distance[current_row][current_col] != 0:
                node_switch = 1
                if distance[current_row][current_col - 1] < min_val and not grid[current_row][current_col - 1].is_barrier(): #Check up node distance
                    node_switch = 1
                elif distance[current_row][current_col + 1] < min_val and not grid[current_row][current_col + 1].is_barrier(): #Check down node distance
                    node_switch = 2
                elif distance[current_row + 1][current_col] < min_val and not grid[current_row + 1][current_col].is_barrier(): #Check down node distance
                    node_switch = 3
                elif distance[current_row - 1][current_col] < min_val and not grid[current_row - 1][current_col].is_barrier(): #Check down node distance
                    node_switch = 4
                if node_switch == 1:
                    current_row, current_col = current_row, current_col - 1
                    min_val = distance[current_row][current_col]
                    current_node = grid[current_row][current_col]
                elif node_switch == 2:
                    current_row, current_col = current_row, current_col + 1
                    min_val = distance[current_row][current_col]
                    current_node = grid[current_row][current_col]
                elif node_switch == 3:
                    current_row, current_col = current_row + 1, current_col
                    min_val = distance[current_row][current_col]
                    current_node = grid[current_row][current_col]
                elif node_switch == 4:
                    current_row, current_col = current_row - 1, current_col
                    min_val = distance[current_row][current_col]
                    current_node = grid[current_row][current_col]

                current_node.set_path()
                drawGrid(grid, 25, 25)
                print(min_val)
                print('Current Row: ', current_row, 'Current Col: ', current_col)
            print('Is finished')
            finished = True

        time.sleep(0.02)
        
grid_loop(25, 25)