#GOAL: the goal of this project is to compare different path planning algorithms
#Two fo the algorithms are popular in robot navigation (1. Potential Field 2. Dynamic Window Approach)
#The other two are grid searches for most optimal path from two distances (1. A* Algorithm 2. Dijkstra Algorithm)

import pygame
import sys
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
    
    def get_pos(self):
        return self.row, self.col
    def clicked(self):
        self.color = BLACK
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
    while True:
        drawGrid(nodes, rows, cols) 
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.MOUSEBUTTONUP:
                mousepos = pygame.mouse.get_pos()
                i, j = mousepos
                i = i // size
                j = j // size
                node = nodes[i][j]
                node.clicked()

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


#Function to identify if a box/rectangle has been clicked
def mouse_click(rows, cols):
    pass

grid_loop(25, 25)