#!/usr/bin/env python3
from typing import List, Set
from dataclasses import dataclass
import pygame
from enum import Enum, unique
import sys
import random
import math
import numpy as np
from collections import defaultdict 


FPS = 15

INIT_LENGTH = 3

WIDTH = 480
HEIGHT = 480
GRID_SIDE = 24
GRID_WIDTH = WIDTH // GRID_SIDE
GRID_HEIGHT = HEIGHT // GRID_SIDE

BRIGHT_BG = (103, 223, 235)
DARK_BG = (78, 165, 173)

SNAKE_COL = (6, 38, 7)
FOOD_COL = (224, 160, 38)
OBSTACLE_COL = (209, 59, 59)
VISITED_COL = (24, 42, 142)



@unique
class Direction(tuple, Enum):
    UP = (0, -1)
    DOWN = (0, 1)
    LEFT = (-1, 0)
    RIGHT = (1, 0)

    def reverse(self):
        x, y = self.value
        return Direction((x * -1, y * -1))


@dataclass
class Position:
    x: int
    y: int

    def check_bounds(self, width: int, height: int):
        return (self.x >= width) or (self.x < 0) or (self.y >= height) or (self.y < 0)

    def draw_node(self, surface: pygame.Surface, color: tuple, background: tuple):
        r = pygame.Rect(
            (int(self.x * GRID_SIDE), int(self.y * GRID_SIDE)), (GRID_SIDE, GRID_SIDE)
        )
        pygame.draw.rect(surface, color, r)
        pygame.draw.rect(surface, background, r, 1)

    def __eq__(self, o: object) -> bool:
        if isinstance(o, Position):
            return (self.x == o.x) and (self.y == o.y)
        else:
            return False

    def __str__(self):
        return f"X{self.x};Y{self.y};"

    def __hash__(self):
        return hash(str(self))


class GameNode:
    nodes: Set[Position] = set()

    def __init__(self):
        self.position = Position(0, 0)
        self.color = (0, 0, 0)

    def randomize_position(self):
        try:
            GameNode.nodes.remove(self.position)
        except KeyError:
            pass

        condidate_position = Position(
            random.randint(0, GRID_WIDTH - 1), random.randint(0, GRID_HEIGHT - 1),
        )

        if condidate_position not in GameNode.nodes:
            self.position = condidate_position
            GameNode.nodes.add(self.position)
        else:
            self.randomize_position()

    def draw(self, surface: pygame.Surface):
        self.position.draw_node(surface, self.color, BRIGHT_BG)


class Food(GameNode):
    def __init__(self):
        super(Food, self).__init__()
        self.color = FOOD_COL
        self.randomize_position()


class Obstacle(GameNode):
    def __init__(self):
        super(Obstacle, self).__init__()
        self.color = OBSTACLE_COL
        self.randomize_position()


class Snake:
    def __init__(self, screen_width, screen_height, init_length):
        self.color = SNAKE_COL
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.init_length = init_length
        self.reset()

    def reset(self):
        self.length = self.init_length
        self.positions = [Position((GRID_SIDE // 2), (GRID_SIDE // 2))]
        self.direction = random.choice([e for e in Direction])
        self.score = 0
        self.hasReset = True

    def get_head_position(self) -> Position:
        return self.positions[0]

    def turn(self, direction: Direction):
        if self.length > 1 and direction.reverse() == self.direction:
            return
        else:
            self.direction = direction

    def move(self):
        self.hasReset = False
        cur = self.get_head_position()
        x, y = self.direction.value
        new = Position(cur.x + x, cur.y + y,)
        if self.collide(new):
            self.reset()
        else:
            self.positions.insert(0, new)
            while len(self.positions) > self.length:
                self.positions.pop()

    def collide(self, new: Position):
        return (new in self.positions) or (new.check_bounds(GRID_WIDTH, GRID_HEIGHT))

    def eat(self, food: Food):
        if self.get_head_position() == food.position:
            self.length += 1
            self.score += 1
            while food.position in self.positions:
                food.randomize_position()

    def hit_obstacle(self, obstacle: Obstacle):
        if self.get_head_position() == obstacle.position:
            self.length -= 1
            self.score -= 1
            if self.length == 0:
                self.reset()

    def draw(self, surface: pygame.Surface):
        for p in self.positions:
            p.draw_node(surface, self.color, BRIGHT_BG)


class Player:
    def __init__(self) -> None:
        self.visited_color = VISITED_COL
        self.visited: Set[Position] = set()
        self.chosen_path: List[Direction] = []

    def move(self, snake: Snake) -> bool:
        try:
            next_step = self.chosen_path.pop(0)
            snake.turn(next_step)
            return False
        except IndexError:
            return True

    def search_path(self, snake: Snake, food: Food, *obstacles: Set[Obstacle]):
        """
        Do nothing, control is defined in derived classes
        """
        pass

    def turn(self, direction: Direction):
        """
        Do nothing, control is defined in derived classes
        """
        pass

    def draw_visited(self, surface: pygame.Surface):
        for p in self.visited:
            p.draw_node(surface, self.visited_color, BRIGHT_BG)


class SnakeGame:
    def __init__(self, snake: Snake, player: Player) -> None:
        pygame.init()
        pygame.display.set_caption("AIFundamentals - SnakeGame")

        self.snake = snake
        self.food = Food()
        self.obstacles: Set[Obstacle] = set()
        for _ in range(40):
            ob = Obstacle()
            while any([ob.position == o.position for o in self.obstacles]):
                ob.randomize_position()
            self.obstacles.add(ob)

        self.player = player

        self.fps_clock = pygame.time.Clock()

        self.screen = pygame.display.set_mode(
            (snake.screen_height, snake.screen_width), 0, 32
        )
        self.surface = pygame.Surface(self.screen.get_size()).convert()
        self.myfont = pygame.font.SysFont("monospace", 16)

    def drawGrid(self):
        for y in range(0, int(GRID_HEIGHT)):
            for x in range(0, int(GRID_WIDTH)):
                p = Position(x, y)
                if (x + y) % 2 == 0:
                    p.draw_node(self.surface, BRIGHT_BG, BRIGHT_BG)
                else:
                    p.draw_node(self.surface, DARK_BG, DARK_BG)

    def run(self):
        while not self.handle_events():
            self.fps_clock.tick(FPS)
            self.drawGrid()
            if self.player.move(self.snake) or self.snake.hasReset:
                self.player.search_path(self.snake, self.food, self.obstacles)
                self.player.move(self.snake)
            self.snake.move()
            self.snake.eat(self.food)
            for ob in self.obstacles:
                self.snake.hit_obstacle(ob)
            for ob in self.obstacles:
                ob.draw(self.surface)
            self.player.draw_visited(self.surface)
            self.snake.draw(self.surface)
            self.food.draw(self.surface)
            self.screen.blit(self.surface, (0, 0))
            text = self.myfont.render(
                "Score {0}".format(self.snake.score), 1, (0, 0, 0)
            )
            self.screen.blit(text, (5, 10))
            pygame.display.update()

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    pygame.quit()
                    sys.exit()
                if event.key == pygame.K_UP:
                    self.player.turn(Direction.UP)
                elif event.key == pygame.K_DOWN:
                    self.player.turn(Direction.DOWN)
                elif event.key == pygame.K_LEFT:
                    self.player.turn(Direction.LEFT)
                elif event.key == pygame.K_RIGHT:
                    self.player.turn(Direction.RIGHT)
        return False


class HumanPlayer(Player):
    def __init__(self):
        super(HumanPlayer, self).__init__()

    def turn(self, direction: Direction):
        self.chosen_path.append(direction)


# ----------------------------------
# DO NOT MODIFY CODE ABOVE THIS LINE
# ----------------------------------

def isElementExistInSet(set, myElement):
    for element in set:
        if element.x == myElement.x and element.y == myElement.y:
            return True
    return False

def getPath(destination, queue):
    if destination == None:
        return queue
    if destination.parent == None:
        queue.append(destination)
        return queue
    else:
        queue.append(destination)
        getPath(destination.parent ,queue)

def blind_search(self, snake, food, obstacles):
    root_x, root_y = snake.get_head_position().x, snake.get_head_position().y
    food_x, food_y = food.position.x, food.position.y
    food = (food_x, food_y)
    #print("Current Position", root_x, root_y)
    self.visited.append(Position(root_x,root_y))

    while len(self.visited) > snake.length:
        self.visited.reverse()
        self.visited.pop()
        self.visited.reverse()
    
    #print(self.visited)

    destination = self.BFS(snake, food, obstacles, self.isLegal)
    path = []
    getPath(destination, path)
    path.reverse()

    if snake.length > 50 or len(path) == 0:
        destination = self.BFS(snake, food, obstacles, self.isLegalLow)
        path = []
        getPath(destination, path)
        path.reverse()
    """
    print("#################################")
    for point in path:
        print(point.x, point.y)append
    print("food: ", food.position.x, food.position.y)
    """ 
    self.followPath(path)

#dijkstra
def dijkstra_search(self, snake, food, obstacles):
    root_x, root_y = snake.get_head_position().x, snake.get_head_position().y
    #print("Current Position", root_x, root_y)
    self.visited.append(Position(root_x,root_y))

    while len(self.visited) > snake.length:
        self.visited.reverse()
        self.visited.pop()
        self.visited.reverse()

    dist, prev = self.dijkstra_v2(snake,food,obstacles)

    stack = []
    u = Position(food.position.x, food.position.y)
    if prev[u.x, u.y] != None or u == Position(root_x, root_y):
        while u != Position(root_x, root_y):
            stack.insert(0,u)
            u = prev[u.x, u.y]
    
    self.followPathDirect(stack, Position(root_x,root_y))



def isObstacle(x,y, obstacles):
    for ob in obstacles:
        for o in ob:
            if o.position.x == x and o.position.y == y:
                return True
    return False

def isSelfVisited(x,y, visitedNode):
    for node in visitedNode:
        if x == node.x and y == node.y:
            return True
    return False

def isInTheMap(x,y):
    if x >= 0 and x < GRID_WIDTH and y >= 0 and y < GRID_HEIGHT:
        return True
    return False

class myGraph:
    def __init__(self, V):
        self.graph = defaultdict(list)
        self.V = V

    def addEdge(self, u, v, w):
        self.graph[u].append([v, w])
    

class Leaf:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
    
    def setParent(self, otherLeaf):
        self.parent = otherLeaf

class SearchBasedPlayer(Player):
    def __init__(self):
        super(SearchBasedPlayer, self).__init__()
        self.visited = list(self.visited)
    
    def isLegal(self, current, obstacles, i, j, visitedNode, snake):
        x, y = current.x+i, current.y+j

        isOnScreen = x >= 0 and x < GRID_WIDTH and y >= 0 and y < GRID_HEIGHT
        isObstacle = False
        isGoingBack = False
        isMoveLegal = i == 0 or j == 0

        for node in visitedNode:
            if x == node.x and y == node.y:
                isGoingBack = True
        
        for ob in obstacles:
            for o in ob:
                if o.position.x == x and o.position.y == y:
                    isObstacle = True

        if isOnScreen and not isObstacle and not isGoingBack and isMoveLegal:
            return True
        return False

    def isLegalLow(self, current, obstacles, i, j, visitedNode, snake):
        x, y = current.x+i, current.y+j

        isOnScreen = x >= 0 and x < GRID_WIDTH and y >= 0 and y < GRID_HEIGHT
        isObstacle = False
        isGoingBack = False
        isMoveLegal = i == 0 or j == 0

        for node in visitedNode:
            if x == node.x and y == node.y:
                isGoingBack = True
        

        if isOnScreen and not isObstacle and not isGoingBack and isMoveLegal:
            return True
        return False

    def getNearestObstacle(self, snake: Snake, obstacles: Set[Obstacle]):
        root_x, root_y = snake.get_head_position().x, snake.get_head_position().y
        min_distance = 5000
        target_x = 0
        target_y = 0

        for ob in obstacles:
            for o in ob:
                distance = math.sqrt(math.pow(root_x - o.position.x,2) + math.pow(root_y - o.position.y ,2))
                if min_distance > distance:
                    min_distance = distance
                    target_x = o.position.x
                    target_y = o.position.y
        return target_x, target_y
                    
    

    def turn(self, direction: Direction):
        self.chosen_path.append(direction)

    def followPathDirect(self, path, current_position):
        if len(path) != 0:
            if path[0].x > current_position.x:
                self.chosen_path.append(Direction.RIGHT)
            if path[0].x < current_position.x:
                self.chosen_path.append(Direction.LEFT)
            if path[0].y > current_position.y:
                self.chosen_path.append(Direction.DOWN)
            if path[0].y < current_position.y:
                self.chosen_path.append(Direction.UP)
            if path[0].x == current_position.x and path[0].y == current_position.y:
                pass
        else:
            print("path is empty")

    def followPath(self, path):
        if len(path) != 0:
            current_position = path[0]
            if path[1].x > current_position.x:
                self.chosen_path.append(Direction.RIGHT)
            if path[1].x < current_position.x:
                self.chosen_path.append(Direction.LEFT)
            if path[1].y > current_position.y:
                self.chosen_path.append(Direction.DOWN)
            if path[1].y < current_position.y:
                self.chosen_path.append(Direction.UP)
            if path[1].x == current_position.x and path[1].y == current_position.y:
                pass
        else:
            print("path is empty")
            #print("myPos", path[0].x, path[0].y , " Next Pos: ", path[1].x, path[1].y )

    def dijkstra(self, snake: Snake, food: Food, obstacles: Set[Obstacle]):
        COST_OF_OBSTACLE = 4
        food_x, food_y = food.position.x, food.position.y
        root_x, root_y = snake.get_head_position().x, snake.get_head_position().y
        infinity = 10000
        graph_distance = np.full((GRID_WIDTH, GRID_HEIGHT), infinity)
        graph_cost = np.full((GRID_WIDTH, GRID_HEIGHT), 1)
        previous = np.full((GRID_WIDTH, GRID_HEIGHT), Position(root_x, root_y))
        unvisited_coordinate = set()

        for i in range(GRID_WIDTH):
            for j in range(GRID_HEIGHT):
                unvisited_coordinate.add(Position(i,j))

        #fill the graph_cost
        for ob in obstacles:
            for o in ob:
                graph_cost[o.position.x, o.position.y] = COST_OF_OBSTACLE

        #initialize graph_distance
        graph_distance[root_x, root_y] = 0
        current = graph_distance[root_x, root_y]
        current_i, current_j = root_x, root_y  

        debug_iteration = 0

        while len(unvisited_coordinate) != 0 and debug_iteration < 90000:
            debug_iteration += 1
            
            print(len(unvisited_coordinate))

            min_distance = 5000
            i_min = 0
            j_min = 0
            if current_i == food_x and current_j == food_y:
                print("target found", current_i, current_j, food_x, food_y)
                return previous

            for i in range(-1,2):
                for j in range(-1,2):
                    #Do something only if the node is reachable
                    if self.isLegalLow(Position(current_i, current_j), obstacles, i, j, self.visited, snake):
                        #If we have a better distance, then replace the previous one by the current distance
                        if graph_distance[current_i+i, current_j+j] > graph_cost[current_i+i, current_j+j] + current:
                            graph_distance[current_i+i, current_j+j] = graph_cost[current_i+i, current_j+j] + current
                            previous[current_i+i, current_j+j] = Position(current_i,current_j)

                        #Search which neighbor has the minimum distance from current to itself AND if this node is not already marked as visited
                        if min_distance > graph_distance[current_i+i, current_j+j] and isElementExistInSet(unvisited_coordinate, Position(current_i+i, current_j+j)):
                            i_min, j_min = current_i+i, current_j+j
                            min_distance = graph_distance[current_i+i, current_j+j]
            
            unvisited_coordinate.discard(Position(current_i, current_j))
            current_i = i_min
            current_j = j_min
            current = graph_distance[current_i, current_j]
        
        return previous

    def dijkstra_v2(self, snake: Snake, food: Food, obstacles: Set[Obstacle]):
        root_x, root_y = snake.get_head_position().x, snake.get_head_position().y
        g = myGraph(GRID_WIDTH*GRID_HEIGHT)
        graph_distance = np.full((GRID_WIDTH, GRID_HEIGHT), 80000)
        previous = np.full((GRID_WIDTH, GRID_HEIGHT), Position(0,0))
        unvisited_nodes = set()

        #build adjacency list
        for i in range(GRID_WIDTH):
            for j in range(GRID_HEIGHT):
                for k in range(-1,2):
                    for l in range(-1,2):
                        if isInTheMap(i+k, j+l) and (k==0 or l==0) and not(k==0 and l==0) and not(isSelfVisited(i+k, j+l, self.visited)):
                            if isObstacle(i+k,j+l, obstacles):
                                g.addEdge(Position(i,j), Position(i+k,j+l), 6)
                            else :
                                g.addEdge(Position(i,j), Position(i+k,j+l), 1)
                unvisited_nodes.add(Position(i,j))

        #print(g.graph[Position(14,6)][neighbors_index][0])
        
        graph_distance[root_x,root_y] = 0

        while len(unvisited_nodes) != 0:
            u = self.getVertexWithMinDistance(graph_distance, unvisited_nodes)

            if u == Position(food.position.x, food.position.y):
                print("Target Found")
                return graph_distance, previous

            unvisited_nodes.remove(u)
        
            for neighbor in g.graph[u]:
                if neighbor[0] in unvisited_nodes:
                    alt = graph_distance[u.x, u.y] + neighbor[1]
                    if alt < graph_distance[neighbor[0].x, neighbor[0].y]:
                        graph_distance[neighbor[0].x, neighbor[0].y] = alt
                        previous[neighbor[0].x, neighbor[0].y] = u
        
        return graph_distance, previous


            
        

    def getVertexWithMinDistance(self, graph_distance, unvisited_nodes):
        min = 80000
        x_min = 0
        y_min = 0
        for node in unvisited_nodes:
            if min > graph_distance[node.x, node.y]:
                min = graph_distance[node.x, node.y]
                x_min = node.x
                y_min = node.y
        return Position(x_min, y_min)



    def BFS(self, snake: Snake, food: Food, obstacles: Set[Obstacle], condition):
        root_x = snake.get_head_position().x
        root_y = snake.get_head_position().y

        mySet = set()
        myQueue = []
        root = Leaf(root_x, root_y)
        myQueue.append(root)

        while len(myQueue) != 0 :
            myQueue.reverse()
            current = myQueue.pop()
            myQueue.reverse()
            if current.x == food[0] and current.y == food[1]:
                return current
            else:
                for i in range(-1,2):
                    for j in range(-1,2):
                        if condition(current, obstacles, i, j, self.visited, snake):
                            newNode = Leaf(current.x+i,current.y+j)
                            if isElementExistInSet(mySet, newNode) == False:
                                newNode.setParent(current)
                                mySet.add(newNode)
                                myQueue.append(newNode)

    

    def search_path(self, snake: Snake, food: Food, *obstacles: Set[Obstacle]):

        #blind_search(self, snake, food, obstacles)
        dijkstra_search(self, snake, food, obstacles)





if __name__ == "__main__":
    snake = Snake(WIDTH, WIDTH, INIT_LENGTH)
    #player = HumanPlayer()
    player = SearchBasedPlayer()
    game = SnakeGame(snake, player)
    game.run()