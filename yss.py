import socket, keepalive
from ast import literal_eval
import random

import sys
import time
import heapq
import math

host = '0.0.0.0' # Listen on all available interfaces
port = 5001      # Port to listen on for yellow snakes (non-privileged ports are > 1023)

WIDTH = 800
HEIGHT = 600
SEG_SIZE = 20

# Variables to keep track of game period and snake immunity period initialized to default
GAME_PERIOD = 1
SNAKE_IMMUNITY_PERIOD = 1

# Setting variables for game_period and snake_immunity period from command line
try:
    GAME_PERIOD = int(sys.argv[1])
except:
    print("Game period command line argument not found so using default value of game period")

try:
    SNAKE_IMMUNITY_PERIOD = int(sys.argv[2])
except:
    print("Snake immunity period command line argument not found so using default value of game period")

class Problem(object):
    """Derived classes should override `actions` and `results`.
    The default heuristic is 0 and the default action cost is 1 for all states.
    When you instantiate a derived class, specify `initial`, and `goal` states 
    (or an `is_goal` method)."""

    def __init__(self, initial=None, goal=None, **kwds): 
        # we can add an initial condition and a goal 
        self.__dict__.update(initial=initial, goal=goal, **kwds) 
        
    def actions(self, state):        raise NotImplementedError
    def result(self, state, action): raise NotImplementedError
    def is_goal(self, state):        return state == self.goal
    def action_cost(self, s, a, s1): return 1
    def h(self, node):               return 0
    
    def __str__(self):
        return '{}({!r}, {!r})'.format(
            type(self).__name__, self.initial, self.goal)
    
class Node:
    def __init__(self, state, parent=None, action=None, path_cost=0):
        # we can add an initial parent, action (movement), and cost
        self.__dict__.update(state=state, parent=parent, action=action, path_cost=path_cost)

    def __repr__(self): return '<{}>'.format(self.state)
    def __len__(self): return 0 if self.parent is None else (1 + len(self.parent))
    def __lt__(self, other): return self.path_cost < other.path_cost
    
failure = Node('failure', path_cost=math.inf) # Indicates an algorithm couldn't find a solution.
cutoff  = Node('cutoff',  path_cost=math.inf) # Indicates iterative deepening search was cut off.

def g(n): return n.path_cost

def straight_line_distance(A, B):
    "Straight-line distance between two points."
    return (abs(A.top_left.x - B.top_left.x)**2 + abs(A.top_left.y - B.top_left.y)**2) ** 0.5

class SnakeProblem(Problem):
    def __init__(self, initial=(0.0, 0.0), goal=(0.0, 0.0), obstacles=(), snake_direction=None, **kwds):
        Problem.__init__(self, initial=initial, goal=goal, **kwds)
        self.obstacles = obstacles
        self.snake_direction = snake_direction
    
    def actions(self, state):
        """You can move one cell in any of `directions` to a non-obstacle cell."""
        allowed_direction =  set(["Up", "Left", "Right", "Down"])
        for game_step in range(1, GAME_PERIOD+1):
            directions = set(["Up", "Left", "Right", "Down"])
            for direction in directions:
                next_segment = state.get_next_segment(direction, step=game_step)
                if not next_segment.is_safe(self.obstacles):
                    if direction in allowed_direction:
                        allowed_direction.remove(direction)
        return allowed_direction
    
    def result(self, state, action):
        return state.get_next_segment(action, step=GAME_PERIOD)
    
    def action_cost(self, s, action, s1): return straight_line_distance(s, s1)
    
    def h(self, node): return straight_line_distance(node.state, self.goal)

def children(problem, node):
    "return the child nodes."
    s = node.state
    for action in problem.actions(s):
        s1 = problem.result(s, action)
        cost = node.path_cost + problem.action_cost(s, action, s1)
        yield Node(s1, node, action, cost)

def path_actions(node):
    "The sequence of actions (movements) to get to this node."
    if node.parent is None:
        return []  
    return path_actions(node.parent) + [node.action]

def path_states(node):
    "The sequence of states (positions) to get to this node."
    if node in (cutoff, failure, None): 
        return []
    return path_states(node.parent) + [node.state]

class PriorityQueue:
    """A min-heap implementation with key/value pair items, defaulting to key/key"""

    def __init__(self, items=(), key=lambda x: x): 
        self.key = key
        self.items = [] # a heap of (score, item) pairs
        for item in items:
            self.add(item)
         
    def add(self, item):
        pair = (self.key(item), item)
        heapq.heappush(self.items, pair)

    def pop(self):
        """item with min f(item) value."""
        return heapq.heappop(self.items)[1]
    
    def top(self): return self.items[0][1]

    def __len__(self): return len(self.items)

def best_first_search(problem, f, cutoff_level):
    """A function to do best first search"""
    global reached
    node = Node(problem.initial)
    frontier = PriorityQueue([node], key=f)
    reached = {problem.initial: node}
    current_level = 0
    
    while frontier:
        node = frontier.pop()
        if problem.is_goal(node.state) or cutoff_level == current_level:
            return node
        increment_level = True
        for child in children(problem, node):
            if increment_level:
                current_level += 1
                increment_level = False
            s = child.state
            if s not in reached or child.path_cost < reached[s].path_cost:
                reached[s] = child
                frontier.add(child)
                
    return failure

def astar_search(problem, h=None, cutoff_level=50):
    """Search nodes with minimum f(n) = g(n) + h(n).
    g is the node cost, and h is the additional heuristic
    cost for that node: Typically straight_line_distance(node.state, self.goal)"""
    h = h or problem.h
    return best_first_search(problem, f=lambda n: g(n) + h(n), cutoff_level=cutoff_level)

# Creating Point class to represent single point having x and y coordinate
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def __eq__(self, other_point):
        if isinstance(other_point, Point):
            return self.x == other_point.x and self.y == other_point.y
        return False
    
    def __add__(self, other_point):
        if isinstance(other_point, Point):
            return Point(self.x + other_point.x, self.y + other_point.y)
        else:
            raise TypeError("Unsupported operand")
    
    def __str__(self):
        return f"({self.x}, {self.y})"

# Creating class Segment to represent single block(rectangle) in snake or apple or obstacle
class Segment:
    def __init__(self, top_left, top_right, bottom_right, bottom_left):
        self.top_left = top_left
        self.top_right = top_right
        self.bottom_right = bottom_right
        self.bottom_left = bottom_left
    
    def __hash__(self):
        return hash(f"{self.top_left}, {self.top_right}, {self.bottom_right}, {self.bottom_left}")
    
    def __eq__(self, other_segment):
        """A function that will return true if segment is colliding with other given segment"""
        if isinstance(other_segment, Segment):
            return self.top_left == other_segment.top_left and self.top_right == other_segment.top_right and self.bottom_right == other_segment.bottom_right and self.bottom_left == other_segment.bottom_left
        return False
    
    def __str__(self):
        return f"Segment({self.top_left}, {self.top_right}, {self.bottom_right}, {self.bottom_left})"
    
    def get_next_segment(self, direction, step=1):
        """A function which returns next segment given direction and step size"""
        if direction == "Up":
            point_to_be_added = Point(0.0,-1.0 * SEG_SIZE * step)
            next_segment = Segment(self.top_left + point_to_be_added, self.top_right + point_to_be_added, self.bottom_right + point_to_be_added, self.bottom_left + point_to_be_added)
            return next_segment
        elif direction == "Right":
            point_to_be_added = Point(1.0 * SEG_SIZE * step,0.0)
            next_segment = Segment(self.top_left + point_to_be_added, self.top_right + point_to_be_added, self.bottom_right + point_to_be_added, self.bottom_left + point_to_be_added)
            return next_segment
        elif direction == "Down":
            point_to_be_added = Point(0.0,1.0 * SEG_SIZE * step)
            next_segment = Segment(self.top_left + point_to_be_added, self.top_right + point_to_be_added, self.bottom_right + point_to_be_added, self.bottom_left + point_to_be_added)
            return next_segment
        elif direction == "Left":
            point_to_be_added = Point(-1.0 * SEG_SIZE * step,0.0)
            next_segment = Segment(self.top_left + point_to_be_added, self.top_right + point_to_be_added, self.bottom_right + point_to_be_added, self.bottom_left + point_to_be_added)
            return next_segment
    
    def is_safe(self, obstacles):
        """A function which returns true if given segment is safe to go to"""
        if self.bottom_right.x <= 0 or self.bottom_right.y <= 0 or self.top_left.x >= WIDTH or self.top_left.y >= HEIGHT or self in obstacles:
            return False
        else:
            return True

class Game:
    def __init__(self):
        """A function which initializes new game by setting all the variables"""
        self.own_length = 5
        self.opposition_length = 5
        self.apple_pos = None
        self.previous_apple_position = None
        self.own_coords = []
        self.opposition_coords = []
        self.snake_travelled = False
        self.snake_direction = None
        self.snake_head = None
        self.opp_snake_head = None
        self.opp_snake_direction = None
        self.goal_failure = 0
        self.goal_change_directions = []
        self.directions = {
            (0.0, -GAME_PERIOD * SEG_SIZE) : "Up",
            (-GAME_PERIOD * SEG_SIZE, 0.0): "Left",
            (GAME_PERIOD * SEG_SIZE,  0.0) : "Right",
            (0.0, GAME_PERIOD * SEG_SIZE) : "Down"
        }

    def increase_length(self, own=True):
        """A function to increase length of a snake"""
        if own:
            self.own_length += 1
        else:
            self.opposition_length += 1
    
    def set_apple_position(self, apple_segment):
        """A function to set apple position"""
        self.apple_pos = apple_segment
    
    def set_apple_pos_and_track_snake_lengths(self, ax, ay):
        """A function to track snake lengths and set apple position"""
        # Storing previous apple position if it is different
        previous_apple_position = None
        apple_segment = Segment(Point(ax, ay), Point(ax + SEG_SIZE, ay), Point(ax + SEG_SIZE, ay + SEG_SIZE), Point(ax, ay + SEG_SIZE))
        if apple_segment != self.apple_pos:
            previous_apple_position = self.apple_pos
        # Setting apple position variable
        self.set_apple_position(apple_segment)
        # If apple is eaten by our snake then incresing its length
        if previous_apple_position:
            game.goal_failure = 0
            game.goal_change_directions = []
            if GAME_PERIOD > 1:
                if previous_apple_position in self.own_coords:
                    self.increase_length()
                elif previous_apple_position in self.opposition_coords:
                    # Incresing other snakes length if apple eaten by him
                    self.increase_length(own=False)
            else:
                if previous_apple_position == self.own_coords[-2]:
                    self.increase_length()
                elif previous_apple_position == self.opposition_coords[-2]:
                    # Incresing other snakes length if apple eaten by him
                    self.increase_length(own=False)
    
    def set_head_coords(self, head_segment, own=True):
        """A function which sets head coordinates of the snakes"""
        if own:
            # Setting own snakes head coordinates
            self.snake_head = head_segment
        else:
            # Setting opposition snakes head coordinates
            self.opp_snake_head = head_segment

    def determine_direction(self, latest_segment, prev_segment, own=True):
        """Function to determine in which direction currently snake is moving"""
        # If there is change in x then snake is moving left or right direction
        if latest_segment.top_left.x - prev_segment.top_left.x > 0:
            if own:
                self.snake_direction = "Right"
            else:
                self.opp_snake_direction = "Right"
        elif latest_segment.top_left.x - prev_segment.top_left.x < 0:
            if own:
                self.snake_direction = "Left"
            else:
                self.opp_snake_direction = "Left"
        # If there is change in y then snake is moving up or down direction
        elif latest_segment.top_left.y - prev_segment.top_left.y > 0:
            if own:
                self.snake_direction = "Down"
            else:
                self.opp_snake_direction = "Down"
        elif latest_segment.top_left.y - prev_segment.top_left.y < 0:
            if own:
                self.snake_direction = "Up"
            else:
                self.opp_snake_direction = "Up"
    
    def update_blocks_of_snakes(self, opp_x1, opp_y1, opp_x2, opp_y2, x1, y1, x2, y2):
        """"A function to keep track of recently visited blocks by snakes"""
        if len(self.own_coords) == 0:
            own_segment = Segment(Point(x1, y1), Point(x2, y1), Point(x2, y2), Point(x1, y2))
            # Setting head coordinates of own snake
            self.set_head_coords(own_segment)
            self.own_coords.append(own_segment)
            opp_segment = Segment(Point(opp_x1, opp_y1), Point(opp_x2, opp_y1), Point(opp_x2, opp_y2), Point(opp_x1, opp_y2))
            # Setting head coordinates of opposition snake
            self.set_head_coords(opp_segment, own=False)
            self.opposition_coords.append(opp_segment)
        else:
            # For own snake
            current_own_segment = Segment(Point(x1, y1), Point(x2, y1), Point(x2, y2), Point(x1, y2))
            # Setting head coordinates of own snake
            self.set_head_coords(current_own_segment)
            previous_own_segment = self.own_coords[-1]
            # Determining snake direction to add snake body coords
            self.determine_direction(current_own_segment, previous_own_segment)
            while previous_own_segment != current_own_segment:
                previous_own_segment = previous_own_segment.get_next_segment(self.snake_direction)
                self.own_coords.append(previous_own_segment)
            # For opposition snake
            current_opp_segment = Segment(Point(opp_x1, opp_y1), Point(opp_x2, opp_y1), Point(opp_x2, opp_y2), Point(opp_x1, opp_y2))
            # Setting head coordinates of opposition snake
            self.set_head_coords(current_opp_segment, own=False)
            previous_opp_segment = self.opposition_coords[-1]
            # Determining snake direction to add snake body coords
            self.determine_direction(current_opp_segment, previous_opp_segment, own=False)
            while previous_opp_segment != current_opp_segment:
                previous_opp_segment = previous_opp_segment.get_next_segment(self.opp_snake_direction)
                self.opposition_coords.append(previous_opp_segment)
        # Removing blocks that are less than the size of the snakes from visited
        self.own_coords = self.own_coords[-self.own_length:]
        self.opposition_coords = self.opposition_coords[-self.opposition_length:]
    
    def add_obstacles(self, extra_obstacles=[]):
        """Function to add obstacles"""
        # Adding another snakes coords as the obstacle in obstacles
        obstacles = set(self.opposition_coords)
        # Adding another snakes coords in all possible directions that snake can move to kill
        if GAME_PERIOD > 1:
            for direction in ["Up", "Left", "Right", "Down"]: # Added later
                for step in range(1, GAME_PERIOD+1):
                    current_obstacle = self.opposition_coords[-1].get_next_segment(direction, step)
                    if current_obstacle not in self.own_coords: # Added later
                        obstacles.add(current_obstacle)
        return obstacles
    
    def set_goal(self):
        """A function to set goal of the snake"""
        # If game period is greater than one and goal is a failure then trying different apple locations as astar is not able to reach that location
        if GAME_PERIOD > 1 and self.goal_failure and len(self.goal_change_directions) > 0:
            if self.apple_pos in self.own_coords:
                # Setting goal to go to three steps ahead of opposition snake
                goal = self.opp_snake_head.get_next_segment(self.opp_snake_direction, 2)
                return goal
            else:
                direction = self.goal_change_directions[0]
                goal = self.apple_pos.get_next_segment(direction, self.goal_failure)
                self.goal_change_directions.remove(direction)
                return goal
        else:
            goal = self.apple_pos
            if self.apple_pos == self.snake_head:
                # Setting goal to go to three steps ahead of opposition snake
                goal = self.opp_snake_head.get_next_segment(self.opp_snake_direction, 2)
            return goal

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    keepalive.set(s)
    s.bind((host, port))
    s.listen()
    print(f"Server listening on {host}:{port}")
    # Creating game
    game = Game()
    while True:
        # begin = time.time()
        # Accept new connections in an infinite loop.
        client_sock, client_addr = s.accept()
        # end = time.time()
        print('New connection from', client_addr)
        # If time between request is larger than 2 then we can say that game is a new game
        # if end-begin > 2:
        #     game = Game()
        while True:
            # accept new data in an infinite loop
            data = client_sock.recv(1024)
            if data:
                received = data.decode()
                data_to_proxy = "Straight"
                x1, y1, x2, y2, rx1, ry1, rx2, ry2, ax, ay = literal_eval(data.decode())
                # Updating snake segment (blocks)
                game.update_blocks_of_snakes(rx1, ry1, rx2, ry2, x1, y1, x2, y2)
                # Setting apple position and tracking lengths of snake
                game.set_apple_pos_and_track_snake_lengths(ax, ay)
                # Let snake travel to locate its blocks and direction
                if not game.snake_travelled:
                    game.snake_travelled = True
                    if x2 + GAME_PERIOD * SEG_SIZE > WIDTH:
                        data_to_proxy = "Left"
                    if x1 + GAME_PERIOD * SEG_SIZE < 0:
                        data_to_proxy = "Right"
                    if y2 + GAME_PERIOD * SEG_SIZE > HEIGHT:
                        data_to_proxy = "Up"
                    if y1 + GAME_PERIOD * SEG_SIZE < 0:
                        data_to_proxy = "Down"
                # If snake travel direction is detected then proceeding with astar search
                if game.snake_direction != None:
                    # Setting goal
                    goal = game.set_goal()
                    # Adding obstacles
                    obstacles = game.add_obstacles()
                    # Creating instance of SnakeProblem
                    problem = SnakeProblem(initial=game.snake_head, goal=goal, obstacles=obstacles, snake_direction=game.snake_direction)
                    # Using astar algorithm to find shortest path to goal
                    goal_node = astar_search(problem, cutoff_level=5000)
                    # If goal is a failure in game step graeter than 1 then changing goal(apple) location so that astar is able to reach apple
                    if goal_node == failure and GAME_PERIOD > 1:
                        if len(game.goal_change_directions) == 0:
                            if game.goal_failure != GAME_PERIOD:
                                game.goal_failure += 1
                                game.goal_change_directions = ["Up", "Left", "Right", "Down"]
                            else:
                                game.goal_failure = 0
                                game.goal_change_directions = []
                    else:
                        game.goal_failure = 0
                        game.goal_change_directions = []
                    # Getting all the actions to be taken to get to goal node
                    movements_to_goal = path_actions(goal_node)
                    # If goal node is not failure and set of actions is greater than zero then returning first action
                    if goal_node != failure and len(movements_to_goal) > 0:
                        data_to_proxy = movements_to_goal[0]
                    # If astar failed to found the goal node then getting all the actions that are available at current state and then randomly chosing from that choice
                    if data_to_proxy not in game.directions.values():
                        allowed_actions = problem.actions(game.snake_head)
                        if len(allowed_actions) > 0:
                            my_choice = random.choice(list(allowed_actions))
                            data_to_proxy = my_choice
                client_sock.sendall(data_to_proxy.encode())
                # hang up?
                if received == "Close" or received == b'Close':
                    # client wants to hang up
                    print("hanging up..")
                    break
            else:
                break
        client_sock.close()
        print("Connection closed!")