from models import Position, RobotWorld
from collections import deque
import heapq
from typing import List, Tuple, Set, Dict
from abc import ABC, abstractmethod
    
class PriorityQueue:
    """
    A priority queue implementation using a heap.
    
    This class provides a priority queue with customizable ordering and priority function.
    """
    def __init__(self, order='min', f=lambda x: x):
        """
        Initialize 

        Args:
            order (str): 'min' for min-heap, anything else for max-heap.
            f (function): A function to compute the priority of an item.
        """
        self.heap = []
        self.order = 1 if order == 'min' else -1
        self.f = f
    
    def append(self, item):
        """
        Add an item to the priority queue.

        Args:
            item: The item to be added.
        """
        priority = self.f(item)
        heapq.heappush(self.heap, (self.order * priority, id(item), item))
    
    def pop(self):
        """Remove and return the item with the highest priority."""
        return heapq.heappop(self.heap)[2]
    
    def __len__(self):
        """Return the number of items in the queue."""
        return len(self.heap)



class SearchNode:
    """
    Represents a node in the search tree.
    
    This class encapsulates the state and metadata for a node in the search process.
    """
    def __init__(self, position: Position, parent=None, cost=0, heuristic=0):
        """
        Initialize a SearchNode.

        Args:
            position (Position): The position of this node
            parent (SearchNode, optional): The parent node.
            cost (int): The cost to reach this node. Defaults to 0.
            heuristic (float): The heuristic estimate to the goal. Defaults to 0.
        """
        self.position = position
        self.parent = parent
        self.cost = cost
        self.heuristic = heuristic
        self.depth = 0 if parent is None else parent.depth + 1
    
    def __lt__(self, other):
        # Define less than comparison for priority queue ordering.
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)
    
    def expand(self, world: RobotWorld):
        """
        Generate child nodes reachable from this node.
        
        Args:
            world (RobotWorld): The world environment
        
        Returns:
            List[SearchNode]: List of child nodes
        """
        return [SearchNode(neighbor, parent=self, cost=self.cost+1) for neighbor in world.get_neighbors(self.position)]


class SearchAlgorithm(ABC):
    """
    Abstract base class for search algorithms.

    This class defines the interface and common functionality for all search algorithms.
    """
    def __init__(self, world: RobotWorld):
        """
        Initialize the search algorithm.

        Args:
            world (RobotWorld): The world environment for the search
        """
        self.world = world
        self.nodes_created = 0
    
    def visit_node(self, position: Position):
        """
        Mark a node as visited. Can be used for visualization or analysis.
        
        Args:
            position (Position): The position of the visited node
        """
        pass
    
    def reconstruct_path(self, node:SearchNode): #  -> List[Position]
        """
        Reconstruct the path from the start to the given node.
        
        Args: 
            node (SearchNode): The end node of the path
        
        Returns:
            List[Position]: The path from start to end
        """
        path = []
        current = node
        while current is not None:
            path.append(current.position)
            current = current.parent
        return list(reversed(path))
    
    def is_goal(self, position: Position): # -> bool
        """
        Check if the given position is a goal state.

        Args:
            position (Position): The position to check

        Returns:
            bool: True if the position is a goal, False otherwise
        """
        return any(position.as_tuple() == goal.as_tuple() for goal in self.world.goals)
    
    @abstractmethod
    def find_path(self): # -> Tuple[List[Position], int]
        """
        Find a path from the start to a goal.
        
        Returns:
            Tuple[List[Position], int]: The path and the number of nodes created 
        """
        pass



#_____________________________________________________________________________________________________________
    
class BFS(SearchAlgorithm):
    """Breath-First Search algorithm implementation."""
    
    def find_path(self): # -> Tuple[List[Position], int]
        """
        Perform Breath-First Search to find a path.

        Returns:
            Tuple[List[Position], int]: The path and the number of nodes created 
        """
        
        queue = deque([SearchNode(self.world.start)])
        visited = {self.world.start.as_tuple()}
        self.nodes_created = 1
        
        while queue:
            current = queue.popleft()
            self.visit_node(current.position)
            
            
            if self.is_goal(current.position):
                return self.reconstruct_path(current), self.nodes_created
            
            for neighbor_node in current.expand(self.world):
                if neighbor_node.position.as_tuple() not in visited:
                    self.nodes_created += 1
                    queue.append(neighbor_node)
                    visited.add(neighbor_node.position.as_tuple())
        
        return [], self.nodes_created
    


#_____________________________________________________________________________________________________________

class DFS(SearchAlgorithm):
    """Depth-First Search algorithm implementation."""
    
    def dfs_recursive(self, node: SearchNode, visited: Set[Tuple[int, int]]): # -> Optional[SearchNode]
        """
        Recursive helper method for DFS.

        Args:
            node (SearchNode): The current node
            visited (Set[Tuple[int, int]]): Set of visited positions

        Returns:
            Optional[SearchNode]: The goal node if found, None otherwise
        """
        self.nodes_created += 1
        self.visit_node(node.position)
        
        if self.is_goal(node.position):
            return node
        
        for neighbor_node in node.expand(self.world):
            neighbor_tuple = neighbor_node.position.as_tuple()
            if neighbor_tuple not in visited:
                visited.add(neighbor_tuple)
                result = self.dfs_recursive(neighbor_node, visited)
                if result is not None:
                    return result
        return None
    
    def find_path(self): # -> Tuple[List[Position], int]
        """
        Perform Depth-First Search to find a path.

        Returns:
            Tuple[List[Position], int]: The path and the number of nodes created 
        """
        self.nodes_created = 0
        start_node = SearchNode(self.world.start)
        visited = {self.world.start.as_tuple()}
        
        final_node = self.dfs_recursive(start_node, visited)
        
        if final_node:
            return self.reconstruct_path(final_node), self.nodes_created
        
            
        return [], self.nodes_created


#_____________________________________________________________________________________________________________
   
class GBFS(SearchAlgorithm):
    """Greedy Best-First Search algorithm implementation."""
    
    def heuristic(self, pos: Position): # -> float
        """
        Compute the heuristic value for a given position.

        Args:
            pos (Position): The position to evaluate

        Returns:
            float: The heuristic value (Manhattan distance to the nearest goal)
        """
        return min(abs(pos.x - goal.x) + abs(pos.y - goal.y) for goal in self.world.goals)
    
    def find_path(self): #  -> Tuple[List[Position], int]
        """
        Perform Greedy Best-First Search to find a path.

        Returns:
            Tuple[List[Position], int]: The path and the numnber of nodes created
        """
        start_node = SearchNode(self.world.start)
        frontier = [(self.heuristic(start_node.position), start_node)]
        heapq.heapify(frontier)
        visited = {self.world.start.as_tuple()}
        self.nodes_created = 1
        
        while frontier:
            _, current = heapq.heappop(frontier)
            self.visit_node(current.position)
            
            
            if self.is_goal(current.position):
                return self.reconstruct_path(current), self.nodes_created
            
            for neighbor_node in current.expand(self.world):
                if neighbor_node.position.as_tuple() not in visited:
                    self.nodes_created += 1
                    visited.add(neighbor_node.position.as_tuple())
                    heapq.heappush(frontier, (self.heuristic(neighbor_node.position), neighbor_node))
                    
        return [], self.nodes_created




#_____________________________________________________________________________________________________________  
        
class AStar(SearchAlgorithm):
    """A* Search algorithm implementation."""
        
    def heuristic(self, position: Position, goal: Position): # -> float
        """
        Compute the heuristic value between two positions.

        Args:
            position (Position): The current position
            goal (Position): The goal position

        Returns:
            float: The heuristic value (Manhattan distance)
        """
        return abs(position.x - goal.x) + abs(position.y - goal.y)
    
    def find_path(self): # -> Tuple[List[Position], int]
        """
        Perform A* Search to find a path.

        Returns:
            Tuple[List[Position], int]: The path and the number of nodes created
        """
        start = self.world.start
        goals = self.world.goals
        
        priority_queue = [(0, self.heuristic(start, goals[0]), SearchNode(start))]
        heapq.heapify(priority_queue)
        visited = set()
        self.nodes_created = 0
        traversed = []
        
        while priority_queue:
            _, _, current = heapq.heappop(priority_queue)
            self.nodes_created += 1
            traversed.append(current.position)
            self.visit_node(current.position)
            
            if self.is_goal(current.position):
                return self.reconstruct_path(current), self.nodes_created 
            
            visited.add(current.position.as_tuple())
            
            for neighbor_node in current.expand(self.world):
                if neighbor_node.position.as_tuple() not in visited:
                    g_cost = current.cost + 1
                    h_cost = self.heuristic(neighbor_node.position, goals[0])
                    f_cost = g_cost + h_cost
                    
                    heapq.heappush(priority_queue, (f_cost, h_cost, neighbor_node))
                    visited.add(neighbor_node.position.as_tuple())
                    
        return [], self.nodes_created





#_____________________________________________________________________________________________________________
      
class CUS1(BFS):
    """CUS1: Bidirectional BFS"""
    
    def find_path(self): #  -> Tuple[List[Position], int]
        """
        Perform Bidirectional BFS to find a path.

        Returns:
            Tuple[List[Position], int]: The path and the number of nodes created
        """
        
        # Initialize start node and goal nodes
        start_node = SearchNode(self.world.start)
        goal_nodes = [SearchNode(goal) for goal in self.world.goals]
        
        # Initialize forward and backward queues
        forward_queue = deque([start_node])
        backward_queues = [deque([goal_node]) for goal_node in goal_nodes]
        
        # Initialize visited dictionaries
        forward_visited = {self.world.start.as_tuple(): start_node}
        backward_visited = {goal.as_tuple(): goal_node for goal, goal_node in zip(self.world.goals, goal_nodes)}
        
        self.nodes_created = 1 + len(self.world.goals)  # Start node and goal nodes
        
        while forward_queue and any(backward_queues):
            # Forward search
            if forward_queue:
                result = self._bidirectional_step(forward_queue, forward_visited, backward_visited, is_forward=True)
                if result:
                    return result
            
            # Backward search
            for backward_queue in backward_queues:
                if backward_queue:
                    result = self._bidirectional_step(backward_queue, backward_visited, forward_visited, is_forward=False)
                    if result:
                        return result
        
        return [], self.nodes_created  # No path found
    
    def _bidirectional_step(self, queue: deque, visited: Dict[Tuple[int, int], SearchNode],
                           other_visited: Dict[Tuple[int, int], SearchNode], is_forward: bool): #  -> Optional[Tuple[SearchNode, SearchNode, int]]
        """
        Perform one step of bidirectional search.

        Args:
            queue (deque): The queue for the current direction
            visited (Dict[Tuple[int, int], SearchNode]): Visited nodes in the current direction
            other_visited (Dict[Tuple[int, int], SearchNode]): Visited nodes in the opposite direction
            is_forward (bool): True if this is a forward search step, False for backward

        Returns:
            Optional[Tuple[SearchNode, SearchNode, int]]: The path and nodes created if solution is found, None otherwise
        """
        current = queue.popleft()
        self.visit_node(current.position)
        
        # Check if we've reached a goal in the forward direction
        if is_forward and self.is_goal(current.position):
            return self.reconstruct_path(current), self.nodes_created
        
        for neighbor_node in current.expand(self.world):
            neighbor_tuple = neighbor_node.position.as_tuple()
            
            if neighbor_tuple in other_visited:
                if is_forward:
                    forward_path = self.reconstruct_path(current)
                    backward_path = self.reconstruct_path(other_visited[neighbor_tuple])
                    # return forward_path + list(reversed(backward_path)), self.nodes_created
                else:
                    backward_path = self.reconstruct_path(current)
                    forward_path = self.reconstruct_path(other_visited[neighbor_tuple])
                    # return forward_path + list(reversed(backward_path)), self.nodes_created
                return forward_path + list(reversed(backward_path)), self.nodes_created
            if neighbor_tuple not in visited:
                self.nodes_created += 1
                queue.append(neighbor_node)
                visited[neighbor_tuple] = neighbor_node
        
        return None
    
    
    def _reconstruct_bidirectional_path(self, forward_node: SearchNode, backward_node: SearchNode): #  -> List[Position]
        """
        Reconstruct the complete path from start to goal using the meeting points of forward and backward searches.
        
        This method combines the paths from the forward search (start to meeting point)
        and the backward search (goal to meeting point) to create a complete path.

        Args:
            forward_node (SearchNode): The node where the forward search met the backward search
            backward_node (SearchNode): The node where the backward search met the forward search

        Returns:
            List[Position]: The complete path from start to goal
        
        Notes:
            The backward path is reversed and its first element (the meeting point) is omitted
            to avoid duplicating the meeting point in the final path.
        """
        forward_path = self.reconstruct_path(forward_node)
        backward_path = self.reconstruct_path(backward_node)
        return forward_path + list(reversed(backward_path[1:]))
            



#_____________________________________________________________________________________________________________


class CUS2(SearchAlgorithm):
    """CUS2 : Bidirectional A*"""

    def heuristic(self, position: Position, goals: List[Position]): #  -> float
        """
        Compute the minimum heuristic value to any goal.

        Args:
            position (Position): The current position
            goals (List[Position]): List of possible goals

        Returns:
            float: The minimum heuristic value to any goal
        """
        return min(abs(position.x - goal.x) + abs(position.y - goal.y) for goal in goals)

    def find_path(self): #  -> Tuple[List[Position], int]
        """
        Perform Bidirectional A* Search to find a path.

        Returns:
            Tuple[List[Position], int]: The path and the number of nodes created
        """
        start = self.world.start
        goals = self.world.goals 
        
        
        # Forward search
        forward_queue = [(0, self.heuristic(start, goals), SearchNode(start))]
        forward_visited = {start.as_tuple(): SearchNode(start)}

        # Initialize backward search from all goals
        backward_queue = []
        backward_visited = {}
        for goal in goals:
            heapq.heappush(backward_queue, (0, self.heuristic(goal, [start]), SearchNode(goal)))
            backward_visited[goal.as_tuple()] = SearchNode(goal)

        self.nodes_created = 0

        while forward_queue and backward_queue:
            # Forward search step
            path = self.search_step(forward_queue, forward_visited, backward_visited, goals, True)
            if path:
                return path, self.nodes_created

            # Backward search step
            path = self.search_step(backward_queue, backward_visited, forward_visited, [start], False)
            if path:
                return path, self.nodes_created

        return [], self.nodes_created

    def search_step(self, queue: List, visited: Dict, other_visited: Dict, target: Position, is_forward: bool): #  -> List[Position]
        """
        Perform on step of the bidirectional A* search.

        Args:
            queue (List): Priority queue for the current search direction
            visited (Dict): Dictionary of visited nodes in the current direction
            other_visited (Dict): Dictionary of visited nodes in the opposite direction
            target (Position): The target position (goal for forward search, start for backward search)
            is_forward (bool): True if this is a forward search step, False for backward

        Returns:
            List[Position]: The complete path if a solution is found, None otherwise
        """
        _, _, current = heapq.heappop(queue)
        self.nodes_created += 1
        self.visit_node(current.position)

        if current.position.as_tuple() in other_visited:
            # Path found, reconstruct
            if is_forward:
                forward_path = self.reconstruct_path(current)
                backward_path = self.reconstruct_path(other_visited[current.position.as_tuple()])
                return forward_path + backward_path[::-1][1:]
            else:
                forward_path = self.reconstruct_path(other_visited[current.position.as_tuple()])
                backward_path = self.reconstruct_path(current)
                return forward_path + backward_path[::-1][1:]

        for neighbor_node in current.expand(self.world):
            if neighbor_node.position.as_tuple() not in visited:
                g_cost = current.cost + 1
                h_cost = self.heuristic(neighbor_node.position, target)
                f_cost = g_cost + h_cost
                heapq.heappush(queue, (f_cost, h_cost, neighbor_node))
                visited[neighbor_node.position.as_tuple()] = neighbor_node

        return None

    def reconstruct_path(self, node: SearchNode): #  -> List[Position]:
        """
        Reconstruct the path from the start to the given node.

        Args:
            node (SearchNode): The end node of the path

        Returns:
            List[Position]: The path from start to end
        """
        path = []
        current = node
        while current:
            path.append(current.position)
            current = current.parent
        return path[::-1]
    
    
            