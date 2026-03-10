from typing import List, Tuple, Dict, Set
from models import Position
from collections import deque
import heapq
from single_goal_search import SearchNode, BFS, DFS, GBFS, AStar, PriorityQueue, CUS2


class MultiGoalSearchMixin:
    """
    A mixin class that provides functionality for searching multiple goals.
    This class is designed to be used with specific search algorithm classes.
    """
    
    def find_all_goals(self): #  -> Tuple[List[List[Position]], int]
        """
        Find paths to all goals in the world.

        Returns:
            Tuple[List[List[Position]], int]: A tuple containing:
                - A list of paths, where each path is a list of Positions
                - The total number of nodes created during the search
        """
        
        remaining_goals = set(goal.as_tuple() for goal in self.world.goals)
        paths = []
        total_nodes_created = 0
        current_start = self.world.start

        while remaining_goals:
            # Find the path to the next closest goal
            path, nodes = self._find_next_goal(current_start, remaining_goals)
            total_nodes_created += nodes    # Always add the number of nodes created
            
            if not path:
                break   # No more reachable goals, but we've still counted the nodes
            
            paths.append(path)
            
            # Update the starting position for the next search
            current_start = path[-1]
            current_start_tuple = current_start.as_tuple()
            if current_start_tuple in remaining_goals:
                remaining_goals.remove(current_start_tuple)

        return paths, total_nodes_created
    
    def _find_next_goal(self, start: Position, goals: Set[Tuple[int, int]]): #  -> Tuple[List[Position], int]
        """
        Find the path to the next goal. This method should be implemented in specific search algorithm classes.

        Args:
            start (Position): The starting position for the search
            goals (Set[Tuple[int, int]]): A set of remaining goals
        
        Returns:
            Tuple[List[Position], int]: A tuple containing:
                - The path to the next goal
                - The number of nodes created during this search

        Raises:
            NotImplementedError: If not implemenetd in the child class
        """
        
        # This method should be implemented in the specific search algorithm classes
        raise NotImplementedError

    def is_goal(self, position: Position, goals: Set[Tuple[int, int]]): #  -> bool
        """
        Check if the given position is one of the goals.

        Args:
            position (Position): The position to check
            goals (Set[Tuple[int, int]]): A set of goal positions

        Returns:
            bool: True if the position is a goal, False otherwise
        """
        return position.as_tuple() in goals

#____________________________________________________________________________________________________________________________________

class MultiGoalBFS(MultiGoalSearchMixin, BFS):
    """Multi-goal Breath-First Search algorithm implementation."""
    
    def _find_next_goal(self, start: Position, goals: Set[Tuple[int, int]]): # -> Tuple[List[Position], int]
        """
        Find the path to the next goal using Breath-First Search.

        Args:
            start (Position): The starting position for the search
            goals (Set[Tuple[int, int]]): A set of remaining goals

        Returns:
            Tuple[List[Position], int]: A tuple containing:
                - The path to the next goal
                - The number of nodes created during this search
        """
        queue = deque([SearchNode(start)])
        visited = {start.as_tuple()}
        
        nodes_created = 1
        
        while queue:
            current = queue.popleft()
            self.visit_node(current.position)
            
            if self.is_goal(current.position, goals):
                return self.reconstruct_path(current), nodes_created
            
            for neighbor_node in current.expand(self.world):
                if neighbor_node.position.as_tuple() not in visited:
                    nodes_created += 1
                    queue.append(neighbor_node)
                    visited.add(neighbor_node.position.as_tuple())
        
        return [], nodes_created  # Return empty path if no goal is found
    
    
#____________________________________________________________________________________________________________________________________
    
class MultiGoalDFS(MultiGoalSearchMixin, DFS):
    """Multi-goal Depth-First Search algorithm implementation."""
    
    def _find_next_goal(self, start: Position, goals: Set[Tuple[int, int]]): # -> Tuple[List[Position], int]
        """
        Find the path to the next goal using Depth-First Search.

        Args:
            start (Position): The starting position for the search
            goals (Set[Tuple[int, int]]): A set of remaining goals

        Returns:
            Tuple[List[Position], int]: A tuple containing:
                - The path to the next goal
                - The number of nodes created during this search
        """
        self.nodes_created = 0
        start_node = SearchNode(start)
        visited = {start.as_tuple()}
        
        final_node = self._dfs_recursive(start_node, visited, goals)
        
        if final_node:
            return self.reconstruct_path(final_node), self.nodes_created
        return [], self.nodes_created
    
    def _dfs_recursive(self, node: SearchNode, visited: Set[Tuple[int, int]], goals: Set[Tuple[int, int]]): #  -> Optional[SearchNode]
        """
        Recursive helper method for Depth-First Search

        Args:
            node (SearchNode): The current node being explored
            visited (Set[Tuple[int, int]]): Set of visited positions
            goals (Set[Tuple[int, int]]): Set of goal positions

        Returns:
            Optional[SearchNode]: The goal node if found, None otherwise
        """
        self.nodes_created += 1
        self.visit_node(node.position)
        
        if self.is_goal(node.position, goals):
            return node
        
        for neighbor_node in node.expand(self.world):
            neighbor_tuple = neighbor_node.position.as_tuple()
            if neighbor_tuple not in visited:
                visited.add(neighbor_tuple)
                result = self._dfs_recursive(neighbor_node, visited, goals)
                if result is not None:
                    return result
        return None
    

#____________________________________________________________________________________________________________________________________
        
class MultiGoalGBFS(MultiGoalSearchMixin, GBFS):
    """Multi-goal Greedy Best-First Search algorithm implementation."""

    def _heuristic(self, pos: Position, goals: Set[Tuple[int, int]]): # -> float
        """
        Calculate the heuristic value for a given position.

        Args:
            pos (Position): The position to evaluate
            goals (Set[Tuple[int, int]]): Set of remaining goal positions

        Returns:
            float: The minimum Manhattan distance to any of the goals
        """
        return min(abs(pos.x - goal[0]) + abs(pos.y - goal[1]) for goal in goals)
    
    def _find_next_goal(self, start: Position, goals: Set[Tuple[int, int]]): #  -> Tuple[List[Position], int]
        """
        Find the path to the next goal using Greedy Best-First Search

        Args:
            start (Position): The starting position for the search
            goals (Set[Tuple[int, int]]): A set of remaining goals

        Returns:
            Tuple[List[Position], int]: A tuple containing:
                - The path to the next goal
                - The number of nodes created during this search
        """
        start_node = SearchNode(start)
        frontier = [(self._heuristic(start_node.position, goals), start_node)]
        heapq.heapify(frontier)
        visited = {start.as_tuple()}
        
        nodes_created = 1
        
        while frontier:
            _, current = heapq.heappop(frontier)
            self.visit_node(current.position)
            
            if self.is_goal(current.position, goals):
                return self.reconstruct_path(current), nodes_created
            
            for neighbor_node in current.expand(self.world):
                if neighbor_node.position.as_tuple() not in visited:
                    nodes_created += 1
                    visited.add(neighbor_node.position.as_tuple())
                    heapq.heappush(frontier, (self._heuristic(neighbor_node.position, goals), neighbor_node))
        return [], nodes_created


#____________________________________________________________________________________________________________________________________
    
class MultiGoalAStar(MultiGoalSearchMixin, AStar):
    """
    A multi-goal search algorithm that uses A* to find paths to multiple goals.
    This class implements a search strategy that finds the nearest goal and then moves on to the next one.
    """
    
    def _heuristic(self, pos: Position, goals: Set[Tuple[int, int]]): # -> float
        """
        Calculate the heuristic value for a given position and set of goals.

        Args:
            pos (Position): The current position
            goals (Set[Tuple[int, int]]): A set of goal positions

        Returns:
            float: The minimum Manhattan distance to any of the goals
        """
        return min(abs(pos.x - goal[0]) + abs(pos.y - goal[1]) for goal in goals)
    
    def _find_next_goal(self, start: Position, goals: Set[Tuple[int, int]]): # -> Tuple[List[Position], int]
        """
        Find the path to the nearest goal using A* search.

        Args:
            start (Position): The starting position
            goals (Set[Tuple[int, int]]): A set of goal positions

        Returns:
            Tuple[List[Position], int]: A tuple containing:
                - The path to the nearest goal
                - The total number of nodes created
        """
        
        # Initialize the start node with cost 0 and calculated heuristic 
        start_node = SearchNode(start, cost=0, heuristic=self._heuristic(start, goals))
        
        # Initialize the priority queue (frontier) with the start bode
        frontier = PriorityQueue('min', lambda n: (n.cost + n.heuristic, n.position.x, n.position.y))
        frontier.append(start_node)
        
        # Initialize the g_scores dictionary with the start node's cost
        g_scores = {start.as_tuple(): 0}
        nodes_created = 1 # Count the start node
        
        while frontier:
            current = frontier.pop() # Get the node with the lowest f_score
            current_tuple = current.position.as_tuple()
            
            # If we've found a better path to this node, skip it
            if current.cost > g_scores[current_tuple]:
                continue
            
            self.visit_node(current.position) # Make the node as visited 
            
            # Check if we've reached a goal
            if self.is_goal(current.position, goals):
                return self.reconstruct_path(current), nodes_created
            
            # Get and sort neighbors to ensure deterministic behavior
            neighbors = sorted(self.world.get_neighbors(current.position), key=lambda p: (p.x, p.y))
            
            for neighbor in neighbors:
                neighbor_tuple = neighbor.as_tuple()
                
                # Calculate the tentative g_score for this neighbor
                tentative_g_score = current.cost + 1
                if neighbor_tuple not in g_scores or tentative_g_score < g_scores[neighbor_tuple]:
                    # Create a new node for this neighbor
                    child = SearchNode(neighbor, parent=current, cost=tentative_g_score, heuristic=self._heuristic(neighbor, goals))
                    
                    # If this is a new node, increment the counter
                    if neighbor_tuple not in g_scores:
                        nodes_created += 1
                    
                    # Update the g_score and add the node to the frontier
                    g_scores[neighbor_tuple] = tentative_g_score
                    frontier.append(child)
        
        # If we've exhausted all nodes without finding a goal, return an empty path
        return [], nodes_created
    

#____________________________________________________________________________________________________________________________________

class MultiGoalCUS1(MultiGoalSearchMixin, BFS): # Bidirectional BFS
    """
    A multi-goal search algorithm that uses a bidirectional BFS approach.
    This class implements a search strategy that simultaneously explores from the start towards all goals
    and from each goal towards the start, finding the nearest goal first.
    """
    def _find_next_goal(self, start: Position, goals: Set[Tuple[int, int]]): #  -> Tuple[List[Position], int]
        """
        Find the path to the nearest goal using Bidirectional BFS

        Args:
            start (Position): The starting position for the search
            goals (Set[Tuple[int, int]]): A set of remaining goals to reach

        Returns:
            Tuple[List[Position], int]: A tuple containing:
                - The path to the nearest goal
                - The number of nodes created in this search
        """
        
        # Initialize start and goal nodes
        start_node = SearchNode(start)
        goal_nodes = [SearchNode(Position(x, y)) for x, y in goals]
        
        # Initialize queues for forward and backward searches
        forward_queue = deque([start_node])
        backward_queues = {goal.position.as_tuple(): deque([goal]) for goal in goal_nodes}
        
        # Initialize visited dictionaries
        forward_visited = {start.as_tuple(): start_node}
        backward_visited = {goal.position.as_tuple(): {goal.position.as_tuple(): goal} for goal in goal_nodes}
        
        nodes_created = 1 + len(goal_nodes) # Count initial nodes
        
        while forward_queue and any(backward_queues.values()):
            # Forward search step
            if forward_queue:
                result = self._bidirectional_step(forward_queue, forward_visited, backward_visited, is_forward=True, goals=goals)
                if result:
                    return result

            # Backward search step for each goal
            for goal_pos, queue in list(backward_queues.items()):
                if queue:
                    result = self._bidirectional_step(queue, backward_visited[goal_pos], forward_visited, is_forward=False, goals={goal_pos})
                    if result:
                        return result

        return [], nodes_created # No path found

    def _bidirectional_step(self, queue: deque, visited: Dict[Tuple[int, int], SearchNode],
                            other_visited: Dict[Tuple[int, int], Dict[Tuple[int, int], SearchNode]], 
                            is_forward: bool, goals: Set[Tuple[int, int]]): # -> Optional[Tuple[List[Position], int]]
        """Perform one step of the bidirectional search.

        Args:
            queue (deque): Queue of nodes to explore
            visited (Dict[Tuple[int, int], SearchNode]): Dictionary of visited nodes in the current direction
            other_visited (Dict[Tuple[int, int], Dict[Tuple[int, int], SearchNode]]): Dictionary of visited nodes in the opposite direction
            is_forward (bool): True if this iis a forward search step, False for backward 
            goals (Set[Tuple[int, int]]): Set of goal positions

        Returns:
            Optional[Tuple[List[Position], int]]: The path and number of nodes created if a goal is reached, None otherwise
        """
        
        # Check if we've reached a goal in the forward direction
        current = queue.popleft()
        self.visit_node(current.position)

        if is_forward and current.position.as_tuple() in goals:
            return self.reconstruct_path(current), len(visited) + sum(len(v) for v in other_visited.values())

        for neighbor_node in current.expand(self.world):
            neighbor_tuple = neighbor_node.position.as_tuple()

            if is_forward:
                # Check if we've met a backward search
                for goal_visited in other_visited.values():
                    if neighbor_tuple in goal_visited:
                        forward_path = self.reconstruct_path(current) + [neighbor_node.position]
                        backward_path = self.reconstruct_path(goal_visited[neighbor_tuple])
                        return forward_path + list(reversed(backward_path)), len(visited) + sum(len(v) for v in other_visited.values())
            else:
                # Check if we've met the forward search
                if neighbor_tuple in other_visited:
                    backward_path = self.reconstruct_path(current) + [neighbor_node.position]
                    forward_path = self.reconstruct_path(other_visited[neighbor_tuple])
                    return forward_path + list(reversed(backward_path)), len(visited) + len(other_visited)
            
            # Add the neighbor to the queue if it hasn't been visited
            if neighbor_tuple not in visited:
                queue.append(neighbor_node)
                visited[neighbor_tuple] = neighbor_node

        return None # No path found in this step
    

#____________________________________________________________________________________________________________________________________

class MultiGoalCUS2(MultiGoalSearchMixin, CUS2): # Bidirectional A*
    """
    A multi-goal search algorithm that uses Bidirectional A* approach.
    This class implements a search strategy that simultaneously explores from the start towards
    multiple goals and from each goal towards the start, finding efficient path to all goals.
    """
    
    def find_all_goals(self):   #  -> Tuple[List[List[Position]], int]
        """
        Find paths to all goals in the world.

        Returns:
            Tuple[List[List[Position]], int]: A tuple containing:
                - A list of paths, where each path is a list of Positions
                - The total number of nodes created during the search
        """
        
        remaining_goals = set(goal.as_tuple() for goal in self.world.goals)
        paths = []
        total_nodes_created = 0
        current_start = self.world.start

        while remaining_goals:
            # Find the next goal
            path, nodes, goal_reached = self._find_next_goal(current_start, remaining_goals)
            total_nodes_created += nodes
            
            if not path:
                break # No more reachable goal
            
            paths.append(path)
            
            if goal_reached in remaining_goals:
                remaining_goals.remove(goal_reached)
            current_start = path[-1] # Set the end of the current path as the new start

        return paths, total_nodes_created

    def _find_next_goal(self, start: Position, goals: Set[Tuple[int, int]]):    # -> Tuple[List[Position], int, Optional[Tuple[int, int]]]
        """
        Find the path to the next closest goal using Bidirectional A*

        Args:
            start (Position): The starting position for the search
            goals (Set[Tuple[int, int]]): A set of remaining goals to reach

        Returns:
            Tuple[List[Position], int, Optional[Tuple[int, int]]]: A tuple containing:
                - The path ot the reached goal
                - The number of nodes created in this search
                - The goal that was reached (or None if no goal was reached)
        """
        
        if not goals:  # Check if there are any goals to search for
            return [], 0, None

        # Initialize forward search
        start_node = SearchNode(start)
        forward_queue = [(0, 0, start_node)]    # Priority queue: (f_score, g_score, node)
        forward_g_scores = {start.as_tuple(): 0}
        forward_f_scores = {start.as_tuple(): min(self._heuristic(start, Position(*goal), 0) for goal in goals)}
        forward_visited = {}

        # Initialize backward searches for each goal
        backward_queues = {}
        backward_g_scores = {}
        backward_f_scores = {}
        backward_visited = {}
        active_goals = set(goals)
        
        for goal in goals:
            goal_pos = Position(*goal)
            goal_node = SearchNode(goal_pos)
            backward_queues[goal] = [(0, 0, goal_node)]
            backward_g_scores[goal] = {goal: 0}
            backward_f_scores[goal] = {goal: self._heuristic(goal_pos, start, 0)}
            backward_visited[goal] = {}

        nodes_created = 1 + len(goals)  # Count initial nodes
        mu = float('inf')    # Upper bound for the optimal solution
        
        while forward_queue or any(backward_queues[goal] for goal in active_goals):
            if not active_goals:  # No more reachable goals
                return [], nodes_created, None

            # Forward search step
            if forward_queue:
                forward_result = self._bidirectional_step(
                    forward_queue, forward_g_scores, forward_f_scores, forward_visited,
                    backward_visited, active_goals, True, mu, nodes_created
                )
                if forward_result:
                    path, goal_reached = forward_result
                    return path, nodes_created, goal_reached

            # Backward search step for each active goal
            goals_to_remove = set()
            for goal in active_goals:
                queue = backward_queues[goal]
                if not queue:  # This goal's backward search is exhausted
                    goals_to_remove.add(goal)
                    continue
                    
                backward_result = self._bidirectional_step(
                    queue, backward_g_scores[goal], backward_f_scores[goal],
                    backward_visited[goal], forward_visited, {start.as_tuple()},
                    False, mu, nodes_created
                )
                
                if backward_result:
                    path, _ = backward_result
                    return path, nodes_created, goal

            # Remove exhausted goals
            active_goals -= goals_to_remove
            
            # If no active goals remain, terminate search
            if not active_goals:
                return [], nodes_created, None

            # Update mu (upper bound) considering only active goals
            if forward_queue and active_goals:
                forward_best = forward_queue[0][0]
                backward_best = float('inf')
                for goal in active_goals:
                    if backward_queues[goal]:
                        goal_best = backward_queues[goal][0][0]
                        backward_best = min(backward_best, goal_best)
                if backward_best != float('inf'):
                    mu = min(mu, forward_best + backward_best)

            nodes_created += 1

        return [], nodes_created, None

    def _bidirectional_step(self, queue, g_scores, f_scores, visited, other_visited, 
                           targets, is_forward, mu, nodes_created): # -> Optional[Tuple[List[Position], Tuple[int, int]]]
        """
        Perform one step of the bidirectional search.

        Args:
            queue (List): Priority queue of nodes to explore
            g_scores (Dict): Dictionary of g-scores for each node
            f_scores (Dict): Dictionary of f-scores for each node
            visited (Dict): Dictionary of visited nodes
            other_visited (Dict): Dictionary of visited nodes in the opposite direction
            targets (Set): Set of target positions
            is_forward (bool): True if this is a forward search step, False for backward
            mu (float): Current upper bound for the optimal solution
            nodes_created (int): Current cound of created nodes

        Returns:
            Optional[Tuple[List[Position], Tuple[int, int]]]: The path and reached goal if found, None otherwise
        """
        
        if not queue or not targets:  # Check for empty targets
            return None

        _, g_cost, current = heapq.heappop(queue)
        current_tuple = current.position.as_tuple()

        if current_tuple in visited:
            return None

        visited[current_tuple] = current
        self.visit_node(current.position)

        # Check if we've reached a target
        if is_forward and current_tuple in targets:
            return self.reconstruct_path(current), current_tuple

        # Check for intersection with other direction
        if is_forward:
            for goal, other_visited_dict in other_visited.items():
                if goal in targets and current_tuple in other_visited_dict:
                    path = self._reconstruct_bidirectional_path(
                        current, other_visited_dict[current_tuple], is_forward
                    )
                    return path, goal
        else:
            if current_tuple in other_visited:
                path = self._reconstruct_bidirectional_path(
                    current, other_visited[current_tuple], is_forward
                )
                return path, path[-1].as_tuple()

        # Expand neighbors
        neighbors = self.world.get_neighbors(current.position)
        for neighbor in neighbors:
            neighbor_tuple = neighbor.as_tuple()
            tentative_g_score = g_cost + 1

            if tentative_g_score < g_scores.get(neighbor_tuple, float('inf')):
                neighbor_node = SearchNode(neighbor, parent=current)
                g_scores[neighbor_tuple] = tentative_g_score

                try:
                    if is_forward:
                        h_score = min(self._heuristic(neighbor, Position(*target), tentative_g_score) 
                                    for target in targets)
                    else:
                        target = next(iter(targets))
                        h_score = self._heuristic(neighbor, Position(*target), tentative_g_score)

                    f_score = tentative_g_score + h_score
                    f_scores[neighbor_tuple] = f_score

                    if f_score <= mu:
                        heapq.heappush(queue, (f_score, tentative_g_score, neighbor_node))
                except ValueError:  # Handle case where targets becomes empty
                    continue

        return None
    
    def _reconstruct_bidirectional_path(self, node1, node2, is_forward): #  -> List[Position]
        """
        Reconstruct the path found by Bidirectional A*.

        Args:
            node1 (SearchNode): The node where the two searches met from one direction
            node2 (SearchNode): The node where the two searches met from the other direction
            is_forward (bool): True if node1 is from the forward search, False otherwise

        Returns:
            List[Position]: The reconstructed path from start to goal
        """
        path1 = self.reconstruct_path(node1)
        path2 = self.reconstruct_path(node2)
        if is_forward:
            return path1 + path2[::-1][1:]
        else:
            return path2 + path1[::-1][1:]
    
    
    def _heuristic(self, pos: Position, goal: Position, g_cost: float): # -> float
        """
        Calculate the Manhattan distance heuristic between current position and goal.
        
        This method estimates the remaining distance from the current position to the goal
        by summing the absolute differences in x and y coordinates (Manhattan distance).

        Args:
            pos (Position): Current position with x,y coordinates
            goal (Position): Goal position with x,y coordinates
            g_cost (float): Cost from start to current position (not used in this heuristic)

        Returns:
            float: Estimated distance from current position to goal position
        """
        return abs(pos.x - goal.x) + abs(pos.y - goal.y)
