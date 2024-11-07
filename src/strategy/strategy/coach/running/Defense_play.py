
import math
from strategy.behaviour import LeafNode, Selector, TaskStatus
from strategy.blackboard import Blackboard
from strategy.skill.route import BreakStrategy, GetInAngleStrategy
from strategy.coach.running.utils import DefenseTriangle, PenaltyArea

class GetPoints():
    def __init__(self):
        self.blackboard = Blackboard()
        self.triangle = DefenseTriangle()
        self.penalty_area = PenaltyArea()
        self.points_to_defend = []

    def run(self):
        self.find_points()
        return self.points_to_defend
    
    def find_intersection(self, m1, n1, m2, n2):
        # Check if lines are parallel
        if m1 == m2:
            return None  # No intersection if lines are parallel
        
        # Calculate intersection points
        x = (n2 - n1) / (m1 - m2)
        y = m1 * x + n1
        return x, y
    
    def find_points(self):
        # Get triangle and penalty area line equations
        m_triangle_left, n_triangle_left = self.triangle.draw_line_left()
        m_triangle_right, n_triangle_right = self.triangle.draw_line_right()
        
        m_penalty_left, n_penalty_left = self.penalty_area.draw_line_left()
        m_penalty_right, n_penalty_right = self.penalty_area.draw_line_rigth()
        x_front = self.penalty_area.draw_line_front()  # x constant for the front line

        # Find intersection between left lines
        left_intersection = self.find_intersection(m_triangle_left, n_triangle_left, m_penalty_left, n_penalty_left)
        if left_intersection:
            self.points_to_defend.append(left_intersection)
        
        # Find intersection between right lines
        right_intersection = self.find_intersection(m_triangle_right, n_triangle_right, m_penalty_right, n_penalty_right)
        if right_intersection:
            self.points_to_defend.append(right_intersection)
        
        # Find intersection with the front line (vertical line)
        # Front line has constant x = x_front, so we use triangle lines to find y at this x.
        
        # Intersection of front line with left triangle line
        if m_triangle_left != 0:  # Avoid division by zero in case of horizontal line
            y_left_front = m_triangle_left * x_front + n_triangle_left
            self.points_to_defend.append((x_front, y_left_front))
        
        # Intersection of front line with right triangle line
        if m_triangle_right != 0:  # Avoid division by zero in case of horizontal line
            y_right_front = m_triangle_right * x_front + n_triangle_right
            self.points_to_defend.append((x_front, y_right_front))

        self.remove_invalid_points()


    def remove_invalid_points(self):
        for point in self.points_to_defend[:]:
            if abs(point[0]) > 2250 or abs(point[0]) < 1750 or abs(point[1]) > 675:
                self.points_to_defend.remove(point) 

class DefensivePlay():
    def __init__(self):
        self.blackboard = Blackboard()
        self.points = GetPoints().run()
        self.distances = {}
        self.assignments = {}

    def run(self):
        self.distance_p2r()
        self.distribute_points()
        return dict(sorted(self.assignments.items()))
    
    """Calculate the distance between a point and a robot"""
    def distance_p2r(self):
        for robot in self.blackboard.ally_robots:
            self.distances[robot] = []
            for point in self.points:
                distance = math.sqrt((point[0] - self.blackboard.ally_robots[robot].position_x) ** 2 + (point[1] - self.blackboard.ally_robots[robot].position_y) ** 2)
                self.distances[robot].append(distance)
    
    """Distribute the position to each defender choosing the most close to point"""
    def distribute_points(self):
        # Initialize a set to keep track of assigned points
        assigned_points = set()
        self.assignments = {}  # Store each robot's assigned point

        # Sort robots by minimum distance to points
        sorted_robots = sorted(self.distances.items(), key=lambda item: min(item[1]))

        # Assign each robot the closest available point
        for robot, distances in sorted_robots:
            closest_point = None
            min_distance = float('inf')

            # Find the closest unassigned point for this robot
            for i, distance in enumerate(distances):
                if distance < min_distance and i not in assigned_points:
                    min_distance = distance
                    closest_point = i  # Index of the closest point

            # Assign this closest point to the robot if found
            if closest_point is not None:
                self.assignments[robot] = self.points[closest_point]
                assigned_points.add(closest_point)  # Mark this point as assigned
                


        
            


