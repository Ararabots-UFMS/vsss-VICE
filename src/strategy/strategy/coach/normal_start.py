import math
from strategy.blackboard import Blackboard

from strategy.behaviour import LeafNode, Sequence, Selector
from strategy.behaviour import TaskStatus
from strategy.coach.running.Defense_play import DefensivePlay
from strategy.robots.penalty.our_penalty.goalkeeper import OurGoalkeeperAction
from strategy.robots.running.attacker import OurActionAttacker
from strategy.robots.running.defensive import OurActionDefender
from strategy.robots.halt.defender import ActionDefender
from strategy.coach.running.command import LastCommand

class CheckZone(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = Blackboard()
        self.name = name
        self.ball = self.blackboard.balls[0]
        self.padding = 500
        for line in self.blackboard.geometry.field_lines:
            if line.name == 'LeftGoalLine':
                self.left_goal = line.x1
            elif line.name == 'CenterLine':
                self.middle_left =  -1*self.padding
                self.middle_right = +1*self.padding
            elif line.name == 'RightGoalLine':
                self.right_goal = line.x1

    def run(self):
        if self.blackboard.gui.is_field_side_left:
            if self.name == "DefenseZone":
                if self.ball.position_x >= self.left_goal and self.ball.position_x < self.middle_left:
                    return TaskStatus.SUCCESS, None
            elif self.name == "MiddleZone":
                if self.ball.position_x >= self.middle_left and self.ball.position_x <= self.middle_right:
                    return TaskStatus.SUCCESS, None
            elif self.name == "AttackZone":
                if self.ball.position_x > self.middle_right and self.ball.position_x <= self.right_goal:
                    return TaskStatus.SUCCESS, None
        else:
            if self.name == "AttackZone":
                if self.ball.position_x >= self.left_goal and self.ball.position_x < self.middle_left:
                    return TaskStatus.SUCCESS, None
            elif self.name == "MiddleZone":
                if self.ball.position_x >= self.middle_left and self.ball.position_x <= self.middle_right:
                    return TaskStatus.SUCCESS, None
            elif self.name == "DefenseZone":
                if self.ball.position_x > self.middle_right and self.ball.position_x <= self.right_goal:
                    return TaskStatus.SUCCESS, None
        
        return TaskStatus.FAILURE, None

class IsOurDefense(LeafNode):
    def __init__(self, name):
        self.name =name
        self.blackboard = Blackboard()
        self.commands = {}

    def run(self):
        self.points = DefensivePlay().run()
        for robot in self.blackboard.ally_robots:
            if robot != self.blackboard.referee.teams[self.blackboard.gui.is_team_color_yellow].goalkeeper:
                if robot in self.points and self.points[robot] is not None:
                    self.commands[robot] = OurActionDefender("Defend!!!", self.points[robot], robot)
            else:
                self.commands[robot] = OurGoalkeeperAction("name")


        return TaskStatus.SUCCESS, self.commands
        

class IsOurAttack(LeafNode):
    def __init__(self, name):
        self.name =name
        self.blackboard = Blackboard()
        self.commands = {}

    def run(self):
        for robot in self.blackboard.ally_robots:
            if robot != self.blackboard.referee.teams[self.blackboard.gui.is_team_color_yellow].goalkeeper:
                self.commands[robot] = OurActionAttacker("Attack!!!", robot)
            else:
                self.commands[robot] = OurGoalkeeperAction("name")

        return TaskStatus.SUCCESS, self.commands
    

class CheckStart(LeafNode):
    def __init__(self, name, commands):
        self.blackboard = Blackboard()
        self.name = name
        self.commands = commands

    def run(self):
        if self.blackboard.referee.command in self.commands:
            return TaskStatus.SUCCESS, None
        
        return TaskStatus.FAILURE, None


class Running(Sequence):
    def __init__(self, name):
        super().__init__(name, [])
        self.last_command = LastCommand().get_command()
        self.test = LastCommand().get_command()
        self.blackboard = Blackboard()    
        
        commands = ["NORMAL_START", "FORCE_START"]

        is_running = CheckStart("CheckStart", commands)
        
        attack_action = IsOurAttack("IsOurAttack")

        defense_action = IsOurDefense("IsOurDefense")

        attack_zone = CheckZone("AttackZone")

        middle_zone = CheckZone("MiddleZone")

        defense_zone = CheckZone("DefenseZone")

        attack = Sequence("AttackSequence", [attack_zone, attack_action])

        if self.last_command == None:
            middle = Sequence("MiddleZone", [middle_zone, IsOurAttack("name")])
        else:
            middle = Sequence("MiddleZone", [middle_zone, self.last_command])

        defend = Sequence("DefenseSequence", [defense_zone, defense_action])
           
        running_selector = Selector("RunningSelector", [defend, middle, attack])

        self.add_children([is_running, running_selector])


    def run(self):
        
        command = super().run()
        dict = command[1]
        robot_id = 0
        if self.blackboard.referee.teams != []:
            for robot in self.blackboard.ally_robots:
                if self.blackboard.referee.teams[self.blackboard.gui.is_team_color_yellow].goalkeeper != robot:
                    robot_id = robot

        if dict != None:
            if isinstance(dict[robot_id], OurActionAttacker):
                self.last_command = IsOurAttack("IsOurAttack")
                LastCommand().set_command(self.last_command)
            else:
                self.last_command = IsOurDefense("IsOurDefense")
                LastCommand().set_command(self.last_command)
        return command


       
if __name__ == "__main__":
    root = Running("StartRoot")
    root.run()