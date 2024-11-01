from strategy.blackboard import Blackboard

from strategy.behaviour import LeafNode, Sequence, Selector
from strategy.behaviour import TaskStatus

class CheckCondition(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.name = name

    def run(self):
        status = True
        if status:
            return TaskStatus.SUCCESS, None
        else:
            return TaskStatus.FAILURE, None



class IsTheirPossession(LeafNode):
    def __init__(self, name):
        self.name =name
        
    def run(self):
        return TaskStatus.SUCCESS, "IS_THEIR_POSSESSION"
        

class IsOurPossession(LeafNode):
    def __init__(self, name):
        self.name =name

    def run(self):    
        return TaskStatus.SUCCESS, "IS_OUR_POSSESSION"



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

        self.blackboard = Blackboard()
        
        commands = ["NORMAL_START", "FORCE_START"]

        is_running = CheckStart("CheckStart", commands)
        
        ours_action = IsOurPossession("IsOurPossession")

        ours_with_ball = CheckCondition("CheckOurCondition")

        theirs_action = IsTheirPossession("IsTheirPossession")

        theirs_with_ball = CheckCondition("CheckTheirCondition")

        ours = Sequence("OursNormalStart", [ours_with_ball, ours_action])

        theirs = Sequence("TheirNormalStart", [theirs_with_ball, theirs_action])

        running_selector = Selector("RunningSelector", [theirs, ours])


        self.add_children([is_running, running_selector])

    def run(self):
        return super().run()

       
if __name__ == "__main__":
    root = Running("StartRoot")
    root.run()