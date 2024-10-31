from strategy.blackboard import Blackboard

from strategy.behaviour import LeafNode, Sequence, Selector
from strategy.behaviour import TaskStatus


class NormalStart(Selector):
    def __init__(self, name):
        super().__init__(name, [])

        

    def run():
        pass


class CheckBallPossession(Selector):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = Blackboard()

        

    def run():
        pass