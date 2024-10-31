from strategy.blackboard import Blackboard

from strategy.behaviour import LeafNode, Sequence, Selector
from strategy.behaviour import TaskStatus


class NormalStart(Selector):
    def __init__(self, name):
        super().__init__(name, [])



        
    def run(self):
        return super().run()



class CheckOurPossession(LeafNode):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = Blackboard()

    def run(self):
        if self.blackboard.get("our_possession"):
            return TaskStatus.SUCCESS
        else:
            return TaskStatus.FAILURE




class BallPossession(Selector):
    def __init__(self, name):
        super().__init__(name, [])
        

        is_our_possession = CheckOurPossession("Check Our Possession")
        is_their_possession = LeafNode("Check Their Possession")

        behavier_with_ball = "vai"

        behavier_no_ball = "nao vai"

        our_possession = Sequence("Our Possession", [is_our_possession, behavier_with_ball]) 

        they_possession = Sequence("They Possession", [is_their_possession, behavier_with_ball])

        self.add_children([our_possession, they_possession])



