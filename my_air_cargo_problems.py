from aimacode.logic import PropKB
from aimacode.planning import Action
from aimacode.search import (
    Node, Problem,
)
from aimacode.utils import expr
from lp_utils import (
    FluentState, encode_state, decode_state,
)
from my_planning_graph import PlanningGraph


class AirCargoProblem(Problem):
    def __init__(self, cargos, planes, airports, initial: FluentState, goal: list):
        """

        :param cargos: list of str
            cargos in the problem
        :param planes: list of str
            planes in the problem
        :param airports: list of str
            airports in the problem
        :param initial: FluentState object
            positive and negative literal fluents (as expr) describing initial state
        :param goal: list of expr
            literal fluents required for goal test
        """
        self.state_map = initial.pos + initial.neg
        self.initial_state_TF = encode_state(initial, self.state_map)
        Problem.__init__(self, self.initial_state_TF, goal=goal)
        self.cargos = cargos
        self.planes = planes
        self.airports = airports
        self.actions_list = self.get_actions()

    def get_actions(self):
        '''
        This method creates concrete actions (no variables) for all actions in the problem
        domain action schema and turns them into complete Action objects as defined in the
        aimacode.planning module. It is computationally expensive to call this method directly;
        however, it is called in the constructor and the results cached in the `actions_list` property.

        Returns:
        ----------
        list<Action>
            list of Action objects
        '''

        # TODO create concrete Action objects based on the domain action schema for: Load, Unload, and Fly
        # concrete actions definition: specific literal action that does not include variables as with the schema
        # for example, the action schema 'Load(c, p, a)' can represent the concrete actions 'Load(C1, P1, SFO)'
        # or 'Load(C2, P2, JFK)'.  The actions for the planning problem must be concrete because the problems in
        # forward search and Planning Graphs must use Propositional Logic

        def load_actions():
            '''Create all concrete Load actions and return a list

            :return: list of Action objects
            '''
            loads = []
            # TODO create all load ground actions from the domain Load action
            for a in self.airports:
                for c in self.cargos:
                    for p in self.planes:
                        precond_pos = [expr("At({}, {})".format(c, a)),
                                       expr("At({}, {})".format(p, a))]
                        precond_neg = []
                        effect_add = [expr("In({}, {})".format(c, p))]
                        effect_rem = [expr("At({}, {})".format(c, a))]
                        load = Action(expr("Load({}, {}, {})".format(c, p, a)),
                                     [precond_pos, precond_neg],
                                     [effect_add, effect_rem])
                        loads.append(load)
            return loads
        """
            precond_pos = [expr("At(c, a)"), expr("At(p, a)"), expr("Cargo(c)"), expr("Plane(p)"), expr("Airport(a)")]
            precond_neg = []
            effect_add = [expr("In(c, p)")]
            effect_rem = [expr("At(c, a)")]
            load = Action(expr("Load(c, p, a)"), [precond_pos, precond_neg], [effect_add, effect_rem])
        """
        def unload_actions():
            '''Create all concrete Unload actions and return a list

            :return: list of Action objects
            '''
            unloads = []
            # TODO create all Unload ground actions from the domain Unload action
            for a in self.airports:
                for c in self.cargos:
                    for p in self.planes:
                        precond_pos = [expr("In({}, {})".format(c, p)),
                                       expr("At({}, {})".format(p, a))]
                        precond_neg = []
                        effect_add = [expr("At({}, {})".format(c, a))]
                        effect_rem = [expr("In({}, {})".format(c, p))]
                        unload = Action(expr("UnLoad({}, {}, {})".format(c, p, a)),
                                     [precond_pos, precond_neg],
                                     [effect_add, effect_rem])
                        unloads.append(unload)
            return unloads
        """
        precond_pos = [expr("In(c, p)"), expr("At(p, a)"), expr("Cargo(c)"), expr("Plane(p)"), expr("Airport(a)")]
        precond_neg = []
        effect_add = [expr("At(c, a)")]
        effect_rem = [expr("In(c, p)")]
        unload = Action(expr("Unload(c, p, a)"), [precond_pos, precond_neg], [effect_add, effect_rem])
        """
        def fly_actions():
            '''Create all concrete Fly actions and return a list

            :return: list of Action objects
            '''
            flys = []
            for fr in self.airports:
                for to in self.airports:
                    if fr != to:
                        for p in self.planes:
                            precond_pos = [expr("At({}, {})".format(p, fr))]
                            precond_neg = []
                            effect_add = [expr("At({}, {})".format(p, to))]
                            effect_rem = [expr("At({}, {})".format(p, fr))]
                            fly = Action(expr("Fly({}, {}, {})".format(p, fr, to)),
                                         [precond_pos, precond_neg],
                                         [effect_add, effect_rem])
                            flys.append(fly)
            return flys

        return load_actions() + unload_actions() + fly_actions()
    """
    precond_pos = [expr("At(p, f)"), expr("Plane(p)"), expr("Airport(f)"), expr("Airport(to)")]
    precond_neg = []
    effect_add = [expr("At(p, to)")]
    effect_rem = [expr("At(p, f)")]
    fly = Action(expr("Fly(p, f, to)"), [precond_pos, precond_neg], [effect_add, effect_rem])
    """
    def actions(self, state: str) -> list:
        """ Return the actions that can be executed in the given state.

        :param state: str
            state represented as T/F string of mapped fluents (state variables)
            e.g. 'FTTTFF'
        :return: list of Action objects
        """
        # TODO implement
        possible_actions = []
        kb = PropKB()
        kb.tell(decode_state(state, self.state_map).pos_sentence())
        # print("clauses=",kb.clauses)
        for action in self.actions_list:
            is_possible = True
            for clause in action.precond_pos:
                if clause not in kb.clauses:
                    is_possible = False
            for clause in action.precond_neg:
                if clause in kb.clauses:
                    is_possible = False
            if is_possible:
                possible_actions.append(action)
        return possible_actions
        # possible_actions = self.get_actions()
        # print(len(possible_actions))
        # print(possible_actions)
        #
        # print(len(state))
        #
        # print(list(state))
        # return possible_actions

    def result(self, state: str, action: Action):
        """ Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state).

        :param state: state entering node
        :param action: Action applied
        :return: resulting state after action
        """
        # TODO implement
        # new_state = FluentState([], [])
        # return encode_state(new_state, self.state_map)
        new_state = FluentState([], [])
        old_state = decode_state(state, self.state_map)
        for fluent in old_state.pos:
            if fluent not in action.effect_rem:
                new_state.pos.append(fluent)
        for fluent in action.effect_add:
            if fluent not in new_state.pos:
                new_state.pos.append(fluent)
        for fluent in old_state.neg:
            if fluent not in action.effect_add:
                new_state.neg.append(fluent)
        for fluent in action.effect_rem:
            if fluent not in new_state.neg:
                new_state.neg.append(fluent)
        return encode_state(new_state, self.state_map)

    def goal_test(self, state: str) -> bool:
        """ Test the state to see if goal is reached

        :param state: str representing state
        :return: bool
        """
        kb = PropKB()
        kb.tell(decode_state(state, self.state_map).pos_sentence())
        for clause in self.goal:
            if clause not in kb.clauses:
                return False
        return True

    def h_1(self, node: Node):
        # note that this is not a true heuristic
        h_const = 1
        return h_const

    def h_pg_levelsum(self, node: Node):
        '''
        This heuristic uses a planning graph representation of the problem
        state space to estimate the sum of all actions that must be carried
        out from the current state in order to satisfy each individual goal
        condition.
        '''
        # requires implemented PlanningGraph class
        pg = PlanningGraph(self, node.state)
        pg_levelsum = pg.h_levelsum()
        return pg_levelsum

    def h_ignore_preconditions(self, node: Node):
        '''
        This heuristic estimates the minimum number of actions that must be
        carried out from the current state in order to satisfy all of the goal
        conditions by ignoring the preconditions required for an action to be
        executed.
        '''
        # TODO implement (see Russell-Norvig Ed-3 10.2.3  or Russell-Norvig Ed-2 11.2)
        # print("node=",node)
        # print("node.state",node.state)
        count = 0
        goals = list(self.goal)
        fs = decode_state(node.state, self.state_map)
        passed = [state for state in fs.pos if state in goals]
        return (len(goals) - len(passed))
        # return count

"""
Here is my implementation of ignore-preconditions:
def h_ignore_preconditions(self, node: Node):
        count = 0
        goals = list(self.goal)
        fs = decode_state(node.state, self.state_map)
        passed = [state for state in fs.pos if state in goals]
        return (len(goals) - len(passed))
"""
"""
- Problem 1 initial state and goal:                                
```                                                                
Init(At(C1, SFO) ∧ At(C2, JFK)                                     
	∧ At(P1, SFO) ∧ At(P2, JFK)                                    
	∧ Cargo(C1) ∧ Cargo(C2)                                        
	∧ Plane(P1) ∧ Plane(P2)                                        
	∧ Airport(JFK) ∧ Airport(SFO))                                 
Goal(At(C1, JFK) ∧ At(C2, SFO))                                    
```                                                                
"""
def air_cargo_p1() -> AirCargoProblem:
    cargos = ['C1', 'C2']
    planes = ['P1', 'P2']
    airports = ['JFK', 'SFO']
    pos = [expr('At(C1, SFO)'),
           expr('At(C2, JFK)'),
           expr('At(P1, SFO)'),
           expr('At(P2, JFK)'),
           ]
    neg = [expr('At(C2, SFO)'),
           expr('In(C2, P1)'),
           expr('In(C2, P2)'),
           expr('At(C1, JFK)'),
           expr('In(C1, P1)'),
           expr('In(C1, P2)'),
           expr('At(P1, JFK)'),
           expr('At(P2, SFO)'),
           ]
    init = FluentState(pos, neg)
    goal = [expr('At(C1, JFK)'),
            expr('At(C2, SFO)'),
            ]
    return AirCargoProblem(cargos, planes, airports, init, goal)

"""
- Problem 2 initial state and goal:
      Init(At(C1, SFO) ∧ At(C2, JFK) ∧ At(C3, ATL)                       
        ∧ At(P1, SFO) ∧ At(P2, JFK) ∧ At(P3, ATL)                      
        ∧ Cargo(C1) ∧ Cargo(C2) ∧ Cargo(C3)                            
        ∧ Plane(P1) ∧ Plane(P2) ∧ Plane(P3)                            
        ∧ Airport(JFK) ∧ Airport(SFO) ∧ Airport(ATL))                  
Goal(At(C1, JFK) ∧ At(C2, SFO) ∧ At(C3, SFO))                      
"""
def air_cargo_p2() -> AirCargoProblem:
    # TODO implement Problem 2 definition
    cargos = ['C1', 'C2', 'C3']
    planes = ['P1', 'P2', 'P3']
    airports = ['SFO', 'JFK','ATL']
    pos = [
           expr('At(P1, SFO)'),
           expr('At(P2, JFK)'),
           expr('At(P3, ATL)'),
           expr('At(C1, SFO)'),
           expr('At(C2, JFK)'),
           expr('At(C3, ATL)'),

           ]
    neg = [expr('At(C1, JFK)'),
           expr('At(C1, ATL)'),
           expr('In(C1, P1)'),
           expr('In(C1, P2)'),
           expr('In(C1, P3)'),
           expr('At(C2, SFO)'),
           expr('At(C2, ATL)'),
           expr('In(C2, P1)'),
           expr('In(C2, P2)'),
           expr('In(C2, P3)'),
           expr('At(C3, JFK)'),
           expr('At(C3, SFO)'),
           expr('In(C3, P1)'),
           expr('In(C3, P2)'),
           expr('In(C3, P3)'),

           expr('At(P1, JFK)'),
           expr('At(P1, ATL)'),
           expr('At(P2, SFO)'),
           expr('At(P2, ATL)'),
           expr('At(P3, JFK)'),
           expr('At(P3, SFO)'),
           ]
    init = FluentState(pos, neg)
    goal = [expr('At(C1, JFK)'),
            expr('At(C2, SFO)'),
            expr('At(C3, SFO)'),
            ]
    return AirCargoProblem(cargos, planes, airports, init, goal)

"""                    
- Problem 3 initial state and goal:                                
```                                                                
Init(At(C1, SFO) ∧ At(C2, JFK) ∧ At(C3, ATL) ∧ At(C4, ORD)         
	∧ At(P1, SFO) ∧ At(P2, JFK)     
	                               
Goal(At(C1, JFK) ∧ At(C3, JFK) ∧ At(C2, SFO) ∧ At(C4, SFO))        
```                                                                

	∧ Cargo(C1) ∧ Cargo(C2) ∧ Cargo(C3) ∧ Cargo(C4)                
	∧ Plane(P1) ∧ Plane(P2)                                        
	∧ Airport(JFK) ∧ Airport(SFO) ∧ Airport(ATL) ∧ Airport(ORD))   
"""
def air_cargo_p3() -> AirCargoProblem:
    # TODO implement Problem 3 definition
    cargos = ['C1', 'C2', 'C3', 'C4']
    planes = ['P1', 'P2']
    airports = ['JFK', 'SFO', 'ATL','ORD']
    pos = [
           expr('At(P1, SFO)'),
           expr('At(C1, SFO)'),
           expr('At(P2, JFK)'),
           expr('At(C2, JFK)'),
           expr('At(C3, ATL)'),
           expr('At(C4, ORD)'),
           ]
    neg = [
           expr('At(P1, JFK)'),
           expr('At(C1, JFK)'),
           expr('At(C3, JFK)'),
           expr('At(C4, JFK)'),

           expr('At(P1, ATL)'),
           expr('At(C1, ATL)'),
           expr('At(P2, ATL)'),
           expr('At(C2, ATL)'),
           expr('At(C4, ATL)'),

           expr('At(P1, ORD)'),
           expr('At(C1, ORD)'),
           expr('At(P2, ORD)'),
           expr('At(C2, ORD)'),
           expr('At(C3, ORD)'),

           expr('At(P2, SFO)'),
           expr('At(C2, SFO)'),
           expr('At(C3, SFO)'),
           expr('At(C4, SFO)'),


           expr('In(C1, P1)'),
           expr('In(C1, P2)'),

           expr('In(C2, P1)'),
           expr('In(C2, P2)'),

           expr('In(C3, P1)'),
           expr('In(C3, P2)'),

           expr('In(C4, P1)'),
           expr('In(C4, P2)'),

           ]
    init = FluentState(pos, neg)
    goal = [expr('At(C1, JFK)'),
            expr('At(C3, JFK)'),
            expr('At(C2, SFO)'),
            expr('At(C4, SFO)')
            ]
    return AirCargoProblem(cargos, planes, airports, init, goal)
