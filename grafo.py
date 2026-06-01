class Graph:
    def __init__(self):
        self.adjacency_list = {}
        self.positions = {}
        self._build_positions()
        self._build_edges()

    def _add_node(self, node, x, y):
        self.positions[node] = (x, y)
        self.adjacency_list[node] = []

    def _add_edge(self, node_a, node_b, travel_time):
        self.adjacency_list[node_a].append((node_b, travel_time))
        self.adjacency_list[node_b].append((node_a, travel_time))

    def _build_positions(self):
        self._add_node("C1", 0, 5)
        self._add_node("C4", 3, 5)
        self._add_node("C6", 5, 5)
        self._add_node("C7", 7, 5)
        self._add_node("C8", 9, 5)

        self._add_node("C9", 1, 3)
        self._add_node("C10", 4, 3)
        self._add_node("C11", 6, 3)
        self._add_node("C12", 8, 3)

        self._add_node("C13", 3, 1)
        self._add_node("C14", 6, 1)

        self._add_node("C5", 10, 6.5)
        self._add_node("C3", 12, 7.2)
        self._add_node("C2", 14, 7.8)

        self._add_node("R1", 1, 6)
        self._add_node("R2", 5, 6.2)
        self._add_node("R3", 2, 2)
        self._add_node("R4", 7, 1.5)
        self._add_node("R5", 9.5, 4)
        self._add_node("R6", 5, 3.8)
        self._add_node("R7", 13, 6.2)

    def _build_edges(self):
        self._add_edge("C1", "R1", 18)
        self._add_edge("C1", "C9", 24)
        self._add_edge("R1", "C4", 22)

        self._add_edge("C4", "R2", 17)
        self._add_edge("C4", "C10", 26)
        self._add_edge("R2", "C6", 18)
        self._add_edge("R2", "C7", 25)

        self._add_edge("C6", "C7", 19)
        self._add_edge("C6", "R6", 13)
        self._add_edge("C7", "C8", 22)
        self._add_edge("C7", "C11", 25)

        self._add_edge("C8", "R5", 18)
        self._add_edge("R5", "C12", 24)

        self._add_edge("C9", "R3", 16)
        self._add_edge("C9", "C10", 31)
        self._add_edge("R3", "C13", 22)

        self._add_edge("C10", "R6", 14)
        self._add_edge("C10", "C11", 20)
        self._add_edge("C10", "C13", 24)

        self._add_edge("C11", "R6", 17)
        self._add_edge("C11", "C12", 21)
        self._add_edge("C11", "C14", 28)

        self._add_edge("C12", "R4", 19)
        self._add_edge("R4", "C14", 17)
        self._add_edge("C13", "C14", 30)

        self._add_edge("C8", "C5", 20)
        self._add_edge("C5", "C3", 23)
        self._add_edge("C3", "C2", 21)
        self._add_edge("C3", "R7", 12)

    def get_neighbors(self, node):
        return self.adjacency_list[node]

    def is_collection_node(self, node):
        return node.startswith("C")

    def is_recharge_node(self, node):
        return node.startswith("R")

    def has_node(self, node):
        return node in self.adjacency_list
