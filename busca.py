import heapq
from dataclasses import dataclass, field

from config import (
    COLLECTION_TIME,
    MAX_BATTERY,
    MIN_BATTERY,
    MIN_COLLECTIONS,
    RECHARGE_RATE,
)
from estado import State
from heuristica import h5


@dataclass
class TreeEdge:
    """Represents one edge in the search tree: a parent expanding into a child."""
    parent_state: object  # State or None for root
    child_state: object   # State
    action: str
    g: float
    h: float
    f: float
    in_solution: bool = False  # will be set to True for edges on the optimal path


@dataclass
class SearchResult:
    path: list
    total_time: float
    expanded_states: int
    open_list_size: int
    closed_list_size: int
    success: bool
    search_tree: list = field(default_factory=list)   # list of TreeEdge
    final_state: object = None
    open_list_history: list = field(default_factory=list)
    closed_list_history: list = field(default_factory=list)
    revisited_states: list = field(default_factory=list)
    expanded_later: list = field(default_factory=list)


class AStarSearch:
    def __init__(self, graph):
        self.graph = graph

    def search(self, start_node, goal_node, heuristic_func=h5):
        self._validate_nodes(start_node, goal_node)

        start_state = self._create_initial_state(start_node)

        initial_g = float(COLLECTION_TIME) if self.graph.is_collection_node(start_node) else 0.0

        open_list = []
        closed_list = set()
        g_score = {start_state: initial_g}
        parents = {start_state: (None, "inicio")}

        search_tree = []
        open_list_history = []
        closed_list_history = []
        revisited_states = []

        counter = 0
        h_start = heuristic_func(self.graph, start_state, goal_node)
        first_f = initial_g + h_start
        heapq.heappush(open_list, (first_f, counter, start_state))

        # Record the root as a special edge (parent=None)
        search_tree.append(TreeEdge(
            parent_state=None,
            child_state=start_state,
            action="inicio",
            g=initial_g,
            h=h_start,
            f=first_f,
        ))

        # Para rastrear saltos temporais na lista aberta (Tópico 5)
        added_at_step = {start_state: 0}
        expanded_later = []
        step = 0

        while open_list:
            _, _, current_state = heapq.heappop(open_list)

            if current_state in closed_list:
                continue
                
            step += 1
            
            # Se o nó entrou na open_list num passo bem anterior ao passo atual (com atraso > 1)
            if added_at_step[current_state] < step - 1:
                expanded_later.append({
                    'state': current_state,
                    'added_step': added_at_step[current_state],
                    'expanded_step': step,
                    'g': g_score[current_state]
                })

            if self._is_goal_state(current_state, goal_node):
                path = self._reconstruct_path(current_state, parents, g_score)

                # Mark edges that belong to the solution path using original states
                solution_states = set()
                for step_dict in path:
                    solution_states.add(step_dict.get("original_state", step_dict["state"]))
                for edge in search_tree:
                    if (edge.child_state in solution_states
                            and (edge.parent_state is None
                                 or edge.parent_state in solution_states)):
                        edge.in_solution = True

                # total_time agora vem do ultimo passo reconstruido (inclui recarga parcial)
                final_total_time = path[-1]["g"] if path else g_score[current_state]
                final_state_reconstructed = path[-1]["state"] if path else current_state

                return SearchResult(
                    path=path,
                    total_time=final_total_time,
                    expanded_states=len(closed_list),
                    open_list_size=len(open_list),
                    closed_list_size=len(closed_list),
                    success=True,
                    search_tree=search_tree,
                    final_state=final_state_reconstructed,
                    open_list_history=open_list_history,
                    closed_list_history=closed_list_history,
                    revisited_states=revisited_states,
                    expanded_later=expanded_later
                )

            closed_list.add(current_state)

            for next_state, action_cost, action_description in self._generate_successors(current_state):
                if next_state in closed_list:
                    continue

                new_g = g_score[current_state] + action_cost
                h_value = heuristic_func(self.graph, next_state, goal_node)
                
                new_f = new_g + h_value

                search_tree.append(TreeEdge(
                    parent_state=current_state,
                    child_state=next_state,
                    action=action_description,
                    g=new_g,
                    h=h_value,
                    f=new_f,
                ))

                if new_g < g_score.get(next_state, float("inf")):
                    # Capturar nó revisitado na open list (ignorando diferenças microscópicas de float)
                    if next_state in g_score and next_state not in closed_list:
                        if g_score[next_state] - new_g > 0.01:
                            revisited_states.append({
                                'state': next_state,
                                'old_g': g_score[next_state],
                                'new_g': new_g,
                                'by_action': action_description
                            })
                        
                    g_score[next_state] = new_g
                    parents[next_state] = (current_state, action_description)
                    added_at_step.setdefault(next_state, step)

                    counter += 1
                    heapq.heappush(open_list, (new_f, counter, next_state))
                    
            # Registrar snapshots no final da expansão do nó
            open_list_history.append([str(item[2]) for item in open_list])
            closed_list_history.append([str(st) for st in closed_list])

        return SearchResult(
            path=[],
            total_time=0.0,
            expanded_states=len(closed_list),
            open_list_size=0,
            closed_list_size=len(closed_list),
            success=False,
            search_tree=search_tree,
            final_state=None,
            open_list_history=open_list_history,
            closed_list_history=closed_list_history,
            revisited_states=revisited_states
        )

    def _validate_nodes(self, start_node, goal_node):
        if not self.graph.has_node(start_node):
            raise ValueError(f"Origem invalida: {start_node}")

        if not self.graph.has_node(goal_node):
            raise ValueError(f"Destino invalido: {goal_node}")

    def _create_initial_state(self, start_node):
        collected_points = set()
        initial_battery = MAX_BATTERY

        # Conforme a especificacao, se o rover inicia em um ponto de coleta,
        # esse ponto ja entra no conjunto de coletas realizadas e consome bateria.
        if self.graph.is_collection_node(start_node):
            collected_points.add(start_node)
            initial_battery -= COLLECTION_TIME

        return State(
            current_node=start_node,
            battery_level=initial_battery,
            collected_points=frozenset(collected_points),
        )

    def _is_goal_state(self, state, goal_node):
        return (
            state.current_node == goal_node
            and state.battery_level >= MIN_BATTERY
            and len(state.collected_points) >= MIN_COLLECTIONS
        )

    def _generate_successors(self, state):
        successors = []

        # Acao 1: recarregar.
        # Ela so existe em nos R. Para evitar muitas recargas pequenas em sequencia,
        # a acao de recarga leva a bateria diretamente ate 100%.
        # Exemplo: sair de 80% para 100% recupera 20%, custando 20 / 5 = 4 min.
        if self.graph.is_recharge_node(state.current_node) and state.battery_level < MAX_BATTERY:
            recharge_time = 0.0 # O tempo real será recalculado no final (optimal partial recharge)

            next_state = State(
                current_node=state.current_node,
                battery_level=MAX_BATTERY,
                collected_points=state.collected_points,
            )

            action = f"recarregar em {state.current_node} (max exploratorio)"
            successors.append((next_state, recharge_time, action))

        # Acao 2: mover para um vizinho conectado por aresta.
        # O peso da aresta e tempo real de deslocamento em minutos.
        # Como o consumo e 1% por minuto, o mesmo valor e subtraido da bateria.
        for neighbor, travel_time in self.graph.get_neighbors(state.current_node):
            battery_after_move = state.battery_level - travel_time

            if battery_after_move < MIN_BATTERY:
                continue

            new_collected_points = set(state.collected_points)
            total_action_cost = travel_time
            battery_after_action = battery_after_move
            action = f"mover {state.current_node} -> {neighbor}"

            # Coleta automatica ao chegar em um ponto C ainda nao visitado,
            # mas somente enquanto o rover nao atingiu o minimo de coletas.
            # Depois de 5 coletas, novas coletas sao opcionais e nao custam tempo.
            can_collect = (
                self.graph.is_collection_node(neighbor)
                and neighbor not in new_collected_points
                and len(new_collected_points) < MIN_COLLECTIONS
            )

            if can_collect:
                battery_after_action -= COLLECTION_TIME

                if battery_after_action < MIN_BATTERY:
                    continue

                total_action_cost += COLLECTION_TIME
                new_collected_points.add(neighbor)
                action += f" e coletar em {neighbor}"

            next_state = State(
                current_node=neighbor,
                battery_level=battery_after_action,
                collected_points=frozenset(new_collected_points),
            )

            successors.append((next_state, total_action_cost, action))

        return successors

    def _reconstruct_path(self, final_state, parents, g_score):
        from config import RECHARGE_RATE, MIN_BATTERY
        raw_path = []
        current_state = final_state

        while current_state is not None:
            previous_state, action = parents[current_state]
            raw_path.append({
                "state": current_state,
                "action": action,
                "raw_g": g_score[current_state],
            })
            current_state = previous_state

        raw_path.reverse()

        needed_battery = MIN_BATTERY

        for i in range(len(raw_path) - 1, 0, -1):
            curr_step = raw_path[i]
            prev_step = raw_path[i - 1]

            action = curr_step["action"]

            if "mover" in action:
                cost = curr_step["raw_g"] - prev_step["raw_g"]
                needed_battery += cost
            elif "recarregar" in action:
                arriving_battery = prev_step["state"].battery_level

                if needed_battery > arriving_battery:
                    recharge_amount = needed_battery - arriving_battery
                    recharge_time = recharge_amount / RECHARGE_RATE

                    curr_step["action"] = f"recarregar em {curr_step['state'].current_node} {recharge_amount:.1f}% ({recharge_time:.1f} min)"
                    curr_step["recharge_time"] = recharge_time
                    curr_step["recharge_amount"] = recharge_amount

                    needed_battery = arriving_battery
                else:
                    curr_step["action"] = f"recarregar em {curr_step['state'].current_node} (ignorada, bateria suficiente)"
                    curr_step["recharge_time"] = 0.0
                    curr_step["recharge_amount"] = 0.0

        final_path = []
        current_g = raw_path[0]["raw_g"]
        current_battery = raw_path[0]["state"].battery_level

        for i, step in enumerate(raw_path):
            if i > 0:
                action = step["action"]
                if "mover" in action:
                    cost = step["raw_g"] - raw_path[i - 1]["raw_g"]
                    current_g += cost
                    current_battery -= cost
                elif "recarregar" in action:
                    current_g += step.get("recharge_time", 0.0)
                    current_battery += step.get("recharge_amount", 0.0)

            # Manter arredondamentos sanos
            current_battery = round(current_battery, 4)
            current_g = round(current_g, 4)

            final_path.append({
                "state": State(step["state"].current_node, current_battery, step["state"].collected_points),
                "original_state": step["state"],  # estado original da busca (para lookup na arvore)
                "action": step["action"],
                "g": current_g,
            })

        return final_path
