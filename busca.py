import heapq
import math
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


# heapq cria a fila de prioridade do A*: o estado com menor f(n) sai primeiro.
# math e usado para arredondar para cima a bateria minima necessaria.
# dataclass reduz codigo repetitivo em estruturas que so guardam dados.


@dataclass
class TreeEdge:
    """Guarda uma transicao da arvore de busca."""
    parent_state: object
    child_state: object
    action: str
    g: float
    h: float
    f: float
    in_solution: bool = False


@dataclass
class SearchResult:
    """Resultado completo da busca, incluindo dados usados no relatorio."""

    path: list
    total_time: float
    expanded_states: int
    open_list_size: int
    closed_list_size: int
    success: bool
    search_tree: list = field(default_factory=list)
    final_state: object = None
    open_list_history: list = field(default_factory=list)
    closed_list_history: list = field(default_factory=list)
    revisited_states: list = field(default_factory=list)
    expanded_later: list = field(default_factory=list)


class AStarSearch:
    def __init__(self, graph):
        self.graph = graph

    def search(self, start_node, goal_node, heuristic_func=h5):
        """Executa o A* entre a origem e o objetivo.

        Cada estado guarda onde o rover esta, quanta bateria sobrou e quais
        coletas ja foram feitas. O A* sempre escolhe para expandir o estado com
        menor f(n), em que f(n) = g(n) + h(n).
        """
        self._validate_nodes(start_node, goal_node)

        start_state = self._create_initial_state(start_node)

        initial_g = float(COLLECTION_TIME) if self.graph.is_collection_node(start_node) else 0.0

        # Lista aberta: estados descobertos, mas ainda nao expandidos.
        # Cada item fica como (f, contador, estado); o contador desempata sem
        # obrigar o Python a comparar dois objetos State.
        open_list = []

        # Lista fechada: estados que ja tiveram seus vizinhos analisados.
        closed_list = set()

        # g_score guarda o custo real mais barato conhecido ate cada estado.
        # parents guarda de onde cada estado veio para reconstruir a rota final.
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

        # A raiz tambem entra na arvore para aparecer na imagem com g, h e f.
        search_tree.append(TreeEdge(
            parent_state=None,
            child_state=start_state,
            action="inicio",
            g=initial_g,
            h=h_start,
            f=first_f,
        ))

        # Estes dados mostram quando um estado foi gerado cedo, mas so foi
        # expandido bem depois. Isso ajuda a explicar a ordem da lista aberta.
        added_at_step = {start_state: 0}
        expanded_later = []
        step = 0

        while open_list:
            _, _, current_state = heapq.heappop(open_list)

            if current_state in closed_list:
                continue

            current_g = g_score[current_state]
            if self._is_dominated(current_state, current_g, g_score):
                continue

            step += 1
            
            if added_at_step[current_state] < step - 1:
                expanded_later.append({
                    'state': current_state,
                    'added_step': added_at_step[current_state],
                    'expanded_step': step,
                    'g': g_score[current_state]
                })

            if self._is_goal_state(current_state, goal_node):
                path = self._reconstruct_path(current_state, parents, g_score)

                # Marcamos apenas as transicoes do caminho final. A arvore ainda
                # guarda os outros estados, mas a visualizacao destaca a solucao.
                solution_states = {step_dict["state"] for step_dict in path}
                solution_edges = set()
                for i in range(1, len(path)):
                    solution_edges.add((path[i - 1]["state"], path[i]["state"]))

                for edge in search_tree:
                    if (
                        edge.parent_state is None
                        and edge.child_state in solution_states
                    ) or (edge.parent_state, edge.child_state) in solution_edges:
                        edge.in_solution = True

                final_total_time = path[-1]["g"] if path else g_score[current_state]
                final_state_reconstructed = path[-1]["state"] if path else current_state

                return SearchResult(
                    path=path,
                    total_time=final_total_time,
                    expanded_states=len(closed_list),
                    open_list_size=len(self._active_open_states(open_list, closed_list, g_score)),
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

            # Evita duas recargas seguidas no mesmo lugar. Depois de recarregar,
            # o rover precisa se mover antes de considerar outra recarga.
            previous_action = parents[current_state][1]
            allow_recharge = "recarregar" not in previous_action

            for next_state, action_cost, action_description in self._generate_successors(
                current_state,
                goal_node,
                allow_recharge=allow_recharge,
            ):
                if next_state in closed_list:
                    continue

                new_g = g_score[current_state] + action_cost
                h_value = heuristic_func(self.graph, next_state, goal_node)
                
                new_f = new_g + h_value

                if new_g < g_score.get(next_state, float("inf")):
                    if self._is_dominated(next_state, new_g, g_score):
                        continue

                    # Se o mesmo estado ja estava na lista aberta com custo
                    # maior, guardamos a troca para mostrar no relatorio.
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
                    added_at_step[next_state] = step

                    search_tree.append(TreeEdge(
                        parent_state=current_state,
                        child_state=next_state,
                        action=action_description,
                        g=new_g,
                        h=h_value,
                        f=new_f,
                    ))

                    counter += 1
                    heapq.heappush(open_list, (new_f, counter, next_state))
                    
            # O historico e registrado no fim da expansao para refletir a lista
            # aberta e fechada depois daquela decisao.
            open_list_history.append(
                self._format_state_list(open_list, closed_list, g_score, heuristic_func, goal_node)
            )
            closed_list_history.append(
                self._format_state_list(closed_list, set(), g_score, heuristic_func, goal_node)
            )

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
        """Confere se origem e destino existem no grafo."""

        if not self.graph.has_node(start_node):
            raise ValueError(f"Origem invalida: {start_node}")

        if not self.graph.has_node(goal_node):
            raise ValueError(f"Destino invalido: {goal_node}")

    def _create_initial_state(self, start_node):
        """Monta o estado inicial, incluindo coleta automatica se a origem for C."""

        collected_points = set()
        initial_battery = MAX_BATTERY

        # Se o rover ja comeca em um ponto de coleta, essa amostra conta para a
        # missao e tambem consome o tempo/bateria da coleta.
        if self.graph.is_collection_node(start_node):
            collected_points.add(start_node)
            initial_battery -= COLLECTION_TIME

        return State(
            current_node=start_node,
            battery_level=initial_battery,
            collected_points=frozenset(collected_points),
        )

    def _is_goal_state(self, state, goal_node):
        """O objetivo so vale quando destino, bateria minima e coletas batem."""

        return (
            state.current_node == goal_node
            and state.battery_level >= MIN_BATTERY
            and len(state.collected_points) >= MIN_COLLECTIONS
        )

    def _generate_successors(self, state, goal_node, allow_recharge=True):
        """Cria todos os proximos estados possiveis a partir do estado atual."""

        successors = []

        # Primeiro vem a acao de recarregar. Ela so aparece em nos R e so usa
        # niveis de bateria que podem ser uteis para chegar a outro ponto.
        if (
            allow_recharge
            and self.graph.is_recharge_node(state.current_node)
            and state.battery_level < MAX_BATTERY
        ):
            for target_battery in self._useful_recharge_targets(state, goal_node):
                recharge_amount = target_battery - state.battery_level
                if recharge_amount <= 0:
                    continue

                recharge_time = recharge_amount / RECHARGE_RATE
                next_state = State(
                    current_node=state.current_node,
                    battery_level=target_battery,
                    collected_points=state.collected_points,
                )

                action = (
                    f"recarregar em {state.current_node} "
                    f"{recharge_amount:.1f}% ({recharge_time:.1f} min)"
                )
                successors.append((next_state, recharge_time, action))

        # Depois vem o deslocamento pelas arestas do grafo. O tempo da aresta
        # tambem e o consumo de bateria, pois o modelo usa 1% por minuto.
        for neighbor, travel_time in self.graph.get_neighbors(state.current_node):
            battery_after_move = state.battery_level - travel_time

            if battery_after_move < MIN_BATTERY:
                continue

            new_collected_points = set(state.collected_points)
            total_action_cost = travel_time
            battery_after_action = battery_after_move
            action = f"mover {state.current_node} -> {neighbor}"

            # A coleta e automatica enquanto a missao ainda nao atingiu o minimo.
            # Depois disso, passar por outro ponto C nao aumenta o custo.
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

    def _useful_recharge_targets(self, state, goal_node):
        """Calcula quais niveis de recarga realmente valem a pena considerar.

        Testar todos os percentuais ate 100 deixaria a busca muito grande.
        Entao a funcao olha adiante a partir da estacao atual e guarda apenas
        baterias que permitem chegar a algum ponto relevante: outra estacao ou
        o objetivo com a quantidade minima de coletas.
        """

        targets = set()
        best_goal_target = None
        start_key = (state.current_node, state.collected_points)
        frontier = [(0, state.current_node, state.collected_points)]
        best_cost = {start_key: 0}

        while frontier:
            battery_cost, node, collected_points = heapq.heappop(frontier)
            key = (node, collected_points)
            if battery_cost > best_cost.get(key, float("inf")):
                continue

            for neighbor, travel_time in self.graph.get_neighbors(node):
                next_collected = set(collected_points)
                step_cost = travel_time
                can_collect = (
                    self.graph.is_collection_node(neighbor)
                    and neighbor not in next_collected
                    and len(next_collected) < MIN_COLLECTIONS
                )
                if can_collect:
                    step_cost += COLLECTION_TIME
                    next_collected.add(neighbor)

                next_cost = battery_cost + step_cost
                target_battery = math.ceil(next_cost + MIN_BATTERY)
                if target_battery > MAX_BATTERY:
                    continue

                next_collected = frozenset(next_collected)

                if (
                    neighbor == goal_node
                    and len(next_collected) >= MIN_COLLECTIONS
                ):
                    if best_goal_target is None or target_battery < best_goal_target:
                        best_goal_target = target_battery
                    continue

                # Ao chegar em outra estacao, paramos a olhada adiante: dali o
                # proprio A* pode decidir se recarrega de novo ou segue caminho.
                if self.graph.is_recharge_node(neighbor):
                    if neighbor != state.current_node and target_battery > state.battery_level:
                        targets.add(target_battery)
                    continue

                next_key = (neighbor, next_collected)
                if next_cost < best_cost.get(next_key, float("inf")):
                    best_cost[next_key] = next_cost
                    heapq.heappush(frontier, (next_cost, neighbor, next_collected))

        if best_goal_target is not None:
            if best_goal_target <= state.battery_level:
                return []

            # Se ja existe uma recarga suficiente para fechar a missao direto,
            # alvos maiores so gastariam mais tempo sem trazer ganho.
            targets = {target for target in targets if target < best_goal_target}
            targets.add(best_goal_target)

        return sorted(targets)

    def _is_dominated(self, state, g_value, g_score):
        """Descarta estados claramente piores que outro ja conhecido.

        Se dois estados estao no mesmo no e possuem as mesmas coletas, aquele
        com menos bateria e custo maior nunca sera melhor no futuro.
        """

        for other_state, other_g in g_score.items():
            if other_state == state:
                continue

            if (
                other_state.current_node != state.current_node
                or other_state.collected_points != state.collected_points
            ):
                continue

            if (
                other_state.battery_level >= state.battery_level
                and other_g <= g_value + 1e-9
            ):
                return True

        return False

    def _active_open_states(self, open_list, closed_list, g_score):
        """Filtra a lista aberta para mostrar apenas candidatos ainda validos."""

        states = []
        seen = set()

        for _, _, state in open_list:
            if state in closed_list or state in seen:
                continue
            if self._is_dominated(state, g_score.get(state, float("inf")), g_score):
                continue

            seen.add(state)
            states.append(state)

        return states

    def _reconstruct_path(self, final_state, parents, g_score):
        """Volta dos pais ate a raiz e depois inverte para obter a rota final."""

        path = []
        current_state = final_state

        while current_state is not None:
            previous_state, action = parents[current_state]
            path.append({
                "state": current_state,
                "action": action,
                "g": round(g_score[current_state], 4),
            })
            current_state = previous_state

        path.reverse()
        return path

    def _format_state_list(self, states_source, closed_list, g_score, heuristic_func, goal_node):
        """Formata estados para o historico exibido no relatorio HTML."""

        if isinstance(states_source, list):
            states = self._active_open_states(states_source, closed_list, g_score)
        else:
            states = sorted(
                states_source,
                key=lambda state: (
                    state.current_node,
                    state.battery_level,
                    tuple(sorted(state.collected_points)),
                ),
            )

        items = []
        for state in states:
            g_value = g_score.get(state, float("inf"))
            h_value = heuristic_func(self.graph, state, goal_node)
            f_value = g_value + h_value
            items.append(
                f"{state} | g={g_value:.1f} h={h_value:.1f} f={f_value:.1f}"
            )

        return items
