"""
Rotinas de visualizacao da busca A*.

Este arquivo junta a arvore impressa no terminal, as imagens em Graphviz e o
relatorio HTML. A ideia e deixar a execucao conferivel sem obrigar ninguem a
reconstruir a busca inteira olhando apenas para o console.
"""

import os
import sys
import html as html_module
import json
from collections import defaultdict

from config import (
    COLLECTION_TIME,
    MAX_SPEED,
    MIN_BATTERY,
    MIN_COLLECTIONS,
    PLANE_UNIT_IN_KM,
    RECHARGE_RATE,
)

# ═══════════════════════════════════════════════════════════════════════════
# 1. Impressão no Console (Árvore de Busca)
# ═══════════════════════════════════════════════════════════════════════════

def print_search_tree(result, max_depth=None, max_children=4):
    """Imprime a hierarquia da árvore de busca no console."""
    if not result.search_tree:
        print("  (arvore vazia)")
        return

    children_map = defaultdict(list)
    root_edge = None

    for edge in result.search_tree:
        if edge.parent_state is None:
            root_edge = edge
        else:
            children_map[edge.parent_state].append(edge)

    if root_edge is None:
        print("  (sem raiz encontrada)")
        return

    solution_states = set()
    if result.success:
        for step in result.path:
            solution_states.add(step["state"])

    def _select_console_children(child_edges):
        if max_children is None or len(child_edges) <= max_children:
            return child_edges, 0

        selected = []
        for child_edge in child_edges:
            if child_edge.child_state in solution_states:
                selected.append(child_edge)
                break

        for child_edge in child_edges:
            if child_edge in selected:
                continue
            if len(selected) >= max_children:
                break
            selected.append(child_edge)

        return selected, len(child_edges) - len(selected)

    def _print_node(edge, prefix, is_last, depth):
        if max_depth is not None and depth > max_depth:
            return
        state = edge.child_state
        marker = "*** " if state in solution_states else ""
        connector = "+-- " if is_last else "+-- "
        line = (
            f"{prefix}{connector}{marker}"
            f"{state.current_node} "
            f"[bat={state.battery_level}%, "
            f"col={len(state.collected_points)}, "
            f"g={edge.g:.1f}, h={edge.h:.1f}, f={edge.f:.1f}]"
        )
        if edge.action and edge.action != "inicio":
            line += f"  <- {edge.action}"
        print(line)
        child_edges = sorted(children_map.get(state, []), key=lambda e: e.f)
        child_edges, omitted = _select_console_children(child_edges)
        new_prefix = prefix + ("    " if is_last else "|   ")
        for i, child_edge in enumerate(child_edges):
            _print_node(child_edge, new_prefix, i == len(child_edges) - 1 and omitted == 0, depth + 1)
        if omitted:
            print(f"{new_prefix}+-- ... +{omitted} ramos omitidos")

    root_state = root_edge.child_state
    root_marker = "*** " if root_state in solution_states else ""
    print(
        f"{root_marker}[RAIZ] {root_state.current_node} "
        f"[bat={root_state.battery_level}%, "
        f"col={len(root_state.collected_points)}, "
        f"g=0.0, h={root_edge.h:.1f}, f={root_edge.f:.1f}]"
    )
    child_edges = sorted(children_map.get(root_state, []), key=lambda e: e.f)
    child_edges, omitted = _select_console_children(child_edges)
    for i, child_edge in enumerate(child_edges):
        _print_node(child_edge, "", i == len(child_edges) - 1 and omitted == 0, 1)
    if omitted:
        print(f"+-- ... +{omitted} ramos omitidos")


# ═══════════════════════════════════════════════════════════════════════════
# 2. Imagens Graphviz (Focada e Completa)
# ═══════════════════════════════════════════════════════════════════════════

def _ensure_graphviz_path():
    """Garante que o diretório binário do Graphviz está no PATH do Windows."""
    if sys.platform == "win32":
        gv_bin = r"C:\Program Files\Graphviz\bin"
        if os.path.isdir(gv_bin) and gv_bin not in os.environ.get("PATH", ""):
            os.environ["PATH"] = gv_bin + ";" + os.environ.get("PATH", "")


def _fmt(value):
    """Formata float com vírgula (padrão brasileiro)."""
    return f"{value:.2f}".replace(".", ",")


def _fmt_edge_cost(value):
    """Formata o custo da aresta para exibição no gráfico."""
    if value == int(value):
        return str(int(value))
    return f"{value:.1f}".replace(".", ",")


COLOR_SOLUTION = "#dbeafe"
COLOR_EXPANDED = "#e5e7eb"
COLOR_OPEN = "white"
COLOR_GOAL = "#b6f2b6"
COLOR_RECHARGE = "#dcfce7"
COLOR_RECHARGE_BORDER = "#059669"
COLOR_PRUNED = "#fef3c7"

FOCUSED_MAX_ALTERNATIVES_PER_STEP = 3
FULL_TREE_MAX_DEPTH = 3
FULL_TREE_MAX_CHILDREN_PER_NODE = 4


def _fmt_battery(value):
    return _fmt_edge_cost(value)


def _edge_label_between(parent_g, child_edge):
    action = child_edge.action or ""
    cost = child_edge.g - parent_g
    if "recarregar" in action:
        if cost <= 0 and child_edge.parent_state is not None:
            recharge_amount = child_edge.child_state.battery_level - child_edge.parent_state.battery_level
            cost = max(0, recharge_amount) / RECHARGE_RATE
        return f"recarga {_fmt_edge_cost(cost)}" if cost > 0 else "recarga"
    if "coletar" in action:
        travel = cost - COLLECTION_TIME
        return f"{_fmt_edge_cost(travel)} + coleta" if travel > 0 else "+ coleta"
    return _fmt_edge_cost(cost)


def _tree_node_label(state, edge_data, is_goal=False, is_recharge_action=False):
    label = (
        f"{state.current_node}\n"
        f"g={_fmt(edge_data.g)} h={_fmt(edge_data.h)} f={_fmt(edge_data.f)}\n"
        f"b={_fmt_battery(state.battery_level)}% coletas={len(state.collected_points)}"
    )
    if is_goal:
        label = "OBJETIVO\n" + label
    if is_recharge_action:
        label += "\nrecarga"
    return label


def _tree_node_attrs(is_goal=False, is_recharge_action=False, is_path=False, is_expanded=False, dashed=False):
    attrs = {"style": "filled,dashed" if dashed else "filled"}

    if is_goal:
        attrs.update({"fillcolor": COLOR_GOAL, "peripheries": "2"})
    elif is_recharge_action:
        attrs.update({"fillcolor": COLOR_RECHARGE, "color": COLOR_RECHARGE_BORDER})
    elif is_path:
        attrs.update({"fillcolor": COLOR_SOLUTION})
    elif is_expanded:
        attrs.update({"fillcolor": COLOR_EXPANDED})
    else:
        attrs.update({"fillcolor": COLOR_OPEN})

    return attrs


def _add_graphviz_legend(dot, focused=False):
    expanded_label = "Expandido / Closed"
    if focused:
        expanded_label = "Expandido / Closed\n(filhos nao desenhados nesta visao)"

    with dot.subgraph(name="cluster_legend") as legend:
        legend.attr(label="Legenda", fontname="Arial", fontsize="11", color="#cccccc")
        legend.attr("node", shape="box", style="filled", fontname="Arial", fontsize="9", color="black")
        legend.node("legend_solution", "Estado no caminho escolhido\narestas azuis ligam a rota final", fillcolor=COLOR_SOLUTION)
        legend.node("legend_recharge", "Acao de recarga executada\nrecarga tambem entra em g(s)", fillcolor=COLOR_RECHARGE, color=COLOR_RECHARGE_BORDER)
        legend.node("legend_goal", "Destino / objetivo", fillcolor=COLOR_GOAL, peripheries="2")
        legend.node("legend_expanded", expanded_label, fillcolor=COLOR_EXPANDED)
        legend.node("legend_open", "Gerado / Open\ncandidato ainda nao expandido", fillcolor=COLOR_OPEN)
        legend.node("legend_alternative", "Alternativa desenhada\naresta cinza", fillcolor=COLOR_OPEN)
        legend.node(
            "legend_pruned",
            "Ramos omitidos por limite visual:\n"
            "estados gerados na execucao,\n"
            "mas ocultados para evitar arvore ilegivel.\n"
            "Nao e poda do algoritmo;\n"
            "Open/Closed e metricas reais nao mudam.",
            fillcolor=COLOR_PRUNED,
        )


def _sorted_edges(edges):
    return sorted(
        edges,
        key=lambda edge: (
            edge.f,
            edge.g,
            edge.child_state.current_node,
            edge.child_state.battery_level,
            tuple(sorted(edge.child_state.collected_points)),
        ),
    )


def _build_tree_indexes(result):
    successors_of = defaultdict(list)
    best_edge = {}
    expanded_set = set()
    root_edge = None

    for edge in result.search_tree:
        if edge.parent_state is None:
            root_edge = edge
        else:
            successors_of[edge.parent_state].append(edge)
            expanded_set.add(edge.parent_state)

        state = edge.child_state
        if state not in best_edge or edge.g < best_edge[state].g:
            best_edge[state] = edge

    for state, edges in list(successors_of.items()):
        successors_of[state] = _sorted_edges(edges)

    return root_edge, successors_of, best_edge, expanded_set


def _path_transition_edges(path_states, successors_of):
    transition_edges = {}
    for i in range(len(path_states) - 1):
        parent = path_states[i]
        child = path_states[i + 1]
        for edge in successors_of.get(parent, []):
            if edge.child_state == child:
                transition_edges[parent] = edge
                break
    return transition_edges


def _add_tree_node(dot, node_id, state, edge_data, result, path_set, expanded_set):
    is_goal = state == result.final_state
    is_path = state in path_set
    is_recharge = "recarregar" in edge_data.action
    label = _tree_node_label(
        state,
        edge_data,
        is_goal=is_goal,
        is_recharge_action=is_recharge,
    )
    attrs = _tree_node_attrs(
        is_goal=is_goal,
        is_recharge_action=is_recharge,
        is_path=is_path,
        is_expanded=state in expanded_set,
    )
    dot.node(node_id, label=label, **attrs)


def _add_pruned_summary(dot, parent_id, summary_id, count):
    if count <= 0:
        return

    label = (
        f"+{count} ramos omitidos\n"
        "limite visual\n"
        "(estados gerados,\n"
        "mas nao desenhados)"
    )
    dot.node(
        summary_id,
        label=label,
        shape="box",
        style="filled,dashed",
        fillcolor=COLOR_PRUNED,
        color="#f59e0b",
        fontcolor="#92400e",
    )
    dot.edge(parent_id, summary_id, style="dashed", color="#d97706", arrowhead="none")


def generate_focused_tree_image(graph, result, start_node, goal_node, output_dir="results"):
    """
    Gera a imagem da arvore FOCADA: caminho final + poucas alternativas imediatas.
    Retorna o nome do arquivo gerado.
    """
    _ensure_graphviz_path()

    try:
        import graphviz
    except ImportError:
        return None

    if not result.success or not result.path:
        return None

    os.makedirs(output_dir, exist_ok=True)

    path_states = [step["state"] for step in result.path]
    path_actions = [step["action"] for step in result.path]
    path_set = set(path_states)
    _, successors_of, best_edge, expanded_set = _build_tree_indexes(result)
    transition_edges = _path_transition_edges(path_states, successors_of)

    dot = graphviz.Digraph("ArvoreBuscaAStar", format="svg")
    dot.attr(rankdir="TB", splines="true", nodesep="0.38", ranksep="0.65")
    dot.attr(
        label=(
            "Arvore focada compactada: mostra o caminho escolhido e algumas alternativas imediatas. "
            "Ramos omitidos sao limite visual, nao poda do A*."
        ),
        labelloc="t",
        fontname="Arial",
        fontsize="12",
    )
    dot.attr("node", shape="ellipse", style="filled", fontname="Arial", fontsize="10", color="black")
    dot.attr("edge", fontname="Arial", fontsize="9", color="#555555")

    state_to_id = {}
    s_counter = [1]
    a_counter = [1]
    p_counter = [1]

    def _next_s_id():
        nid = f"S{s_counter[0]}"
        s_counter[0] += 1
        return nid

    def _next_a_id():
        nid = f"A{a_counter[0]}"
        a_counter[0] += 1
        return nid

    def _next_p_id():
        nid = f"P{p_counter[0]}"
        p_counter[0] += 1
        return nid

    # Caminho escolhido: a espinha dorsal da explicacao.
    for i, state in enumerate(path_states):
        edge_data = best_edge[state]
        is_goal = (state == result.final_state)
        is_recharge = (i > 0 and "recarregar" in path_actions[i])

        node_id = _next_s_id()
        state_to_id[state] = node_id

        label = _tree_node_label(state, edge_data, is_goal, is_recharge)

        attrs = _tree_node_attrs(
            is_goal=is_goal,
            is_recharge_action=is_recharge,
            is_path=True,
        )
        dot.node(node_id, label=label, **attrs)

        if i > 0:
            prev_state = path_states[i - 1]
            prev_id = state_to_id[prev_state]
            connecting_edge = transition_edges.get(prev_state)
            if connecting_edge:
                elabel = _edge_label_between(best_edge[prev_state].g, connecting_edge)
                dot.edge(prev_id, node_id, label=f"escolhido\n{elabel}", color="#2563eb", penwidth="2")
            else:
                cost = result.path[i]["g"] - result.path[i - 1]["g"]
                dot.edge(prev_id, node_id, label=f"escolhido\n{_fmt_edge_cost(cost)}", color="#2563eb", penwidth="2")

    # Alternativas imediatas de cada decisao do caminho, sem abrir sub-ramos.
    for state in path_states:
        parent_id = state_to_id[state]
        parent_g = best_edge[state].g
        alternatives = [
            edge for edge in successors_of.get(state, [])
            if edge.child_state not in path_set
        ]
        selected = alternatives[:FOCUSED_MAX_ALTERNATIVES_PER_STEP]

        drawn = 0
        for succ_edge in selected:
            child = succ_edge.child_state
            if child in state_to_id:
                continue

            was_expanded = child in expanded_set
            child_id = _next_s_id() if was_expanded else _next_a_id()
            state_to_id[child] = child_id

            label = _tree_node_label(child, succ_edge, is_recharge_action="recarregar" in succ_edge.action)
            attrs = _tree_node_attrs(
                is_recharge_action="recarregar" in succ_edge.action,
                is_expanded=was_expanded,
                dashed=was_expanded,
            )
            dot.node(child_id, label=label, **attrs)

            elabel = _edge_label_between(parent_g, succ_edge)
            dot.edge(parent_id, child_id, label=f"opcao\n{elabel}", color="#9ca3af")
            drawn += 1

        omitted_count = max(0, len(alternatives) - drawn)
        _add_pruned_summary(
            dot,
            parent_id,
            _next_p_id(),
            omitted_count,
        )

    _add_graphviz_legend(dot, focused=True)

    filename = "arvore_busca_focada.svg"
    
    dot_path = os.path.join(output_dir, "arvore_busca_focada.dot")
    with open(dot_path, "w", encoding="utf-8") as f:
        f.write(dot.source)

    try:
        dot.render(os.path.join(output_dir, "arvore_busca_focada"), format="svg", cleanup=True)
        return filename
    except Exception:
        return None


def generate_full_tree_image(graph, result, start_node, goal_node, output_dir="results"):
    """
    Gera uma arvore COMPLETA compactada: primeiros niveis + caminho ate o objetivo.
    Retorna o nome do arquivo gerado.
    """
    _ensure_graphviz_path()

    try:
        import graphviz
    except ImportError:
        return None

    if not result.success or not result.path:
        return None

    os.makedirs(output_dir, exist_ok=True)

    root_edge, successors_of, best_edge, expanded_set = _build_tree_indexes(result)
    if root_edge is None:
        return None

    path_states_list = [step["state"] for step in result.path]
    path_set = set(path_states_list)
    transition_edges = _path_transition_edges(path_states_list, successors_of)
    path_next = {
        path_states_list[i]: path_states_list[i + 1]
        for i in range(len(path_states_list) - 1)
    }

    dot = graphviz.Digraph("ArvoreBuscaCompleta", format="svg")
    dot.attr(rankdir="TB", splines="true", nodesep="0.35", ranksep="0.62")
    dot.attr(
        label=(
            "Arvore completa compactada por limite visual: estados omitidos existiram na execucao, "
            "mas nao foram desenhados para manter a figura legivel."
        ),
        labelloc="t",
        fontname="Arial",
        fontsize="12",
    )
    dot.attr("node", shape="ellipse", style="filled", fontname="Arial", fontsize="10", color="black")
    dot.attr("edge", fontname="Arial", fontsize="9", color="#555555")

    state_to_id = {}
    node_counter = [1]
    summary_counter = [1]
    rendered_states = set()
    expanded_rendered_depth = {}
    drawn_edges = set()

    def _get_id(state):
        if state not in state_to_id:
            state_to_id[state] = f"N{node_counter[0]}"
            node_counter[0] += 1
        return state_to_id[state]

    def _next_summary_id():
        nid = f"X{summary_counter[0]}"
        summary_counter[0] += 1
        return nid

    def _draw_state(state):
        nid = _get_id(state)
        if state not in rendered_states:
            _add_tree_node(dot, nid, state, best_edge[state], result, path_set, expanded_set)
            rendered_states.add(state)
        return nid

    def _draw_edge(parent_state, edge):
        parent_id = _get_id(parent_state)
        child_id = _get_id(edge.child_state)
        edge_key = (parent_id, child_id)
        if edge_key in drawn_edges:
            return

        parent_edge = best_edge.get(parent_state)
        parent_g = parent_edge.g if parent_edge else 0
        elabel = _edge_label_between(parent_g, edge)

        if transition_edges.get(parent_state) == edge:
            dot.edge(parent_id, child_id, label=elabel, color="#2563eb", penwidth="2")
        else:
            dot.edge(parent_id, child_id, label=elabel)

        drawn_edges.add(edge_key)

    def _select_children(state, depth):
        edges = successors_of.get(state, [])
        if not edges:
            return [], 0

        path_child = path_next.get(state)
        path_edge = None
        if path_child is not None:
            for edge in edges:
                if edge.child_state == path_child:
                    path_edge = edge
                    break

        if depth >= FULL_TREE_MAX_DEPTH:
            selected = [path_edge] if path_edge is not None else []
            omitted = len(edges) - len(selected)
            return selected, max(0, omitted)

        selected = []
        if path_edge is not None:
            selected.append(path_edge)

        for edge in edges:
            if path_edge is not None and edge.child_state == path_edge.child_state:
                continue
            if len(selected) >= FULL_TREE_MAX_CHILDREN_PER_NODE:
                break
            selected.append(edge)

        omitted = len(edges) - len(selected)
        return selected, max(0, omitted)

    def _draw_subtree(state, depth):
        previous_depth = expanded_rendered_depth.get(state)
        if previous_depth is not None and previous_depth <= depth:
            return
        expanded_rendered_depth[state] = depth

        parent_id = _draw_state(state)
        children, omitted = _select_children(state, depth)

        for edge in children:
            child = edge.child_state
            _draw_state(child)
            _draw_edge(state, edge)

            should_continue = (
                child in path_set
                or child in expanded_set
                or depth + 1 < FULL_TREE_MAX_DEPTH
            )
            if should_continue:
                _draw_subtree(child, depth + 1)

        if omitted:
            _add_pruned_summary(dot, parent_id, _next_summary_id(), omitted)

    _draw_subtree(root_edge.child_state, 0)
    _add_graphviz_legend(dot)

    filename = "arvore_busca_completa.svg"
    
    dot_path = os.path.join(output_dir, "arvore_busca_completa.dot")
    with open(dot_path, "w", encoding="utf-8") as f:
        f.write(dot.source)

    try:
        dot.render(os.path.join(output_dir, "arvore_busca_completa"), format="svg", cleanup=True)
        return filename
    except Exception:
        return None


# ═══════════════════════════════════════════════════════════════════════════
# 3. Geracao da interface HTML interativa
# ═══════════════════════════════════════════════════════════════════════════

def generate_html(graph, result_h5, result_h4, start_node, goal_node, output_file="resultado_busca.html"):
    """Gera um arquivo HTML limpo, claro e acadêmico, com lightbox para as imagens."""
    nodes_js = _build_nodes_js(graph)
    edges_js = _build_edges_js(graph)
    from heuristica import h5, h4
    from functools import partial
    alpha = 5 # mesmo valor usado no main.py
    
    path_js = _build_path_js(result_h5, graph, goal_node, h5)
    heuristic_table_js = _build_heuristic_table_js(result_h5, graph, start_node, goal_node, h5)
    heuristic_table_h4_js = _build_heuristic_table_js(
        result_h4, graph, start_node, goal_node, partial(h4, alpha=alpha)
    )
    tree_js = _build_focused_tree_js(result_h5)
    from estado import State
    initial_battery = 100
    if graph.is_collection_node(start_node):
        initial_battery -= 2  # COLLECTION_TIME
    start_state = State(start_node, battery_level=initial_battery, collected_points=frozenset([start_node]) if graph.is_collection_node(start_node) else frozenset())
    h_start_h5 = h5(graph, start_state, goal_node)
    h_start_h4 = h4(graph, start_state, goal_node, alpha)

    stats_js = _build_stats_js(result_h5, start_node, goal_node, h_start_h5)
    
    path_h4_js = _build_path_js(result_h4, graph, goal_node, partial(h4, alpha=alpha))
    stats_h4_js = _build_stats_js(result_h4, start_node, goal_node, h_start_h4)
    
    tree_h4_js = _build_focused_tree_js(result_h4)
    
    output_dir = os.path.dirname(output_file) or "."
    
    # Gera primeiro as imagens da h5 para salvar com sufixo proprio.
    generate_focused_tree_image(graph, result_h5, start_node, goal_node, output_dir)
    generate_full_tree_image(graph, result_h5, start_node, goal_node, output_dir)
    
    # Renomeia para nao serem sobrescritas quando a h4 for desenhada.
    import shutil
    try:
        shutil.move(os.path.join(output_dir, "arvore_busca_focada.svg"), os.path.join(output_dir, "arvore_busca_focada_h5.svg"))
        shutil.move(os.path.join(output_dir, "arvore_busca_completa.svg"), os.path.join(output_dir, "arvore_busca_completa_h5.svg"))
    except:
        pass
        
    # Agora gera as imagens da h4, usando o mesmo formato visual.
    generate_focused_tree_image(graph, result_h4, start_node, goal_node, output_dir)
    generate_full_tree_image(graph, result_h4, start_node, goal_node, output_dir)
    
    # Renomeia as imagens da h4 para ficarem lado a lado no relatorio.
    try:
        shutil.move(os.path.join(output_dir, "arvore_busca_focada.svg"), os.path.join(output_dir, "arvore_busca_focada_h4.svg"))
        shutil.move(os.path.join(output_dir, "arvore_busca_completa.svg"), os.path.join(output_dir, "arvore_busca_completa_h4.svg"))
    except:
        pass

    # Historicos
    history_h5_js = json.dumps({
        "open": result_h5.open_list_history, 
        "closed": result_h5.closed_list_history, 
        "revisited": [
            {"state": str(r['state']), "old_g": r['old_g'], "new_g": r['new_g'], "by_action": r['by_action']} for r in result_h5.revisited_states
        ],
        "expanded_later": [
            {"state": str(r['state']), "added_step": r['added_step'], "expanded_step": r['expanded_step'], "g": r['g']} for r in getattr(result_h5, 'expanded_later', [])
        ]
    })
    history_h4_js = json.dumps({
        "open": result_h4.open_list_history, 
        "closed": result_h4.closed_list_history, 
        "revisited": [
            {"state": str(r['state']), "old_g": r['old_g'], "new_g": r['new_g'], "by_action": r['by_action']} for r in result_h4.revisited_states
        ],
        "expanded_later": [
            {"state": str(r['state']), "added_step": r['added_step'], "expanded_step": r['expanded_step'], "g": r['g']} for r in getattr(result_h4, 'expanded_later', [])
        ]
    })

    def read_svg(filename):
        try:
            with open(os.path.join(output_dir, filename), 'r', encoding='utf-8') as f:
                content = f.read()
                # extrair apenas a tag <svg ...> até o final
                import re
                match = re.search(r'<svg.*</svg>', content, re.DOTALL)
                if match:
                    return match.group(0).replace('width="', 'data-w="').replace('height="', 'data-h="').replace('<svg', '<svg width="100%" height="100%"')
        except:
            pass
        return "SVG indisponível"

    svg_foc_h5 = read_svg("arvore_busca_focada_h5.svg")
    svg_cmp_h5 = read_svg("arvore_busca_completa_h5.svg")
    svg_foc_h4 = read_svg("arvore_busca_focada_h4.svg")
    svg_cmp_h4 = read_svg("arvore_busca_completa_h4.svg")

    html_content = f"""<!DOCTYPE html>
<html lang="pt-BR">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Relatório A* - Rover Marciano</title>
<link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;600;800&family=JetBrains+Mono:wght@400;700&display=swap" rel="stylesheet">
<style>{_get_css()}</style>
</head>
<body>
  <div class="app-container">
    <header class="header">
      <h1>Navegação A* - Solução Ótima (h5)</h1>
      <p>Análise de rota autônoma do Rover Marciano utilizando heurística admissível e otimização de bateria.</p>
    </header>

    <div style="background: rgba(5, 150, 105, 0.1); border-left: 4px solid var(--node-r); padding: 16px; margin-bottom: 24px; border-radius: 4px; color: var(--node-r); font-size: 1.05rem;">
      <strong>Missão Bem-Sucedida:</strong> O rover chegou ao estado objetivo corretamente nesta execução. Os dados abaixo representam a rota ótima garantida pela heurística admissível (h5).
    </div>

    <div class="stats-grid" id="stats-row"></div>
    <div class="panel metric-note">
      <strong>Leitura das métricas:</strong> estados expandidos são contados pela Closed List real da execução. O tamanho da Open List representa os estados ativos ainda disponíveis para expansão ao final da busca. Estados gerados correspondem aos estados aceitos na estrutura de execução, incluindo a raiz. A árvore Graphviz pode ocultar ramos por limite visual, mas essa compactação não altera Open, Closed nem as contagens do algoritmo.
    </div>

    <div class="section-title">Visualização do Terreno</div>
    <div class="panel">
      <div class="map-legend">
        <div class="l-item"><div class="dot bg-c"></div> Ponto de Coleta</div>
        <div class="l-item"><div class="dot bg-r"></div> Estação de Recarga</div>
        <div class="l-item"><div class="line bg-p"></div> Caminho Ótimo</div>
      </div>
      <canvas id="mapCanvas"></canvas>
    </div>

    <div class="split">
      <div>
        <div class="section-title">Trajetória (Log)</div>
        <div class="panel log-area" id="path-container"></div>
      </div>
      <div>
        <div class="section-title" style="display:flex; justify-content:space-between;">
          Hierarquia
          <button class="btn" id="btn-expand-all">Expandir Tudo</button>
        </div>
        <div class="panel log-area" id="tree-container"></div>
      </div>
    </div>

    <div class="section-title">Detalhamento da Heurística (h5) por Passo</div>
    <div class="panel table-panel">
      <p style="margin-bottom: 12px; color: var(--text-mut);">A tabela abaixo apresenta como a heurística admissível <strong>h5</strong> estimou o custo restante em cada passo da trajetória apresentada. A soma dos componentes (deslocamento, coleta e recarga mínimos) resulta no valor final de <code>h5(s)</code>.</p>
      <table class="heuristic-table">
        <thead>
          <tr style="border-bottom: 2px solid var(--border);">
            <th style="padding: 12px 8px;">Etapa</th>
            <th style="padding: 12px 8px;">Estado (p,b,K)</th>
            <th style="padding: 12px 8px;">Ação</th>
            <th style="padding: 12px 8px;">g(s)</th>
            <th style="padding: 12px 8px;">r(s)</th>
            <th style="padding: 12px 8px;">Tdesloc_min(s)</th>
            <th style="padding: 12px 8px;">Tcoleta_min(s)</th>
            <th style="padding: 12px 8px;">Trecarga_min(s)</th>
            <th style="padding: 12px 8px; color: var(--node-r);">h5(s)</th>
            <th style="padding: 12px 8px; color: var(--node-c);">f(s)</th>
          </tr>
        </thead>
        <tbody id="heuristic-tbody">
        </tbody>
      </table>
      <p class="formula-note">Na tabela, <code>s = (p,b,K)</code>: <code>p</code> é a posição atual, <code>b</code> é a bateria e <code>K</code> é o conjunto de coletas realizadas. O valor <code>g(s)</code> é o custo real acumulado até o estado, considerando deslocamentos, coletas e recargas reais já executadas. O valor <code>r(s)</code> é o número de coletas restantes para cumprir a missão: <code>r(s) = max(0, m - |K|)</code>, com <code>m = {MIN_COLLECTIONS}</code>.</p>
      <p class="formula-note">Os termos da heurística são calculados explicitamente assim: <code>Tdesloc_min(s) = d_km(p,G) / v_max</code>, em que <code>d_km(p,G) = dist_euclidiana(p,G) × {PLANE_UNIT_IN_KM}</code>. De forma expandida, <code>Tdesloc_min(s) = sqrt((x_G - x_p)^2 + (y_G - y_p)^2) × {PLANE_UNIT_IN_KM} / {MAX_SPEED}</code>. O termo de coleta é <code>Tcoleta_min(s) = 2 × r(s)</code>, porque cada coleta custa 2 minutos. O termo de recarga é <code>Trecarga_min(s) = max(0, Tdesloc_min(s) + Tcoleta_min(s) + {MIN_BATTERY} - b) / {RECHARGE_RATE}</code>, que estima a recarga mínima necessária para não chegar ao objetivo com bateria nula. Assim, <code>h5(s) = Tdesloc_min(s) + Tcoleta_min(s) + Trecarga_min(s)</code> e <code>f(s) = g(s) + h5(s)</code>. O termo <code>Trecarga_min(s)</code> faz parte da heurística, enquanto a recarga real executada no caminho entra em <code>g(s)</code>.</p>
    </div>

    <div class="section-title">Detalhamento da Heurística (h4) por Passo</div>
    <div class="panel table-panel">
      <p style="margin-bottom: 12px; color: var(--text-mut);">A tabela abaixo usa os mesmos estados e os mesmos termos mínimos da tabela h5. A diferença é que a heurística inflada calcula <code>h4(s) = α × h5(s)</code>, com <code>α = 5</code>, e depois calcula <code>f(s) = g(s) + h4(s)</code>.</p>
      <table class="heuristic-table">
        <thead>
          <tr style="border-bottom: 2px solid var(--border);">
            <th style="padding: 12px 8px;">Etapa</th>
            <th style="padding: 12px 8px;">Estado (p,b,K)</th>
            <th style="padding: 12px 8px;">Ação</th>
            <th style="padding: 12px 8px;">g(s)</th>
            <th style="padding: 12px 8px;">r(s)</th>
            <th style="padding: 12px 8px;">Tdesloc_min(s)</th>
            <th style="padding: 12px 8px;">Tcoleta_min(s)</th>
            <th style="padding: 12px 8px;">Trecarga_min(s)</th>
            <th style="padding: 12px 8px; color: var(--node-r);">h4(s)</th>
            <th style="padding: 12px 8px; color: var(--node-c);">f(s)</th>
          </tr>
        </thead>
        <tbody id="heuristic-h4-tbody">
        </tbody>
      </table>
      <p class="formula-note">Para comparação, <code>Tdesloc_min(s)</code>, <code>Tcoleta_min(s)</code>, <code>Trecarga_min(s)</code>, <code>r(s)</code> e <code>g(s)</code> permanecem iguais. Apenas o valor da estimativa muda: <code>h4(s) = 5 × h5(s)</code>. Por isso, a h4 pode ficar mais agressiva, mas deixa de ser admissível porque pode superestimar o custo restante.</p>
    </div>

    <div class="section-title">Árvores de Busca Geradas</div>
    <div class="panel visual-note">
      <strong>Como ler as árvores compactadas:</strong> os ramos omitidos por limite visual representam estados que foram gerados durante a execução do A*, mas não foram desenhados para evitar uma figura excessivamente grande e difícil de ler. Isso não é poda do algoritmo: a execução, as listas Open/Closed e as métricas reais continuam considerando os estados normalmente. Sem esse limite visual, a imagem precisaria desenhar todos os sucessores e sub-ramos gerados pela busca.
    </div>
    
    <h3 style="font-size: 1.1rem; margin-bottom: 12px; color: var(--path);">Heurística Admissível (h5)</h3>
    <div class="images-grid" style="margin-bottom: 24px;">
      <div class="img-card" onclick="openLightbox(this)">
        <div class="svg-container">{svg_foc_h5}</div>
        <div class="img-title">Árvore Focada (h5)</div>
        <div class="img-desc">Azul é o caminho escolhido. Verde é ação de recarga. Branco é nó gerado/Open. Cinza é nó expandido/Closed. Caixas amarelas indicam ramos ocultados apenas por limite visual.</div>
      </div>
      <div class="img-card" onclick="openLightbox(this)">
        <div class="svg-container">{svg_cmp_h5}</div>
        <div class="img-title">Árvore Completa Compactada (h5)</div>
        <div class="img-desc">Mostra uma árvore compactada. As caixas amarelas são estados gerados, mas não desenhados para manter a figura legível; não representam poda do A*.</div>
      </div>
    </div>

    <h3 style="font-size: 1.1rem; margin-bottom: 12px; color: var(--node-c);">Heurística Não Admissível (h4)</h3>
    <div class="images-grid">
      <div class="img-card" onclick="openLightbox(this)">
        <div class="svg-container">{svg_foc_h4}</div>
        <div class="img-title">Árvore Focada (h4)</div>
        <div class="img-desc">Azul é o caminho escolhido. Verde é ação de recarga. Branco é nó gerado/Open. Cinza é nó expandido/Closed. Caixas amarelas indicam ramos ocultados apenas por limite visual.</div>
      </div>
      <div class="img-card" onclick="openLightbox(this)">
        <div class="svg-container">{svg_cmp_h4}</div>
        <div class="img-title">Árvore Completa Compactada (h4)</div>
        <div class="img-desc">Mostra uma árvore compactada da h4. As caixas amarelas são estados gerados, mas não desenhados para manter a figura legível; não representam poda do A*.</div>
      </div>
    </div>

    <div class="section-title" style="margin-top: 60px;">Análise de Heurísticas (Admissível vs Não Admissível)</div>
    <div class="panel">
      <h3 style="font-size: 1.3rem; margin-bottom: 8px;">1. A Heurística Admissível (h5)</h3>
      <p style="margin-bottom: 12px; color: var(--text-mut);">No algoritmo A*, a função heurística h(s) tem o papel de estimar o custo restante entre o estado atual do agente e o objetivo final. Essa estimativa é usada junto ao custo real acumulado g(n), formando a função: <strong>f(s) = g(s) + h(s)</strong>.</p>
      <p style="margin-bottom: 12px; color: var(--text-mut);">No contexto deste trabalho, g(s) representa o tempo real já gasto até o estado atual, enquanto h(s) estima, de forma otimista, o tempo restante até concluir a missão. A heurística adotada como principal neste trabalho é a <strong>h5</strong>, que considera três componentes mínimos do custo restante:</p>
      <div style="background: rgba(0,0,0,0.05); padding: 12px; border-radius: 6px; font-family: 'JetBrains Mono'; margin-bottom: 16px;">h5(s) = T_min_desloc(s) + T_min_coleta(s) + T_min_recarga(s)</div>
      <p style="margin-bottom: 24px; color: var(--text-mut);">Ela é mais adequada ao problema porque leva em conta a bateria atual do agente e o número de coletas restantes, mas continua <strong>admissível</strong> por usar apenas estimativas otimistas (distância em linha reta, tempo mínimo sem desvios).</p>

      <h3 style="font-size: 1.3rem; margin-bottom: 8px;">2. A Heurística Não Admissível (h4 - Inflada)</h3>
      <p style="margin-bottom: 12px; color: var(--text-mut);">Foi considerada uma heurística não admissível, chamada de heurística inflada: <code>h4(s) = α * h5(s)</code>. Neste caso, utilizamos α = 5. Essa heurística pode tornar a busca mais agressiva e reduzir o número de estados expandidos, mas pode superestimar o custo real restante. Por isso, <strong>ela não garante que o A* encontre o caminho ótimo</strong>.</p>
      
      <div class="split" style="margin-top: 24px;">
        <div style="border: 1px solid var(--border); padding: 16px; border-radius: 8px;">
          <h4 style="margin-bottom: 12px; color: var(--path);">Resultado h5 (Admissível)</h4>
          <ul style="list-style: none; font-family: 'JetBrains Mono'; font-size: 0.9rem;">
            <li>Estimativa Inicial h(start): <span id="h5-hstart"></span> min</li>
            <li>Custo Total: <span id="h5-cost"></span> min</li>
            <li>Estados Expandidos: <span id="h5-expanded"></span></li>
            <li>Passos no Caminho: <span id="h5-path-len"></span></li>
          </ul>
        </div>
        <div style="border: 1px solid var(--node-c); padding: 16px; border-radius: 8px; background: rgba(234, 88, 12, 0.05);">
          <h4 style="margin-bottom: 12px; color: var(--node-c);">Resultado h4 (Não Admissível)</h4>
          <ul style="list-style: none; font-family: 'JetBrains Mono'; font-size: 0.9rem;">
            <li>Estimativa Inicial h(start): <span id="h4-hstart"></span> min</li>
            <li>Custo Total: <span id="h4-cost"></span> min</li>
            <li>Estados Expandidos: <span id="h4-expanded"></span></li>
            <li>Passos no Caminho: <span id="h4-path-len"></span></li>
          </ul>
        </div>
      </div>
      <p style="margin-top: 16px; color: var(--node-c); font-weight: bold; padding: 12px; border-left: 4px solid var(--node-c); background: rgba(234, 88, 12, 0.1);">
        Note que se o custo da solução com h4 for maior que o de h5, fica provado empiricamente que a solução encontrada por h4 não é a ótima.
      </p>

      <h3 style="font-size: 1.3rem; margin-top: 40px; margin-bottom: 16px;">3. Histórico de Execução (Listas Open e Closed)</h3>
      <p style="margin-bottom: 12px; color: var(--text-mut);">Abaixo está a listagem de como as listas aberta (nós disponíveis para expansão) e fechada (nós já expandidos) evoluem. Para não sobrecarregar, estamos exibindo as últimas 10 interações que culminaram no fim da execução.</p>
      <div style="display: flex; gap: 12px; margin-bottom: 16px;">
        <button class="btn" id="btn-hist-h5">Ver Histórico h5</button>
        <button class="btn" id="btn-hist-h4" style="background: var(--node-c);">Ver Histórico h4</button>
      </div>
      
      <div class="log-area" id="history-log" style="background: #1e1e1e; color: #d4d4d4; padding: 16px; border-radius: 8px; font-family: 'JetBrains Mono', monospace; font-size: 0.85rem; border: 1px solid #333; height: 350px; white-space: pre-wrap;">
        Selecione uma heurística acima para ver o passo a passo.
      </div>

      <h3 style="font-size: 1.3rem; margin-top: 40px; margin-bottom: 16px;">4. Possíveis Soluções e a Solução Escolhida pelo A*</h3>
      <p style="margin-bottom: 12px; color: var(--text-mut);">Durante a busca, o A* com a heurística admissível avalia várias rotas em potencial antes de cravar a melhor. A solução escolhida pelo algoritmo é sempre aquela que minimiza <strong>f(n) = g(n) + h(n)</strong>.</p>
      <div id="topic4-container" style="background: var(--bg); padding: 16px; border-radius: 8px; border: 1px solid var(--border);">
        <!-- Preenchido dinamicamente pelo JavaScript da pagina -->
      </div>
      
      <h3 style="font-size: 1.3rem; margin-top: 40px; margin-bottom: 16px;">5. Dinâmica da Open List (Permanência de Estados)</h3>
      <p style="margin-bottom: 12px; color: var(--text-mut);">Durante a execução, alguns estados foram gerados e permaneceram na Open List por vários passos antes de serem expandidos. Isso mostra que o A* não segue necessariamente o último estado gerado, mas sempre seleciona da Open List o estado com menor valor de f(n).</p>
      <div class="log-area" id="revisited-log" style="background: rgba(5, 150, 105, 0.1); padding: 16px; border-radius: 8px; font-family: 'JetBrains Mono', monospace; font-size: 0.85rem; border: 1px solid var(--node-r); color: var(--node-r); white-space: pre-wrap;">
      </div>
    </div>
  </div>

  <!-- Janela de ampliacao das imagens -->
  <div class="lightbox" id="lightbox" onclick="closeLightbox()">
    <div class="lb-close">&times;</div>
    <div id="lb-content" style="width: 90vw; height: 90vh; display: flex; justify-content: center; align-items: center; background: white; border-radius: 8px; padding: 20px; box-shadow: 0 10px 40px rgba(0,0,0,0.3);"></div>
  </div>

<script>
const NODES={nodes_js};
const EDGES={edges_js};
const PATH={path_js};
const HEURISTIC_TABLE={heuristic_table_js};
const HEURISTIC_TABLE_H4={heuristic_table_h4_js};
const TREE={tree_js};
const STATS={stats_js};
const PATH_H4={path_h4_js};
const STATS_H4={stats_h4_js};
const HIST_H5={history_h5_js};
const HIST_H4={history_h4_js};
const START_NODE="{start_node}";
const GOAL_NODE="{goal_node}";
{_get_javascript()}
</script>
</body>
</html>"""

    with open(output_file, "w", encoding="utf-8") as f:
        f.write(html_content)
    return output_file


# ── Funções Auxiliares para Serialização em JavaScript ──

def _build_nodes_js(graph):
    items = []
    for node, (x, y) in graph.positions.items():
        ntype = "collection" if graph.is_collection_node(node) else "recharge"
        items.append(f'{{"id":"{node}","x":{x},"y":{y},"type":"{ntype}"}}')
    return "[" + ",".join(items) + "]"

def _build_edges_js(graph):
    seen = set()
    items = []
    for na in graph.adjacency_list:
        for nb, w in graph.adjacency_list[na]:
            key = tuple(sorted([na, nb]))
            if key not in seen:
                seen.add(key)
                items.append(f'{{"from":"{na}","to":"{nb}","weight":{w}}}')
    return "[" + ",".join(items) + "]"

def _build_path_js(result, graph, goal_node, h_func):
    if not result.success:
        return "[]"
    
    from heuristica import (
        min_collection_time,
        min_displacement_time,
        min_recharge_time,
        remaining_collections,
    )
    items = []
    for step in result.path:
        state = step["state"]
        col = ",".join(sorted(state.collected_points))
        action = html_module.escape(step["action"], quote=True)
        h_val = h_func(graph, state, goal_node)
        f_val = step["g"] + h_val
        
        h_desloc = min_displacement_time(graph, state, goal_node)
        h_coleta = min_collection_time(state)
        h_recarga = min_recharge_time(graph, state, goal_node)
        remaining = remaining_collections(state)

        items.append(
            f'{{"node":"{state.current_node}",'
            f'"battery":{state.battery_level},'
            f'"collected":"{col}",'
            f'"numCollected":{len(state.collected_points)},'
            f'"action":"{action}",'
            f'"g":{step["g"]:.2f},'
            f'"h":{h_val:.2f},'
            f'"f":{f_val:.2f},'
            f'"h_desloc":{h_desloc:.2f},'
            f'"remaining":{remaining},'
            f'"h_coleta":{h_coleta:.2f},'
            f'"h_recarga":{h_recarga:.2f}}}'
        )
    return "[" + ",".join(items) + "]"


def _build_heuristic_table_js(result, graph, start_node, goal_node, h_func):
    return _build_path_js(result, graph, goal_node, h_func)


def _build_focused_tree_js(result):
    if not result.success or not result.path:
        return "null"
    path_states = [step["state"] for step in result.path]
    path_set = set(path_states)
    _, successors_of, best_edge, expanded_set = _build_tree_indexes(result)

    def _build_node(state, edge_data, is_on_path):
        children = []
        if is_on_path:
            alternatives = [
                edge for edge in successors_of.get(state, [])
                if edge.child_state not in path_set
            ][:FOCUSED_MAX_ALTERNATIVES_PER_STEP]
            for succ_edge in alternatives:
                child = succ_edge.child_state
                was_expanded = child in expanded_set
                child_node = {
                    "node": child.current_node, "battery": child.battery_level, "numCollected": len(child.collected_points),
                    "g": succ_edge.g, "h": succ_edge.h, "f": succ_edge.f, "action": succ_edge.action,
                    "isSolution": False, "isExpanded": was_expanded, "children": []
                }
                children.append(child_node)
        return {
            "node": state.current_node, "battery": state.battery_level, "numCollected": len(state.collected_points),
            "g": edge_data.g, "h": edge_data.h, "f": edge_data.f, "action": edge_data.action,
            "isSolution": is_on_path, "isExpanded": True, "children": children
        }

    nodes = []
    for i, state in enumerate(path_states):
        nodes.append(_build_node(state, best_edge[state], True))
    for i in range(len(nodes) - 1):
        nodes[i]["children"].insert(0, nodes[i + 1])
    return json.dumps(nodes[0])

def _build_stats_js(result, start_node, goal_node, h_start):
    fb = 0
    fc = 0
    if result.success and result.final_state:
        fb = result.final_state.battery_level
        fc = len(result.final_state.collected_points)
    return (
        f'{{"success":{str(result.success).lower()},'
        f'"startNode":"{start_node}","goalNode":"{goal_node}",'
        f'"hStart":{h_start:.2f},'
        f'"totalTime":{result.total_time:.2f},'
        f'"expandedStates":{result.expanded_states},'
        f'"openListSize":{result.open_list_size},'
        f'"closedListSize":{result.closed_list_size},'
        f'"finalBattery":{fb},"finalCollected":{fc},'
        f'"treeEdges":{len(result.search_tree)}}}'
    )

# CSS da pagina HTML.
def _get_css():
    return """
:root {
  --bg: #fafafa; --surface: #ffffff; --border: #eaeaea;
  --text-main: #111111; --text-mut: #666666;
  --accent: #111111; --path: #2563eb; --node-c: #ea580c; --node-r: #059669;
}
* { margin:0; padding:0; box-sizing:border-box; }
body { font-family: 'Inter', sans-serif; background: var(--bg); color: var(--text-main); line-height: 1.5; padding: 40px 20px; }
.app-container { max-width: 1200px; margin: 0 auto; }
.header { text-align: center; margin-bottom: 60px; padding-bottom: 40px; border-bottom: 1px solid var(--border); }
.header h1 { font-size: 3.5rem; font-weight: 800; letter-spacing: 0; margin-bottom: 16px; color: var(--text-main); }
.header p { font-size: 1.2rem; color: var(--text-mut); max-width: 600px; margin: 0 auto; }
.stats-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 40px; margin-bottom: 60px; text-align: center; }
.stat-item .value { font-size: 4rem; font-weight: 800; letter-spacing: 0; margin-bottom: 8px; color: var(--text-main); }
.stat-item .label { font-size: 0.85rem; text-transform: uppercase; letter-spacing: 1px; color: var(--text-mut); font-weight: 600; }
.section-title { font-size: 1.8rem; font-weight: 800; letter-spacing: 0; margin-bottom: 24px; border-bottom: 1px solid var(--border); padding-bottom: 16px; }
.panel { background: var(--surface); border: 1px solid var(--border); border-radius: 12px; padding: 32px; margin-bottom: 40px; box-shadow: 0 4px 20px rgba(0,0,0,0.02); }
.table-panel { overflow-x: auto; -webkit-overflow-scrolling: touch; }
.heuristic-table { width: 100%; min-width: 1080px; border-collapse: collapse; text-align: left; font-size: 0.95rem; }
.heuristic-table th, .heuristic-table td { white-space: nowrap; vertical-align: top; }
.heuristic-table td:nth-child(3) { white-space: normal; min-width: 210px; }
.formula-note { margin-top: 16px; margin-bottom: 0; color: var(--text-mut); }
.metric-note { border-left: 4px solid var(--path); background: #eff6ff; color: #1e3a8a; padding: 18px 22px; }
.visual-note { border-left: 4px solid #f59e0b; background: #fffbeb; color: #78350f; padding: 18px 22px; }
.map-legend { display: flex; gap: 20px; margin-bottom: 20px; font-size: 0.9rem; color: var(--text-mut); }
.l-item { display: flex; align-items: center; gap: 8px; }
.dot { width: 12px; height: 12px; border-radius: 50%; }
.bg-c { background: var(--node-c); } .bg-r { background: var(--node-r); } 
.line { width: 20px; height: 3px; border-radius: 2px; } .bg-p { background: var(--path); }
#mapCanvas { width: 100%; border: 1px solid var(--border); border-radius: 8px; background: #fff; }
.split { display: grid; grid-template-columns: 1fr 1fr; gap: 32px; }
@media (max-width: 800px) { .split { grid-template-columns: 1fr; } }
.log-area { max-height: 500px; overflow-y: auto; padding-right: 16px; }
.log-area::-webkit-scrollbar { width: 6px; } .log-area::-webkit-scrollbar-thumb { background: #ccc; border-radius: 3px; }
.path-step { padding: 16px 0; border-bottom: 1px solid var(--border); }
.path-step:last-child { border-bottom: none; }
.p-node { font-size: 1.2rem; font-weight: 700; margin-bottom: 4px; }
.p-act { color: var(--text-mut); font-size: 0.95rem; margin-bottom: 8px; }
.p-metrics { display: flex; gap: 12px; font-family: 'JetBrains Mono', monospace; font-size: 0.8rem; color: var(--text-main); }
.tree-node { margin-left: 24px; position: relative; border-left: 1px solid #ddd; padding-left: 16px; margin-top: 8px; }
.tree-item { display: inline-flex; align-items: center; flex-wrap: wrap; max-width: 100%; gap: 12px; padding: 8px 12px; cursor: pointer; border-radius: 6px; background: #f9f9f9; transition: background 0.2s; border: 1px solid transparent; }
.tree-item:hover { background: #f1f1f1; }
.tree-item.is-sol { background: #eff6ff; border-color: #bfdbfe; }
.tree-toggle { width: 16px; font-weight: bold; color: var(--text-mut); text-align: center; }
.t-node { font-weight: 700; font-size: 1rem; }
.t-data { color: var(--text-mut); font-size: 0.85rem; font-family: 'JetBrains Mono', monospace; overflow-wrap: anywhere; }
.tree-kids { display: none; } .tree-kids.open { display: block; }
.btn { background: var(--accent); color: #fff; border: none; padding: 10px 20px; font-size: 0.9rem; font-weight: 600; border-radius: 6px; cursor: pointer; transition: opacity 0.2s; }
.btn:hover { opacity: 0.8; }
.images-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 32px; }
@media (max-width: 800px) { .images-grid { grid-template-columns: 1fr; } }
.img-card { cursor: zoom-in; transition: transform 0.2s; background: var(--surface); border: 1px solid var(--border); padding: 16px; border-radius: 12px; box-shadow: 0 4px 20px rgba(0,0,0,0.02); }
.img-card:hover { transform: translateY(-4px); }
.img-card img { width: 100%; border: 1px solid var(--border); border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.05); background: #fff; }
.img-title { font-size: 1.2rem; font-weight: 700; margin-top: 16px; margin-bottom: 8px; }
.img-desc { color: var(--text-mut); font-size: 0.95rem; }
/* Lightbox */
.lightbox { position: fixed; top: 0; left: 0; width: 100%; height: 100%; background: rgba(255,255,255,0.95); display: flex; align-items: center; justify-content: center; z-index: 1000; opacity: 0; pointer-events: none; transition: opacity 0.3s; padding: 40px; }
.lightbox.active { opacity: 1; pointer-events: auto; }
.lightbox img { max-width: 100%; max-height: 100%; box-shadow: 0 10px 40px rgba(0,0,0,0.1); border-radius: 8px; }
.lb-close { position: absolute; top: 30px; right: 40px; font-size: 2.5rem; cursor: pointer; font-weight: 300; color: #000; }
@media (max-width: 700px) {
  body { padding: 20px 12px; }
  .header { margin-bottom: 32px; padding-bottom: 24px; }
  .header h1 { font-size: 2rem; }
  .header p { font-size: 1rem; }
  .stats-grid { grid-template-columns: repeat(auto-fit, minmax(140px, 1fr)); gap: 20px; margin-bottom: 36px; }
  .stat-item .value { font-size: 2.4rem; }
  .section-title { font-size: 1.35rem; margin-bottom: 16px; padding-bottom: 12px; }
  .panel { padding: 16px; margin-bottom: 28px; border-radius: 8px; }
  .table-panel { margin-left: -12px; margin-right: -12px; border-left: 0; border-right: 0; border-radius: 0; }
  .heuristic-table { min-width: 1080px; font-size: 0.82rem; }
  .map-legend { flex-wrap: wrap; gap: 12px; }
  .p-metrics { flex-wrap: wrap; gap: 8px; }
  .tree-node { margin-left: 12px; padding-left: 10px; }
  .tree-item { max-width: 100%; overflow-x: auto; flex-wrap: wrap; }
  .images-grid { gap: 20px; }
  .img-card { border-radius: 8px; padding: 12px; }
  .lightbox { padding: 16px; }
  .lb-close { top: 14px; right: 18px; }
}
"""

# ── JavaScript ──
def _get_javascript():
    return r"""
function renderStats(){
  const r=document.getElementById('stats-row');
  const d=[
    {l:'Bateria Final',v:STATS.finalBattery+'%'},
    {l:'Estados Expandidos',v:STATS.expandedStates},
    {l:'Minutos Operacionais',v:STATS.totalTime.toFixed(1)},
    {l:'Coletas Realizadas',v:STATS.finalCollected},
    {l:'Open final ativa',v:STATS.openListSize},
    {l:'Closed final',v:STATS.closedListSize},
    {l:'Estados gerados',v:STATS.treeEdges}
  ];
  r.innerHTML=d.map(x=>`<div class="stat-item"><div class="value">${x.v}</div><div class="label">${x.l}</div></div>`).join('');
}
function drawMap(){
  const canvas=document.getElementById('mapCanvas');const ctx=canvas.getContext('2d');
  let minX=Infinity,maxX=-Infinity,minY=Infinity,maxY=-Infinity;
  NODES.forEach(n=>{minX=Math.min(minX,n.x);maxX=Math.max(maxX,n.x);minY=Math.min(minY,n.y);maxY=Math.max(maxY,n.y)});
  const padding=40,cw=canvas.parentElement.clientWidth,rx=maxX-minX||1,ry=maxY-minY||1;
  const scale=Math.min((cw-padding*2)/rx,400/ry),ch=ry*scale+padding*2;
  canvas.width=cw*2;canvas.height=ch*2;canvas.style.height=ch+'px';ctx.scale(2,2);
  function tx(x){return padding+(x-minX)*scale} function ty(y){return ch-padding-(y-minY)*scale}
  
  const pE=new Set(),pN=new Set();
  for(let i=0;i<PATH.length;i++){pN.add(PATH[i].node);if(i>0)pE.add([PATH[i-1].node,PATH[i].node].sort().join('-'))}
  
  EDGES.forEach(e=>{
    const nA=NODES.find(n=>n.id===e.from),nB=NODES.find(n=>n.id===e.to);
    const isP=pE.has([e.from,e.to].sort().join('-'));
    ctx.beginPath();ctx.moveTo(tx(nA.x),ty(nA.y));ctx.lineTo(tx(nB.x),ty(nB.y));
    if(isP){ctx.strokeStyle='#2563eb';ctx.lineWidth=3;}
    else{ctx.strokeStyle='#eaeaea';ctx.lineWidth=1;ctx.setLineDash([4,4]);}
    ctx.stroke();ctx.setLineDash([]);
    const mx=(tx(nA.x)+tx(nB.x))/2,my=(ty(nA.y)+ty(nB.y))/2;
    ctx.font='10px "JetBrains Mono"';ctx.fillStyle=isP?'#2563eb':'#aaa';ctx.textAlign='center';ctx.textBaseline='middle';
    ctx.fillText(e.weight,mx,my-8);
  });
  
  NODES.forEach(n=>{
    const x=tx(n.x),y=ty(n.y),isP=pN.has(n.id);
    const color=n.type==='collection'?'#ea580c':'#059669';
    // Desenhando em círculo como solicitado
    ctx.beginPath();ctx.arc(x,y,isP?20:16,0,2*Math.PI);
    ctx.fillStyle=isP?color:'#fff'; ctx.fill();
    ctx.strokeStyle=color; ctx.lineWidth=2; ctx.stroke();
    ctx.font='600 12px "Inter"';ctx.fillStyle=isP?'#fff':color;ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText(n.id,x,y);
  });
}
function renderPath(){
  const c=document.getElementById('path-container');
  if(!STATS.success){c.innerHTML='Caminho não encontrado.';return}
  c.innerHTML=PATH.map((s,i)=>`<div class="path-step"><div class="p-node">Passo ${i}: ${s.node}</div><div class="p-act">${s.action}</div><div class="p-metrics"><span>Custo (g): ${s.g.toFixed(1)}</span><span>h(n): ${s.h.toFixed(1)}</span><span>f(n): ${s.f.toFixed(1)}</span><span>Bateria: ${s.battery}%</span><span>Coletas: ${s.numCollected}</span></div></div>`).join('');
}
function buildT(n){
  const has=n.children&&n.children.length>0;
  let cls=''; if(n.isSolution)cls=' is-sol';
  let tog='<span class="tree-toggle"> </span>'; if(has)tog='<span class="tree-toggle" data-toggle>+</span>';
  let ch=''; if(has)ch=`<div class="tree-kids open">${n.children.map(buildT).join('')}</div>`;
  return `<div class="tree-node"><div class="tree-item${cls}">${tog}<span class="t-node">${n.node}</span><span class="t-data"> g:${n.g.toFixed(1)} | h:${n.h.toFixed(1)} | f:${n.f.toFixed(1)} | Bateria:${n.battery}% | Coletas:${n.numCollected}</span></div>${ch}</div>`;
}
function renderTree(){
  const c=document.getElementById('tree-container');
  if(!TREE)return;
  c.innerHTML=buildT(TREE);
  document.querySelectorAll('[data-toggle]').forEach(t=>t.addEventListener('click',function(e){
    e.stopPropagation(); const p=this.closest('.tree-node').querySelector('.tree-kids');
    if(p){p.classList.toggle('open'); this.textContent=p.classList.contains('open')?'+':'-';}
  }));
  let exp=true;
  document.getElementById('btn-expand-all').addEventListener('click',function(){
    exp=!exp; document.querySelectorAll('.tree-kids').forEach(k=>exp?k.classList.add('open'):k.classList.remove('open'));
    document.querySelectorAll('[data-toggle]').forEach(t=>t.textContent=exp?'+':'-');
    this.textContent=exp?'Recolher Tudo':'Expandir Tudo';
  });
}

// Lightbox Logic
function openLightbox(el) {
  const svgContent = el.querySelector('svg');
  if(svgContent) {
    const lbContainer = document.getElementById('lb-content');
    lbContainer.innerHTML = '';
    const cloned = svgContent.cloneNode(true);
    cloned.style.width = '100%';
    cloned.style.height = '100%';
    lbContainer.appendChild(cloned);
  }
  document.getElementById('lightbox').classList.add('active');
}
function closeLightbox() {
  document.getElementById('lightbox').classList.remove('active');
  document.getElementById('lb-content').innerHTML = '';
}

function renderAnalysis() {
  document.getElementById('h5-hstart').innerText = (STATS.hStart !== undefined) ? STATS.hStart.toFixed(1) : 'N/A';
  document.getElementById('h5-cost').innerText = STATS.totalTime.toFixed(1);
  document.getElementById('h5-expanded').innerText = STATS.expandedStates;
  document.getElementById('h5-path-len').innerText = PATH.length;
  
  document.getElementById('h4-hstart').innerText = (STATS_H4.hStart !== undefined) ? STATS_H4.hStart.toFixed(1) : 'N/A';
  document.getElementById('h4-cost').innerText = STATS_H4.totalTime.toFixed(1);
  document.getElementById('h4-expanded').innerText = STATS_H4.expandedStates;
  document.getElementById('h4-path-len').innerText = PATH_H4.length;
  
  function formatLog(hist) {
    if(!hist || !hist.open || hist.open.length===0) return 'Nenhum histórico capturado.';
    let txt = '';
    // Pegar as ultimas 10 interacoes para nao travar o browser
    let start = Math.max(0, hist.open.length - 10);
    txt += `Exibindo as últimas ${hist.open.length - start} iterações (Total: ${hist.open.length} expansões realizadas):\n========================================================\n\n`;
    for(let i=start; i<hist.open.length; i++) {
       txt += `--- PASSO ${i+1} ---\n`;
       txt += `[OPEN LIST] (${hist.open[i].length} estados não expandidos):\n${hist.open[i].join('\n')}\n\n`;
       txt += `[CLOSED LIST] (${hist.closed[i].length} estados já expandidos):\n${hist.closed[i].join('\n')}\n\n`;
    }
    return txt;
  }
  
  function formatRevisited(hist) {
     let txt = 'Esse comportamento ocorre porque os estados permanecem na Open List até que se tornem os mais promissores segundo f(n)=g(n)+h(n). Assim, o algoritmo pode voltar a expandir estados gerados anteriormente, caso eles passem a ter a melhor prioridade entre as opções disponíveis.\n\n';
     txt += 'Exemplos observados na execução (estados que esperaram na Open List):\n';
     txt += '-'.repeat(100) + '\n';
     if(hist && hist.expanded_later && hist.expanded_later.length > 0) {
        hist.expanded_later.slice(0, 10).forEach(r => {
           txt += `- ${r.state} entrou na Open List no passo ${r.added_step} e foi expandido apenas no passo ${r.expanded_step}, com g(n)=${r.g.toFixed(1)}.\n`;
        });
     } else {
        txt += 'Não houve exemplos nesta execução específica.\n';
     }
     return txt;
  }
  
  document.getElementById('btn-hist-h5').onclick = function() {
    document.getElementById('history-log').innerText = formatLog(HIST_H5);
    document.getElementById('revisited-log').innerText = formatRevisited(HIST_H5);
  };
  document.getElementById('btn-hist-h4').onclick = function() {
    document.getElementById('history-log').innerText = formatLog(HIST_H4);
    document.getElementById('revisited-log').innerText = formatRevisited(HIST_H4);
  };
  
  function renderPossibleSolutions() {
    const container = document.getElementById('topic4-container');
    if(!container) return;

    let html = `<p><strong>A Solução Escolhida (Ótima) - h5:</strong></p>`;
    html += `<table style="width: 100%; border-collapse: collapse; margin-bottom: 20px; font-family: 'JetBrains Mono', monospace; font-size: 0.85rem; text-align: left;">`;
    html += `<thead><tr style="background: var(--surface); border-bottom: 2px solid var(--border);"><th style="padding: 8px;">Passo</th><th style="padding: 8px;">Nó Atual</th><th style="padding: 8px;">Ação</th><th style="padding: 8px;">Custo (g)</th><th style="padding: 8px;">Bateria</th></tr></thead>`;
    html += `<tbody>`;
    PATH.forEach((step, idx) => {
      html += `<tr style="border-bottom: 1px solid var(--border);"><td style="padding: 8px;">${idx}</td><td style="padding: 8px;">${step.node}</td><td style="padding: 8px; color: var(--text-mut);">${step.action}</td><td style="padding: 8px; color: var(--path); font-weight: bold;">${step.g.toFixed(1)}</td><td style="padding: 8px;">${step.battery}%</td></tr>`;
    });
    html += `</tbody></table>`;
    html += `<p style="margin-bottom: 16px;"><strong>Motivo da Escolha:</strong> Este caminho foi o primeiro a alcançar o destino tendo expandido sempre pelo menor custo <strong>f(n) = g(n) + h(n)</strong>, garantindo a otimalidade.</p>`;

    html += `<p><strong>Outras Soluções Possíveis / Caminhos Explorados (Sub-ótimos na lista fechada ou abertos na Open List):</strong></p>`;
    
    // Find some alternative paths by traversing the tree
    let altPaths = [];
    function traverse(node, currentPath) {
      currentPath.push(node);
      if(!node.isSolution && (!node.children || node.children.length === 0)) {
        if (currentPath.length > 2) {
            altPaths.push([...currentPath]);
        }
      }
      if(node.children) {
        node.children.forEach(c => traverse(c, currentPath));
      }
      currentPath.pop();
    }
    traverse(TREE, []);
    
    // Sort by length descending, and pick 2 diverse ones
    altPaths.sort((a,b) => b.length - a.length);
    let selectedAlts = altPaths.slice(0, 3);

    if (selectedAlts.length > 0) {
        html += `<table style="width: 100%; border-collapse: collapse; font-family: 'JetBrains Mono', monospace; font-size: 0.85rem; text-align: left;">`;
        html += `<thead><tr style="background: rgba(234, 88, 12, 0.05); border-bottom: 2px solid var(--node-c);"><th style="padding: 8px;">Alternativa</th><th style="padding: 8px;">Caminho Parcial</th><th style="padding: 8px;">Custo f(n) no momento</th><th style="padding: 8px;">Status</th></tr></thead>`;
        html += `<tbody>`;
        selectedAlts.forEach((pathArr, idx) => {
            let pathStr = pathArr.map(n => n.node).join(' -> ');
            let lastNode = pathArr[pathArr.length-1];
            let reason = lastNode.isExpanded ? "Expandida, mas o ramo provou ser sub-ótimo." : "Na Open List (f(n) maior que o caminho ótimo).";
            html += `<tr style="border-bottom: 1px solid var(--border);"><td style="padding: 8px; font-weight: bold;">Alt ${idx+1}</td><td style="padding: 8px; color: var(--text-mut);">${pathStr} ...</td><td style="padding: 8px;">${lastNode.f.toFixed(1)}</td><td style="padding: 8px;">${reason}</td></tr>`;
        });
        html += `</tbody></table>`;
    } else {
        html += `<p style="color: var(--text-mut);">A busca encontrou a solução rapidamente e não restaram ramos profundos o suficiente para exibir como alternativas completas.</p>`;
    }

    container.innerHTML = html;
  }
  
  renderPossibleSolutions();

  // click h5 by default
  document.getElementById('btn-hist-h5').click();
}

function renderHeuristicTableRows(tbodyId, rows){
  const tbody=document.getElementById(tbodyId);
  if(!tbody || !STATS.success)return;
  function fmt2(v){ return v.toFixed(2).replace('.', ','); }
  tbody.innerHTML=rows.map((s,i)=>`
    <tr style="border-bottom: 1px solid var(--border);">
      <td style="padding: 10px 8px;">${i}</td>
      <td style="padding: 10px 8px; font-family: 'JetBrains Mono';">(${s.node}, ${s.battery}%, {${s.collected}})</td>
      <td style="padding: 10px 8px;">${s.action}</td>
      <td style="padding: 10px 8px;">${fmt2(s.g)}</td>
      <td style="padding: 10px 8px;">${s.remaining}</td>
      <td style="padding: 10px 8px;">${fmt2(s.h_desloc)}</td>
      <td style="padding: 10px 8px;">${fmt2(s.h_coleta)}</td>
      <td style="padding: 10px 8px;">${fmt2(s.h_recarga)}</td>
      <td style="padding: 10px 8px; color: var(--node-r); font-weight: bold;">${fmt2(s.h)}</td>
      <td style="padding: 10px 8px; color: var(--node-c); font-weight: bold;">${fmt2(s.f)}</td>
    </tr>
  `).join('');
}
function renderHeuristicTable(){
  renderHeuristicTableRows('heuristic-tbody', HEURISTIC_TABLE);
  renderHeuristicTableRows('heuristic-h4-tbody', HEURISTIC_TABLE_H4);
}
renderStats();drawMap();renderPath();renderTree();renderHeuristicTable();
renderAnalysis();
window.addEventListener('resize',drawMap);
"""
