import math

from config import (
    COLLECTION_TIME,
    MAX_SPEED,
    MIN_BATTERY,
    MIN_COLLECTIONS,
    PLANE_UNIT_IN_KM,
    RECHARGE_RATE,
)


def euclidean_distance_on_plane(graph, node_a, node_b):
    x_a, y_a = graph.positions[node_a]
    x_b, y_b = graph.positions[node_b]

    return math.sqrt((x_b - x_a) ** 2 + (y_b - y_a) ** 2)


def distance_in_km(graph, node_a, node_b):
    plane_distance = euclidean_distance_on_plane(graph, node_a, node_b)
    return PLANE_UNIT_IN_KM * plane_distance


def min_displacement_time(graph, state, goal_node):
    # T_min_desloc(s): tempo em linha reta da posicao atual p(s) ate o objetivo G.
    # A distancia vem das coordenadas DOT, convertida pela escala 1 unidade = 0.7 km.
    km_distance = distance_in_km(graph, state.current_node, goal_node)
    return km_distance / MAX_SPEED


def remaining_collections(state):
    return max(0, MIN_COLLECTIONS - len(state.collected_points))


def min_collection_time(state):
    # T_min_coleta(s): cada coleta restante custa no minimo 2 minutos.
    return COLLECTION_TIME * remaining_collections(state)


def min_recharge_time(graph, state, goal_node):
    # Como o rover consome 1% por minuto de atividade, o tempo minimo restante
    # tambem funciona como uma estimativa minima de consumo de bateria.
    min_consumption = (
        min_displacement_time(graph, state, goal_node)
        + min_collection_time(state)
    )

    missing_battery = max(
        0,
        min_consumption + MIN_BATTERY - state.battery_level,
    )

    return missing_battery / RECHARGE_RATE


def h5(graph, state, goal_node):
    return (
        min_displacement_time(graph, state, goal_node)
        + min_collection_time(state)
        + min_recharge_time(graph, state, goal_node)
    )

def h4(graph, state, goal_node, alpha=1.5):
    if alpha <= 1:
        raise ValueError("alpha deve ser maior que 1.")
    return alpha * h5(graph, state, goal_node)