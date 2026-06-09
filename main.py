import argparse
import os

from config import (
    MAX_BATTERY,
    MIN_BATTERY,
    MIN_COLLECTIONS,
)
from busca import AStarSearch
from grafo import Graph
from functools import partial
from heuristica import h5, h4
from visualizacao import (
    print_search_tree,
    generate_focused_tree_image,
    generate_full_tree_image,
    generate_html,
)


# argparse le os parametros passados no terminal, como --start e --goal.
# os ajuda a criar a pasta de resultados antes de salvar HTML e imagens.
# partial fixa o alpha da h4 sem precisar criar uma funcao nova so para isso.


def build_parser():
    parser = argparse.ArgumentParser(
        description="Busca A* para navegacao autonoma de um rover marciano."
    )
    parser.add_argument(
        "--start",
        default="C10",
        help="No inicial do rover. Exemplo: C1, R2, C9.",
    )
    parser.add_argument(
        "--goal",
        default="C4",
        help="No objetivo G. Exemplo: C3, C14, R7.",
    )
    parser.add_argument(
        "--output-dir",
        default="results",
        help="Pasta de saida para a imagem da arvore. Padrao: results.",
    )
    parser.add_argument(
        "--tree-depth",
        type=int,
        default=3,
        help="Profundidade maxima da arvore no console. Padrao: 3. Para sem limite passe um valor alto.",
    )
    parser.add_argument(
        "--alpha",
        type=float,
        default=5.0,
        help="Fator de inflacao da heuristica h4. Use 1.5 para reproduzir o caso C10 -> C4 antigo.",
    )
    return parser


def print_separator():
    print("=" * 60)


def print_problem_configuration(graph, start_node, goal_node, h4_alpha):
    print_separator()
    print("  CONFIGURACAO DA BUSCA A*")
    print_separator()
    print(f"  Origem:           {start_node}")
    print(f"  Destino (G):      {goal_node}")
    print(f"  Bateria inicial:  {MAX_BATTERY}%")
    print(f"  Bateria minima:   {MIN_BATTERY}%")
    print(f"  Coletas minimas:  {MIN_COLLECTIONS}")
    print(f"  Alpha da h4:      {h4_alpha:g}")
    print(f"  Total de nos:     {len(graph.adjacency_list)}")
    print_separator()
    print()


def print_result(result, graph, goal_node, heuristic_func=h5, heuristic_label="h5", path_title="CAMINHO FINAL"):
    if not result.success:
        print_separator()
        print("  RESULTADO: NENHUMA SOLUCAO ENCONTRADA")
        print_separator()
        print(f"  Estados expandidos:  {result.expanded_states}")
        print(f"  Lista fechada:       {result.closed_list_size}")
        print()
        return

    print_separator()
    print("  SOLUCAO ENCONTRADA!")
    print_separator()
    print(f"  Tempo total g(n):        {result.total_time:.2f} min")
    print(f"  Bateria final:           {result.final_state.battery_level}%")
    print(f"  Coletas realizadas:      {len(result.final_state.collected_points)}")
    print(f"  Estados expandidos:      {result.expanded_states}")
    print(f"  Tam. lista aberta:       {result.open_list_size}")
    print(f"  Tam. lista fechada:      {result.closed_list_size}")
    print(f"  Nos gerados na busca:    {len(result.search_tree)}")
    print_separator()
    print()

    print(f"  {path_title}:")
    print("-" * 60)

    for step_index, step in enumerate(result.path):
        state = step["state"]
        heuristic_value = heuristic_func(graph, state, goal_node)
        f_value = step["g"] + heuristic_value

        print(f"  Passo {step_index}: {state}")
        print(f"    Acao:  {step['action']}")
        print(
            f"    g(n)={step['g']:.2f}  "
            f"{heuristic_label}(n)={heuristic_value:.2f}  "
            f"f(n)={f_value:.2f}"
        )

        if step_index < len(result.path) - 1:
            print(f"    {'|':>6}")

    print("-" * 60)
    print()


def print_tree_section(result, max_depth):
    print_separator()
    print("  ARVORE DE BUSCA (*** = caminho solucao)")
    if max_depth is not None:
        print(f"  (Mostrando apenas profundidade maxima = {max_depth} para nao poluir o terminal)")
        print("  (Ramos omitidos sao limite visual: estados gerados, mas nao desenhados aqui.)")
        print("  (Isso nao e poda do A*: Open/Closed e metricas reais nao mudam.)")
        print("  (Acesse as imagens SVG ou o HTML gerado para ver a arvore compactada.)")
    print_separator()
    print_search_tree(result, max_depth=max_depth)
    print_separator()
    print()


def main():
    parser = build_parser()
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    graph = Graph()
    search = AStarSearch(graph)

    start_node = args.start.upper()
    goal_node = args.goal.upper()

    print()
    print_problem_configuration(graph, start_node, goal_node, args.alpha)

    print("=== EXECUTANDO COM HEURISTICA ADMISSIVEL (h5) ===")
    result_h5 = search.search(start_node, goal_node, heuristic_func=h5)
    print_result(
        result_h5,
        graph,
        goal_node,
        heuristic_func=h5,
        heuristic_label="h5",
        path_title="CAMINHO OTIMO (h5)",
    )
    
    print("=== EXECUTANDO COM HEURISTICA INFLADA NAO ADMISSIVEL (h4) ===")
    h4_func = partial(h4, alpha=args.alpha)
    result_h4 = search.search(start_node, goal_node, heuristic_func=h4_func)
    print_result(
        result_h4,
        graph,
        goal_node,
        heuristic_func=h4_func,
        heuristic_label="h4",
        path_title="CAMINHO FINAL (h4, sem garantia de otimalidade)",
    )
    
    # Gerar a arvore com o result_h5 para manter compatibilidade com o log no terminal
    print_tree_section(result_h5, max_depth=args.tree_depth)

    # Gera as imagens da arvore usando o resultado da h5.
    img_focada = generate_focused_tree_image(
        graph, result_h5, start_node, goal_node, output_dir=args.output_dir,
    )
    if img_focada:
        print(f"  Imagem da arvore focada (h5): {img_focada}")

    img_completa = generate_full_tree_image(
        graph, result_h5, start_node, goal_node, output_dir=args.output_dir,
    )
    if img_completa:
        print(f"  Imagem da arvore completa compactada (h5): {img_completa}")

    html_out = os.path.join(args.output_dir, "resultado_busca.html")
    html_path = generate_html(
        graph, result_h5, result_h4, start_node, goal_node,
        output_file=html_out,
        h4_alpha=args.alpha,
    )
    print(f"  Visualizacao HTML: {html_path}")
    print()


if __name__ == "__main__":
    main()
