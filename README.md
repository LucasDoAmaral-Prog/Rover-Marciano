# Rover Marciano: Navegacao Autonoma com Busca A*

Projeto academico de Inteligencia Artificial para planejamento de rota de um rover em um ambiente marciano modelado como grafo. O sistema utiliza o algoritmo A* para encontrar uma rota ate um objetivo, respeitando restricoes de bateria, pontos de coleta e estacoes de recarga.

O projeto tambem gera uma visualizacao interativa em HTML e arvores de busca em SVG, permitindo analisar o comportamento do algoritmo, comparar heuristicas e observar o impacto entre otimalidade e reducao de expansoes.

## Sumario

- [Objetivo do projeto](#objetivo-do-projeto)
- [Como instalar](#como-instalar)
- [Como executar](#como-executar)
- [Como acessar os resultados](#como-acessar-os-resultados)
- [Modelagem do problema](#modelagem-do-problema)
- [Heuristicas utilizadas](#heuristicas-utilizadas)
- [Arquitetura do projeto](#arquitetura-do-projeto)
- [Resultados de referencia](#resultados-de-referencia)
- [Relatorio, video e apresentacao](#relatorio-video-e-apresentacao)
- [Competencias demonstradas](#competencias-demonstradas)

## Objetivo do projeto

O Rover Marciano tem como finalidade simular uma missao autonoma em que um agente precisa se deslocar por uma rede de pontos, coletar amostras e chegar a um objetivo final sem violar as restricoes de energia.

Em termos de IA, o problema e tratado como uma busca em espaco de estados, onde cada estado representa:

| Componente | Descricao |
|---|---|
| No atual | Localizacao do rover no grafo, como `C10`, `R6` ou `C4`. |
| Bateria | Energia restante, consumida conforme o tempo de deslocamento e coleta. |
| Coletas realizadas | Conjunto de pontos de coleta ja visitados e coletados. |

A solucao desejada e um caminho que minimize o tempo total da missao, considerando deslocamento, coleta e recarga.

## Como instalar

### 1. Requisitos

| Requisito | Finalidade | Observacao |
|---|---|---|
| Python 3.10+ | Executar o algoritmo e gerar as saidas | O projeto foi validado com Python 3.13.1. |
| Graphviz | Renderizar as arvores de busca em SVG | Necessario para as imagens geradas automaticamente. |
| `graphviz` Python package | Interface Python para gerar arquivos Graphviz | Instalado por `requirements.txt`. |

### 2. Baixar o projeto

Se estiver usando Git:

```bash
git clone <url-do-repositorio>
cd Rover
```

Se recebeu a pasta compactada, basta extrair o arquivo e abrir o terminal dentro da pasta `Rover`.

### 3. Criar e ativar ambiente virtual

No Windows:

```powershell
python -m venv .venv
.\.venv\Scripts\activate
```

No Linux ou macOS:

```bash
python -m venv .venv
source .venv/bin/activate
```

### 4. Instalar dependencias Python

```bash
python -m pip install --upgrade pip
pip install -r requirements.txt
```

### 5. Instalar Graphviz no sistema

O pacote Python `graphviz` cria os arquivos, mas o renderizador `dot` precisa estar instalado no sistema operacional.

| Sistema | Comando ou acao recomendada |
|---|---|
| Windows | Instalar o Graphviz e garantir que `C:\Program Files\Graphviz\bin` esteja no `PATH`. |
| Ubuntu/Debian | `sudo apt install graphviz` |
| macOS | `brew install graphviz` |

Para verificar:

```bash
dot -V
```

## Como executar

Execucao padrao:

```bash
python main.py
```

Execucao definindo origem e destino:

```bash
python main.py --start C10 --goal C4
```

Execucao com pasta de saida personalizada:

```bash
python main.py --start C10 --goal C4 --output-dir results
```

Execucao controlando a profundidade da arvore impressa no terminal:

```bash
python main.py --tree-depth 2
```

### Parametros disponiveis

| Parametro | Padrao | Descricao |
|---|---:|---|
| `--start` | `C10` | No inicial do rover. |
| `--goal` | `C4` | No objetivo final da missao. |
| `--output-dir` | `results` | Pasta onde serao salvos HTML, SVG e DOT. |
| `--tree-depth` | `3` | Profundidade maxima exibida no terminal para evitar poluicao visual. |

## Como acessar os resultados

Apos executar `python main.py`, os principais artefatos ficam em `results/`.

| Arquivo | Finalidade |
|---|---|
| `results/resultado_busca.html` | Relatorio visual interativo com mapa, caminho, metricas, comparacao de heuristicas e historico da busca. |
| `results/arvore_busca_focada_h5.svg` | Arvore focada da `h5`: caminho final + principais alternativas imediatas. |
| `results/arvore_busca_completa_h5.svg` | Arvore completa compactada da `h5`, limitada para apresentacao. |
| `results/arvore_busca_focada_h4.svg` | Arvore focada da `h4`: caminho final + principais alternativas imediatas. |
| `results/arvore_busca_completa_h4.svg` | Arvore completa compactada da `h4`, limitada para comparacao visual. |
| `results/*.dot` | Representacao textual Graphviz das arvores. |

Na arvore focada, azul indica o caminho escolhido, branco indica no gerado/Open
e cinza indica no expandido/Closed. Nas arvores compactadas, as caixas amarelas
indicam ramos omitidos por limite visual: estados que foram gerados durante a
execucao, mas nao foram desenhados para evitar uma figura excessivamente grande
e dificil de ler. Isso nao e poda do algoritmo; e apenas compactacao visual. A
execucao do A*, as listas Open/Closed e as metricas reais continuam considerando
esses estados normalmente.

As contagens de estados expandidos sao obtidas da Closed List real da execucao,
nao da quantidade de nos desenhados na arvore compactada.

Para visualizar o relatorio, abra o arquivo:

```text
results/resultado_busca.html
```

Para gerar um PDF, abra o HTML no navegador e use a opcao de impressao do navegador em "Salvar como PDF".

## Modelagem do problema

O ambiente e modelado como um grafo nao direcionado, composto por pontos de coleta (`C`) e pontos de recarga (`R`).

| Elemento | Representacao | Papel no problema |
|---|---|---|
| Pontos de coleta | `C1`, `C4`, `C10`, etc. | Locais onde o rover pode realizar coleta de amostras. |
| Pontos de recarga | `R1`, `R2`, `R6`, etc. | Locais onde o rover pode recuperar bateria. |
| Arestas | Tempo em minutos | Representam deslocamentos possiveis entre dois pontos. |
| Bateria | Percentual de 0 a 100 | Consumida durante deslocamento e coleta. |
| Custo `g(n)` | Tempo real acumulado | Soma de deslocamento, coleta e recarga efetivamente realizados. |
| Funcao `f(n)` | `f(n) = g(n) + h(n)` | Prioridade usada pelo A* para escolher o proximo estado. |

### Restricoes da missao

| Restricao | Valor usado | Justificativa |
|---|---:|---|
| Bateria inicial | `100%` | O rover parte com carga completa. |
| Bateria minima | `1%` | O rover nao pode chegar a um estado com bateria nula. |
| Coletas minimas | `3` | A missao so e valida apos coletar amostras suficientes. |
| Tempo por coleta | `2 min` | Cada coleta possui custo operacional fixo. |
| Consumo de bateria | `1% por minuto` | O custo temporal tambem representa consumo energetico. |
| Taxa de recarga | `5% por minuto` | Define o tempo necessario para recuperar energia. |
| Escala espacial | `1 unidade = 0.7 km` | Converte coordenadas do grafo em distancia fisica. |
| Velocidade maxima | `0.1 km/min` | Usada para estimar o tempo minimo de deslocamento. |

## Heuristicas utilizadas

O projeto compara duas heuristicas principais: uma admissivel (`h5`) e uma inflada nao admissivel (`h4`). Essa comparacao e importante porque mostra, na pratica, o equilibrio entre garantia de otimalidade e reducao do numero de estados expandidos.

### Heuristica h5: admissivel

A heuristica principal e:

```text
h5(s) = T_min_desloc(s) + T_min_coleta(s) + T_min_recarga(s)
```

| Termo | Formula | Interpretacao |
|---|---|---|
| `T_min_desloc(s)` | `distancia_em_linha_reta(p, G) / velocidade_maxima` | Menor tempo teorico para ir da posicao atual ao objetivo. |
| `T_min_coleta(s)` | `tempo_coleta * coletas_restantes` | Tempo minimo para cumprir as coletas que ainda faltam. |
| `T_min_recarga(s)` | `max(0, consumo_minimo + bateria_minima - bateria_atual) / taxa_recarga` | Recarga minima estimada para nao violar a restricao de bateria. |

A `h5` e considerada admissivel porque utiliza estimativas otimistas:

| Motivo | Por que nao superestima |
|---|---|
| Distancia em linha reta | E sempre menor ou igual ao caminho real no grafo. |
| Coleta minima | Considera apenas a quantidade estritamente necessaria de coletas. |
| Recarga minima | Estima somente a energia indispensavel para manter a bateria acima do minimo. |

Por isso, quando usada com A*, a `h5` preserva a garantia de encontrar uma solucao otima, desde que os custos sejam nao negativos.

### Heuristica h4: inflada e nao admissivel

A heuristica `h4` usa a propria `h5` multiplicada por um fator `alpha`:

```text
h4(s) = alpha * h5(s)
```

No projeto, o valor usado e:

```text
alpha = 5
```

| Caracteristica | Consequencia |
|---|---|
| Estimativa mais agressiva | A busca tende a expandir menos estados. |
| Possivel superestimativa | A heuristica deixa de ser admissivel. |
| Menor custo computacional em alguns casos | Pode encontrar respostas rapidamente. |
| Sem garantia de otimalidade | Pode escolher um caminho mais caro que o otimo. |

Essa heuristica e util como experimento comparativo: ela mostra como uma busca mais direcionada pode reduzir expansoes, mas tambem pode sacrificar a qualidade da solucao.

## Arquitetura do projeto

| Arquivo | Responsabilidade |
|---|---|
| `main.py` | Ponto de entrada do projeto, leitura de parametros, execucao das heuristicas e geracao das saidas. |
| `busca.py` | Implementacao do algoritmo A*, reconstrucao do caminho, controle de listas aberta/fechada e arvore de busca. |
| `grafo.py` | Definicao dos nos, coordenadas, arestas e classificacao entre coleta e recarga. |
| `estado.py` | Modelo imutavel do estado do rover: no atual, bateria e coletas realizadas. |
| `heuristica.py` | Calculo das distancias, tempos minimos e heuristicas `h5` e `h4`. |
| `config.py` | Parametros globais da missao, como bateria, coleta, recarga, escala e velocidade. |
| `visualizacao.py` | Geracao de arvores Graphviz e relatorio HTML interativo. |
| `requirements.txt` | Dependencias Python do projeto. |
| `results/` | Pasta de saida com relatorio, imagens e arquivos DOT. |

## Resultados de referencia

Execucao validada com:

```bash
python main.py --tree-depth 1
```

Configuracao padrao:

| Item | Valor |
|---|---:|
| Origem | `C10` |
| Objetivo | `C4` |
| Bateria inicial | `100%` |
| Coletas minimas | `3` |
| Total de nos | `21` |

Comparacao entre heuristicas:

| Heuristica | Tipo | Tempo total | Bateria final | Coletas | Estados expandidos | Nos gerados |
|---|---|---:|---:|---:|---:|---:|
| `h5` | Admissivel | `68.00 min` | `32.0%` | `3` | `14` | `49` |
| `h4` | Inflada, nao admissivel | `72.00 min` | `28.0%` | `3` | `9` | `27` |

Interpretacao:

| Observacao | Analise |
|---|---|
| `h5` encontrou custo menor | A heuristica admissivel preservou a otimalidade do A*. |
| `h4` expandiu menos estados | A heuristica inflada tornou a busca mais direcionada. |
| `h4` retornou custo maior | A perda de admissibilidade reduziu a garantia de qualidade da solucao. |

Caminho otimo encontrado pela `h5`:

| Passo | Estado | Acao | `g(n)` |
|---:|---|---|---:|
| 0 | `(C10, 98%, {C10})` | Inicio | `2.00` |
| 1 | `(R6, 84.0%, {C10})` | Mover `C10 -> R6` | `16.00` |
| 2 | `(C6, 69.0%, {C10,C6})` | Mover `R6 -> C6` e coletar em `C6` | `31.00` |
| 3 | `(R2, 51.0%, {C10,C6})` | Mover `C6 -> R2` | `49.00` |
| 4 | `(C4, 32.0%, {C10,C4,C6})` | Mover `R2 -> C4` e coletar em `C4` | `68.00` |

## Relatorio, video e apresentacao

O projeto ja gera um relatorio visual em HTML, que pode ser usado como material de apresentacao:

```text
results/resultado_busca.html
```

## Competencias demonstradas

| Area | Evidencia no projeto |
|---|---|
| Inteligencia Artificial | Implementacao de busca A* com funcao de avaliacao `f(n) = g(n) + h(n)`. |
| Heuristicas | Projeto e comparacao entre heuristica admissivel e heuristica inflada. |
| Modelagem de estados | Representacao formal de localizacao, bateria e coletas. |
| Otimizacao | Busca por menor tempo total respeitando restricoes operacionais. |
| Analise experimental | Comparacao de custos, estados expandidos e comportamento da arvore de busca. |
| Visualizacao de dados | Relatorio HTML interativo e geracao de arvores Graphviz. |
| Engenharia de software | Separacao clara entre grafo, estado, busca, heuristica, configuracao e visualizacao. |

## Possiveis melhorias futuras

| Melhoria | Beneficio |
|---|---|
| Interface grafica para alterar origem/destino | Facilita experimentos sem uso do terminal. |
| Exportacao automatica para PDF | Torna a entrega academica ainda mais direta. |
| Novas heuristicas | Permite comparar mais estrategias de busca. |
| Testes automatizados | Aumenta a confiabilidade de alteracoes futuras. |
| Persistencia de cenarios | Permite salvar diferentes configuracoes de missao. |

## Solucao de problemas

| Problema | Causa provavel | Como resolver |
|---|---|---|
| `ModuleNotFoundError: graphviz` | Dependencias Python nao instaladas | Executar `pip install -r requirements.txt`. |
| Erro relacionado ao comando `dot` | Graphviz do sistema nao instalado ou fora do `PATH` | Instalar Graphviz e verificar com `dot -V`. |
| HTML gerado sem imagens | Renderizacao Graphviz falhou | Conferir instalacao do Graphviz e reexecutar `python main.py`. |
| Origem ou destino invalido | No nao existe no grafo | Usar identificadores presentes em `grafo.py`, como `C10`, `C4`, `R6`. |

## Licenca

Este projeto foi desenvolvido para fins academicos. Caso seja publicado em um repositorio, recomenda-se adicionar uma licenca formal, como MIT, Apache-2.0 ou GPL, conforme o objetivo de uso e distribuicao.
