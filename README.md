# Practica Inteligencia Artificial 2

## Agentes Deliberativos: Los Extraños mundos de Belkan 

### Nivel 0: Búsqueda de Profundidad
- Implementación del algoritmo de búsqueda de profundidad.
- Exploración recursiva de nodos en un camino específico.
- Retroceso cuando no hay más nodos por visitar en un camino.

### Nivel 1: Búsqueda de Anchura
- Aplicación del algoritmo de búsqueda de anchura para calcular el número mínimo de acciones para alcanzar un objetivo.
- Utilización de una cola para recorrer los nodos nivel por nivel.
- Empleo del método top en la cola en lugar de front.

### Nivel 2: Algoritmos CU y A*
- Implementación de los algoritmos de Costo Uniforme (CU) y A*.
- Cálculo del costo de un nodo para determinar la ruta óptima.
- Adaptación del movimiento según la presencia de zapatillas o bikini en el nodo.

### Nivel 3: Comportamiento Reactivo y Deliberativo
- Desarrollo de funciones para dirigir al agente hacia casillas de interés detectadas por los sensores.
- Implementación de una función de emergencia para habilitar los sensores en situaciones críticas.
- Comportamiento reactiva al explorar aleatoriamente y deliberativo al encontrar una casilla de interés.

### Nivel 4: Determinación de Posición y Recálculo de Rutas
- Creación de una función para calcular la posición del agente cuando los sensores fallan.
- Planificación y recálculo de rutas para alcanzar objetivos, adaptándose a obstáculos en el camino.
- Restablecimiento de valores y determinación de ubicación en caso de colisión
