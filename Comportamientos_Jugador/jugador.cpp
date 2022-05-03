#include "../Comportamientos_Jugador/jugador.hpp"
#include "motorlib/util.h"

#include <cmath>
#include <iostream>
#include <queue>
#include <set>
#include <stack>

// Este es el método principal que se piden en la practica.
// Tiene como entrada la información de los sensores y devuelve la acción a
// realizar. Para ver los distintos sensores mirar fichero "comportamiento.hpp"
Action ComportamientoJugador::think(Sensores sensores) {
  // Actualizo la variable accion y actual
  Action accion = actIDLE;

  actual.fila = sensores.posF; // fila
  actual.columna = sensores.posC; // columna 
  actual.orientacion = sensores.sentido; // brujula

  

  cout << "Fila: " << actual.fila << endl;
  cout << "Col : " << actual.columna << endl;
  cout << "Ori : " << actual.orientacion << endl;
  cout << "Tiene Zapatillas: " << actual.TieneZapatillas << endl;
  cout << "Tiene Bikini: " << actual.TieneBikini << endl;
  cout << "Tiempo Recarga: " << tiempo_recarga << endl;
  cout << "Recarga: " << actual.recarga << endl;

  // Capturo los destinos
  cout << "sensores.num_destinos : " << sensores.num_destinos << endl;
  objetivos.clear();
  for (int i = 0; i < sensores.num_destinos; i++) {
    estado aux;
    aux.fila = sensores.destino[2 * i];
    aux.columna = sensores.destino[2 * i + 1];
    objetivos.push_back(aux);
  }

  // nivel 3 donde hago hibrido reactivo y deliberativo

  /*
	
  */
  if (sensores.nivel == 3) {
    
    //ActualizarMapa(sensores);
	/*
	int ejemplo=0;

	ejemplo = (rand() % 4 );
	cout << " " << ejemplo << endl;
	*/
	
	if (comienzo){

		ActualizarMapa(sensores);

		if (actual.recarga){
			accion = actIDLE;
    		actual = Recargar(sensores,actual);
		}

		else if (SensoresAvanzar(sensores, actual)){
			accion = actFORWARD; 
			// accion = moverAleatorio;
		} else {
			accion = Girar(sensores);
		}
	}

	








  } else {
    if (!hayPlan)
      hayPlan = pathFinding(sensores.nivel, actual, objetivos, plan);

    if (hayPlan and plan.size() > 0) { // hay un plan no vacio
      accion = plan.front();           // tomo la siguiente accion del Plan
      plan.erase(plan.begin());        // eliminamos la accion del plan
    } else {
      cout << "no se pudo encontrar un plan\n" << endl;
    }
  }
  //

  // actualizo tmb bikini y zapas solo puedo tener uno
  actual = SensorCasilla(sensores, actual);
  comienzo = true;
  return accion;
}

// Llama al algoritmo de busqueda que se usara en cada comportamiento del agente
// Level representa el comportamiento en el que fue iniciado el agente.
bool ComportamientoJugador::pathFinding(int level, const estado &origen,
                                        const list<estado> &destino,
                                        list<Action> &plan) {
  estado un_objetivo;
  switch (level) {
  case 0:
    cout << "Demo\n";
    un_objetivo = objetivos.front();
    cout << "fila: " << un_objetivo.fila << " col:" << un_objetivo.columna
         << endl;
    return pathFinding_Profundidad(origen, un_objetivo, plan);
    break;

  case 1:
    cout << "Optimo numero de acciones\n";
    un_objetivo = objetivos.front();
    cout << "fila: " << un_objetivo.fila << " col:" << un_objetivo.columna
         << endl;
    return pathFinding_Anchura(origen, un_objetivo, plan);

    break;
  case 2:
    cout << "Optimo en coste Costo Uniforme\n";
    un_objetivo = objetivos.front();
    cout << "fila: " << un_objetivo.fila << " col:" << un_objetivo.columna
         << endl;
    return pathFinding_CostoUniforme(origen, un_objetivo, plan);

    break;
  case 3:
    cout << "Reto 1: Descubrir el mapa\n";

    break;
  case 4:
    cout << "Reto 2: Maximizar objetivos\n";
    // Incluir aqui la llamada al algoritmo de busqueda para el Reto 2
    cout << "No implementado aun\n";
    break;
  }
  return false;
}

//---------------------- Implementación de la busqueda en profundidad
//---------------------------

// Dado el codigo en caracter de una casilla del mapa dice si se puede
// pasar por ella sin riegos de morir o chocar.
bool EsObstaculo(unsigned char casilla) {
  if (casilla == 'P' or casilla == 'M')
    return true;
  else
    return false;
}

// Comprueba si la casilla que hay delante es un obstaculo. Si es un
// obstaculo devuelve true. Si no es un obstaculo, devuelve false y
// modifica st con la posición de la casilla del avance.
bool ComportamientoJugador::HayObstaculoDelante(estado &st) {
  int fil = st.fila, col = st.columna;

  // calculo cual es la casilla de delante del agente
  switch (st.orientacion) {
  case 0:
    fil--;
    break;
  case 1:
    fil--;
    col++;
    break;
  case 2:
    col++;
    break;
  case 3:
    fil++;
    col++;
    break;
  case 4:
    fil++;
    break;
  case 5:
    fil++;
    col--;
    break;
  case 6:
    col--;
    break;
  case 7:
    fil--;
    col--;
    break;
  }

  // Compruebo que no me salgo fuera del rango del mapa
  if (fil < 0 or fil >= mapaResultado.size())
    return true;
  if (col < 0 or col >= mapaResultado[0].size())
    return true;

  // Miro si en esa casilla hay un obstaculo infranqueable
  if (!EsObstaculo(mapaResultado[fil][col])) {
    // No hay obstaculo, actualizo el parametro st poniendo la casilla de
    // delante.
    st.fila = fil;
    st.columna = col;
    return false;
  } else {
    return true;
  }
}

struct nodo {
  estado st;
  list<Action> secuencia;
};

struct ComparaEstados {
  bool operator()(const estado &a, const estado &n) const {
    if ((a.fila > n.fila) or (a.fila == n.fila and a.columna > n.columna) or
        (a.fila == n.fila and a.columna == n.columna and
         a.orientacion > n.orientacion))
      return true;
    else
      return false;
  }
};

// Sacar por la consola la secuencia del plan obtenido
void ComportamientoJugador::PintaPlan(list<Action> plan) {
  auto it = plan.begin();
  while (it != plan.end()) {
    if (*it == actFORWARD) {
      cout << "A ";
    } else if (*it == actTURN_R) {
      cout << "D ";
    } else if (*it == actSEMITURN_R) {
      cout << "d ";
    } else if (*it == actTURN_L) {
      cout << "I ";
    } else if (*it == actSEMITURN_L) {
      cout << "i ";
    } else {
      cout << "- ";
    }
    it++;
  }
  cout << endl;
}

// Funcion auxiliar para poner a 0 todas las casillas de una matriz
void AnularMatriz(vector<vector<unsigned char>> &m) {
  for (int i = 0; i < m[0].size(); i++) {
    for (int j = 0; j < m.size(); j++) {
      m[i][j] = 0;
    }
  }
}

// Pinta sobre el mapa del juego el plan obtenido
void ComportamientoJugador::VisualizaPlan(const estado &st,
                                          const list<Action> &plan) {
  AnularMatriz(mapaConPlan);
  estado cst = st;

  auto it = plan.begin();
  while (it != plan.end()) {
    if (*it == actFORWARD) {
      switch (cst.orientacion) {
      case 0:
        cst.fila--;
        break;
      case 1:
        cst.fila--;
        cst.columna++;
        break;
      case 2:
        cst.columna++;
        break;
      case 3:
        cst.fila++;
        cst.columna++;
        break;
      case 4:
        cst.fila++;
        break;
      case 5:
        cst.fila++;
        cst.columna--;
        break;
      case 6:
        cst.columna--;
        break;
      case 7:
        cst.fila--;
        cst.columna--;
        break;
      }
      mapaConPlan[cst.fila][cst.columna] = 1;
    } else if (*it == actTURN_R) {
      cst.orientacion = (cst.orientacion + 2) % 8;
    } else if (*it == actSEMITURN_R) {
      cst.orientacion = (cst.orientacion + 1) % 8;
    } else if (*it == actTURN_L) {
      cst.orientacion = (cst.orientacion + 6) % 8;
    } else if (*it == actSEMITURN_L) {
      cst.orientacion = (cst.orientacion + 7) % 8;
    }
    it++;
  }
}

//------------------------------------------------------------------------
//						NIVEL 0: BUSQUEDA EN PROFUNDIDAD
//------------------------------------------------------------------------

// Implementación de la busqueda en profundidad.
// Entran los puntos origen y destino y devuelve la
// secuencia de acciones en plan, una lista de acciones.
bool ComportamientoJugador::pathFinding_Profundidad(const estado &origen,
                                                    const estado &destino,
                                                    list<Action> &plan) {
  // Borro la lista
  cout << "Calculando plan\n";
  plan.clear();
  set<estado, ComparaEstados> Cerrados; // Lista de Cerrados
  stack<nodo> Abiertos;                 // Lista de Abiertos

  nodo current;
  current.st = origen;
  current.secuencia.empty();

  Abiertos.push(current);

  while (!Abiertos.empty() and (current.st.fila != destino.fila or
                                current.st.columna != destino.columna)) {

    Abiertos.pop();
    Cerrados.insert(current.st);

    // Generar descendiente de girar a la derecha 90 grados
    nodo hijoTurnR = current;
    hijoTurnR.st.orientacion = (hijoTurnR.st.orientacion + 2) % 8;
    if (Cerrados.find(hijoTurnR.st) == Cerrados.end()) {
      hijoTurnR.secuencia.push_back(actTURN_R);
      Abiertos.push(hijoTurnR);
    }

    // Generar descendiente de girar a la derecha 45 grados
    nodo hijoSEMITurnR = current;
    hijoSEMITurnR.st.orientacion = (hijoSEMITurnR.st.orientacion + 1) % 8;
    if (Cerrados.find(hijoSEMITurnR.st) == Cerrados.end()) {
      hijoSEMITurnR.secuencia.push_back(actSEMITURN_R);
      Abiertos.push(hijoSEMITurnR);
    }

    // Generar descendiente de girar a la izquierda 90 grados
    nodo hijoTurnL = current;
    hijoTurnL.st.orientacion = (hijoTurnL.st.orientacion + 6) % 8;
    if (Cerrados.find(hijoTurnL.st) == Cerrados.end()) {
      hijoTurnL.secuencia.push_back(actTURN_L);
      Abiertos.push(hijoTurnL);
    }

    // Generar descendiente de girar a la izquierda 45 grados
    nodo hijoSEMITurnL = current;
    hijoSEMITurnL.st.orientacion = (hijoSEMITurnL.st.orientacion + 7) % 8;
    if (Cerrados.find(hijoSEMITurnL.st) == Cerrados.end()) {
      hijoSEMITurnL.secuencia.push_back(actSEMITURN_L);
      Abiertos.push(hijoSEMITurnL);
    }

    // Generar descendiente de avanzar
    nodo hijoForward = current;
    if (!HayObstaculoDelante(hijoForward.st)) {
      if (Cerrados.find(hijoForward.st) == Cerrados.end()) {
        hijoForward.secuencia.push_back(actFORWARD);
        Abiertos.push(hijoForward);
      }
    }

    // Tomo el siguiente valor de la Abiertos
    if (!Abiertos.empty()) {
      current = Abiertos.top();
    }
  }

  cout << "Terminada la busqueda\n";

  if (current.st.fila == destino.fila and
      current.st.columna == destino.columna) {
    cout << "Cargando el plan\n";
    plan = current.secuencia;
    cout << "Longitud del plan: " << plan.size() << endl;
    PintaPlan(plan);
    // ver el plan en el mapa
    VisualizaPlan(origen, plan);
    return true;
  } else {
    cout << "No encontrado plan\n";
  }

  return false;
}

//------------------------------------------------------------------------
//						NIVEL 1: BUSQUEDA EN ANCHURA
//------------------------------------------------------------------------

/*
        Para conseguir el minimo numero de acciones basta con implementar la
   busqueda en anchura Entran los puntos origen y destino y devuelve la
   secuancia de ciiones en plan, una lista de acciones

        nodo - estado
                 - secuencia (lista acciones)
*/

bool ComportamientoJugador::pathFinding_Anchura(const estado &origen,
                                                const estado &destino,
                                                list<Action> &plan) {

  // busqueda en grafo
  // lista abiertos - para almacenar nodos sucesores que se han generado en
  // etapas anteriores lista cerrado - almaceno los nodos que hayan sido
  // abiertos

  // Primero meto el nodo inicial en la lista de abiertos, luego meto sus hijos
  // si uno es solucion se acaba, en otro caso el nodo revisado se elimina de la
  // cola de abiertos y pasa a Cerrados

  // Borro la lista , borro mi plan
  cout << "Calculando plan\n";
  plan.clear();
  set<estado, ComparaEstados> Cerrados; // Lista de Cerrados
  queue<nodo> Abiertos; // Lista de Abiertos FIFO , stack es lifo

  nodo current;
  current.st = origen;
  current.secuencia.empty();

  Abiertos.push(current);

  while (!Abiertos.empty() and
         (current.st.fila != destino.fila or
          current.st.columna !=
              destino.columna)) // mientras abiertos no sea fin y no se haya
                                // llegado al destino creamos el arbol
  {
    Abiertos.pop();
    Cerrados.insert(current.st);

    // para cada accion generamos un hijo

    // genera hijo giro a la derecha 90
    nodo hijoTurnR = current;
    hijoTurnR.st.orientacion =
        (hijoTurnR.st.orientacion + 2) % 8; // giro der 90
    if (Cerrados.find(hijoTurnR.st) == Cerrados.end()) {
      hijoTurnR.secuencia.push_back(actTURN_R);
      Abiertos.push(hijoTurnR);
    }

    // genera hijo giro a la izquierda 90
    nodo hijoTurnL = current;
    hijoTurnL.st.orientacion = (hijoTurnL.st.orientacion + 6) % 8;
    if (Cerrados.find(hijoTurnL.st) == Cerrados.end()) {
      hijoTurnL.secuencia.push_back(actTURN_L);
      Abiertos.push(hijoTurnL);
    }

    // genera gijo giro a la derecha 45
    nodo hijoSEMITurnR = current;
    hijoSEMITurnR.st.orientacion = (hijoSEMITurnR.st.orientacion + 1) % 8;
    if (Cerrados.find(hijoSEMITurnR.st) == Cerrados.end()) {
      hijoSEMITurnR.secuencia.push_back(actSEMITURN_R);
      Abiertos.push(hijoSEMITurnR);
    }

    // genera hijo giro a la izquierda 45
    nodo hijoSEMITurnL = current;
    hijoSEMITurnL.st.orientacion = (hijoSEMITurnL.st.orientacion + 7) % 8;
    if (Cerrados.find(hijoSEMITurnL.st) == Cerrados.end()) {
      hijoSEMITurnL.secuencia.push_back(actSEMITURN_L);
      Abiertos.push(hijoSEMITurnL);
    }

    // genero hijo para avanzar
    nodo hijoForward = current;
    if (!HayObstaculoDelante(hijoForward.st)) {
      if (Cerrados.find(hijoForward.st) == Cerrados.end()) {
        hijoForward.secuencia.push_back(actFORWARD);
        Abiertos.push(hijoForward);
      }
    }

    // tomo el siguiente valor de Abiertos
    if (!Abiertos.empty())
      current = Abiertos.front();
  }

  // cargamos el plan

  cout << "Terminada la busqueda\n";

  if (current.st.fila == destino.fila and
      current.st.columna == destino.columna) {
    cout << "Cargando el plan\n";
    plan = current.secuencia;
    cout << "Longitud del plan: " << plan.size() << endl;
    PintaPlan(plan);
    // ver el plan en el mapa
    VisualizaPlan(origen, plan);
    return true;
  } else {
    cout << "No encontrado plan\n";
  }

  return false;
}

//------------------------------------------------------------------------
//						NIVEL 2: COSTO UNIFORME O A*
//------------------------------------------------------------------------

// menor consumo de bateria en acciones
// estructura de nodos especializada
// para la priority queue necesito crear sus operandos

/*
        - Definir sensor casilla
        - nodoCoste
        - CalculoCoste
        - CostoUniforme
*/

estado ComportamientoJugador::SensorCasilla(Sensores sensores, estado aux) {

  if (sensores.terreno[2] == 'K') {
    aux.TieneBikini = true;
    aux.TieneZapatillas = false;
  }

  if (sensores.terreno[2] == 'D') {
    aux.TieneZapatillas = true;
    aux.TieneBikini = false;
  }

  if (sensores.terreno[2] == 'X' ) {
    aux.recarga = true;
  }

  return aux;
}

struct nodoCoste {
  estado st;
  list<Action> secuencia;
  unsigned int coste;
};

bool operator<(const nodoCoste &a, const nodoCoste &n) {
  return a.coste > n.coste;
}

struct ComparaEstadosCostes {
  bool operator()(const estado &a, const estado &n) const {
    if ((a.fila > n.fila) or (a.fila == n.fila and a.columna > n.columna) or
        (a.fila == n.fila and a.columna == n.columna and
         a.orientacion > n.orientacion) or
        (a.fila == n.fila and a.columna == n.columna and
         a.orientacion == n.orientacion and
         a.TieneZapatillas > n.TieneZapatillas) or
        (a.fila == n.fila and a.columna == n.columna and
         a.orientacion == n.orientacion and
         a.TieneZapatillas == n.TieneZapatillas and
         a.TieneBikini > n.TieneBikini))
      return true;
    else
      return false;
  }
};

// calculo el coste de una accion en un nodo
unsigned int ComportamientoJugador::CalculoCoste(int fil, int col,
                                                 Action accion,
                                                 bool TieneBikini,
                                                 bool TieneZapatillas) {
  unsigned int coste = 0;
  // char casilla = sensores.terreno[0]; // necesito saber que casilla estoy,
  // ahora la comparo y veo su coste
  char casilla = mapaResultado[fil][col];

  if (accion != actIDLE) { // si no es IDLE
    coste = 1;

    // mirar la tabla de costes de bateria
    if (casilla == 'A') {

      if (accion == actFORWARD) {
        coste = 200;
        if (TieneBikini)
          coste = 10;
      } else if (accion == actTURN_L or accion == actTURN_R) {
        coste = 500;
        if (TieneBikini)
          coste = 5;
      } else {
        coste = 300;
        if (TieneBikini)
          coste = 2;
      }

    } else if (casilla == 'B') {

      if (accion == actFORWARD) {
        coste = 100;
        if (TieneZapatillas)
          coste = 15;
      } else if (accion == actTURN_L or accion == actTURN_R) {
        coste = 3;
        if (TieneZapatillas)
          coste = 1;
      } else {
        coste = 2;
        if (TieneZapatillas)
          coste = 1;
      }

    } else if (casilla == 'T') {

      if (accion == actFORWARD or accion == actTURN_L or accion == actTURN_R) {
        coste = 2;

      } else {
        coste = 1;
      }
    }
  }
  return coste;
}

bool ComportamientoJugador::pathFinding_CostoUniforme(const estado &origen,
                                                      const estado &destino,
                                                      list<Action> &plan) {

  // Borro la lista , borro mi plan
  cout << "Calculando plan\n";
  plan.clear();
  set<estado, ComparaEstadosCostes> Cerrados; // Lista de Cerrados
  priority_queue<nodoCoste> Abiertos;         // Lista de Abiertos

  nodoCoste current;
  current.st = origen;
  current.coste = 0;
  current.secuencia.empty();

  Abiertos.push(current);
  // hijoSEMITurnR.coste += coste(hijoSEMITurnR.fil,hijoSEMITurnR.col,tiene
  // bikini, tiene zapas, actuo semiturn R ) creo que tengo que dar el
  // equipamiento

  while (!Abiertos.empty() and
         (current.st.fila != destino.fila or
          current.st.columna !=
              destino.columna)) // mientras abiertos no sea fin y no se haya
                                // llegado al destino creamos el arbol
  {

    Abiertos.pop();
    Cerrados.insert(current.st);

    // veo bikini y zapas
    // current.st = SensorCasilla(sensores, current.st);
    if (mapaResultado[current.st.fila][current.st.columna] == 'K') {
      current.st.TieneBikini = true;
      current.st.TieneZapatillas = false;
    }

    if (mapaResultado[current.st.fila][current.st.columna] == 'D') {
      current.st.TieneZapatillas = true;
      current.st.TieneBikini = false;
    }

    cout << "Estoy generando los hijos de "
         << " " << current.coste << endl;

    // genera hijo giro a la derecha 90
    nodoCoste hijoTurnR = current;
    hijoTurnR.st.orientacion =
        (hijoTurnR.st.orientacion + 2) % 8; // giro der 90
    hijoTurnR.coste += CalculoCoste(
        current.st.fila, current.st.columna, actTURN_R, current.st.TieneBikini,
        current.st.TieneZapatillas); // calculo el coste
    if (Cerrados.find(hijoTurnR.st) == Cerrados.end()) {
      hijoTurnR.secuencia.push_back(actTURN_R);
      Abiertos.push(hijoTurnR);
    }

    // genera hijo giro a la izquierda 90
    nodoCoste hijoTurnL = current;
    hijoTurnL.st.orientacion = (hijoTurnL.st.orientacion + 6) % 8;
    hijoTurnL.coste += CalculoCoste(
        current.st.fila, current.st.columna, actTURN_L, current.st.TieneBikini,
        current.st.TieneZapatillas); // calculo el coste

    if (Cerrados.find(hijoTurnL.st) == Cerrados.end()) {
      hijoTurnL.secuencia.push_back(actTURN_L);
      Abiertos.push(hijoTurnL);
    }

    // genera gijo giro a la derecha 45
    nodoCoste hijoSEMITurnR = current;
    hijoSEMITurnR.st.orientacion = (hijoSEMITurnR.st.orientacion + 1) % 8;
    hijoSEMITurnR.coste += CalculoCoste(
        current.st.fila, current.st.columna, actSEMITURN_R,
        current.st.TieneBikini, current.st.TieneZapatillas); // calculo el coste

    if (Cerrados.find(hijoSEMITurnR.st) == Cerrados.end()) {
      hijoSEMITurnR.secuencia.push_back(actSEMITURN_R);
      Abiertos.push(hijoSEMITurnR);
    }

    // genera hijo giro a la izquierda 45
    nodoCoste hijoSEMITurnL = current;
    hijoSEMITurnL.st.orientacion = (hijoSEMITurnL.st.orientacion + 7) % 8;
    hijoSEMITurnL.coste += CalculoCoste(
        current.st.fila, current.st.columna, actSEMITURN_L,
        current.st.TieneBikini, current.st.TieneZapatillas); // calculo el coste

    if (Cerrados.find(hijoSEMITurnL.st) == Cerrados.end()) {
      hijoSEMITurnL.secuencia.push_back(actSEMITURN_L);
      Abiertos.push(hijoSEMITurnL);
    }

    // genero hijo para avanzar
    nodoCoste hijoForward = current;
    hijoForward.coste += CalculoCoste(
        current.st.fila, current.st.columna, actFORWARD, current.st.TieneBikini,
        current.st.TieneZapatillas); // calculo el coste

    if (!HayObstaculoDelante(hijoForward.st)) {
      if (Cerrados.find(hijoForward.st) == Cerrados.end()) {
        hijoForward.secuencia.push_back(actFORWARD);
        Abiertos.push(hijoForward);
      }
    }

    // tomo el siguiente valor de Abiertos
    if (!Abiertos.empty())
      current = Abiertos.top();
  }

  // cargamos el plan

  cout << "Terminada la busqueda\n";

  if (current.st.fila == destino.fila and
      current.st.columna == destino.columna) {
    cout << "Cargando el plan\n";
    plan = current.secuencia;
    cout << "Longitud del plan: " << plan.size() << endl;
    PintaPlan(plan);
    // ver el plan en el mapa
    VisualizaPlan(origen, plan);
    cout << "Coste: " << current.coste << endl;
    return true;
  } else {
    cout << "No encontrado plan\n";
  }

  return false;
}

//------------------------------------------------------------------------
//			NIVEL 3: RETO 1 (DESCUBRIR MAYOR PORCENTAJE MAPA)
//------------------------------------------------------------------------

int ComportamientoJugador::interact(Action accion, int valor) { return false; }

bool ComportamientoJugador::SensoresAvanzar(Sensores sensores, estado st) {

  bool b1 = (sensores.terreno[2] == 'T' or sensores.terreno[2] == 'S' or
             sensores.terreno[2] == 'G' or sensores.terreno[2] == 'D' or
             sensores.terreno[2] == 'K' or sensores.terreno[2] == 'X');
  bool b2 = (sensores.superficie[2] == '_');
  bool condicion_zapatillas = (sensores.terreno[2] == 'B');
  bool condicion_bikini = (sensores.terreno[2] == 'A');

  
  if (st.TieneZapatillas) // ahora uso estado porque las variables de zapas y bikini estan ahi
    b1 = b1 || condicion_zapatillas;

  if (st.TieneBikini)
    b1 = b1 || condicion_bikini;
  

  return b1 && b2;
}

void ComportamientoJugador::ActualizarMapa(Sensores sensores) {

  int fil=sensores.posF;
  int col=sensores.posC;
  int brujula=sensores.sentido;

  mapaResultado[fil][col] = sensores.terreno[0];

  switch (brujula) {
  case 0:
    mapaResultado[fil - 1][col - 1] = sensores.terreno[1];
    mapaResultado[fil - 1][col] = sensores.terreno[2];
    mapaResultado[fil - 1][col + 1] = sensores.terreno[3];
    mapaResultado[fil - 2][col - 2] = sensores.terreno[4];
    mapaResultado[fil - 2][col - 1] = sensores.terreno[5];
    mapaResultado[fil - 2][col] = sensores.terreno[6];
    mapaResultado[fil - 2][col + 1] = sensores.terreno[7];
    mapaResultado[fil - 2][col + 2] = sensores.terreno[8];

    mapaResultado[fil - 3][col - 3] = sensores.terreno[9];
    mapaResultado[fil - 3][col - 2] = sensores.terreno[10];
    mapaResultado[fil - 3][col - 1] = sensores.terreno[11];
    mapaResultado[fil - 3][col] = sensores.terreno[12];
    mapaResultado[fil - 3][col + 1] = sensores.terreno[13];
    mapaResultado[fil - 3][col + 2] = sensores.terreno[14];
    mapaResultado[fil - 3][col + 3] = sensores.terreno[15];
    break;

  case 1:
  	mapaResultado[fil - 1][col] = sensores.terreno[1];
	mapaResultado[fil - 1][col+1] = sensores.terreno[2];
	mapaResultado[fil][col + 1] = sensores.terreno[3];
	mapaResultado[fil - 2][col] = sensores.terreno[4];
	mapaResultado[fil - 2][col + 1] = sensores.terreno[5];
	mapaResultado[fil - 2][col + 2] = sensores.terreno[6];
	mapaResultado[fil - 1][col + 2] = sensores.terreno[7];
    mapaResultado[fil][col + 2] = sensores.terreno[8];
	mapaResultado[fil - 3][col] = sensores.terreno[9];
	mapaResultado[fil - 3][col + 1] = sensores.terreno[10];
	mapaResultado[fil - 3][col + 2] = sensores.terreno[11];
	mapaResultado[fil - 3][col + 3] = sensores.terreno[12];
	mapaResultado[fil -2 ][col +3 ] = sensores.terreno[13];
	mapaResultado[fil -1][col+3 ] = sensores.terreno[14];
	mapaResultado[fil][col + 3] = sensores.terreno[15];
  break;

  case 2:
    mapaResultado[fil - 1][col + 1] = sensores.terreno[1];
    mapaResultado[fil][col + 1] = sensores.terreno[2];
    mapaResultado[fil + 1][col + 1] = sensores.terreno[3];

    mapaResultado[fil - 2][col + 2] = sensores.terreno[4];
    mapaResultado[fil - 1][col + 2] = sensores.terreno[5];
    mapaResultado[fil][col + 2] = sensores.terreno[6];
    mapaResultado[fil + 1][col + 2] = sensores.terreno[7];
    mapaResultado[fil + 2][col + 2] = sensores.terreno[8];

    mapaResultado[fil - 3][col + 3] = sensores.terreno[9];
    mapaResultado[fil - 2][col + 3] = sensores.terreno[10];
    mapaResultado[fil - 1][col + 3] = sensores.terreno[11];
    mapaResultado[fil][col + 3] = sensores.terreno[12];
    mapaResultado[fil + 1][col + 3] = sensores.terreno[13];
    mapaResultado[fil + 2][col + 3] = sensores.terreno[14];
    mapaResultado[fil + 3][col + 3] = sensores.terreno[15];
    break;

	case 3:
		mapaResultado[fil ][col+1] = sensores.terreno[1];
		mapaResultado[fil + 1][col+1] = sensores.terreno[2];
		mapaResultado[fil+1][col] = sensores.terreno[3];
		mapaResultado[fil][col+2] = sensores.terreno[4];
		mapaResultado[fil + 1][col+2] = sensores.terreno[5];
		mapaResultado[fil + 2][col+2] = sensores.terreno[6];
		mapaResultado[fil + 2][col+1] = sensores.terreno[7];
		mapaResultado[fil+2][col] = sensores.terreno[8];
		mapaResultado[fil][col+3] = sensores.terreno[9];
		mapaResultado[fil + 1][col+3] = sensores.terreno[10];
		mapaResultado[fil + 2][col+3] = sensores.terreno[11];
		mapaResultado[fil + 3][col+3] = sensores.terreno[12];
		mapaResultado[fil + 3][col+2] = sensores.terreno[13];
		mapaResultado[fil + 3][col+1] = sensores.terreno[14];
		mapaResultado[fil+3][col] = sensores.terreno[15];
	break;

  case 4:
    mapaResultado[fil + 1][col + 1] = sensores.terreno[1];
    mapaResultado[fil + 1][col] = sensores.terreno[2];
    mapaResultado[fil + 1][col - 1] = sensores.terreno[3];

    mapaResultado[fil + 2][col + 2] = sensores.terreno[4];
    mapaResultado[fil + 2][col + 1] = sensores.terreno[5];
    mapaResultado[fil + 2][col] = sensores.terreno[6];
    mapaResultado[fil + 2][col - 1] = sensores.terreno[7];
    mapaResultado[fil + 2][col - 2] = sensores.terreno[8];

    mapaResultado[fil + 3][col + 3] = sensores.terreno[9];
    mapaResultado[fil + 3][col + 2] = sensores.terreno[10];
    mapaResultado[fil + 3][col + 1] = sensores.terreno[11];
    mapaResultado[fil + 3][col] = sensores.terreno[12];
    mapaResultado[fil + 3][col - 1] = sensores.terreno[13];
    mapaResultado[fil + 3][col - 2] = sensores.terreno[14];
    mapaResultado[fil + 3][col - 3] = sensores.terreno[15];
    break;

	case 5:
		mapaResultado[fil + 1][col] = sensores.terreno[1];
		mapaResultado[fil + 1][col-1] = sensores.terreno[2];
		mapaResultado[fil][col - 1] = sensores.terreno[3];
		mapaResultado[fil + 2][col] = sensores.terreno[4];
		mapaResultado[fil + 2][col - 1] = sensores.terreno[5];
		mapaResultado[fil + 2][col - 2] = sensores.terreno[6];
		mapaResultado[fil + 1][col - 2] = sensores.terreno[7];
		mapaResultado[fil][col - 2] = sensores.terreno[8];
		mapaResultado[fil + 3][col] = sensores.terreno[9];
		mapaResultado[fil + 3][col - 1] = sensores.terreno[10];
		mapaResultado[fil + 3][col - 2] = sensores.terreno[11];
		mapaResultado[fil + 3][col - 3] = sensores.terreno[12];
		mapaResultado[fil +2 ][col - 3 ] = sensores.terreno[13];
		mapaResultado[fil +1][col-3 ] = sensores.terreno[14];
		mapaResultado[fil][col - 3] = sensores.terreno[15];
	break;

  case 6:
    mapaResultado[fil + 1][col - 1] = sensores.terreno[1];
    mapaResultado[fil][col - 1] = sensores.terreno[2];
    mapaResultado[fil - 1][col - 1] = sensores.terreno[3];

    mapaResultado[fil + 2][col - 2] = sensores.terreno[4];
    mapaResultado[fil + 1][col - 2] = sensores.terreno[5];
    mapaResultado[fil][col - 2] = sensores.terreno[6];
    mapaResultado[fil - 1][col - 2] = sensores.terreno[7];
    mapaResultado[fil - 2][col - 2] = sensores.terreno[8];

    mapaResultado[fil + 3][col - 3] = sensores.terreno[9];
    mapaResultado[fil + 2][col - 3] = sensores.terreno[10];
    mapaResultado[fil + 1][col - 3] = sensores.terreno[11];
    mapaResultado[fil][col - 3] = sensores.terreno[12];
    mapaResultado[fil - 1][col - 3] = sensores.terreno[13];
    mapaResultado[fil - 2][col - 3] = sensores.terreno[14];
    mapaResultado[fil - 3][col - 3] = sensores.terreno[15];
    break;


	case 7:
		mapaResultado[fil ][col-1] = sensores.terreno[1];
		mapaResultado[fil - 1][col-1] = sensores.terreno[2];
		mapaResultado[fil-1][col] = sensores.terreno[3];
		mapaResultado[fil][col-2] = sensores.terreno[4];
		mapaResultado[fil - 1][col-2] = sensores.terreno[5];
		mapaResultado[fil - 2][col-2] = sensores.terreno[6];
		mapaResultado[fil - 2][col-1] = sensores.terreno[7];
		mapaResultado[fil-2][col] = sensores.terreno[8];
		mapaResultado[fil][col-3] = sensores.terreno[9];
		mapaResultado[fil - 1][col-3] = sensores.terreno[10];
		mapaResultado[fil - 2][col-3] = sensores.terreno[11];
		mapaResultado[fil - 3][col-3] = sensores.terreno[12];
		mapaResultado[fil - 3][col-2] = sensores.terreno[13];
		mapaResultado[fil - 3][col-1] = sensores.terreno[14];
		mapaResultado[fil-3][col] = sensores.terreno[15];

	break;
  }
}


estado ComportamientoJugador::Recargar(Sensores sensores,estado st) {
  tiempo_recarga--;
  if (tiempo_recarga == 0) {
    st.recarga = false;
    tiempo_recarga = 100;
  }
  return st;
}

Action ComportamientoJugador::Girar(Sensores sensores) {
  Action accion;
  // generar valores aleatorios del 1 al 4

	switch (girar) {
		case 0:
			accion = actTURN_L;
			girar = (rand() % 4);
		break;

		case 1: 
			accion = actTURN_R;
			girar = (rand() % 4);
		break;

		case 2:
			accion = actSEMITURN_L;
			girar = (rand() % 4);
		break;

		case 3: 
			accion = actSEMITURN_L;
			girar = (rand() % 4);
		break;
	}

/* 
  if (!girar_derecha) {
    accion = actTURN_L;
    girar_derecha = (rand() % 2 == 0);

  } else {
    accion = actTURN_R;
    girar_derecha = (rand() % 2 == 0);
  }
*/

  return accion;
}