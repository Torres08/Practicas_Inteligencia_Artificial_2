#ifndef COMPORTAMIENTOJUGADOR_H
#define COMPORTAMIENTOJUGADOR_H

#include "comportamientos/comportamiento.hpp"

#include <list>

struct estado {
  int fila;
  int columna;
  int orientacion;
  bool TieneBikini;
  bool TieneZapatillas;
  bool recarga;
  int num_objetivos_visitados;
  bool objetivos_visitados[3];
};

class ComportamientoJugador : public Comportamiento {
  public:
    ComportamientoJugador(unsigned int size) : Comportamiento(size) {
      // Inicializar Variables de Estado PARA NIVEL 3 Y 4
      Inicializar();
      /*
      comienzo = false;
      tiempo_recarga = 50;
      girar=0; // valores 0,1,2,3
      encontrada_zapas = false;
      encontrada_bikini = true;
      encontrada_recarga = false;
      tiempo_intervalo = 350; // cada 10 pasos busco zapas o busco bikini, 
      contador = 50; // si llega a 0 habilito para que no se quede pillado
      emergencia = false;
      contador_emergencia = 25;
      bien_busqueda = true;
      */
    }


    ComportamientoJugador(std::vector< std::vector< unsigned char> > mapaR) : Comportamiento(mapaR) {
      // Inicializar Variables de Estado PARA NIVEL 0 1 Y 2
      Inicializar();
    }
    ComportamientoJugador(const ComportamientoJugador & comport) : Comportamiento(comport){}
    ~ComportamientoJugador(){}

    Action think(Sensores sensores);
    int interact(Action accion, int valor);
    void VisualizaPlan(const estado &st, const list<Action> &plan);
    ComportamientoJugador * clone(){return new ComportamientoJugador(*this);}

  private:
    // Declarar Variables de Estado
    estado actual; // fil col brujula TieneBikini TieneZapatillas
    list<estado> objetivos;
    list<Action> plan;
    bool hayPlan, comienzo;
    int tiempo_recarga;
    int girar;
    bool encontrada_zapas, encontrada_bikini, encontrada_recarga;
    int tiempo_intervalo;
    int contador;
    Action ultimaAccion;
    bool emergencia;
    int contador_emergencia;
    int num_objetivos;
    bool cambio;
    bool bien_busqueda;
    estado vector_objetivos[3];
    bool bien_situado;

    // Métodos privados de la clase
    bool pathFinding(int level, const estado &origen, const list<estado> &destino, list<Action> &plan);
    bool pathFinding_Profundidad(const estado &origen, const estado &destino, list<Action> &plan);
    bool pathFinding_Anchura(const estado &origen, const estado &destino, list<Action> &plan);
    bool pathFinding_CostoUniforme(const estado &origen, const estado &destino, list<Action> &plan);
    bool pathFinding_AEstrella(const estado &origen, const estado &destino, list<Action> &plan);


    //
    estado SensorCasilla(Sensores sensores, estado actual);
    unsigned int CalculoCoste(int fil, int col, Action accion, bool TieneBikini, bool TieneZapatillas);
    void PintaPlan(list<Action> plan);
    bool HayObstaculoDelante(estado &st);
    int distancia_estimada(const estado actual, const estado destino);
    
    //
    bool SensoresAvanzar(Sensores sensores, estado st);
    void ActualizarMapa(Sensores sensores);
    estado Recargar(Sensores sensores, estado st);
    Action Girar(Sensores sensores);
    estado CalculoPunto(int i,Sensores sensores);
    void VistaAgente(Sensores sensores);
    void EmergenciaBelkan();
    void IntervaloBusqueda();
    Action MoverAleatorio(Sensores sensores);
    void MovimientoRepetido(Action accion, Action ultimaAccion);

    //
    void SensorVistaNivel(Sensores sensores);
    bool replanificar(Sensores sensores, estado ac, estado dest); // true si tengo que replanificar

    //
    void Inicializar(){
      hayPlan = false;
      actual.TieneBikini = actual.TieneZapatillas = actual.recarga = false;
      num_objetivos = 3;
      
      comienzo = false;
      tiempo_recarga = 50;
      girar=0; // valores 0,1,2,3
      encontrada_zapas = false;
      encontrada_bikini = true;
      encontrada_recarga = false;
      tiempo_intervalo = 350; // cada 10 pasos busco zapas o busco bikini, 
      contador = 50; // si llega a 0 habilito para que no se quede pillado
      emergencia = false;
      contador_emergencia = 25;

      bien_busqueda = true;
      
      bien_situado = false;
      cambio = true;
    }
};



#endif
