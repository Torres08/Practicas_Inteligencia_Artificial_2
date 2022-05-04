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
};

class ComportamientoJugador : public Comportamiento {
  public:
    ComportamientoJugador(unsigned int size) : Comportamiento(size) {
      // Inicializar Variables de Estado PARA NIVEL 3 Y 4
      Inicializar();
      comienzo = false;
      tiempo_recarga = 50;
      girar=0; // valores 0,1,2,3
      encontrada_zapas = false;
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
    bool encontrada_zapas;

    // Métodos privados de la clase
    bool pathFinding(int level, const estado &origen, const list<estado> &destino, list<Action> &plan);
    bool pathFinding_Profundidad(const estado &origen, const estado &destino, list<Action> &plan);
    bool pathFinding_Anchura(const estado &origen, const estado &destino, list<Action> &plan);
    bool pathFinding_CostoUniforme(const estado &origen, const estado &destino, list<Action> &plan);

    estado SensorCasilla(Sensores sensores, estado actual);
    unsigned int CalculoCoste(int fil, int col, Action accion, bool TieneBikini, bool TieneZapatillas);
    void PintaPlan(list<Action> plan);
    bool HayObstaculoDelante(estado &st);
    
    //
    bool SensoresAvanzar(Sensores sensores, estado st);
    void ActualizarMapa(Sensores sensores);
    estado Recargar(Sensores sensores, estado st);
    Action Girar(Sensores sensores);
    estado CalculoPunto(int i,Sensores sensores);

    void Inicializar(){
      hayPlan = false;
      actual.TieneBikini = actual.TieneZapatillas = actual.recarga = false;
    }
};



#endif
