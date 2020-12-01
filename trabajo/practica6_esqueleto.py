#! /usr/bin/python

# 6ta Practica Laboratorio 
# Complementos Matematicos I
# Ejemplo parseo argumentos

import argparse
import matplotlib.pyplot as plt
import numpy as np
from numpy import random


class LayoutGraph:

    def __init__(self, tamano: float, grafo, iters, refresh, c1, c2, initial_t, update_temp, verbose=False):
        """
        Parámetros:
        tamano: tamaño de uno de los lados del cuadrado que limita la representacion
        grafo: grafo en formato lista
        iters: cantidad de iteraciones a realizar
        refresh: cada cuántas iteraciones graficar. Si su valor es cero, entonces debe graficarse solo al final.
        c1: constante de attracion
        c2: constante de repulsion
        initial_t: constante inicial de temperatura
        update_temp: constante de actualizacion de la temperatura
        verbose: si está encendido, activa los comentarios
        """

        # Guardo el grafo
        self.grafo = grafo

        # Guardo opciones
        self.iters = iters
        self.verbose = verbose
        self.size = tamano
        self.refresh = refresh
        self.c1 = c1
        self.c2 = c2
        self.initial_t = initial_t
        self.update_temp = update_temp

    def layout(self):
        """
        Aplica el algoritmo de Fruchtermann-Reingold para obtener (y mostrar)
        un layout
        """
        V, E = self.grafo

        self.k1 = self.c1 * np.sqrt(self.size * self.size / len(V))
        self.k2 = self.c2 * np.sqrt(self.size * self.size / len(V))

        self._randomize_positions()
        self._display_step()

        for k in range(self.iters):
            self._log("Inicio iteracion {}".format(k))

            self._step()

            self._log("Fin iteracion {}".format(k))
            self._log("Posiciones " + str(self.posiciones.tolist()))
            self._display_step()

    def _randomize_positions(self):
        V, E = self.grafo
        size = self.size #float tamaño del espacio

        # "np.finfo(float).eps" para que no genere vertices sobre el borde,
        # porque el final del rango es abierto pero el inicio es cerrado
        self.posiciones = np.random.default_rng().uniform(np.finfo(float).eps, size, (len(V), 2))

    def _step(self):
        self._initialize_temperature()
        self._initialize_accumulators()
        self._compute_attractions()
        self._compute_repulsions()
        self._compute_gravity()
        self._separate_overlapping_vertex()
        self._update_positions()
        self._update_temperature()

    def _display_step(self):
        V, E = self.grafo
        plt.scatter(self.posiciones[:, 0], self.posiciones[:, 1])
        for a, b in E:
            x = [self.posiciones[V.index(a), 0], self.posiciones[V.index(b), 0]]
            y = [self.posiciones[V.index(a), 1], self.posiciones[V.index(b), 1]]
            plt.plot(x, y)
        plt.show()

    def _initialize_temperature(self):
        self.temperature = self.initial_t

    def _initialize_accumulators(self):
        V, E = self.grafo
        self.accumulator = np.zeros((len(V), 2))

    def _compute_attractions(self):
        V, E = self.grafo
        for a, b in E:
            i = V.index(a)
            j = V.index(b)

            vector = self.posiciones[i] - self.posiciones[j]
            distance = np.linalg.norm(vector)
            delta_attraction = self._force_attraction(distance)
            force = delta_attraction * (vector / distance)
            self.accumulator[i] += force
            self.accumulator[j] -= force

            self._log("[A] Sobre ({},{}) se aplica |({:.2f},{:.2f})|={:.2f}"
                      .format(a, b, force[0], force[1], np.linalg.norm(force)))

    def _force_attraction(self, x):
        return x * x / self.k1

    def _compute_repulsions(self):
        V, E = self.grafo
        for i in range(len(V)):
            for j in range(len(V)):
                if i == j:
                    continue

                vector = self.posiciones[i] - self.posiciones[j]
                distance = np.linalg.norm(vector)
                delta_attraction = self._force_repulsion(distance)
                force = delta_attraction * (vector / distance)
                self.accumulator[i] += force
                self.accumulator[j] -= force

                self._log("[R] Sobre ({},{}) se aplica |({:.2f},{:.2f})|={:.2f}"
                          .format(V[i], V[j], force[0], force[1], np.linalg.norm(force)))

    def _force_repulsion(self, x):
        return -self.k2 * self.k2 / x

    def _compute_gravity(self):
        eps = np.finfo(float).eps
        gravity_origin = np.asarray([self.size / 2, self.size / 2])

        V, E = self.grafo
        for i in range(len(V)):
            vector = gravity_origin - self.posiciones[i]
            distance = np.linalg.norm(vector) + eps
            delta_attraction = self._force_gravity(distance)
            force = delta_attraction * (vector / distance)
            self.accumulator[i] += force

            self._log("[G] Sobre {} se aplica ({:.2f},{:.2f})".format(V[i], force[0], force[1]))

    def _force_gravity(self, x):
        return 0.1 * self._force_attraction(x)

    def _separate_overlapping_vertex(self):
        if not np.any(np.isnan(self.accumulator)):
            return

        # Esto solo se ejecuta si alguna division dio nan,
        # es decir, si la distancia fue 0 en alguna cuenta
        eps = 0.005

        for i in range(self.accumulator.shape[0]):
            if np.any(np.isnan(self.accumulator[i])):
                small_force = np.random.normal(0, 1, 2)
                small_force = np.linalg.norm(small_force)
                small_force *= eps
                self.accumulator[i] = small_force

    def _update_positions(self):
        V, E = self.grafo
        for i in range(len(V)):
            distance = np.linalg.norm(self.accumulator[i])

            if distance > self.temperature:
                self.accumulator[i] /= distance
                self.accumulator[i] *= self.temperature

            self.posiciones[i] += self.accumulator[i]

        self.posiciones = np.clip(self.posiciones, 0, self.size)

    def _update_temperature(self):
        self.temperature *= self.update_temp

    def _log(self, msg: str):
        if self.verbose:
            print(msg)


def lee_grafo_archivo(file_path):
    '''
    Lee un grafo desde un archivo y devuelve su representacion como lista.
    Ejemplo Entrada:
        3
        A
        B
        C
        A B
        B C
        C B
    Ejemplo retorno:
        (['A','B','C'],[('A','B'),('B','C'),('C','B')])
    '''
    with open(file_path, 'r') as iterable:
        cant = int(iterable.readline())
        vertices = []
        while len(vertices) != cant:
            vertices.append(iterable.readline().strip())

        aristas = []
        while True:
            v = iterable.readline().strip().split(" ")
            if v[0] == "":
                break
            if (v[0], v[1]) in aristas:
                continue
            if v[0] in vertices and v[1] in vertices:
                aristas.append((v[0], v[1]))
        return (vertices, aristas)


def main():
    # Definimos los argumentos de linea de comando que aceptamos
    parser = argparse.ArgumentParser()

    # Verbosidad, opcional, False por defecto
    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        help='Muestra mas informacion al correr el programa'
    )
    # Cantidad de iteraciones, opcional, 50 por defecto
    parser.add_argument(
        '--iters',
        type=int,
        help='Cantidad de iteraciones a efectuar',
        default=50
    )
    # Temperatura inicial
    parser.add_argument(
        '--temp',
        type=float,
        help='Temperatura inicial',
        default=100.0
    )
    # Archivo del cual leer el grafo
    parser.add_argument(
        'file_name',
        help='Archivo del cual leer el grafo a dibujar'
    )

    args = parser.parse_args()

    grafo = lee_grafo_archivo(args.file_name)

    # Creamos nuestro objeto LayoutGraph
    layout_gr = LayoutGraph(
        1000,
        grafo,
        iters=args.iters,
        refresh=1,
        c1=5.0,
        c2=0.1,
        initial_t=1,
        update_temp=0.95,
        verbose=args.verbose
    )

    # Ejecutamos el layout
    layout_gr.layout()
    return


if __name__ == '__main__':
    main()
