package tracks.singlePlayer.evaluacion.src_GARCIA_LOPEZ_MIGUEL;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.Stack;

import core.game.Observation;
import core.game.StateObservation;
import core.player.AbstractPlayer;
import ontology.Types.ACTIONS;
import tools.ElapsedCpuTimer;
import tools.Vector2d;

public class AgenteAStar extends AbstractPlayer {

	private Vector2d fescala;
	private Vector2d portal;
	private Stack<ACTIONS> camino;
	private boolean camino_encontrado;
	private boolean prim_acc_real = false;
	private Node nodo_final;
	private int Cerrados[][];
	private int Abiertos_aux[][];
	private int nd_exp;
	private int A, B, AB;

	public AgenteAStar(StateObservation stateObs, ElapsedCpuTimer elapsedTimer) {
		// Cambio de pixeles a coordenadas del mapa
		fescala = new Vector2d(stateObs.getWorldDimension().width / stateObs.getObservationGrid().length,
				stateObs.getWorldDimension().height / stateObs.getObservationGrid()[0].length);

		A = 0;
		B = 0;
		AB = 0;
		nd_exp = 0;

		// Encuentro el objetivo
		ArrayList<Observation>[] posiciones = stateObs.getPortalsPositions(stateObs.getAvatarPosition());
		portal = posiciones[0].get(0).position;
		portal.x = Math.floor(portal.x / fescala.x);
		portal.y = Math.floor(portal.y / fescala.y);

		camino_encontrado = false;
		camino = new Stack<ACTIONS>();
		
		/* Tenemos dos matrices de enteros que representan si una posición está o no visitada,
		 * en caso de estarlo contendrá la g de cada nodo de manera que podremos actualizarla fácilmente
		 */
		int iMAX = stateObs.getWorldDimension().width / (int) fescala.x;
		int jMAX = stateObs.getWorldDimension().height / (int) fescala.y;
		Cerrados = new int[iMAX][jMAX];
		for (int i = 0; i < iMAX; i++) {
			for (int j = 0; j < jMAX; j++)
				Cerrados[i][j] = -1;
		}

		Abiertos_aux = new int[iMAX][jMAX];
		for (int i = 0; i < iMAX; i++) {
			for (int j = 0; j < jMAX; j++)
				Abiertos_aux[i][j] = -1;
		}
	}

	@Override
	public ACTIONS act(StateObservation stateObs, ElapsedCpuTimer elapsedTimer) {
		// Posicion inicial del avatar
		Vector2d avatar = new Vector2d(stateObs.getAvatarPosition().x / fescala.x,
				stateObs.getAvatarPosition().y / fescala.y);

		Node nodo;
		Node nodo_hijo;
		ACTIONS accion;
		ACTIONS prim_accion = ACTIONS.ACTION_NIL;
		double newH = 0;
		boolean terminar = false;

		// Mientras el camino no sea encontrado seguimos ejecutando el bucle infinito de búsqueda
		if (!camino_encontrado) {
			long tInicio = System.nanoTime();
			
			// Cola con prioridad que ordena según la f de cada nodo
			Comparator<Node> nodeComparator = Comparator.comparing(Node::getF);
			PriorityQueue<Node> Abiertos = new PriorityQueue<Node>(nodeComparator);

			double H = distObjetivo(avatar);
			Node nodo_ini = new Node(ACTIONS.ACTION_NIL, null, avatar, 0, H);

			// Añadimos a abiertos el nodo inicial y actualizamos la matriz de abiertos con su g
			Abiertos.add(nodo_ini);
			Abiertos_aux[(int) nodo_ini.getPos().x][(int) nodo_ini.getPos().y] = (int) nodo_ini.getG();
			A++;

			while (true) {
				
				// Sacamos el primer nodo de abiertos, el que mejor f tenga
				nodo = Abiertos.poll();
				A--;

				// Si es objetivo hemos terminado
				if (nodo.getPos().x == portal.x && nodo.getPos().y == portal.y) {
					nodo_final = nodo;
					nd_exp++;
					break;
				} else
					nd_exp++;

				/*
				 * Expandimos nodos, si está en cerrados, pero hemos encontrado el mismo nodo con mejor coste, lo sacamos de Cerrados.
				 * En caso de que no esté ni en cerrados ni en abiertos, lo metemos en abiertos para que sea explorado.
				 * Si está en Abiertos, pero el nodo que queremos expandir es mejor, actualizamos la g (al ser en java las instancias
				 * referencias, no hace falta sacar el nodo de Abiertos, se actualiza solo)
				 */
				Vector2d newPos_up = nodo.getPos(), newPos_down = nodo.getPos(), newPos_left = nodo.getPos(),
						newPos_right = nodo.getPos();

				if (nodo.getPos().y - 1 >= 0) {
					newPos_up = new Vector2d(nodo.getPos().x, nodo.getPos().y - 1);
					newH = this.distObjetivo(newPos_up);
					nodo_hijo = new Node(ACTIONS.ACTION_UP, nodo, newPos_up, nodo.getG() + 1, newH);

					if ((nodo.getPadre() == null || (nodo_hijo.getPos() != nodo.getPadre().getPos()))
							&& !this.isObstaculo(stateObs, nodo_hijo)) {
						if (Cerrados[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] != -1 && nodo_hijo
								.getG() < Cerrados[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y]) {
							Cerrados[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] = -1;
							B--;
							Abiertos.add(nodo_hijo);
							A++;
							Abiertos_aux[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] = (int) nodo_hijo
									.getG();
						} else if (Cerrados[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] == -1
								&& Abiertos_aux[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] == -1) {
							Abiertos.add(nodo_hijo);
							A++;
							Abiertos_aux[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] = (int) nodo_hijo
									.getG();
						} else if (Abiertos_aux[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] != -1
								&& nodo_hijo.getG() < Abiertos_aux[(int) nodo_hijo.getPos().x][(int) nodo_hijo
										.getPos().y]) {
							Abiertos_aux[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] = -1;
							Abiertos.add(nodo_hijo);
							Abiertos_aux[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] = (int) nodo_hijo
									.getG();
						}
					}
				}
				if (nodo.getPos().y + 1 <= stateObs.getObservationGrid()[0].length - 1) {
					newPos_down = new Vector2d(nodo.getPos().x, nodo.getPos().y + 1);
					newH = this.distObjetivo(newPos_down);
					nodo_hijo = new Node(ACTIONS.ACTION_DOWN, nodo, newPos_down, nodo.getG() + 1, newH);

					if ((nodo.getPadre() == null || (nodo_hijo.getPos() != nodo.getPadre().getPos()))
							&& !this.isObstaculo(stateObs, nodo_hijo)) {
						if (Cerrados[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] != -1 && nodo_hijo
								.getG() < Cerrados[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y]) {
							Cerrados[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] = -1;
							Abiertos.add(nodo_hijo);
							A++;
							B--;
							Abiertos_aux[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] = (int) nodo_hijo
									.getG();
						} else if (Cerrados[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] == -1
								&& Abiertos_aux[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] == -1) {
							Abiertos.add(nodo_hijo);
							A++;
							Abiertos_aux[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] = (int) nodo_hijo
									.getG();
						} else if (Abiertos_aux[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] != -1
								&& nodo_hijo.getG() < Abiertos_aux[(int) nodo_hijo.getPos().x][(int) nodo_hijo
										.getPos().y]) {
							Abiertos_aux[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] = -1;
							Abiertos.add(nodo_hijo);
							Abiertos_aux[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] = (int) nodo_hijo
									.getG();
						}
					}
				}
				if (nodo.getPos().x - 1 >= 0) {
					newPos_left = new Vector2d(nodo.getPos().x - 1, nodo.getPos().y);
					newH = this.distObjetivo(newPos_left);
					nodo_hijo = new Node(ACTIONS.ACTION_LEFT, nodo, newPos_left, nodo.getG() + 1, newH);

					if ((nodo.getPadre() == null || (nodo_hijo.getPos() != nodo.getPadre().getPos()))
							&& !this.isObstaculo(stateObs, nodo_hijo)) {
						if (Cerrados[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] != -1 && nodo_hijo
								.getG() < Cerrados[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y]) {
							Cerrados[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] = -1;
							Abiertos.add(nodo_hijo);
							A++;
							B--;
							Abiertos_aux[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] = (int) nodo_hijo
									.getG();
						} else if (Cerrados[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] == -1
								&& Abiertos_aux[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] == -1) {
							Abiertos.add(nodo_hijo);
							A++;
							Abiertos_aux[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] = (int) nodo_hijo
									.getG();
						} else if (Abiertos_aux[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] != -1
								&& nodo_hijo.getG() < Abiertos_aux[(int) nodo_hijo.getPos().x][(int) nodo_hijo
										.getPos().y]) {
							Abiertos_aux[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] = -1;
							Abiertos.add(nodo_hijo);
							Abiertos_aux[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] = (int) nodo_hijo
									.getG();
						}
					}
				}
				if (nodo.getPos().x + 1 <= stateObs.getObservationGrid().length - 1) {
					newPos_right = new Vector2d(nodo.getPos().x + 1, nodo.getPos().y);
					newH = this.distObjetivo(newPos_right);
					nodo_hijo = new Node(ACTIONS.ACTION_RIGHT, nodo, newPos_right, nodo.getG() + 1, newH);

					if ((nodo.getPadre() == null || (nodo_hijo.getPos() != nodo.getPadre().getPos()))
							&& !this.isObstaculo(stateObs, nodo_hijo)) {
						if (Cerrados[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] != -1 && nodo_hijo
								.getG() < Cerrados[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y]) {
							Cerrados[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] = -1;
							Abiertos.add(nodo_hijo);
							A++;
							B--;
							Abiertos_aux[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] = (int) nodo_hijo
									.getG();
						} else if (Cerrados[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] == -1
								&& Abiertos_aux[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] == -1) {
							Abiertos.add(nodo_hijo);
							A++;
							Abiertos_aux[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] = (int) nodo_hijo
									.getG();
						} else if (Abiertos_aux[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] != -1
								&& nodo_hijo.getG() < Abiertos_aux[(int) nodo_hijo.getPos().x][(int) nodo_hijo
										.getPos().y]) {
							Abiertos_aux[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] = -1;
							Abiertos.add(nodo_hijo);
							Abiertos_aux[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] = (int) nodo_hijo
									.getG();
						}
					}
				}
				Cerrados[(int) nodo.getPos().x][(int) nodo.getPos().y] = (int) nodo.getG();
				B++;
			}
			long tFin = System.nanoTime();
			long tiempoTotal = (tFin - tInicio) / 1000000;
			System.out.println("Runtime: " + tiempoTotal);
		}

		// Contruimos camino a través de los padres
		if (!camino_encontrado) {
			camino_encontrado = true;
			AB = A + B;
			camino.push(nodo_final.getAccion());
			nodo = nodo_final;
			while (!terminar) {
				if (nodo.getPadre() == null) {
					prim_accion = nodo.getAccion();
					terminar = true;
				} else {
					nodo = nodo.getPadre();
					accion = nodo.getAccion();
					camino.push(accion);
				}
			}
			System.out.println("Tamaño ruta: " + camino.size());
			System.out.println("Nodos expandidos: " + nd_exp);
			System.out.println("Máx nodos en memoria: " + AB);
		}

		if (camino_encontrado && prim_acc_real)
			if (!camino.isEmpty())
				return camino.pop();

		prim_acc_real = true;
		return prim_accion;
	}

	public boolean isObstaculo(StateObservation stateObs, Node nodo) {
		ArrayList<Observation>[][] map = stateObs.getObservationGrid();

		for (Observation obs : map[(int) nodo.getPos().x][(int) nodo.getPos().y])
			if (obs.itype != 3)
				return true;

		return false;
	}

	// Información heurística, distancia Manhattan
	public double distObjetivo(Vector2d pos_act) {
		return Math.abs(pos_act.x - portal.x) + Math.abs(pos_act.y - portal.y);
	}
}
