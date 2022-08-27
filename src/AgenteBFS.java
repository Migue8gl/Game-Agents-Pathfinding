package tracks.singlePlayer.evaluacion.src_GARCIA_LOPEZ_MIGUEL;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Stack;

import core.game.Observation;
import core.game.StateObservation;
import core.player.AbstractPlayer;
import ontology.Types.ACTIONS;
import tools.ElapsedCpuTimer;
import tools.Vector2d;

public class AgenteBFS extends AbstractPlayer {

	private Vector2d fescala;
	private Vector2d portal;
	private Stack<ACTIONS> camino;
	private boolean camino_encontrado;
	private boolean prim_acc_real = false;
	static private boolean pos_visitadas[][];
	private int nd_exp;
	private int A;

	public AgenteBFS(StateObservation stateObs, ElapsedCpuTimer elapsedTimer) {
		// Cambio de pixeles a coordenadas del mapa
		fescala = new Vector2d(stateObs.getWorldDimension().width / stateObs.getObservationGrid().length,
				stateObs.getWorldDimension().height / stateObs.getObservationGrid()[0].length);

		// Encuentro el objetivo
		ArrayList<Observation>[] posiciones = stateObs.getPortalsPositions(stateObs.getAvatarPosition());
		portal = posiciones[0].get(0).position;
		portal.x = Math.floor(portal.x / fescala.x);
		portal.y = Math.floor(portal.y / fescala.y);

		// Contador de nodos expandidos y nodos en memoria
		nd_exp = 0;
		A = 0;

		camino_encontrado = false;
		camino = new Stack<ACTIONS>();

		// Matriz con las posiciones visitadas, más eficiente en tiempo que recorrer un
		// contenedor
		int fil = stateObs.getWorldDimension().width / (int) fescala.x;
		int col = stateObs.getWorldDimension().height / (int) fescala.y;
		pos_visitadas = new boolean[fil][col];

		// Todas las posiciones a false (no visitadas)
		for (int i = 0; i < fil; i++) {
			for (int j = 0; j < col; j++)
				pos_visitadas[i][j] = false;
		}
	}

	@Override
	public ACTIONS act(StateObservation stateObs, ElapsedCpuTimer elapsedTimer) {
		// Posicion inicial del avatar
		Vector2d avatar = new Vector2d(stateObs.getAvatarPosition().x / fescala.x,
				stateObs.getAvatarPosition().y / fescala.y);

		Queue<Node> cola = new LinkedList<Node>();
		Node nodo_ini = new Node(ACTIONS.ACTION_NIL, null, avatar);

		// Actualizar estado comprueba la matriz de posiciones
		this.actEstado(nodo_ini.getPos(), nodo_ini);
		Node nodo;
		Node nodo_hijo;
		ACTIONS accion;
		ACTIONS prim_accion = ACTIONS.ACTION_NIL;
		cola.add(nodo_ini);
		A++;
		boolean terminar = false;

		if (!camino_encontrado) {

			long tInicio = System.nanoTime();

			// Mientras la cola esté llena debo seguir iterando nodos
			while (!cola.isEmpty()) {

				// Saco primer nodo de la cola
				nodo = cola.poll();

				// Si es objetivo terminamos y contruimos el camino a través de los padres
				if (nodo.getPos().x == portal.x && nodo.getPos().y == portal.y) {
					nd_exp++;
					camino.push(nodo.getAccion());
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
					camino_encontrado = true;
				} else
					nd_exp++;

				// Expando todos los hijos del nodo actual
				if (!camino_encontrado) {
					Vector2d newPos_up = nodo.getPos(), newPos_down = nodo.getPos(), newPos_left = nodo.getPos(),
							newPos_right = nodo.getPos();
					if (nodo.getPos().y - 1 >= 0) {
						newPos_up = new Vector2d(nodo.getPos().x, nodo.getPos().y - 1);
						nodo_hijo = new Node(ACTIONS.ACTION_UP, nodo, newPos_up);
						this.actEstado(nodo_hijo.getPos(), nodo_hijo);

						if (nodo_hijo.getEstado() == "no_visitado" && !isObstaculo(stateObs, nodo_hijo)) {
							nodo_hijo.setEstado("visitado");
							cola.add(nodo_hijo);
							A++;
						}
					}
					if (nodo.getPos().y + 1 <= stateObs.getObservationGrid()[0].length - 1) {
						newPos_down = new Vector2d(nodo.getPos().x, nodo.getPos().y + 1);
						nodo_hijo = new Node(ACTIONS.ACTION_DOWN, nodo, newPos_down);
						this.actEstado(nodo_hijo.getPos(), nodo_hijo);

						if (nodo_hijo.getEstado() == "no_visitado" && !isObstaculo(stateObs, nodo_hijo)) {
							nodo_hijo.setEstado("visitado");
							cola.add(nodo_hijo);
							A++;
						}
					}
					if (nodo.getPos().x - 1 >= 0) {
						newPos_left = new Vector2d(nodo.getPos().x - 1, nodo.getPos().y);
						nodo_hijo = new Node(ACTIONS.ACTION_LEFT, nodo, newPos_left);
						this.actEstado(nodo_hijo.getPos(), nodo_hijo);

						if (nodo_hijo.getEstado() == "no_visitado" && !isObstaculo(stateObs, nodo_hijo)) {
							nodo_hijo.setEstado("visitado");
							cola.add(nodo_hijo);
							A++;
						}
					}
					if (nodo.getPos().x + 1 <= stateObs.getObservationGrid().length - 1) {
						newPos_right = new Vector2d(nodo.getPos().x + 1, nodo.getPos().y);
						nodo_hijo = new Node(ACTIONS.ACTION_RIGHT, nodo, newPos_right);
						this.actEstado(nodo_hijo.getPos(), nodo_hijo);

						if (nodo_hijo.getEstado() == "no_visitado" && !isObstaculo(stateObs, nodo_hijo)) {
							nodo_hijo.setEstado("visitado");
							cola.add(nodo_hijo);
							A++;
						}
					}
				}
			}
			long tFin = System.nanoTime();
			long tiempoTotal = (tFin - tInicio) / 1000000;
			System.out.println("Runtime: " + tiempoTotal);
			System.out.println("Tamaño ruta: " + camino.size());
			System.out.println("Nodos expandidos: " + nd_exp);
			System.out.println("Máx nodos en memoria: " + A);
		}

		// Si hay camino y se ha impreso la primera acción, las siguientes acciones
		// serán las del camino
		if (camino_encontrado && prim_acc_real)
			if (!camino.isEmpty())
				return camino.pop();

		// La primera acción es la primera acción del camino una vez se ha calculado
		// este
		prim_acc_real = true;
		return prim_accion;
	}

	/*
	 * Método para comprobar obstáculos, todo lo que no tenga id = 3 es considerado
	 * obstáculo, pero esto no afecta al objetivo ya que se comprueba siempre al
	 * inicio si se ha llegado o no al final
	 */
	public boolean isObstaculo(StateObservation stateObs, Node nodo) {
		ArrayList<Observation>[][] map = stateObs.getObservationGrid();

		for (Observation obs : map[(int) nodo.getPos().x][(int) nodo.getPos().y])
			if (obs.itype != 3)
				return true;

		return false;
	}

	// Comprueba matriz de posiciones visitadas y actualiza el estado del nodo en
	// cuestión
	public void actEstado(Vector2d pos, Node nodo) {
		if (pos_visitadas[(int) pos.x][(int) pos.y] == true)
			nodo.setEstado("visitado");
		else {
			pos_visitadas[(int) pos.x][(int) pos.y] = true;
			nodo.setEstado("no_visitado");
		}
	}
}
