package tracks.singlePlayer.evaluacion.src_GARCIA_LOPEZ_MIGUEL;

import java.util.ArrayList;
import java.util.Stack;

import core.game.Observation;
import core.game.StateObservation;
import core.player.AbstractPlayer;
import ontology.Types.ACTIONS;
import tools.ElapsedCpuTimer;
import tools.Vector2d;

public class AgenteIDAStar extends AbstractPlayer {

	private Vector2d fescala;
	private Vector2d portal;
	private Stack<ACTIONS> camino;
	private boolean camino_encontrado;
	private boolean prim_acc_real;
	private Node nodo_final;
	private int nd_exp;
	private int A;
	private boolean Ruta_grid[][];

	public AgenteIDAStar(StateObservation stateObs, ElapsedCpuTimer elapsedTimer) {
		// Cambio de pixeles a coordenadas del mapa
		fescala = new Vector2d(stateObs.getWorldDimension().width / stateObs.getObservationGrid().length,
				stateObs.getWorldDimension().height / stateObs.getObservationGrid()[0].length);

		// Encuentro el objetivo
		ArrayList<Observation>[] posiciones = stateObs.getPortalsPositions(stateObs.getAvatarPosition());
		portal = posiciones[0].get(0).position;
		portal.x = Math.floor(portal.x / fescala.x);
		portal.y = Math.floor(portal.y / fescala.y);
		
		A = 0;
		nd_exp = 0;

		camino_encontrado = false;
		prim_acc_real = false;
		camino = new Stack<ACTIONS>();

		// Matriz auxiliar para saber si una posición ha sido visitada
		int iMAX = stateObs.getWorldDimension().width / (int) fescala.x;
		int jMAX = stateObs.getWorldDimension().height / (int) fescala.y;
		Ruta_grid = new boolean[iMAX][jMAX];
		for (int i = 0; i < iMAX; i++) {
			for (int j = 0; j < jMAX; j++)
				Ruta_grid[i][j] = false;
		}
	}

	@Override
	public ACTIONS act(StateObservation stateObs, ElapsedCpuTimer elapsedTimer) {
		// Posicion inicial del avatar
		Vector2d avatar = new Vector2d(stateObs.getAvatarPosition().x / fescala.x,
				stateObs.getAvatarPosition().y / fescala.y);

		Node nodo;
		ACTIONS accion;
		ACTIONS prim_accion = ACTIONS.ACTION_NIL;
		double t = 0;
		boolean terminar = false;

		if (!camino_encontrado) {
			ArrayList<Node> Ruta = new ArrayList<Node>();

			double H = distObjetivo(avatar);
			double cota = H;
			Node nodo_ini = new Node(ACTIONS.ACTION_NIL, null, avatar, 0, H);
			Ruta.add(nodo_ini);
			A++;
			Ruta_grid[(int) nodo_ini.getPos().x][(int) nodo_ini.getPos().y] = true;

			long tInicio = System.nanoTime();
			while (!camino_encontrado) {

				// Iniciamos la búsqueda acotada recursivamente
				t = IDA_search(Ruta, 0, cota, stateObs);
				
				// Actualizamos la cota
				cota = t;

				// Si t = 0, hemos encontrado el camino
				if (t == 0) {
					camino_encontrado = true;
					nodo = nodo_final;
					camino.push(nodo_final.getAccion());
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
					break;
				}
			}
			long tFin = System.nanoTime();
			long tiempoTotal = (tFin - tInicio) / 1000000;
			System.out.println("Runtime: " + tiempoTotal);
			System.out.println("Tamaño ruta: " + camino.size());
			System.out.println("Nodos expandidos: " + nd_exp);
			System.out.println("Máx nodos en memoria: " + A);
		}

		if (camino_encontrado && prim_acc_real)
			if (!camino.isEmpty())
				return camino.pop();

		prim_acc_real = true;
		return prim_accion;
	}

	public double IDA_search(ArrayList<Node> Ruta, double coste, double cota, StateObservation stateObs) {
		
		// Cogemos el nodo a expandir
		Node nodo = Ruta.get(Ruta.size() - 1);
		Node nodo_hijo;
		
		// Actualizamos f
		double f = coste + nodo.getH();
		double t = 0;

		// Si nos pasamos de la cota, retornamos f, que será la nueva cota
		if (f > cota)
			return f;

		// Si hemos encontrado el objetivo, retornamos 0 y terminamos
		if (nodo.getPos().x == portal.x && nodo.getPos().y == portal.y) {
			nodo_final = nodo;
			nd_exp++;
			return 0;
		} else
			nd_exp++;

		double min = 9999.9;
		double newH = 0;

		/* Nodos a expandir recursivamente, los vamos añadiendo a la ruta siempre y cuando no
		 * hayan sido visitados o no sean un obstáculo. Si la respuesta es 0, significa que hemos terminado.
		 * Si la cota es menor al minimo, el nuevo minimo será la cota. Si no hemos encontrado el objetivo
		 * quitamos ese nodo de la ruta pues ya no será explorado
		 */
		Vector2d newPos_up = nodo.getPos(), newPos_down = nodo.getPos(), newPos_left = nodo.getPos(),
				newPos_right = nodo.getPos();
		if (nodo.getPos().y - 1 >= 0) {
			newPos_up = new Vector2d(nodo.getPos().x, nodo.getPos().y - 1);
			newH = this.distObjetivo(newPos_up);
			nodo_hijo = new Node(ACTIONS.ACTION_UP, nodo, newPos_up, nodo.getG() + 1, newH);

			if (!Ruta_grid[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y]
					&& !isObstaculo(stateObs, nodo_hijo)) {
				Ruta.add(nodo_hijo);
				A++;
				Ruta_grid[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] = true;
				t = this.IDA_search(Ruta, nodo_hijo.getG(), cota, stateObs);
				if (t == 0)
					return t;
				if (t < min)
					min = t;
				A--;
				Ruta.remove(nodo_hijo);
				Ruta_grid[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] = false;
			}
		}
		if (nodo.getPos().y + 1 <= stateObs.getObservationGrid()[0].length - 1) {
			newPos_down = new Vector2d(nodo.getPos().x, nodo.getPos().y + 1);
			newH = this.distObjetivo(newPos_down);
			nodo_hijo = new Node(ACTIONS.ACTION_DOWN, nodo, newPos_down, nodo.getG() + 1, newH);

			if (!Ruta_grid[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y]
					&& !isObstaculo(stateObs, nodo_hijo)) {
				Ruta.add(nodo_hijo);
				A++;
				Ruta_grid[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] = true;
				t = this.IDA_search(Ruta, nodo_hijo.getG(), cota, stateObs);
				if (t == 0)
					return t;
				if (t < min)
					min = t;
				A--;
				Ruta.remove(nodo_hijo);
				Ruta_grid[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] = false;
			}
		}
		if (nodo.getPos().x - 1 >= 0) {
			newPos_left = new Vector2d(nodo.getPos().x - 1, nodo.getPos().y);
			newH = this.distObjetivo(newPos_left);
			nodo_hijo = new Node(ACTIONS.ACTION_LEFT, nodo, newPos_left, nodo.getG() + 1, newH);

			if (!Ruta_grid[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y]
					&& !isObstaculo(stateObs, nodo_hijo)) {
				Ruta.add(nodo_hijo);
				A++;
				Ruta_grid[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] = true;
				t = this.IDA_search(Ruta, nodo_hijo.getG(), cota, stateObs);
				if (t == 0)
					return t;
				if (t < min)
					min = t;
				A--;
				Ruta.remove(nodo_hijo);
				Ruta_grid[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] = false;
			}
		}
		if (nodo.getPos().x + 1 <= stateObs.getObservationGrid().length - 1) {
			newPos_right = new Vector2d(nodo.getPos().x + 1, nodo.getPos().y);
			newH = this.distObjetivo(newPos_right);
			nodo_hijo = new Node(ACTIONS.ACTION_RIGHT, nodo, newPos_right, nodo.getG() + 1, newH);

			if (!Ruta_grid[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y]
					&& !isObstaculo(stateObs, nodo_hijo)) {
				Ruta.add(nodo_hijo);
				A++;
				Ruta_grid[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] = true;
				t = this.IDA_search(Ruta, nodo_hijo.getG(), cota, stateObs);
				if (t == 0)
					return t;
				if (t < min)
					min = t;
				A--;
				Ruta.remove(nodo_hijo);
				Ruta_grid[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] = false;
			}
		}
		return min;
	}

	public boolean isObstaculo(StateObservation stateObs, Node nodo) {
		ArrayList<Observation>[][] map = stateObs.getObservationGrid();

		for (Observation obs : map[(int) nodo.getPos().x][(int) nodo.getPos().y])
			if (obs.itype != 3)
				return true;

		return false;
	}

	public double distObjetivo(Vector2d pos_act) {
		return Math.abs(pos_act.x - portal.x) + Math.abs(pos_act.y - portal.y);
	}
}
