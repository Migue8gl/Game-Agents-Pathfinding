package tracks.singlePlayer.evaluacion.src_GARCIA_LOPEZ_MIGUEL;

import java.util.ArrayList;
import java.util.Collections;

import core.game.Observation;
import core.game.StateObservation;
import core.player.AbstractPlayer;
import ontology.Types.ACTIONS;
import tools.ElapsedCpuTimer;
import tools.Vector2d;

public class AgenteRTAStar extends AbstractPlayer {

	private Vector2d fescala;
	private Vector2d portal;
	private int nd_exp;
	private int A;
	private long tTotal;
	private double grid_pos[][];

	public AgenteRTAStar(StateObservation stateObs, ElapsedCpuTimer elapsedTimer) {
		// Cambio de pixeles a coordenadas del mapa
		fescala = new Vector2d(stateObs.getWorldDimension().width / stateObs.getObservationGrid().length,
				stateObs.getWorldDimension().height / stateObs.getObservationGrid()[0].length);

		// Encuentro el objetivo
		ArrayList<Observation>[] posiciones = stateObs.getPortalsPositions(stateObs.getAvatarPosition());
		portal = posiciones[0].get(0).position;
		portal.x = Math.floor(portal.x / fescala.x);
		portal.y = Math.floor(portal.y / fescala.y);

		tTotal = 0;
		nd_exp = 0;
		A = 0;

		int iMAX = stateObs.getWorldDimension().width / (int) fescala.x;
		int jMAX = stateObs.getWorldDimension().height / (int) fescala.y;
		grid_pos = new double[iMAX][jMAX];
		for (int i = 0; i < iMAX; i++) {
			for (int j = 0; j < jMAX; j++)
				grid_pos[i][j] = -1;
		}
	}

	@Override
	public ACTIONS act(StateObservation stateObs, ElapsedCpuTimer elapsedTimer) {
		// Posicion inicial del avatar
		Vector2d avatar = new Vector2d(stateObs.getAvatarPosition().x / fescala.x,
				stateObs.getAvatarPosition().y / fescala.y);

		double H = distObjetivo(avatar);
		Node nodo = new Node(ACTIONS.ACTION_NIL, null, avatar, 0, H);

		Node nodo_hijo;
		boolean meta_alcanzada = false;
		
		// Costes iniciales, al estar restringidos al espacio local de aprendizaje, el coste g siempre es 1
		double costeUp = 1, costeDown = 1, costeRight = 1, costeLeft = 1;
		ACTIONS accion = ACTIONS.ACTION_NIL;
		double min = 999999.9;
		double newH = 0;
		ArrayList<Double> z = new ArrayList<Double>();

		long tInicio = System.nanoTime();
		
		/* Miramos cada nodo a expandir, nos quedamos siempre con el que tenga coste mínimo, siendo
		 * el coste la suma del coste del propio movimiento más el de la heurística, la cuál se va actualizando con
		 * el tiempo (evitando ciclos infinitos). Si el nodo es visitado por primera vez el valor de la 
		 * heurística será la H del propio nodo, es decir,
		 * la distancia Manhattan hacia el objetivo desde ese nodo concreto
		 */
		Vector2d newPos_up = nodo.getPos(), newPos_down = nodo.getPos(), newPos_left = nodo.getPos(),
				newPos_right = nodo.getPos();
		if (nodo.getPos().y - 1 >= 0) {
			newPos_up = new Vector2d(nodo.getPos().x, nodo.getPos().y - 1);
			newH = this.distObjetivo(newPos_up);
			nodo_hijo = new Node(ACTIONS.ACTION_UP, nodo, newPos_up, nodo.getG() + 1, newH);

			if (!isObstaculo(stateObs, nodo_hijo)) {
				if (grid_pos[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] != -1)
					costeUp += grid_pos[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y];
				else
					costeUp += nodo_hijo.getH();

				if (costeUp < min) {
					min = costeUp;
					accion = ACTIONS.ACTION_UP;
				}
				z.add(costeUp);
			}

			if (nodo_hijo.getPos().x == portal.x && nodo_hijo.getPos().y == portal.y)
				meta_alcanzada = true;
		}
		if (nodo.getPos().y + 1 <= stateObs.getObservationGrid()[0].length - 1) {
			newPos_down = new Vector2d(nodo.getPos().x, nodo.getPos().y + 1);
			newH = this.distObjetivo(newPos_down);
			nodo_hijo = new Node(ACTIONS.ACTION_DOWN, nodo, newPos_down, nodo.getG() + 1, newH);

			if (!isObstaculo(stateObs, nodo_hijo)) {
				if (grid_pos[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] != -1)
					costeDown += grid_pos[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y];
				else
					costeDown += nodo_hijo.getH();

				if (costeDown < min) {
					min = costeDown;
					accion = ACTIONS.ACTION_DOWN;
				}
				z.add(costeDown);
			}

			if (nodo_hijo.getPos().x == portal.x && nodo_hijo.getPos().y == portal.y)
				meta_alcanzada = true;
		}
		if (nodo.getPos().x - 1 >= 0) {
			newPos_left = new Vector2d(nodo.getPos().x - 1, nodo.getPos().y);
			newH = this.distObjetivo(newPos_left);
			nodo_hijo = new Node(ACTIONS.ACTION_LEFT, nodo, newPos_left, nodo.getG() + 1, newH);

			if (!isObstaculo(stateObs, nodo_hijo)) {
				if (grid_pos[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] != -1)
					costeLeft += grid_pos[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y];
				else
					costeLeft += nodo_hijo.getH();

				if (costeLeft < min) {
					min = costeLeft;
					accion = ACTIONS.ACTION_LEFT;
				}
				z.add(costeLeft);
			}

			if (nodo_hijo.getPos().x == portal.x && nodo_hijo.getPos().y == portal.y)
				meta_alcanzada = true;
		}
		if (nodo.getPos().x + 1 <= stateObs.getObservationGrid().length - 1) {
			newPos_right = new Vector2d(nodo.getPos().x + 1, nodo.getPos().y);
			newH = this.distObjetivo(newPos_right);
			nodo_hijo = new Node(ACTIONS.ACTION_RIGHT, nodo, newPos_right, nodo.getG() + 1, newH);

			if (!isObstaculo(stateObs, nodo_hijo)) {

				if (grid_pos[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y] != -1)
					costeRight += grid_pos[(int) nodo_hijo.getPos().x][(int) nodo_hijo.getPos().y];
				else
					costeRight += nodo_hijo.getH();

				if (costeRight < min) {
					min = costeRight;
					accion = ACTIONS.ACTION_RIGHT;
				}
				z.add(costeRight);
			}

			if (nodo_hijo.getPos().x == portal.x && nodo_hijo.getPos().y == portal.y)
				meta_alcanzada = true;
		}
		long tFin = System.nanoTime();
		tTotal += (tFin - tInicio) / 1000000;

		Collections.sort(z);

		if (meta_alcanzada) {
			System.out.println("Runtime: " + tTotal);
			System.out.println("Tamaño ruta: " + nd_exp);
			System.out.println("Nodos expandidos: " + nd_exp);
			System.out.println("Máx nodos en memoria: " + A);
		}
		
		if(grid_pos[(int) nodo.getPos().x][(int) nodo.getPos().y] == -1)
			A++;

		// Nos quedamos con el segundo mínimo, si no lo hay, el primero, y actualizamos la matriz de costes heurísticos
		if (z.size() > 1)
			grid_pos[(int) nodo.getPos().x][(int) nodo.getPos().y] = Math.max(z.get(1), nodo.getH());
		else
			grid_pos[(int) nodo.getPos().x][(int) nodo.getPos().y] = Math.max(z.get(0), nodo.getH());

		nd_exp++;
		return accion;
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
