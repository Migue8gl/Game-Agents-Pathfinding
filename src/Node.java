package tracks.singlePlayer.evaluacion.src_GARCIA_LOPEZ_MIGUEL;

import ontology.Types.ACTIONS;
import tools.Vector2d;

public class Node {
	private String estado;
	private ACTIONS accion;
	private Node padre;
	private Vector2d posicion;
	private double g;
	private double h;

	public Node(ACTIONS acc_ini, Node pad_ini, Vector2d pos_ini) {
		estado = "no_visitado";
		accion = acc_ini;
		padre = pad_ini;
		posicion = pos_ini;
		g = 0;
		h = 0;
	}

	public Node(ACTIONS acc_ini, Node pad_ini, Vector2d pos_ini, double G, double H) {
		estado = "no_visitado";
		accion = acc_ini;
		padre = pad_ini;
		posicion = pos_ini;
		g = G;
		h = H;
	}

	public boolean igual(Node n) {
		boolean eq = false;

		if (this.posicion.x == n.getPos().x && this.posicion.y == n.getPos().y)
			eq = true;
		return eq;
	}

	public double getF() {
		return g + h;
	}

	public double getG() {
		return g;
	}

	public double getH() {
		return h;
	}

	public void setH(double H) {
		h = H;
	}

	public void setG(double G) {
		g = G;
	}

	public Vector2d getPos() {
		return posicion;
	}

	public Node getPadre() {
		return padre;
	}

	public String getEstado() {
		return estado;
	}

	public ACTIONS getAccion() {
		return accion;
	}

	public void setPos(Vector2d pos) {
		posicion = pos;
	}

	public void setPadre(Node pad) {
		padre = pad;
	}

	public void setEstado(String est) {
		estado = est;
	}

	public void setAccion(ACTIONS act) {
		accion = act;
	}
}
