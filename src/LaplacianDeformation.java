import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;

import com.joptimizer.algebra.CholeskyFactorization;

import cern.colt.matrix.DoubleMatrix1D;
import cern.colt.matrix.impl.DenseDoubleMatrix1D;
import cern.colt.matrix.impl.DenseDoubleMatrix2D;
import processing.core.PShape;
import processing.core.PVector;
import remixlab.dandelion.core.MatrixHelper;
import remixlab.dandelion.geom.Vec;
import smile.data.Dataset;
import smile.data.SparseDataset;
import smile.data.parser.SparseDatasetParser;
import smile.math.Math;
import smile.math.SparseArray;
import smile.math.matrix.CholeskyDecomposition;
import smile.math.matrix.IMatrix;
import smile.math.matrix.LUDecomposition;
import smile.math.matrix.Matrix;
import smile.math.matrix.QRDecomposition;
import smile.math.matrix.SparseMatrix;
import smile.util.SmileUtils;

/*
 * November 19 2015
 * Sebastian Chaparro
 * 
 * This is an implementation of Laplacian Surface Deformation 
 * The paper can be found at: 
 * http://igl.ethz.ch/projects/Laplacian-mesh-processing/Laplacian-mesh-editing/laplacian-mesh-editing.pdf
 * 
 * */

public class LaplacianDeformation {
	public HashMap<PVector, Vertex> vertices;
	public ArrayList<Face> faces;	
	public ArrayList<Edge> edges;	
	public ArrayList<Anchor> anchors;
	public SparseDataset A, L, M, M_T; 
	static boolean debug = false;
	public boolean first_iteration = true;
	public CholeskyFactorization ch_t;
	public Matrix LHS;
	
	
	public class Anchor{
		public class AnchorAttribs{
			Bone related_bone;
			PVector initial_pos;
			float weight;
			PVector initial_angle;
			PVector pos;
		}
		
		ArrayList<AnchorAttribs> attribs;		
		Vertex vertex;
		PVector pos;
		float weight = 0;
		int idx;//id of the control point
		
		
		public Anchor(Bone b, Vertex vv, int ii, PVector p_model, float weight){
			attribs = new ArrayList<AnchorAttribs>();
			AnchorAttribs ats = new AnchorAttribs();
			ats.related_bone = b;
			vertex = vv;
			pos = vv.v;
			idx = ii;
			ats.initial_angle = new PVector(0,0,0); 
			ats.initial_angle.x = b.joint_axis_x.angle;
			ats.initial_angle.y = b.joint_axis_y.angle;
			ats.initial_angle.z = b.joint_axis_z.angle;
			ats.initial_pos = p_model;
			ats.weight = weight;
			attribs.add(ats);
		}

		public Anchor(Vertex vv, int ii){
			vertex = vv;
			idx = ii;
		}

		public void addAttrib(Bone b, PVector pVector, float weight) {
			AnchorAttribs ats =	new AnchorAttribs();
			ats.related_bone = b;
			ats.initial_angle = new PVector(0,0,0); 
			ats.initial_angle.x = b.joint_axis_x.angle;
			ats.initial_angle.y = b.joint_axis_y.angle;
			ats.initial_angle.z = b.joint_axis_z.angle;
			ats.initial_pos = pVector;
			ats.weight = weight;
			attribs.add(ats);
		}
	}
	
	public class Face{
		ArrayList<Vertex> vertices = new ArrayList<Vertex>();
	}

	public class Edge{
		Vertex v1;
		Vertex v2;
		
		public Edge(Vertex v, Vertex u){
			v1 = v;
			v2 = u;
		}
	}
	
	public static class Vertex{
		PVector v;
		PVector d;
		ArrayList<int[]> idx_shape;
		int idx; //according to way the vertices are traversed
		ArrayList<Vertex> neighbors;		
		ArrayList<Face> faces;		
		ArrayList<Edge> edges;		
		
		public Vertex(PVector vv, int is, int ic, int idx){
			v = vv;			
			idx_shape = new ArrayList<int[]>();
			int[] idxs = new int[]{ic,is}; 
			idx_shape.add(idxs);
			this.idx = idx;
			neighbors = new ArrayList<Vertex>();
		}
		
		public void addIdxs(int is, int ic){
			int[] idxs = new int[]{ic,is}; 
			idx_shape.add(idxs);
		}
		
		public void addNeighbor(Vertex n){
			if(!neighbors.contains(n)) neighbors.add(n);
			if(!n.neighbors.contains(this)) n.neighbors.add(this);
		}
	}
	
	public void addEdge(SparseDataset A, Vertex v1, Vertex v2){
		//The whole vetex is used as arg if its desired to use other weight scheme
		A.set(v1.idx, v2.idx, 1);
		A.set(v2.idx, v1.idx, 1);		
	}
	
	public void setup(PShape shape){
		getNeighbors(shape);
		getLaplacian();
		anchors = new ArrayList<Anchor>();
	}
	
	public void getNeighbors(PShape shape){
		A = new SparseDataset();
		vertices = new HashMap<PVector,Vertex>();
		edges = new ArrayList<Edge>();
		faces = new ArrayList<Face>();
		int idx = 0;
		
		for(int i = 0; i < shape.getChildCount(); i++){
			PShape child = shape.getChild(i);
			//create a new Face
			Face face = new Face();
			Vertex prev = null;
			for(int j = 0; j < child.getVertexCount(); j++){
				PVector vec = child.getVertex(j);
				if(!vertices.containsKey(vec)){
					vertices.put(vec, new Vertex(vec, j, i, idx));
					idx++;
				}else{
					Vertex v = vertices.get(vec);
					//add the idx of the face and the num
					v.addIdxs(j,i);									
				}
				Vertex v = vertices.get(vec);
				//add edge with the previous vertex
				if(prev != null){
					v.addNeighbor(prev);
					addEdge(A,v,prev);
					edges.add(new Edge(v,prev));
				}
				prev = v;
				//add the vertex to the new face
				face.vertices.add(v);
			}
			//add an edge between the last and the first vertex
			Vertex v0 = vertices.get(child.getVertex(0));
			v0.addNeighbor(prev);
			addEdge(A,v0,prev);			
			edges.add(new Edge(v0,prev));
		}
	}
	
	public void getLaplacian(){
		int n = vertices.size();
		//M is used as the matrix to get the new positions of the vertices
		M = new SparseDataset();
		L = new SparseDataset();
		for(Vertex v_i : vertices.values()){
			double dx = v_i.v.x;
			double dy = v_i.v.y;
			double dz = v_i.v.z;			
			L.set(v_i.idx, v_i.idx, 1);
			M.set(v_i.idx, v_i.idx, 1);
			M.set(v_i.idx + n, v_i.idx + n, 1);
			M.set(v_i.idx + 2*n, v_i.idx + 2*n, 1);
			int degree = v_i.neighbors.size();
			for(Vertex v_j : v_i.neighbors){
				L.set(v_i.idx, v_j.idx, -1./degree);
				dx = dx  -((1./degree) * v_j.v.x);
				dy = dy  -((1./degree) * v_j.v.y);
				dz = dz  -((1./degree) * v_j.v.z);
				M.set(v_i.idx, v_j.idx, -1./degree);
				M.set(v_i.idx + n, v_j.idx + n, -1./degree);				
				M.set(v_i.idx + 2*n, v_j.idx + 2*n, -1./degree);				
			}
			v_i.d = new PVector((float)dx, (float)dy, (float)dz);
		}
		if(debug) printMat("Laplacian", L.toArray());
		if(debug) printMat("Initial M", M.toArray());		
	}
	
	public Vertex getNearest(Collection<Vertex> vertices, PVector p){
		float min_dist = 99999;
		Vertex min = null;
		for(Vertex v : vertices){
			if(PVector.dist(v.v, p) < min_dist){
				min = v;
				min_dist = PVector.dist(v.v, p);
			}
		}
		return min;
	}
		
	public Vertex getFarthest(Collection<Vertex> vertices, PVector p){
		float max_dist = -99999;
		Vertex max = null;
		for(Vertex v : vertices){
			if(PVector.dist(v.v, p) > max_dist){
				max = v;
				max_dist = PVector.dist(v.v, p);
			}
		}
		return max;
	}
	
	public Vertex getNearest(PVector p){
		return getNearest(this.vertices.values(), p);
	}

	public Vertex getFarthest(PVector p){
		return getFarthest(this.vertices.values(), p);
	}

	public Anchor containsAnchor(ArrayList<Anchor> anchors, Vertex v){
		for(Anchor a : anchors){
			if(a.vertex == v) return a;
		}
		return null;
	}

	  public void addAnchorByDist(ArrayList<LaplacianDeformation.Anchor> anchors, Utilities.CustomModelFrame model, Bone b, int i, float percentage){
			PVector p2 = new PVector(b.model_pos.x(), b.model_pos.y(),b.model_pos.z());
			PVector p1 = new PVector(b.parent.model_pos.x(), b.parent.model_pos.y(), b.parent.model_pos.z());
		    float rad = b.radiusX/model.magnitude();
			//orthogonal axis
			//percentage is the total of vertices to add to the anchor
			percentage = percentage > 1  ? 1 : percentage;
			if(debug)System.out.println("num anchors %%%%" + percentage + "num vert" + vertices.size());
			ArrayList<Vertex> sorted = new ArrayList<Vertex>();
			for(Vertex v : vertices.values()){
				float[] dist = Utilities.getDistance(new Vec(v.v.x, v.v.y, v.v.z), b);
				if((percentage/2. >= Math.abs(0.5 - dist[0]) 
						|| (b.children().size() == 0 && dist[0] > 0.5)) && dist[0]<1){
					System.out.println("dist ----- u: " + dist[0] + " -----  dist : " + dist[1]);
					System.out.println("percentage ----- %: " + percentage);
					System.out.println("rad ----- %: " + rad);
					System.out.println("rad ----- %: " + rad*model.magnitude());
					Vec algo = model.coordinatesOf(new Vec(0,0, b.radiusX));
					System.out.println("rad ----- %: " + algo.magnitude());
					if(dist[1] <= rad){
						sorted.add(v);
					}
				}
			}
			Vertex n = getNearest(sorted, p1);
			Vertex f = getFarthest(sorted, p1);		
			//float nearest = PVector.dist(n.v, p1);
			//float farthest = PVector.dist(f.v, p1);
			for(Vertex v : sorted){
				//float weight = Math.abs((farthest - PVector.dist(p1, v.v)))/Math.abs((farthest - nearest ));
				if(debug)System.out.println("################# <<<<<<<<<<<<<---- : d " + PVector.dist(p1, v.v));			
				//if(debug)System.out.println("################# <<<<<<<<<<<<<---- : weight " + weight);
				Vec initial = model.coordinatesOf(b.parent.position().get());
				Anchor anchor = new Anchor(b, v, i, new PVector(initial.x(), initial.y(), initial.z()), 0);
				Anchor a = containsAnchor(this.anchors, v);
				if(a == null){
					anchors.add(anchor);
				}else{
					a.addAttrib(b,new PVector(initial.x(), initial.y(), initial.z()),0);
				}			
			}
	  }	

	
	public void calculateLaplacian(){
		int n = vertices.size();	
		if(debug) printMat("laplacian",L.toArray(),30,30);
		for(Vertex v_i : vertices.values()){
			int num_n = v_i.neighbors.size();			
			double[][] T_data = new double[(num_n+1)*3][7];
			int idx = 0;
            T_data[idx] = 
            		new double[]{v_i.v.x, 0, v_i.v.z, -v_i.v.y, 1, 0, 0};
			T_data[idx + num_n + 1] = 
					new double[]{v_i.v.y, -v_i.v.z, 0, v_i.v.x, 0, 1, 0};			
			T_data[idx + 2*(num_n + 1)] = 
					new double[]{v_i.v.z, v_i.v.y, -v_i.v.x, 0, 0, 0, 1};			
			idx++;
			for(Vertex v_j : v_i.neighbors){
	            T_data[idx] = 
	            		new double[]{v_j.v.x, 0, v_j.v.z, -v_j.v.y, 1, 0, 0};
				T_data[idx + num_n + 1] = 
						new double[]{v_j.v.y, -v_j.v.z, 0, v_j.v.x, 0, 1, 0};			
				T_data[idx + 2*(num_n + 1)] = 
						new double[]{v_j.v.z, v_j.v.y, -v_j.v.x, 0, 0, 0, 1};			
				idx++;
			}
			
			QRDecomposition qr = new QRDecomposition(T_data);
			//Matrix T = new Matrix(T_data);
			//qr.inverse();
			double[][] T_inv = new double[7][(num_n+1)*3];
			qr.solve(Math.eye((num_n+1)*3, (num_n+1)*3), T_inv);
			if(debug)printMat("inverse T implicit",T_inv);
			
			//get the linear transformation coefficients
			double[] s =  T_inv[0];
			double[] h1 = T_inv[1];
			double[] h2 = T_inv[2];
			double[] h3 = T_inv[3];

			//s = new double[]{-1,0,0,-1,0,0};
			//a = new double[]{0,-1,0,0,-1,0};

			//apply the transformation to laplacian coords
			double[][] T_delta = new double[3][(num_n+1)*3];
			for(int i = 0; i < T_delta[0].length; i++){
				T_delta[0][i] =   s[i] * v_i.d.x - h3[i] * v_i.d.y + h2[i] * v_i.d.z;
				T_delta[1][i] =  h3[i] * v_i.d.x +  s[i] * v_i.d.y - h1[i] * v_i.d.z;				
				T_delta[2][i] = -h2[i] * v_i.d.x + h1[i] * v_i.d.y +  s[i] * v_i.d.z;				
			}
			//Update values on M
			idx = 0;
			M.set(v_i.idx      , v_i.idx      , M.get(v_i.idx      , v_i.idx      ) - T_delta[0][idx]);
			M.set(v_i.idx +   n, v_i.idx      , M.get(v_i.idx +   n, v_i.idx      ) - T_delta[1][idx]);
			M.set(v_i.idx + 2*n, v_i.idx      , M.get(v_i.idx + 2*n, v_i.idx      ) - T_delta[2][idx]);			
			M.set(v_i.idx      , v_i.idx +   n, M.get(v_i.idx      , v_i.idx +   n) - T_delta[0][idx + num_n + 1]);
			M.set(v_i.idx +   n, v_i.idx +   n, M.get(v_i.idx +   n, v_i.idx +   n) - T_delta[1][idx + num_n + 1]);
			M.set(v_i.idx + 2*n, v_i.idx +   n, M.get(v_i.idx + 2*n, v_i.idx +   n) - T_delta[2][idx + num_n + 1]);
			M.set(v_i.idx      , v_i.idx + 2*n, M.get(v_i.idx      , v_i.idx + 2*n) - T_delta[0][idx + 2*(num_n + 1)]);
			M.set(v_i.idx +   n, v_i.idx + 2*n, M.get(v_i.idx +   n, v_i.idx + 2*n) - T_delta[1][idx + 2*(num_n + 1)]);
			M.set(v_i.idx + 2*n, v_i.idx + 2*n, M.get(v_i.idx + 2*n, v_i.idx + 2*n) - T_delta[2][idx + 2*(num_n + 1)]);

			idx++;
			for(Vertex v_j : v_i.neighbors){
				M.set(v_i.idx      , v_j.idx      , M.get(v_i.idx      , v_j.idx      ) - T_delta[0][idx]);
				M.set(v_i.idx +   n, v_j.idx      , M.get(v_i.idx +   n, v_j.idx      ) - T_delta[1][idx]);
				M.set(v_i.idx + 2*n, v_j.idx      , M.get(v_i.idx + 2*n, v_j.idx      ) - T_delta[2][idx]);			
				M.set(v_i.idx      , v_j.idx +   n, M.get(v_i.idx      , v_j.idx +   n) - T_delta[0][idx + num_n + 1]);
				M.set(v_i.idx +   n, v_j.idx +   n, M.get(v_i.idx +   n, v_j.idx +   n) - T_delta[1][idx + num_n + 1]);
				M.set(v_i.idx + 2*n, v_j.idx +   n, M.get(v_i.idx + 2*n, v_j.idx +   n) - T_delta[2][idx + num_n + 1]);
				M.set(v_i.idx      , v_j.idx + 2*n, M.get(v_i.idx      , v_j.idx + 2*n) - T_delta[0][idx + 2*(num_n + 1)]);
				M.set(v_i.idx +   n, v_j.idx + 2*n, M.get(v_i.idx +   n, v_j.idx + 2*n) - T_delta[1][idx + 2*(num_n + 1)]);
				M.set(v_i.idx + 2*n, v_j.idx + 2*n, M.get(v_i.idx + 2*n, v_j.idx + 2*n) - T_delta[2][idx + 2*(num_n + 1)]);
				idx++;
			}
		}
	}	
	

	public void getLHS(){
		if(!first_iteration) return;
		int n = vertices.size();			
		SparseDataset M = new SparseDataset();
		SparseDataset M_T = new SparseDataset();
		for(int i = 0; i < this.M.size(); i++){
			for(int j = 0; j < this.M.ncols(); j++){
				double val = this.M.get(i, j);
				M.set(i,j,val);
				M_T.set(j,i,val);					
			}
		}
		int m_dim = M.size();
		ArrayList<Anchor> anchors = new ArrayList<Anchor>();
		anchors.addAll(this.anchors);
		if(debug)System.out.println("m_dim : " + m_dim);		
		for(Anchor anchor : anchors){
			M.set(m_dim, anchor.vertex.idx, anchor.weight);
			M_T.set(anchor.vertex.idx, m_dim, anchor.weight);
			m_dim++;
			M.set(m_dim, anchor.vertex.idx + n, anchor.weight);
			M_T.set(anchor.vertex.idx + n, m_dim, anchor.weight);
			m_dim++;
			M.set(m_dim, anchor.vertex.idx + 2*n, anchor.weight);
			M_T.set(anchor.vertex.idx + 2*n, m_dim, anchor.weight);
			m_dim++;
		}
		//Solve
		System.out.println("antes de mul");
		SparseMatrix MMT = M.toSparseMatrix().transpose().times(M.toSparseMatrix()); 
		this.LHS = new Matrix(matrixToArray(MMT), true, true);

		this.M_T = M_T;
		
		DenseDoubleMatrix2D LHS_T = new DenseDoubleMatrix2D(matrixToArray(LHS));
		//SparseDoubleMatrix2D LHS_T = new SparseDoubleMatrix2D(matrixToArray(LHS));
		//LHS_T.trimToSize();
		ch_t = null;
		try {
			ch_t = new CholeskyFactorization(LHS_T);
		} catch (Exception e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		//CholeskySparseFactorization ch_t = new CholeskySparseFactorization(LHS_T);
		try {
			ch_t.factorize();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		first_iteration = false;
	}
	
	
	public ArrayList<PVector> solveLaplacian(){
		int n = vertices.size();			
		int m_dim = M.size();		
		double[] RHS = new double[m_dim + 3*anchors.size()];		
		for(Anchor anchor : anchors){
			RHS[m_dim++] = anchor.weight*anchor.pos.x;
			RHS[m_dim++] = anchor.weight*anchor.pos.y;
			RHS[m_dim++] = anchor.weight*anchor.pos.z;
		}
		Matrix M_aux = new Matrix(M_T.toArray());		
		System.out.println("despues de mul");

		//double 
		double[] RHSS = new double[M_aux.nrows()];
		M_aux.ax(RHS, RHSS);
		if(debug)printArr("rhs " + RHS.length, RHS);
		if(debug)printArr("new rhs", RHSS);
		if(debug)printMat("m cond", M.toArray());
		double[] new_coords = new double[LHS.ncols()];	
		DenseDoubleMatrix1D RHSS_T = new DenseDoubleMatrix1D(RHSS);
		DoubleMatrix1D new_coords_t = ch_t.solve(RHSS_T);
		new_coords = new_coords_t.toArray();
		//CholeskyDecomposition ch = LHS.cholesky();
		//QRDecomposition ch = new QRDecomposition(M.toArray());
		//ch.solve(RHSS, new_coords);		
		ArrayList<PVector> new_img = new ArrayList<PVector>();
		for(int i = 0; i < n; i++){
			new_img.add(new PVector((float)new_coords[i], (float)new_coords[i+n], (float)new_coords[i+2*n]));
		}
		return new_img;		
	}
	
	public ArrayList<Vertex> sortByDistance(ArrayList<Vertex> vertices, PVector p){
		if(vertices.size() == 1) return vertices;
		if(vertices.size() == 0) return vertices;
		int p_idx = vertices.size()/2;
		Vertex pivot = vertices.get(p_idx); 
		float dist_p = PVector.dist(pivot.v, p);
		//divide
		ArrayList<Vertex> leftside = new ArrayList<Vertex>();
		ArrayList<Vertex> rightside = new ArrayList<Vertex>();
		for(Vertex v : vertices){
			if(v == pivot) continue;
			float dist = PVector.dist(v.v, p);
			if(dist < dist_p){
				leftside.add(v);
			}else{
				rightside.add(v);
			}
		}
		//sort the sublists
		leftside = sortByDistance(leftside, p);
		rightside = sortByDistance(rightside, p);
		//join the sublists
		ArrayList<Vertex> sorted = new ArrayList<Vertex>();
		sorted.addAll(leftside);
		sorted.add(pivot);
		sorted.addAll(rightside);		
		return sorted;
	}
	
	static void printMat(String name, double[][] m){
		System.out.println("---------------------");
		System.out.println(name);		
		System.out.println("---------------------");		
		for(int i = 0; i < m.length; i++){
			for(int j = 0; j < m[0].length; j++){
				System.out.printf("%.2f" + ", " , m[i][j]);
			}
			System.out.println();
		}
		System.out.println("---------------------");		
		System.out.println("---------------------");		
	}

	static void printMat(String name, double[][] m, int r, int c){
		System.out.println("---------------------");
		System.out.println(name);		
		System.out.println("---------------------");
		int rr = Math.min(r, m.length);
		int cc = Math.min(c, m[0].length);		
		for(int i = 0; i < rr; i++){
			for(int j = 0; j < cc; j++){
				System.out.printf("%.2f" + ", \t" , m[i][j]);
			}
			System.out.println();
		}
		System.out.println("---------------------");		
		System.out.println("---------------------");		
	}
	
	
	static void printArr(String name, double[] m){
		System.out.println("---------------------");
		System.out.println(name);		
		System.out.println("---------------------");		
		for(int i = 0; i < m.length; i++){
				System.out.printf("%.2f" + ", " , m[i]);
		}
		System.out.println("---------------------");		
		System.out.println("---------------------");		
	}
	
	static double[][] matrixMult(double[][] m1, double[][] m2){
		int n = m1.length;
		int m = m1[0].length;
		int l = m2[0].length;
		double[][] result = new double[n][l];
		for(int i = 0; i < n; i++){
			for(int k = 0; k < m; k++){
				for(int j = 0; j < l; j++){
					result[i][j] += m1[i][j] * m2[j][k]; 
				}			
			}
		}
		return result;
	}
	
	static double[][] matrixToArray(IMatrix m){
		double[][] result = new double[m.nrows()][m.ncols()];
		for(int i = 0; i < m.nrows(); i++){
			for(int j = 0; j < m.ncols(); j++){
				result[i][j] = m.get(i, j);	
			}
		}
		return result;
	}
}
