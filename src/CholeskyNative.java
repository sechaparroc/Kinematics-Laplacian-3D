import java.util.ArrayList;

public class CholeskyNative {
	
	static boolean first_time = true;
	
	public static native void sparseCholesky(int[] ri, int[] ci, float[] A, 
			float[] x, float[] b, int n, int nnz, boolean reconstruct);	
	
	public static void getNewCoords(ArrayList<Integer> ri, ArrayList<Integer> ci, ArrayList<Float> v, float[] x, float[] y,
			int n, boolean reconstruct){
		int[] ri_l = new int[ri.size()];
		int[] ci_l = new int[ci.size()];
		float[]  v_l = new float[v.size()];
		for(int i = 0; i < ri.size(); i++){
			ri_l[i] = ri.get(i);ci_l[i] = ci.get(i);v_l[i] = v.get(i);
		}
		sparseCholesky(ri_l, ci_l, v_l, x, y, n, v.size(), first_time);
		first_time = false;
	} 
	
	public static void startNative(){
		System.loadLibrary("SparseCholesky");
	} 
}
