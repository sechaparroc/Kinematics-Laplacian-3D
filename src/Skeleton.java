import java.util.ArrayList;
import remixlab.proscene.Scene;

public class Skeleton{
	  ArrayList<Bone> bones;
	  Bone frame;//used just for translations
	  Scene scene;
	  
	  public Skeleton(Scene sc){
	    bones = new ArrayList<Bone>();
	    scene = sc;
	  }
	
	  public Skeleton(Scene sc, float x, float y, float z){
		scene = sc;  
	    bones = new ArrayList<Bone>();
	    Bone b = new Bone(scene);
	    b.createBone(x, y, z, this);
	    frame = b;
	  }
	
	  //Cause is expected a short list the exhaustive application of the method is not costly
	  public void updateAngles(){
	    //traverse the list
	    for(Bone b : bones){
	      b.updateAngle();
	    }
	  }
}
