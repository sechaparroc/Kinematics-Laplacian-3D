public class Joint{
  float max_angle;
  float min_angle;  
  float stable_angle;
  float angle;  
  float PI = (float) Math.PI;
  
  public Joint(){
    max_angle = PI;
    min_angle = -1*PI;
    stable_angle = (float) 0.;
    angle = stable_angle;
  }
  public Joint(float min, float max, float st){
    max_angle = max;
    min_angle = min;
    stable_angle = st;
    angle = stable_angle;
  }
  
  public void setMax(float a){
    max_angle = a < min_angle ? max_angle : a;
    angle = angle > max_angle ? max_angle : angle;  
    stable_angle = stable_angle > max_angle ? max_angle : stable_angle;  
  }
  
  public void setMin(float a){
    min_angle = a > max_angle ? min_angle : a;
    angle = angle < min_angle ? min_angle : angle;    
    stable_angle = stable_angle < min_angle ? min_angle : stable_angle;    
  }
  public void setStable(float a){
    stable_angle = a > max_angle || a < min_angle ? stable_angle : a;
  }
  public void setAngle(float a){
    angle = a > max_angle || a < min_angle ? angle : a;  
  }
}
