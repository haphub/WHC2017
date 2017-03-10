/**
 ***********************************************************************************************
 * @file       Mechanisms.java
 * @author     
 * @version    V0.1.0
 * @date       01-March-2017
 * @brief      Mechanisms class definition, supports base mechanism designs
 ***********************************************************************************************
 * @attention
 *
 *
 ***********************************************************************************************
 */

/* imports ************************************************************************************/
import static java.lang.Math.*;

public class Mechanisms{
	
	/* Mechanisms object fields */
	public 	int 	  l, L, d;
	public 	float 	th1, th2, th3, th4;
	public 	float 	tau1, tau2, tau3, tau4;
	public 	float 	x_E, y_E, z_E;
	
	private float 	pi = 3.14159265359f;
	private float 	J11, J12, J21, J22;
	
 /**
	* @brief    one degree of freedom mechanism constructor
	* @param 	l: arm length
	*/
	public Mechanisms(int l){
		this(l, 0, 0);
	}
	
	
 /**
	* @brief    two degree of freedom mechanism constructor
	* @param 	l: arm length 1
	* @param	L: arm length 2
	* @param	d: motor distance
	*/ 
	public Mechanisms(int l, int L, int d){
		this.l = l;
		this.L = L;
		this.d = d;
	}
	
	
 /**
	* @brief    one degree of freedom forwardKinematics calculations
	* @param 	angle1: relative angle seen at encoder/actuator
	* @note 	incomplete !!!! 
	*/ 
	public void forwardKinematics(float angle1){
	}
	
	
 /**
	* @brief    two degree of freedom forwardKinematics calculations
	* @param 	angle1: relative angle seen at encoder1/actuator2
	* @param	angle2: relative angle seen at encoder2/actuator2
	*/ 
	public void forwardKinematics(float angle1, float angle2){
		
		th1 = pi/180*angle1;
		th2 = pi/180*angle2;
		
		// Forward Kinematics
		float c1 = (float)cos(this.th1);
		float c2 = (float)cos(this.th2);
		float s1 = (float)sin(this.th1);
		float s2 = (float)sin(this.th2);
    
		float xA = l*c1;
		float yA = l*s1;
		float xB = d+l*c2;
		float yB = l*s2;
		float R = (float)pow(xA,2) + (float)pow(yA,2);
		float S = (float)pow(xB,2) + (float)pow(yB,2);
		float M = (yA-yB)/(xB-xA);
		float N = (float)0.5*(S-R)/(xB-xA);
		float a = (float)pow(M,2)+1;
		float b = 2*(M*N-M*xA-yA);
		float c = (float)pow(N,2)-2*N*xA+R-(float)pow(L,2);
		float Delta = (float)pow(b,2)-4*a*c;
		
		y_E = (-b+(float)sqrt(Delta))/(2*a);
		x_E = M*y_E+N;	
		
		float phi1 = (float)acos((x_E-l*c1)/L);
		float phi2 = (float)acos((x_E-d-l*c2)/L);
		float s21 = (float)sin(phi2-phi1);
		float s12 = (float)sin(th1-phi2);
		float s22 = (float)sin(th2-phi2);
		J11 = -(s1*s21 + (float)sin(phi1)*s12)/s21;
		J12 = (c1*s21 + (float)cos(phi1)*s12)/s21;
		J21 = (float)sin(phi1)*s22/s21;
		J22 =-(float)cos(phi1)*s22/s21;
	}
	
	
 /**
	* @brief    two degree of freedom forwardKinematics calculations
	* @param 	angle1: relative angle seen at encoder1/actuator2
	* @param	angle2: relative angle seen at encoder2/actuator2
	* @param	angle3: relative angle seen at encoder3/actuator3
	* @note		incomplete!!!!
	*/ 
	public void forwardKinematics(float angle1, float angle2, float angle3){
	}
	
	
 /**
	* @brief    Torque calculations in 2D space
	* @param 	f_x: end effector x position
	* @param	f_y: end effector y position
	* @param 	divisor: scaling divisor
	*/ 
	public void torqueCalculation(float f_x, float f_y, int divisor){
		
		tau1 = J11*f_x + J12*f_y;
		tau2 = J21*f_x + J22*f_y;
		
		tau2 = -tau2;
		
		tau1 = tau1/divisor;
		tau2 = tau2/divisor;
	}



}
